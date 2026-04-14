#include <udp/simple_udp.h>

#include <app_config.h>
#include <common/mavlink.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <string>

namespace {

enum class RequestedMode : uint32_t {
	GuidedDisarmed = 0,
	GuidedArmed = 1,
	Land = 2,
};

std::atomic_bool g_running{true};
UdpSocket* g_socket_ptr = nullptr;
IpAddress g_drone_addr{};
SharedConfig g_shared_config;
GcsConfig g_gcs_config;
DroneConfig g_drone_config;

void signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		g_running = false;
	}
}

void handle_mavlink_message(const mavlink_message_t& msg)
{
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t hb{};
		mavlink_msg_heartbeat_decode(&msg, &hb);
		static bool hb_tick = false;
		hb_tick = !hb_tick;
		std::printf(
			"\rGCS: HEARTBEAT [%c] sys=%u comp=%u type=%u mode=0x%02X custom=%u state=%u   ",
			hb_tick ? '*' : ' ',
			msg.sysid,
			msg.compid,
			hb.type,
			hb.base_mode,
			hb.custom_mode,
			hb.system_status);
		std::fflush(stdout);
		break;
	}
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
		mavlink_local_position_ned_t pos{};
		mavlink_msg_local_position_ned_decode(&msg, &pos);
		static bool pos_tick = false;
		pos_tick = !pos_tick;
		std::printf(
			"\rGCS: LOCAL_POSITION_NED [%c] t=%u x=%.2f y=%.2f z=%.2f vx=%.2f vy=%.2f vz=%.2f   ",
			pos_tick ? '*' : ' ',
			pos.time_boot_ms,
			pos.x,
			pos.y,
			pos.z,
			pos.vx,
			pos.vy,
			pos.vz);
		std::fflush(stdout);
		break;
	}
	default:
		break;
	}
}

void send_set_mode_command(RequestedMode mode)
{
	if (!g_socket_ptr) return;

	constexpr uint8_t kGcsSystemId = 255;
	constexpr uint8_t kGcsComponentId = MAV_COMP_ID_SYSTEM_CONTROL;
	constexpr uint8_t kDroneSystemId = 1;

	mavlink_message_t msg;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
	if (mode != RequestedMode::GuidedDisarmed) {
		base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	mavlink_msg_set_mode_pack(
		kGcsSystemId,
		kGcsComponentId,
		&msg,
		kDroneSystemId,
		base_mode,
		static_cast<uint32_t>(mode));

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	g_socket_ptr->sendto(tx_buf, len, g_drone_addr);

	const char* label = "DISARM";
	if (mode == RequestedMode::GuidedArmed) {
		label = "ARM";
	} else if (mode == RequestedMode::Land) {
		label = "LAND";
	}

	std::printf("GCS: Sent SET_MODE command - %s\n", label);
}

void command_input_thread()
{
	std::printf("GCS: Commands available:\n");
	std::printf("  'a' - ARM drone (target altitude %.1fm)\n", g_drone_config.arm_target_altitude_m);
	std::printf("  'l' - LAND drone\n");
	std::printf("  'd' - DISARM drone (lands first if airborne)\n");
	std::printf("  'q' - quit\n");

	while (g_running) {
		char cmd;
		if (scanf("%c", &cmd) != 1) {
			continue;
		}

		switch (cmd) {
		case 'a':
		case 'A':
			send_set_mode_command(RequestedMode::GuidedArmed);
			break;
		case 'l':
		case 'L':
			send_set_mode_command(RequestedMode::Land);
			break;
		case 'd':
		case 'D':
			send_set_mode_command(RequestedMode::GuidedDisarmed);
			break;
		case 'q':
		case 'Q':
			g_running = false;
			break;
		default:
			break;
		}
	}
}

} // namespace

int main()
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	// Provisioning
	g_shared_config = app_config::load_shared_config();
	g_gcs_config = app_config::load_gcs_config();
	g_drone_config = app_config::load_drone_config();

	if (!app_config::find_config_path("shared_config.json").has_value() || !app_config::is_valid(g_shared_config)) {
		std::fprintf(stderr, "GCS: shared_config.json is missing or invalid\n");
		return 1;
	}

	if (!app_config::find_config_path("gcs_config.json").has_value() || !app_config::is_valid(g_gcs_config)) {
		std::fprintf(stderr, "GCS: gcs_config.json is missing or invalid\n");
		return 1;
	}

	if (!app_config::find_config_path("drone_config.json").has_value() || !app_config::is_valid(g_drone_config)) {
		std::fprintf(stderr, "GCS: drone_config.json is missing or invalid\n");
		return 1;
	}

	g_drone_addr = IpAddress{g_shared_config.host.c_str(), g_gcs_config.drone_port};

	UdpSocket server;
	if (!server.create(g_gcs_config.gcs_port, false)) {
		std::fprintf(stderr, "GCS: failed to bind UDP server on port %u\n", g_gcs_config.gcs_port);
		return 1;
	}

	g_socket_ptr = &server;

	std::printf("GCS: listening on %s:%u\n", g_shared_config.host.c_str(), g_gcs_config.gcs_port);

	// Start command input thread
	std::thread cmd_thread(command_input_thread);

	uint8_t rx_buf[2048];
	mavlink_status_t parse_status{};

	while (g_running) {
		if (!server.poll_read(g_gcs_config.poll_timeout_ms)) {
			continue;
		}

		while (server.available() > 0) {
			IpAddress from;
			const int bytes = server.recvfrom(rx_buf, sizeof(rx_buf), from);
			if (bytes <= 0) {
				continue;
			}

			for (int i = 0; i < bytes; ++i) {
				mavlink_message_t msg;
				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &parse_status)) {
					handle_mavlink_message(msg);
				}
			}
		}
	}

	cmd_thread.join();
	server.close();
	std::printf("GCS: stopped\n");
	return 0;
}
