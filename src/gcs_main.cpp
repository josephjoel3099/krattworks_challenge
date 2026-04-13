#include <udp/simple_udp.h>

#include <common/mavlink.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <string>

namespace {

std::atomic_bool g_running{true};
UdpSocket* g_socket_ptr = nullptr;
IpAddress g_drone_addr{"127.0.0.1", 14551};

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
		std::printf(
			"GCS: HEARTBEAT sys=%u comp=%u type=%u mode=0x%02X state=%u\n",
			msg.sysid,
			msg.compid,
			hb.type,
			hb.base_mode,
			hb.system_status);
		break;
	}
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
		mavlink_local_position_ned_t pos{};
		mavlink_msg_local_position_ned_decode(&msg, &pos);
		std::printf(
			"GCS: LOCAL_POSITION_NED t=%u x=%.2f y=%.2f z=%.2f vx=%.2f vy=%.2f vz=%.2f\n",
			pos.time_boot_ms,
			pos.x,
			pos.y,
			pos.z,
			pos.vx,
			pos.vy,
			pos.vz);
		break;
	}
	default:
		break;
	}
}

void send_set_mode_command(bool arm)
{
	if (!g_socket_ptr) return;

	constexpr uint8_t kGcsSystemId = 255;
	constexpr uint8_t kGcsComponentId = MAV_COMP_ID_SYSTEM_CONTROL;
	constexpr uint8_t kDroneSystemId = 1;

	mavlink_message_t msg;
	mavlink_msg_set_mode_pack(
		kGcsSystemId,
		kGcsComponentId,
		&msg,
		kDroneSystemId,
		arm ? MAV_MODE_GUIDED_ARMED : MAV_MODE_GUIDED_DISARMED,
		0);

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	g_socket_ptr->sendto(tx_buf, len, g_drone_addr);

	std::printf("GCS: Sent SET_MODE command - %s\n", arm ? "ARM" : "DISARM");
}

void command_input_thread()
{
	std::printf("GCS: Commands available:\n");
	std::printf("  'a' - ARM drone (will climb to 20m)\n");
	std::printf("  'd' - DISARM drone\n");
	std::printf("  'q' - quit\n");

	while (g_running) {
		char cmd;
		if (scanf("%c", &cmd) != 1) {
			continue;
		}

		switch (cmd) {
		case 'a':
		case 'A':
			send_set_mode_command(true);
			break;
		case 'd':
		case 'D':
			send_set_mode_command(false);
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

	constexpr uint16_t kGcsPort = 14550;
	UdpSocket server;
	if (!server.create(kGcsPort, false)) {
		std::fprintf(stderr, "GCS: failed to bind UDP server on port %u\n", kGcsPort);
		return 1;
	}

	g_socket_ptr = &server;

	std::printf("GCS: listening on 127.0.0.1:%u\n", kGcsPort);

	// Start command input thread
	std::thread cmd_thread(command_input_thread);

	uint8_t rx_buf[2048];
	mavlink_status_t parse_status{};

	while (g_running) {
		if (!server.poll_read(100)) {
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
