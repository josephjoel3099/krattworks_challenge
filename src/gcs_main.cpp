#include <udp/simple_udp.h>

#include <common/mavlink.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>

namespace {

std::atomic_bool g_running{true};

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

	std::printf("GCS: listening on 127.0.0.1:%u\n", kGcsPort);

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

	server.close();
	std::printf("GCS: stopped\n");
	return 0;
}
