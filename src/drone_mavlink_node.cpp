#include <drone_mavlink_node.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

DroneMavlinkNode::DroneMavlinkNode(
	DroneTelemetrySimulator& telemetry,
	const SharedConfig& shared_config,
	const DroneConfig& drone_config,
	std::atomic_bool& running)
	: telemetry_(telemetry),
	  shared_config_(shared_config),
	  drone_config_(drone_config),
	  running_(running)
{
	ids_.drone_system_id = static_cast<uint8_t>(drone_config_.drone_system_id);
	ids_.drone_component_id = static_cast<uint8_t>(drone_config_.drone_component_id);
	ids_.gcs_system_id = static_cast<uint8_t>(drone_config_.gcs_system_id);
	ids_.gcs_component_id = static_cast<uint8_t>(drone_config_.gcs_component_id);
}

void DroneMavlinkNode::run()
{
	UdpSocket client;
	if (!client.create(shared_config_.drone_port, true)) {
		std::fprintf(stderr, "Drone: failed to create UDP client on port %u\n", shared_config_.drone_port);
		return;
	}

	const IpAddress gcs_addr{shared_config_.host.c_str(), shared_config_.gcs_port};
	std::printf(
		"Drone: UDP client created on %s:%u, sending to %s\n",
		shared_config_.host.c_str(),
		shared_config_.drone_port,
		gcs_addr.to_string().c_str());

	using clock = std::chrono::steady_clock; // sim clock
	const auto start = clock::now();
	auto next_hb = start;
	auto next_pos = start;
	const auto hb_interval = std::chrono::duration<double>(
		1.0 / std::max(0.1, static_cast<double>(drone_config_.heartbeat_rate_hz)));
	const auto pos_interval = std::chrono::duration<double>(
		1.0 / std::max(0.1, static_cast<double>(drone_config_.position_rate_hz)));

	mavlink_status_t parse_status{};

	while (running_) {
		const auto now = clock::now();

		if (client.poll_read(std::max(1, drone_config_.rx_poll_timeout_ms))) {
			while (client.available() > 0) {
				char buffer[1024];
				IpAddress from;
				const int bytes = client.recvfrom(buffer, sizeof(buffer), from);
				if (bytes <= 0) {
					continue;
				}

				for (int i = 0; i < bytes; ++i) {
					mavlink_message_t msg;
					if (mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(buffer[i]), &msg, &parse_status)) {
						handleMavlinkCommand(msg, client, gcs_addr);
					}
				}
			}
		}

		if (now >= next_hb) {
			telemetry_.sendHeartbeat();

			mavlink_message_t hb_msg;
			mavlink_msg_heartbeat_pack(
				ids_.drone_system_id,
				ids_.drone_component_id,
				&hb_msg,
				MAV_TYPE_QUADROTOR,
				MAV_AUTOPILOT_GENERIC,
				telemetry_.getMode() == MAVMode::GUIDED_DISARMED ? MAV_MODE_GUIDED_DISARMED : MAV_MODE_GUIDED_ARMED,
				static_cast<uint32_t>(telemetry_.getMode()),
				MAV_STATE_ACTIVE);
			sendMavlinkMessage(client, gcs_addr, hb_msg);
			next_hb += std::chrono::duration_cast<clock::duration>(hb_interval);
		}

		if (now >= next_pos) {
			telemetry_.sendLocalPositionNED();

			mavlink_message_t pos_msg;
			mavlink_msg_local_position_ned_pack(
				ids_.drone_system_id,
				ids_.drone_component_id,
				&pos_msg,
				static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()),
				telemetry_.getX(),
				telemetry_.getY(),
				telemetry_.getAltitude(),
				telemetry_.getVx(),
				telemetry_.getVy(),
				telemetry_.getVz());
			sendMavlinkMessage(client, gcs_addr, pos_msg);
			next_pos += std::chrono::duration_cast<clock::duration>(pos_interval);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	client.close();
}

void DroneMavlinkNode::sendMavlinkMessage(
	UdpSocket& socket,
	const IpAddress& gcs_addr,
	const mavlink_message_t& msg) const
{
	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, gcs_addr);
}

void DroneMavlinkNode::sendGeofencePoint(UdpSocket& socket, const IpAddress& gcs_addr, uint8_t point_index) const
{
	if (point_index >= drone_config_.geofence_corners_m.size()) {
		return;
	}

	const XYPoint& point = drone_config_.geofence_corners_m[point_index];
	mavlink_message_t fence_msg;
	mavlink_msg_fence_point_pack(
		ids_.drone_system_id,
		ids_.drone_component_id,
		&fence_msg,
		ids_.gcs_system_id,
		ids_.gcs_component_id,
		point_index,
		static_cast<uint8_t>(drone_config_.geofence_corners_m.size()),
		point.x,
		point.y);
	sendMavlinkMessage(socket, gcs_addr, fence_msg);
}

void DroneMavlinkNode::handleMavlinkCommand(
	const mavlink_message_t& msg,
	UdpSocket& socket,
	const IpAddress& gcs_addr)
{
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_SET_MODE: {
		mavlink_set_mode_t cmd{};
		mavlink_msg_set_mode_decode(&msg, &cmd);
		std::printf(
			"Drone: SET_MODE command received - base_mode=0x%02X custom_mode=%u\n",
			cmd.base_mode,
			cmd.custom_mode);

		if (cmd.custom_mode == static_cast<uint32_t>(MAVMode::LAND)) {
			telemetry_.setMode(MAVMode::LAND);
		} else if ((cmd.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) && (cmd.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
			telemetry_.setMode(MAVMode::GUIDED_ARMED);
		} else if ((cmd.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) && !(cmd.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
			telemetry_.setMode(MAVMode::GUIDED_DISARMED);
		}
		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG: {
		mavlink_command_long_t command{};
		mavlink_msg_command_long_decode(&msg, &command);
		if (command.target_system != ids_.drone_system_id || command.target_component != ids_.drone_component_id) {
			break;
		}

		if (command.command != MAV_CMD_OVERRIDE_GOTO) {
			break;
		}

		if (static_cast<int>(command.param1) == MAV_GOTO_DO_CONTINUE) {
			telemetry_.holdCurrentPosition();
			std::printf("\nDrone: MAV_CMD_OVERRIDE_GOTO continue treated as hold-current\n");
			break;
		}

		if (static_cast<int>(command.param2) == MAV_GOTO_HOLD_AT_CURRENT_POSITION) {
			telemetry_.holdCurrentPosition();
			std::printf("\nDrone: holding current position\n");
			break;
		}

		if (static_cast<int>(command.param2) != MAV_GOTO_HOLD_AT_SPECIFIED_POSITION
			|| static_cast<int>(command.param3) != MAV_FRAME_LOCAL_NED) {
			std::printf("\nDrone: rejected MAV_CMD_OVERRIDE_GOTO with unsupported parameters\n");
			break;
		}

		if (!telemetry_.setGuidedHoldTarget(command.param5, command.param6, command.param7)) {
			std::printf("\nDrone: rejected MAV_CMD_OVERRIDE_GOTO target outside geofence or while not armed\n");
		}
		break;
	}
	case MAVLINK_MSG_ID_MANUAL_CONTROL: {
		mavlink_manual_control_t control{};
		mavlink_msg_manual_control_decode(&msg, &control);
		if (control.target != ids_.drone_system_id) {
			break;
		}

		if (!telemetry_.setManualControl(
			std::clamp(static_cast<float>(control.x) / 1000.0f, -1.0f, 1.0f),
			std::clamp(static_cast<float>(control.y) / 1000.0f, -1.0f, 1.0f),
			std::clamp(static_cast<float>(control.z) / 1000.0f, -1.0f, 1.0f))) {
			std::printf("\nDrone: ignored MANUAL_CONTROL while not in GUIDED_ARMED\n");
		}
		break;
	}
	case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
		mavlink_fence_fetch_point_t request{};
		mavlink_msg_fence_fetch_point_decode(&msg, &request);
		if (request.target_system != ids_.drone_system_id || request.target_component != ids_.drone_component_id) {
			break;
		}

		sendGeofencePoint(socket, gcs_addr, request.idx);
		break;
	}
	default:
		break;
	}
}
