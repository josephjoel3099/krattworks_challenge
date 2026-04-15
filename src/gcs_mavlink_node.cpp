#include <gcs_mavlink_node.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

GcsMavlinkNode::GcsMavlinkNode(
	const SharedConfig& shared_config,
	const GcsConfig& gcs_config,
	const DroneConfig& drone_config,
	std::atomic_bool& running)
	: shared_config_(shared_config),
	  gcs_config_(gcs_config),
	  drone_config_(drone_config),
	  running_(running),
	  drone_addr_(shared_config.host.c_str(), shared_config.drone_port)
{
	ids_.gcs_system_id = static_cast<uint8_t>(drone_config_.gcs_system_id);
	ids_.gcs_component_id = static_cast<uint8_t>(drone_config_.gcs_component_id);
	ids_.drone_system_id = static_cast<uint8_t>(drone_config_.drone_system_id);
	ids_.drone_component_id = static_cast<uint8_t>(drone_config_.drone_component_id);
}

void GcsMavlinkNode::run()
{
	UdpSocket server;
	if (!server.create(shared_config_.gcs_port, true)) {
		std::fprintf(stderr, "GCS: failed to create UDP server on port %u\n", shared_config_.gcs_port);
		running_ = false;
		return;
	}

	std::printf("GCS: listening on port %u\n", shared_config_.gcs_port);

	mavlink_status_t parse_status{};
	auto last_manual_control_time = std::chrono::steady_clock::time_point{};
	const auto manual_control_interval = std::chrono::milliseconds(gcs_config_.manual_control_interval_ms);

	while (running_) {
		PendingCommand command{};
		while (popPendingCommand(command)) {
			if (command.type == PendingCommandType::ModeChange) {
				sendSetModeCommand(server, command.mode);
			} else if (command.type == PendingCommandType::OverrideGoto) {
				sendOverrideGotoCommand(server, command.target, command.target_altitude_ned_m);
			}
		}

		const auto now = std::chrono::steady_clock::now();
		requestGeofenceIfNeeded(server, now);

		const TeleopState teleop_state = getTeleopState();
		if (teleop_state.enabled
			&& (last_manual_control_time.time_since_epoch().count() == 0
				|| now - last_manual_control_time >= manual_control_interval)) {
			sendManualControl(server, teleop_state);
			last_manual_control_time = now;
		}

		if (!server.poll_read(gcs_config_.poll_timeout_ms)) {
			continue;
		}

		while (server.available() > 0) {
			char buffer[1024];
			IpAddress from;
			const int bytes = server.recvfrom(buffer, sizeof(buffer), from);
			if (bytes <= 0) {
				continue;
			}

			for (int i = 0; i < bytes; ++i) {
				mavlink_message_t msg;
				if (mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(buffer[i]), &msg, &parse_status)) {
					handleMavlinkMessage(msg);
				}
			}
		}
	}

	server.close();
}

const char* GcsMavlinkNode::modeLabel(RequestedMode mode) const
{
	switch (mode) {
	case RequestedMode::GuidedArmed:
		return "ARM";
	case RequestedMode::Land:
		return "LAND";
	case RequestedMode::GuidedDisarmed:
	default:
		return "DISARM";
	}
}

void GcsMavlinkNode::queueModeCommand(RequestedMode mode)
{
	if (mode != RequestedMode::GuidedArmed) {
		std::lock_guard<std::mutex> teleop_lock(teleop_mutex_);
		teleop_state_ = {};
	}

	std::lock_guard<std::mutex> lock(command_mutex_);
	pending_commands_.push_back(PendingCommand{
		PendingCommandType::ModeChange,
		mode,
		{},
		-drone_config_.arm_target_altitude_m,
	});
}

void GcsMavlinkNode::setTeleopEnabled(bool enabled)
{
	std::lock_guard<std::mutex> lock(teleop_mutex_);
	teleop_state_.enabled = enabled;
	if (!enabled) {
		teleop_state_.x_velocity_mps = 0.0f;
		teleop_state_.y_velocity_mps = 0.0f;
		teleop_state_.z_velocity_mps = 0.0f;
	}
}

void GcsMavlinkNode::updateTeleopAxes(float x_axis, float y_axis, float z_axis)
{
	std::lock_guard<std::mutex> lock(teleop_mutex_);
	if (!teleop_state_.enabled) {
		teleop_state_.x_velocity_mps = 0.0f;
		teleop_state_.y_velocity_mps = 0.0f;
		teleop_state_.z_velocity_mps = 0.0f;
		return;
	}

	teleop_state_.x_velocity_mps = std::clamp(
		x_axis,
		-drone_config_.manual_horizontal_velocity_mps,
		drone_config_.manual_horizontal_velocity_mps);
	teleop_state_.y_velocity_mps = std::clamp(
		y_axis,
		-drone_config_.manual_horizontal_velocity_mps,
		drone_config_.manual_horizontal_velocity_mps);
	teleop_state_.z_velocity_mps = std::clamp(
		z_axis,
		-drone_config_.manual_vertical_velocity_mps,
		drone_config_.manual_vertical_velocity_mps);
}

TeleopState GcsMavlinkNode::getTeleopState() const
{
	std::lock_guard<std::mutex> lock(teleop_mutex_);
	return teleop_state_;
}

bool GcsMavlinkNode::queueOverrideGotoCommand(float x, float y, float altitude_m)
{
	std::lock_guard<std::mutex> lock(command_mutex_);
	const auto geofence = app_config::sort_geofence_corners(drone_config_.geofence_corners_m);
	if (!app_config::is_point_inside_polygon(XYPoint{x, y}, geofence) || altitude_m < 0.0f) {
		return false;
	}

	pending_commands_.push_back(PendingCommand{
		PendingCommandType::OverrideGoto,
		RequestedMode::GuidedArmed,
		XYPoint{x, y},
		-altitude_m,
	});
	return true;
}

bool GcsMavlinkNode::popPendingCommand(PendingCommand& command)
{
	std::lock_guard<std::mutex> lock(command_mutex_);
	if (pending_commands_.empty()) {
		return false;
	}

	command = pending_commands_.front();
	pending_commands_.pop_front();
	return true;
}

void GcsMavlinkNode::updateHeartbeat(const mavlink_message_t& msg)
{
	mavlink_heartbeat_t heartbeat{};
	mavlink_msg_heartbeat_decode(&msg, &heartbeat);

	std::lock_guard<std::mutex> lock(telemetry_mutex_);
	telemetry_.system_id = msg.sysid;
	telemetry_.component_id = msg.compid;
	telemetry_.vehicle_type = heartbeat.type;
	telemetry_.base_mode = heartbeat.base_mode;
	telemetry_.custom_mode = heartbeat.custom_mode;
	telemetry_.system_status = heartbeat.system_status;
	telemetry_.has_heartbeat = true;
	telemetry_.last_message_time = std::chrono::steady_clock::now();
}

void GcsMavlinkNode::updateLocalPosition(const mavlink_message_t& msg)
{
	mavlink_local_position_ned_t position{};
	mavlink_msg_local_position_ned_decode(&msg, &position);

	std::lock_guard<std::mutex> lock(telemetry_mutex_);
	telemetry_.time_boot_ms = position.time_boot_ms;
	telemetry_.x = position.x;
	telemetry_.y = position.y;
	telemetry_.z = position.z;
	telemetry_.vx = position.vx;
	telemetry_.vy = position.vy;
	telemetry_.vz = position.vz;
	telemetry_.has_position = true;
	telemetry_.last_message_time = std::chrono::steady_clock::now();

	const float altitude_m = -position.z;
	if (!telemetry_.altitude_history.empty() && telemetry_.altitude_history.back().time_boot_ms == position.time_boot_ms) {
		telemetry_.altitude_history.back().altitude_m = altitude_m;
	} else {
		telemetry_.altitude_history.push_back(gcs_ui::AltitudeSample{position.time_boot_ms, altitude_m});
		while (telemetry_.altitude_history.size() > static_cast<size_t>(gcs_config_.max_altitude_history_points)) {
			telemetry_.altitude_history.pop_front();
		}
	}
}

void GcsMavlinkNode::updateGeofencePoint(const mavlink_message_t& msg)
{
	mavlink_fence_point_t fence_point{};
	mavlink_msg_fence_point_decode(&msg, &fence_point);
	if (fence_point.idx >= telemetry_.geofence.vertices.size()) {
		return;
	}

	std::lock_guard<std::mutex> lock(telemetry_mutex_);
	telemetry_.geofence.expected_count = std::min<uint8_t>(
		fence_point.count,
		static_cast<uint8_t>(telemetry_.geofence.vertices.size()));
	if (!telemetry_.geofence.received[fence_point.idx]) {
		telemetry_.geofence.received[fence_point.idx] = true;
		++telemetry_.geofence.received_count;
	}
	telemetry_.geofence.vertices[fence_point.idx] = ImVec2(fence_point.lat, fence_point.lng);
	telemetry_.last_message_time = std::chrono::steady_clock::now();
}

void GcsMavlinkNode::handleMavlinkMessage(const mavlink_message_t& msg)
{
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		updateHeartbeat(msg);
		break;
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		updateLocalPosition(msg);
		break;
	case MAVLINK_MSG_ID_FENCE_POINT:
		updateGeofencePoint(msg);
		break;
	default:
		break;
	}
}

bool GcsMavlinkNode::nextMissingGeofenceIndex(uint8_t& index) const
{
	std::lock_guard<std::mutex> lock(telemetry_mutex_);
	const size_t target_count = telemetry_.geofence.expected_count == 0
		? telemetry_.geofence.vertices.size()
		: std::min<size_t>(telemetry_.geofence.expected_count, telemetry_.geofence.vertices.size());

	for (size_t point_index = 0; point_index < target_count; ++point_index) {
		if (!telemetry_.geofence.received[point_index]) {
			index = static_cast<uint8_t>(point_index);
			return true;
		}
	}

	return false;
}

void GcsMavlinkNode::sendSetModeCommand(UdpSocket& socket, RequestedMode mode) const
{
	mavlink_message_t msg;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
	if (mode != RequestedMode::GuidedDisarmed) {
		base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	mavlink_msg_set_mode_pack(
		ids_.gcs_system_id,
		ids_.gcs_component_id,
		&msg,
		ids_.drone_system_id,
		base_mode,
		static_cast<uint32_t>(mode));

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, drone_addr_);

	std::printf("GCS: Sent SET_MODE command - %s\n", modeLabel(mode));
}

void GcsMavlinkNode::sendOverrideGotoCommand(
	UdpSocket& socket,
	const XYPoint& target,
	float target_altitude_ned_m) const
{
	mavlink_message_t msg;
	mavlink_msg_command_long_pack(
		ids_.gcs_system_id,
		ids_.gcs_component_id,
		&msg,
		ids_.drone_system_id,
		ids_.drone_component_id,
		MAV_CMD_OVERRIDE_GOTO,
		0,
		static_cast<float>(MAV_GOTO_DO_HOLD),
		static_cast<float>(MAV_GOTO_HOLD_AT_SPECIFIED_POSITION),
		static_cast<float>(MAV_FRAME_LOCAL_NED),
		std::numeric_limits<float>::quiet_NaN(),
		target.x,
		target.y,
		target_altitude_ned_m);

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, drone_addr_);

	std::printf(
		"GCS: Sent MAV_CMD_OVERRIDE_GOTO - X=%.2f Y=%.2f Z=%.2f\n",
		target.x,
		target.y,
		target_altitude_ned_m);
}

void GcsMavlinkNode::sendGeofenceFetchRequest(UdpSocket& socket, uint8_t point_index) const
{
	mavlink_message_t msg;
	mavlink_msg_fence_fetch_point_pack(
		ids_.gcs_system_id,
		ids_.gcs_component_id,
		&msg,
		ids_.drone_system_id,
		ids_.drone_component_id,
		point_index);

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, drone_addr_);
}

void GcsMavlinkNode::sendManualControl(UdpSocket& socket, const TeleopState& teleop_state) const
{
	mavlink_message_t msg;
	const float x_normalized = drone_config_.manual_horizontal_velocity_mps > 0.0f
		? teleop_state.x_velocity_mps / drone_config_.manual_horizontal_velocity_mps
		: 0.0f;
	const float y_normalized = drone_config_.manual_horizontal_velocity_mps > 0.0f
		? teleop_state.y_velocity_mps / drone_config_.manual_horizontal_velocity_mps
		: 0.0f;
	const float z_normalized = drone_config_.manual_vertical_velocity_mps > 0.0f
		? teleop_state.z_velocity_mps / drone_config_.manual_vertical_velocity_mps
		: 0.0f;
	const int16_t x = static_cast<int16_t>(std::lround(std::clamp(x_normalized, -1.0f, 1.0f) * 1000.0f));
	const int16_t y = static_cast<int16_t>(std::lround(std::clamp(y_normalized, -1.0f, 1.0f) * 1000.0f));
	const int16_t z = static_cast<int16_t>(std::lround(std::clamp(z_normalized, -1.0f, 1.0f) * 1000.0f));

	mavlink_msg_manual_control_pack(
		ids_.gcs_system_id,
		ids_.gcs_component_id,
		&msg,
		ids_.drone_system_id,
		x,
		y,
		z,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0);

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, drone_addr_);
}

void GcsMavlinkNode::requestGeofenceIfNeeded(UdpSocket& socket, const std::chrono::steady_clock::time_point& now)
{
	const auto geofence_interval = std::chrono::milliseconds(gcs_config_.geofence_fetch_interval_ms);
	if (last_geofence_request_time_.time_since_epoch().count() != 0
		&& now - last_geofence_request_time_ < geofence_interval) {
		return;
	}

	uint8_t point_index = 0;
	if (!nextMissingGeofenceIndex(point_index)) {
		return;
	}

	sendGeofenceFetchRequest(socket, point_index);
	last_geofence_request_time_ = now;
}

gcs_ui::DashboardState GcsMavlinkNode::makeDashboardState() const
{
	gcs_ui::DashboardState state;
	state.host = shared_config_.host.c_str();
	state.gcs_port = shared_config_.gcs_port;
	state.drone_port = shared_config_.drone_port;
	state.arm_target_altitude_m = drone_config_.arm_target_altitude_m;
	state.manual_horizontal_velocity_mps = drone_config_.manual_horizontal_velocity_mps;
	state.manual_vertical_velocity_mps = drone_config_.manual_vertical_velocity_mps;
	state.telemetry_stale_timeout_ms = gcs_config_.telemetry_stale_timeout_ms;
	state.is_running = running_.load();
	state.teleop_enabled = getTeleopState().enabled;

	std::lock_guard<std::mutex> lock(telemetry_mutex_);
	state.telemetry = telemetry_;
	return state;
}

gcs_ui::DashboardActions GcsMavlinkNode::makeDashboardActions()
{
	gcs_ui::DashboardActions actions;
	actions.on_arm = [this] { queueModeCommand(RequestedMode::GuidedArmed); };
	actions.on_land = [this] { queueModeCommand(RequestedMode::Land); };
	actions.on_disarm = [this] { queueModeCommand(RequestedMode::GuidedDisarmed); };
	actions.on_send_goto = [this](float x, float y, float altitude_m) {
		return queueOverrideGotoCommand(x, y, altitude_m);
	};
	actions.on_set_teleop_enabled = [this](bool enabled) { setTeleopEnabled(enabled); };
	actions.on_update_teleop_axes = [this](float x_axis, float y_axis, float z_axis) {
		updateTeleopAxes(x_axis, y_axis, z_axis);
	};
	return actions;
}
