#include <udp/simple_udp.h>

#include <app_config.h>
#include <ardupilotmega/mavlink.h>
#include <gcs_gui_host.h>

#include <array>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <limits>
#include <mutex>
#include <thread>

namespace {

using gcs_ui::DashboardActions;
using gcs_ui::DashboardState;
using gcs_ui::GuiHost;
using gcs_ui::TelemetrySnapshot;

enum class RequestedMode : uint32_t {
	GuidedDisarmed = 0,
	GuidedArmed = 1,
	Land = 2,
};

enum class PendingCommandType {
	ModeChange,
	OverrideGoto,
};

struct PendingCommand {
	PendingCommandType type = PendingCommandType::ModeChange;
	RequestedMode mode = RequestedMode::GuidedDisarmed;
	XYPoint target{};
	float target_altitude_ned_m = 0.0f;
};

struct TeleopState {
	bool enabled = false;
	float x_velocity_mps = 0.0f;
	float y_velocity_mps = 0.0f;
	float z_velocity_mps = 0.0f;
};

IpAddress g_drone_addr{};
SharedConfig g_shared_config;
GcsConfig g_gcs_config;
DroneConfig g_drone_config;
TelemetrySnapshot g_telemetry;

std::atomic_bool g_running{true};
std::mutex g_telemetry_mutex;
std::mutex g_command_mutex;
std::mutex g_teleop_mutex;
std::deque<PendingCommand> g_pending_commands;
TeleopState g_teleop_state;

constexpr size_t kMaxAltitudeHistoryPoints = 512;
constexpr auto kGeofenceFetchInterval = std::chrono::milliseconds(250);
constexpr auto kManualControlInterval = std::chrono::milliseconds(50);

uint8_t gcs_system_id()
{
	return static_cast<uint8_t>(g_drone_config.gcs_system_id);
}

uint8_t gcs_component_id()
{
	return static_cast<uint8_t>(g_drone_config.gcs_component_id);
}

uint8_t drone_system_id()
{
	return static_cast<uint8_t>(g_drone_config.drone_system_id);
}

uint8_t drone_component_id()
{
	return static_cast<uint8_t>(g_drone_config.drone_component_id);
}

void signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		g_running = false;
	}
}

const char* mode_label(RequestedMode mode)
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

void queue_mode_command(RequestedMode mode)
{
	if (mode != RequestedMode::GuidedArmed) {
		std::lock_guard<std::mutex> teleop_lock(g_teleop_mutex);
		g_teleop_state = {};
	}

	std::lock_guard<std::mutex> lock(g_command_mutex);
	g_pending_commands.push_back(PendingCommand{
		PendingCommandType::ModeChange,
		mode,
		{},
		-g_drone_config.arm_target_altitude_m,
	});
}

void set_teleop_enabled(bool enabled)
{
	std::lock_guard<std::mutex> lock(g_teleop_mutex);
	g_teleop_state.enabled = enabled;
	if (!enabled) {
		g_teleop_state.x_velocity_mps = 0.0f;
		g_teleop_state.y_velocity_mps = 0.0f;
		g_teleop_state.z_velocity_mps = 0.0f;
	}
}

void update_teleop_axes(float x_axis, float y_axis, float z_axis)
{
	std::lock_guard<std::mutex> lock(g_teleop_mutex);
	if (!g_teleop_state.enabled) {
		g_teleop_state.x_velocity_mps = 0.0f;
		g_teleop_state.y_velocity_mps = 0.0f;
		g_teleop_state.z_velocity_mps = 0.0f;
		return;
	}

	g_teleop_state.x_velocity_mps = std::clamp(
		x_axis,
		-g_drone_config.manual_horizontal_velocity_mps,
		g_drone_config.manual_horizontal_velocity_mps);
	g_teleop_state.y_velocity_mps = std::clamp(
		y_axis,
		-g_drone_config.manual_horizontal_velocity_mps,
		g_drone_config.manual_horizontal_velocity_mps);
	g_teleop_state.z_velocity_mps = std::clamp(
		z_axis,
		-g_drone_config.manual_vertical_velocity_mps,
		g_drone_config.manual_vertical_velocity_mps);
}

TeleopState get_teleop_state()
{
	std::lock_guard<std::mutex> lock(g_teleop_mutex);
	return g_teleop_state;
}

bool queue_override_goto_command(float x, float y, float altitude_m)
{
	std::lock_guard<std::mutex> lock(g_command_mutex);
	const auto geofence = app_config::sort_geofence_corners(g_drone_config.geofence_corners_m);
	if (!app_config::is_point_inside_polygon(XYPoint{x, y}, geofence) || altitude_m < 0.0f) {
		return false;
	}

	g_pending_commands.push_back(PendingCommand{
		PendingCommandType::OverrideGoto,
		RequestedMode::GuidedArmed,
		XYPoint{x, y},
		-altitude_m,
	});
	return true;
}

bool pop_pending_command(PendingCommand& command)
{
	std::lock_guard<std::mutex> lock(g_command_mutex);
	if (g_pending_commands.empty()) {
		return false;
	}

	command = g_pending_commands.front();
	g_pending_commands.pop_front();
	return true;
}

void update_heartbeat(const mavlink_message_t& msg)
{
	mavlink_heartbeat_t hb{};
	mavlink_msg_heartbeat_decode(&msg, &hb);

	std::lock_guard<std::mutex> lock(g_telemetry_mutex);
	g_telemetry.system_id = msg.sysid;
	g_telemetry.component_id = msg.compid;
	g_telemetry.vehicle_type = hb.type;
	g_telemetry.base_mode = hb.base_mode;
	g_telemetry.custom_mode = hb.custom_mode;
	g_telemetry.system_status = hb.system_status;
	g_telemetry.has_heartbeat = true;
	g_telemetry.last_message_time = std::chrono::steady_clock::now();
}

void update_local_position(const mavlink_message_t& msg)
{
	mavlink_local_position_ned_t pos{};
	mavlink_msg_local_position_ned_decode(&msg, &pos);

	std::lock_guard<std::mutex> lock(g_telemetry_mutex);
	g_telemetry.time_boot_ms = pos.time_boot_ms;
	g_telemetry.x = pos.x;
	g_telemetry.y = pos.y;
	g_telemetry.z = pos.z;
	g_telemetry.vx = pos.vx;
	g_telemetry.vy = pos.vy;
	g_telemetry.vz = pos.vz;
	g_telemetry.has_position = true;
	g_telemetry.last_message_time = std::chrono::steady_clock::now();
	const float altitude_m = -pos.z;
	if (!g_telemetry.altitude_history.empty() && g_telemetry.altitude_history.back().time_boot_ms == pos.time_boot_ms) {
		g_telemetry.altitude_history.back().altitude_m = altitude_m;
	} else {
		g_telemetry.altitude_history.push_back(gcs_ui::AltitudeSample{pos.time_boot_ms, altitude_m});
		if (g_telemetry.altitude_history.size() > kMaxAltitudeHistoryPoints) {
			g_telemetry.altitude_history.pop_front();
		}
	}
}

void update_geofence_point(const mavlink_message_t& msg)
{
	mavlink_fence_point_t fence_point{};
	mavlink_msg_fence_point_decode(&msg, &fence_point);

	if (fence_point.idx >= g_telemetry.geofence.vertices.size()) {
		return;
	}

	std::lock_guard<std::mutex> lock(g_telemetry_mutex);
	g_telemetry.geofence.expected_count = std::min<uint8_t>(
		fence_point.count,
		static_cast<uint8_t>(g_telemetry.geofence.vertices.size()));
	if (!g_telemetry.geofence.received[fence_point.idx]) {
		g_telemetry.geofence.received[fence_point.idx] = true;
		++g_telemetry.geofence.received_count;
	}
	g_telemetry.geofence.vertices[fence_point.idx] = ImVec2(fence_point.lat, fence_point.lng);
	g_telemetry.last_message_time = std::chrono::steady_clock::now();
}

void handle_mavlink_message(const mavlink_message_t& msg)
{
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		update_heartbeat(msg);
		break;
	case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		update_local_position(msg);
		break;
	case MAVLINK_MSG_ID_FENCE_POINT:
		update_geofence_point(msg);
		break;
	default:
		break;
	}
}

bool next_missing_geofence_index(uint8_t& index)
{
	std::lock_guard<std::mutex> lock(g_telemetry_mutex);
	const size_t target_count = g_telemetry.geofence.expected_count == 0
		? g_telemetry.geofence.vertices.size()
		: std::min<size_t>(g_telemetry.geofence.expected_count, g_telemetry.geofence.vertices.size());
	for (size_t point_index = 0; point_index < target_count; ++point_index) {
		if (!g_telemetry.geofence.received[point_index]) {
			index = static_cast<uint8_t>(point_index);
			return true;
		}
	}

	return false;
}

void send_set_mode_command(UdpSocket& socket, RequestedMode mode)
{
	mavlink_message_t msg;
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
	if (mode != RequestedMode::GuidedDisarmed) {
		base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	mavlink_msg_set_mode_pack(
		gcs_system_id(),
		gcs_component_id(),
		&msg,
		drone_system_id(),
		base_mode,
		static_cast<uint32_t>(mode));

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, g_drone_addr);

	std::printf("GCS: Sent SET_MODE command - %s\n", mode_label(mode));
}

void send_override_goto_command(UdpSocket& socket, const XYPoint& target, float target_altitude_ned_m)
{
	mavlink_message_t msg;
	mavlink_msg_command_long_pack(
		gcs_system_id(),
		gcs_component_id(),
		&msg,
		drone_system_id(),
		drone_component_id(),
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
	socket.sendto(tx_buf, len, g_drone_addr);

	std::printf(
		"GCS: Sent MAV_CMD_OVERRIDE_GOTO - X=%.2f Y=%.2f Z=%.2f\n",
		target.x,
		target.y,
		target_altitude_ned_m);
}

void send_geofence_fetch_request(UdpSocket& socket, uint8_t point_index)
{
	mavlink_message_t msg;
	mavlink_msg_fence_fetch_point_pack(
		gcs_system_id(),
		gcs_component_id(),
		&msg,
		drone_system_id(),
		drone_component_id(),
		point_index);

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, g_drone_addr);
}

void send_manual_control(UdpSocket& socket, const TeleopState& teleop_state)
{
	mavlink_message_t msg;
	const float x_normalized = g_drone_config.manual_horizontal_velocity_mps > 0.0f
		? teleop_state.x_velocity_mps / g_drone_config.manual_horizontal_velocity_mps
		: 0.0f;
	const float y_normalized = g_drone_config.manual_horizontal_velocity_mps > 0.0f
		? teleop_state.y_velocity_mps / g_drone_config.manual_horizontal_velocity_mps
		: 0.0f;
	const float z_normalized = g_drone_config.manual_vertical_velocity_mps > 0.0f
		? teleop_state.z_velocity_mps / g_drone_config.manual_vertical_velocity_mps
		: 0.0f;
	const int16_t x = static_cast<int16_t>(std::lround(std::clamp(x_normalized, -1.0f, 1.0f) * 1000.0f));
	const int16_t y = static_cast<int16_t>(std::lround(std::clamp(y_normalized, -1.0f, 1.0f) * 1000.0f));
	const int16_t z = static_cast<int16_t>(std::lround(std::clamp(z_normalized, -1.0f, 1.0f) * 1000.0f));
	mavlink_msg_manual_control_pack(
		gcs_system_id(),
		gcs_component_id(),
		&msg,
		drone_system_id(),
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
	socket.sendto(tx_buf, len, g_drone_addr);
}

void request_geofence_if_needed(UdpSocket& socket)
{
	static auto last_request_time = std::chrono::steady_clock::time_point{};
	const auto now = std::chrono::steady_clock::now();
	if (last_request_time.time_since_epoch().count() != 0 && now - last_request_time < kGeofenceFetchInterval) {
		return;
	}

	uint8_t point_index = 0;
	if (!next_missing_geofence_index(point_index)) {
		return;
	}

	send_geofence_fetch_request(socket, point_index);
	last_request_time = now;
}

void mavlink_worker()
{
	UdpSocket server;
	if (!server.create(g_shared_config.gcs_port, false)) {
		std::fprintf(stderr, "GCS: failed to bind UDP server on port %u\n", g_shared_config.gcs_port);
		g_running = false;
		return;
	}

	std::printf("GCS: listening on %s:%u\n", g_shared_config.host.c_str(), g_shared_config.gcs_port);

	uint8_t rx_buf[2048];
	mavlink_status_t parse_status{};
	auto last_manual_control_time = std::chrono::steady_clock::time_point{};

	while (g_running) {
		PendingCommand command{};
		while (pop_pending_command(command)) {
			if (command.type == PendingCommandType::ModeChange) {
				send_set_mode_command(server, command.mode);
			} else if (command.type == PendingCommandType::OverrideGoto) {
				send_override_goto_command(server, command.target, command.target_altitude_ned_m);
			}
		}

		request_geofence_if_needed(server);

		const TeleopState teleop_state = get_teleop_state();
		const auto now = std::chrono::steady_clock::now();
		if (teleop_state.enabled
			&& (last_manual_control_time.time_since_epoch().count() == 0
				|| now - last_manual_control_time >= kManualControlInterval)) {
			send_manual_control(server, teleop_state);
			last_manual_control_time = now;
		}

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

	server.close();
}

TelemetrySnapshot get_telemetry_snapshot()
{
	std::lock_guard<std::mutex> lock(g_telemetry_mutex);
	return g_telemetry;
}

DashboardState make_dashboard_state()
{
	DashboardState state;
	state.host = g_shared_config.host.c_str();
	state.gcs_port = g_shared_config.gcs_port;
	state.drone_port = g_shared_config.drone_port;
	state.arm_target_altitude_m = g_drone_config.arm_target_altitude_m;
	state.manual_horizontal_velocity_mps = g_drone_config.manual_horizontal_velocity_mps;
	state.manual_vertical_velocity_mps = g_drone_config.manual_vertical_velocity_mps;
	state.is_running = g_running.load();
	state.teleop_enabled = get_teleop_state().enabled;
	state.telemetry = get_telemetry_snapshot();
	return state;
}

DashboardActions make_dashboard_actions()
{
	DashboardActions actions;
	actions.on_arm = [] { queue_mode_command(RequestedMode::GuidedArmed); };
	actions.on_land = [] { queue_mode_command(RequestedMode::Land); };
	actions.on_disarm = [] { queue_mode_command(RequestedMode::GuidedDisarmed); };
	actions.on_send_goto = [](float x, float y, float altitude_m) {
		return queue_override_goto_command(x, y, altitude_m);
	};
	actions.on_set_teleop_enabled = [](bool enabled) { set_teleop_enabled(enabled); };
	actions.on_update_teleop_axes = [](float x_axis, float y_axis, float z_axis) {
		update_teleop_axes(x_axis, y_axis, z_axis);
	};
	return actions;
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


	g_drone_addr = IpAddress{g_shared_config.host.c_str(), g_shared_config.drone_port};

	GuiHost gui;
	if (!gui.initialize()) {
		return 1;
	}

	std::thread mavlink_thread(mavlink_worker);

	while (g_running && !gui.should_close()) {
		gui.begin_frame();
		gui.render_dashboard(make_dashboard_state(), make_dashboard_actions());
		gui.end_frame();
	}

	g_running = false;
	mavlink_thread.join();
	std::printf("GCS: stopped\n");
	return 0;
}
