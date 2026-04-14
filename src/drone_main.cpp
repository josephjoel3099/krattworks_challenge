#include <udp/simple_udp.h>

#include <app_config.h>
#include <ardupilotmega/mavlink.h>
#include <drone_telemtry_abc.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <memory>
#include <thread>

std::atomic_bool g_running{true};

SharedConfig g_shared_config;
DroneConfig g_drone_config;

namespace {

constexpr float kAltitudeArrivalToleranceM = 0.5f;
constexpr float kHorizontalArrivalToleranceM = 0.75f;
constexpr uint8_t kDroneSystemId = 1;
constexpr uint8_t kDroneComponentId = MAV_COMP_ID_AUTOPILOT1;
constexpr uint8_t kGcsSystemId = 255;
constexpr uint8_t kGcsComponentId = MAV_COMP_ID_SYSTEM_CONTROL;

} // namespace

// Concrete implementation of DroneTelemetryABC
class DroneTelemetrySimulator : public DroneTelemetryABC {
private:
	mutable std::mutex state_mutex_;
	float x_ = 0.0f;
	float y_ = 0.0f;
	float altitude_ = 0.0f;
	float target_altitude_ = 0.0f;
	float vx_ = 0.0f;
	float vy_ = 0.0f;
	float vz_ = 0.0f;
	float max_velocity_mps_;
	float climb_rate_mps_;
	float land_rate_mps_;
	float arm_target_altitude_m_;
	std::array<XYPoint, 4> geofence_corners_;
	XYPoint hold_target_{};
	MAVMode mode_ = MAVMode::GUIDED_DISARMED;
	std::chrono::steady_clock::time_point last_update_time_;

public:
	explicit DroneTelemetrySimulator(
		float max_velocity_mps,
		float climb_rate_mps,
		float land_rate_mps,
		float arm_target_altitude_m,
		const std::array<XYPoint, 4>& geofence_corners)
		: max_velocity_mps_(max_velocity_mps),
		  climb_rate_mps_(climb_rate_mps),
		  land_rate_mps_(land_rate_mps),
		  arm_target_altitude_m_(arm_target_altitude_m),
		  geofence_corners_(app_config::sort_geofence_corners(geofence_corners)) {
		const XYPoint launch_position = app_config::compute_polygon_centroid(geofence_corners_);
		x_ = launch_position.x;
		y_ = launch_position.y;
		hold_target_ = launch_position;
		last_update_time_ = std::chrono::steady_clock::now();
	}

	float getX() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return x_;
	}

	float getY() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return y_;
	}

	float getAltitude() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return altitude_;
	}

	MAVMode getMode() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return mode_;
	}

	void setMode(MAVMode mode) override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (mode == MAVMode::GUIDED_ARMED && mode_ != MAVMode::GUIDED_ARMED) {
			// NED convention: negative Z is up.
			target_altitude_ = -arm_target_altitude_m_;
			hold_target_ = XYPoint{x_, y_};
			vx_ = 0.0f;
			vy_ = 0.0f;
			std::printf("\nDrone: ARMED - climbing to %.1fm altitude\n", arm_target_altitude_m_);
		} else if (mode == MAVMode::LAND && mode_ != MAVMode::GUIDED_DISARMED) {
			target_altitude_ = 0.0f;
			hold_target_ = XYPoint{x_, y_};
			vx_ = 0.0f;
			vy_ = 0.0f;
			std::printf("\nDrone: LAND mode engaged\n");
		} else if (mode == MAVMode::GUIDED_DISARMED) {
			if (altitude_ < -0.5f) {
				mode_ = MAVMode::LAND;
				target_altitude_ = 0.0f;
				hold_target_ = XYPoint{x_, y_};
				std::printf("\nDrone: DISARM requested while airborne - entering LAND mode\n");
				return;
			}

			target_altitude_ = 0.0f;
			altitude_ = 0.0f;
			vx_ = 0.0f;
			vy_ = 0.0f;
			vz_ = 0.0f;
			hold_target_ = XYPoint{x_, y_};
			std::printf("\nDrone: DISARMED\n");
		}
		mode_ = mode;
	}

	bool setGuidedHoldTarget(float x, float y, float altitude)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (mode_ != MAVMode::GUIDED_ARMED) {
			return false;
		}

		const XYPoint target{x, y};
		if (!app_config::is_point_inside_polygon(target, geofence_corners_)) {
			return false;
		}

		hold_target_ = target;
		target_altitude_ = altitude;
		std::printf(
			"\nDrone: received MAV_CMD_OVERRIDE_GOTO -> X=%.2f Y=%.2f Z=%.2f\n",
			hold_target_.x,
			hold_target_.y,
			target_altitude_);
		return true;
	}

	void holdCurrentPosition()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		hold_target_ = XYPoint{x_, y_};
	}

	void setTargetAltitude(float altitude) override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		target_altitude_ = altitude;
	}

	float getTargetAltitude() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return target_altitude_;
	}

	void startTelemetryStream() override {
		std::printf("Telemetry stream started\n");
	}

	void stopTelemetryStream() override {
		std::printf("Telemetry stream stopped\n");
	}

	void sendHeartbeat() override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		static bool hb_tick = false;
		hb_tick = !hb_tick;
		std::printf("\rHEARTBEAT: [%c] System armed=%s   ",
			hb_tick ? '*' : ' ',
			mode_ == MAVMode::GUIDED_DISARMED ? "false" : "true");
		std::fflush(stdout);
	}

	void sendLocalPositionNED() override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		static bool pos_tick = false;
		pos_tick = !pos_tick;
		std::printf("\rLOCAL_POSITION_NED: [%c] X=%.2f Y=%.2f Z=%.2f VX=%.2f VY=%.2f VZ=%.2f   ",
			pos_tick ? '*' : ' ',
			x_, y_, altitude_, vx_, vy_, vz_);
		std::fflush(stdout);
	}

	void update() override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		
		const auto now = std::chrono::steady_clock::now();
		float dt = std::chrono::duration<float>(now - last_update_time_).count();
		if (dt <= 0.0f) {
			dt = 0.01f;
		}
		last_update_time_ = now;
		
		// If armed, climb towards target altitude.
		if (mode_ == MAVMode::GUIDED_ARMED) {
			const float alt_diff = target_altitude_ - altitude_;
			const float max_step = climb_rate_mps_ * dt;
			const float alt_step = std::clamp(alt_diff, -max_step, max_step);
			altitude_ += alt_step;
			vz_ = alt_step / dt;
		} else if (mode_ == MAVMode::LAND) {
			const float alt_diff = 0.0f - altitude_;
			const float alt_step = std::clamp(alt_diff, 0.0f, land_rate_mps_ * dt);
			altitude_ += alt_step;
			vz_ = alt_step / dt;

			if (altitude_ >= -0.5f) {
				altitude_ = 0.0f;
				mode_ = MAVMode::GUIDED_DISARMED;
				vx_ = 0.0f;
				vy_ = 0.0f;
				vz_ = 0.0f;
				std::printf("\nDrone: Landed and disarmed\n");
			}
		} else {
			// When disarmed, maintain current altitude (zero vertical velocity)
			vz_ = 0.0f;
		}
		
		// After reaching the commanded altitude, move only to the commanded hold target.
		if (mode_ == MAVMode::GUIDED_ARMED && std::abs(target_altitude_ - altitude_) <= kAltitudeArrivalToleranceM) {
			const float delta_x = hold_target_.x - x_;
			const float delta_y = hold_target_.y - y_;
			const float remaining_distance = std::hypot(delta_x, delta_y);
			if (remaining_distance <= kHorizontalArrivalToleranceM) {
				x_ = hold_target_.x;
				y_ = hold_target_.y;
				vx_ = 0.0f;
				vy_ = 0.0f;
			} else {
				const float max_step = max_velocity_mps_ * dt;
				const float step = std::min(remaining_distance, max_step);
				const float direction_x = delta_x / remaining_distance;
				const float direction_y = delta_y / remaining_distance;
				const float step_x = direction_x * step;
				const float step_y = direction_y * step;
				x_ += step_x;
				y_ += step_y;
				vx_ = step_x / dt;
				vy_ = step_y / dt;
			}
		} else {
			// During climb, land, and disarmed states, hold XY position.
			vx_ = 0.0f;
			vy_ = 0.0f;
			if (mode_ != MAVMode::GUIDED_ARMED) {
				hold_target_ = XYPoint{x_, y_};
			}
		}
	}

	float getVx() const {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return vx_;
	}

	float getVy() const {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return vy_;
	}

	float getVz() const {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return vz_;
	}
};

std::unique_ptr<DroneTelemetrySimulator> g_telemetry;

void signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		g_running = false;
	}
}

void send_mavlink_message(UdpSocket& socket, const IpAddress& gcs_addr, const mavlink_message_t& msg)
{
	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, gcs_addr);
}

void send_geofence_point(UdpSocket& socket, const IpAddress& gcs_addr, uint8_t point_index)
{
	if (point_index >= g_drone_config.geofence_corners_m.size()) {
		return;
	}

	const XYPoint& point = g_drone_config.geofence_corners_m[point_index];
	mavlink_message_t fence_msg;
	mavlink_msg_fence_point_pack(
		kDroneSystemId,
		kDroneComponentId,
		&fence_msg,
		kGcsSystemId,
		kGcsComponentId,
		point_index,
		static_cast<uint8_t>(g_drone_config.geofence_corners_m.size()),
		point.x,
		point.y);
	send_mavlink_message(socket, gcs_addr, fence_msg);
}

void handle_mavlink_command(const mavlink_message_t& msg, UdpSocket& socket, const IpAddress& gcs_addr)
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
			if (g_telemetry) g_telemetry->setMode(MAVMode::LAND);
		} else if ((cmd.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) && (cmd.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
			if (g_telemetry) g_telemetry->setMode(MAVMode::GUIDED_ARMED);
		} else if ((cmd.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) && !(cmd.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
			if (g_telemetry) g_telemetry->setMode(MAVMode::GUIDED_DISARMED);
		}
		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG: {
		mavlink_command_long_t command{};
		mavlink_msg_command_long_decode(&msg, &command);
		if (command.target_system != kDroneSystemId || command.target_component != kDroneComponentId) {
			break;
		}

		if (command.command != MAV_CMD_OVERRIDE_GOTO || !g_telemetry) {
			break;
		}

		if (static_cast<int>(command.param1) == MAV_GOTO_DO_CONTINUE) {
			g_telemetry->holdCurrentPosition();
			std::printf("\nDrone: MAV_CMD_OVERRIDE_GOTO continue treated as hold-current\n");
			break;
		}

		if (static_cast<int>(command.param2) == MAV_GOTO_HOLD_AT_CURRENT_POSITION) {
			g_telemetry->holdCurrentPosition();
			std::printf("\nDrone: holding current position\n");
			break;
		}

		if (static_cast<int>(command.param2) != MAV_GOTO_HOLD_AT_SPECIFIED_POSITION
			|| static_cast<int>(command.param3) != MAV_FRAME_LOCAL_NED) {
			std::printf("\nDrone: rejected MAV_CMD_OVERRIDE_GOTO with unsupported parameters\n");
			break;
		}

		if (!g_telemetry->setGuidedHoldTarget(command.param5, command.param6, command.param7)) {
			std::printf("\nDrone: rejected MAV_CMD_OVERRIDE_GOTO target outside geofence or while not armed\n");
		}
		break;
	}
	case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
		mavlink_fence_fetch_point_t request{};
		mavlink_msg_fence_fetch_point_decode(&msg, &request);
		if (request.target_system != kDroneSystemId || request.target_component != kDroneComponentId) {
			break;
		}

		send_geofence_point(socket, gcs_addr, request.idx);
		break;
	}
	default:
		break;
	}
}

void simulation_worker()
{
	const double update_rate_hz = std::max(1.0, static_cast<double>(g_drone_config.update_rate_hz));
	const auto update_interval = std::chrono::duration<double>(1.0 / update_rate_hz);

	while (g_running) {
		if (g_telemetry) {
			g_telemetry->update();
		}
		std::this_thread::sleep_for(update_interval);
	}
}

void mavlink_worker(const IpAddress& gcs_addr, uint16_t drone_port)
{
	UdpSocket socket;
	if (!socket.create(drone_port, false)) {
		std::fprintf(stderr, "Drone: failed to create UDP socket on port %u\n", drone_port);
		return;
	}

	std::printf(
		"Drone: sending telemetry from %s:%u to %s\n",
		g_shared_config.host.c_str(),
		drone_port,
		gcs_addr.to_string().c_str());

	using clock = std::chrono::steady_clock;
	const auto start = clock::now();
	auto next_hb = start;
	auto next_pos = start;
	const auto hb_interval = std::chrono::duration<double>(
		1.0 / std::max(0.1, static_cast<double>(g_drone_config.heartbeat_rate_hz)));
	const auto pos_interval = std::chrono::duration<double>(
		1.0 / std::max(0.1, static_cast<double>(g_drone_config.position_rate_hz)));

	uint8_t rx_buf[2048];
	mavlink_status_t parse_status{};

	while (g_running) {
		const auto now = clock::now();

		// Handle incoming commands
		if (socket.poll_read(std::max(1, g_drone_config.rx_poll_timeout_ms))) {
			while (socket.available() > 0) {
				IpAddress from;
				const int bytes = socket.recvfrom(rx_buf, sizeof(rx_buf), from);
				if (bytes <= 0) {
					continue;
				}

				for (int i = 0; i < bytes; ++i) {
					mavlink_message_t msg;
					if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &parse_status)) {
						handle_mavlink_command(msg, socket, gcs_addr);
					}
				}
			}
		}

		if (now >= next_hb && g_telemetry) {
			g_telemetry->sendHeartbeat();

			mavlink_message_t hb_msg;
			mavlink_msg_heartbeat_pack(
				kDroneSystemId,
				kDroneComponentId,
				&hb_msg,
				MAV_TYPE_QUADROTOR,
				MAV_AUTOPILOT_GENERIC,
				g_telemetry->getMode() == MAVMode::GUIDED_DISARMED ? MAV_MODE_GUIDED_DISARMED : MAV_MODE_GUIDED_ARMED,
				static_cast<uint32_t>(g_telemetry->getMode()),
				MAV_STATE_ACTIVE);

			uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
			const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &hb_msg);
			socket.sendto(tx_buf, len, gcs_addr);

			next_hb += std::chrono::duration_cast<clock::duration>(hb_interval);
		}

		if (now >= next_pos && g_telemetry) {
			g_telemetry->sendLocalPositionNED();

			const float x = g_telemetry->getX();
			const float y = g_telemetry->getY();
			const float z = g_telemetry->getAltitude();
			const float vx = g_telemetry->getVx();
			const float vy = g_telemetry->getVy();
			const float vz = g_telemetry->getVz();

			mavlink_message_t pos_msg;
			mavlink_msg_local_position_ned_pack(
				kDroneSystemId,
				kDroneComponentId,
				&pos_msg,
				static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()),
				x,
				y,
				z,
				vx,
				vy,
				vz);

			uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
			const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &pos_msg);
			socket.sendto(tx_buf, len, gcs_addr);

			next_pos += std::chrono::duration_cast<clock::duration>(pos_interval);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	socket.close();
}

int main()
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	// Provisioning
	g_shared_config = app_config::load_shared_config();
	g_drone_config = app_config::load_drone_config();

	if (!app_config::find_config_path("shared_config.json").has_value() || !app_config::is_valid(g_shared_config)) {
		std::fprintf(stderr, "Drone: shared_config.json is missing or invalid\n");
		return 1;
	}

	if (!app_config::find_config_path("drone_config.json").has_value() || !app_config::is_valid(g_drone_config)) {
		std::fprintf(stderr, "Drone: drone_config.json is missing or invalid\n");
		return 1;
	}

	g_drone_config.arm_target_altitude_m = 20.0f;

	if (const auto cfg_path = app_config::find_config_path("drone_config.json"); cfg_path.has_value()) {
		std::printf(
			"Drone: loaded config from %s (climb_rate_mps=%.2f, land_rate_mps=%.2f, arm_target_altitude_m=%.2f)\n",
			cfg_path->string().c_str(),
			g_drone_config.climb_rate_mps,
			g_drone_config.land_rate_mps,
			g_drone_config.arm_target_altitude_m);
		for (size_t i = 0; i < g_drone_config.geofence_corners_m.size(); ++i) {
			const auto& corner = g_drone_config.geofence_corners_m[i];
			std::printf("Drone: geofence corner %zu -> X=%.2f Y=%.2f\n", i + 1, corner.x, corner.y);
		}
	} else {
		std::printf(
			"Drone: drone_config.json not found; using defaults (climb_rate_mps=%.2f, land_rate_mps=%.2f, arm_target_altitude_m=%.2f)\n",
			g_drone_config.climb_rate_mps,
			g_drone_config.land_rate_mps,
			g_drone_config.arm_target_altitude_m);
	}

	g_telemetry = std::make_unique<DroneTelemetrySimulator>(
		g_drone_config.max_velocity_mps,
		g_drone_config.climb_rate_mps,
		g_drone_config.land_rate_mps,
		g_drone_config.arm_target_altitude_m,
		g_drone_config.geofence_corners_m);

	const IpAddress gcs_addr{g_shared_config.host.c_str(), g_drone_config.gcs_port};

	// Start simulation thread
	std::thread sim_thread(simulation_worker);

	// Start MAVLink communication thread
	std::thread mavlink_thread(mavlink_worker, gcs_addr, g_drone_config.drone_port);

	// Wait for threads to complete
	sim_thread.join();
	mavlink_thread.join();

	std::printf("Drone: stopped\n");
	return 0;
}

