#include <udp/simple_udp.h>

#include <app_config.h>
#include <common/mavlink.h>
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
#include <random>
#include <thread>

std::atomic_bool g_running{true};

SharedConfig g_shared_config;
DroneConfig g_drone_config;

namespace {

constexpr float kAltitudeArrivalToleranceM = 0.5f;
constexpr float kWaypointArrivalToleranceM = 0.75f;
constexpr int kWaypointSamplingAttempts = 64;

std::array<XYPoint, 4> sort_geofence_corners(std::array<XYPoint, 4> corners)
{
	float center_x = 0.0f;
	float center_y = 0.0f;
	for (const auto& corner : corners) {
		center_x += corner.x;
		center_y += corner.y;
	}
	center_x /= static_cast<float>(corners.size());
	center_y /= static_cast<float>(corners.size());

	std::sort(corners.begin(), corners.end(), [center_x, center_y](const XYPoint& lhs, const XYPoint& rhs) {
		return std::atan2(lhs.y - center_y, lhs.x - center_x)
			< std::atan2(rhs.y - center_y, rhs.x - center_x);
	});

	return corners;
}

XYPoint compute_polygon_centroid(const std::array<XYPoint, 4>& polygon)
{
	float center_x = 0.0f;
	float center_y = 0.0f;
	for (const auto& point : polygon) {
		center_x += point.x;
		center_y += point.y;
	}

	return XYPoint{
		center_x / static_cast<float>(polygon.size()),
		center_y / static_cast<float>(polygon.size()),
	};
}

bool is_point_inside_polygon(const XYPoint& point, const std::array<XYPoint, 4>& polygon)
{
	bool inside = false;
	for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
		const auto& current = polygon[i];
		const auto& previous = polygon[j];
		const bool crosses_scanline = (current.y > point.y) != (previous.y > point.y);
		if (!crosses_scanline) {
			continue;
		}

		const float edge_delta_y = previous.y - current.y;
		const float interpolated_x = ((previous.x - current.x) * (point.y - current.y) / edge_delta_y) + current.x;
		if (point.x <= interpolated_x) {
			inside = !inside;
		}
	}

	return inside;
}

XYPoint sample_random_point_in_geofence(const std::array<XYPoint, 4>& polygon, std::mt19937& random_engine)
{
	float min_x = polygon.front().x;
	float max_x = polygon.front().x;
	float min_y = polygon.front().y;
	float max_y = polygon.front().y;
	for (const auto& point : polygon) {
		min_x = std::min(min_x, point.x);
		max_x = std::max(max_x, point.x);
		min_y = std::min(min_y, point.y);
		max_y = std::max(max_y, point.y);
	}

	std::uniform_real_distribution<float> x_dist(min_x, max_x);
	std::uniform_real_distribution<float> y_dist(min_y, max_y);
	for (int attempt = 0; attempt < kWaypointSamplingAttempts; ++attempt) {
		const XYPoint candidate{x_dist(random_engine), y_dist(random_engine)};
		if (is_point_inside_polygon(candidate, polygon)) {
			return candidate;
		}
	}

	return compute_polygon_centroid(polygon);
}

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
	XYPoint waypoint_target_{};
	bool has_waypoint_target_ = false;
	MAVMode mode_ = MAVMode::GUIDED_DISARMED;
	std::mt19937 random_engine_;
	std::chrono::steady_clock::time_point last_update_time_;

	void select_next_waypoint_locked()
	{
		const XYPoint current_position{x_, y_};
		for (int attempt = 0; attempt < kWaypointSamplingAttempts; ++attempt) {
			const XYPoint candidate = sample_random_point_in_geofence(geofence_corners_, random_engine_);
			if (std::hypot(candidate.x - current_position.x, candidate.y - current_position.y)
				< kWaypointArrivalToleranceM * 2.0f) {
				continue;
			}

			waypoint_target_ = candidate;
			has_waypoint_target_ = true;
			std::printf("\nDrone: new waypoint X=%.2f Y=%.2f\n", waypoint_target_.x, waypoint_target_.y);
			return;
		}

		waypoint_target_ = sample_random_point_in_geofence(geofence_corners_, random_engine_);
		has_waypoint_target_ = true;
		std::printf("\nDrone: new waypoint X=%.2f Y=%.2f\n", waypoint_target_.x, waypoint_target_.y);
	}

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
		  geofence_corners_(sort_geofence_corners(geofence_corners)),
		  random_engine_(std::random_device{}()) {
		const XYPoint launch_position = compute_polygon_centroid(geofence_corners_);
		x_ = launch_position.x;
		y_ = launch_position.y;
		waypoint_target_ = launch_position;
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
			has_waypoint_target_ = false;
			vx_ = 0.0f;
			vy_ = 0.0f;
			std::printf("\nDrone: ARMED - climbing to %.1fm altitude\n", arm_target_altitude_m_);
		} else if (mode == MAVMode::LAND && mode_ != MAVMode::GUIDED_DISARMED) {
			target_altitude_ = 0.0f;
			has_waypoint_target_ = false;
			vx_ = 0.0f;
			vy_ = 0.0f;
			std::printf("\nDrone: LAND mode engaged\n");
		} else if (mode == MAVMode::GUIDED_DISARMED) {
			if (altitude_ < -0.5f) {
				mode_ = MAVMode::LAND;
				target_altitude_ = 0.0f;
				has_waypoint_target_ = false;
				std::printf("\nDrone: DISARM requested while airborne - entering LAND mode\n");
				return;
			}

			target_altitude_ = 0.0f;
			altitude_ = 0.0f;
			vx_ = 0.0f;
			vy_ = 0.0f;
			vz_ = 0.0f;
			has_waypoint_target_ = false;
			std::printf("\nDrone: DISARMED\n");
		}
		mode_ = mode;
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
		
		// After reaching the commanded altitude, move between random waypoints inside the geofence.
		if (mode_ == MAVMode::GUIDED_ARMED && std::abs(target_altitude_ - altitude_) <= kAltitudeArrivalToleranceM) {
			if (!has_waypoint_target_) {
				select_next_waypoint_locked();
			}

			const float delta_x = waypoint_target_.x - x_;
			const float delta_y = waypoint_target_.y - y_;
			const float remaining_distance = std::hypot(delta_x, delta_y);
			if (remaining_distance <= kWaypointArrivalToleranceM) {
				x_ = waypoint_target_.x;
				y_ = waypoint_target_.y;
				vx_ = 0.0f;
				vy_ = 0.0f;
				select_next_waypoint_locked();
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
				has_waypoint_target_ = false;
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

void handle_mavlink_command(const mavlink_message_t& msg)
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
	default:
		break;
	}
}

void send_mavlink_message(UdpSocket& socket, const IpAddress& gcs_addr, const mavlink_message_t& msg)
{
	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, gcs_addr);
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

	constexpr uint8_t kSystemId = 1;
	constexpr uint8_t kComponentId = MAV_COMP_ID_AUTOPILOT1;

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
						handle_mavlink_command(msg);
					}
				}
			}
		}

		if (now >= next_hb && g_telemetry) {
			g_telemetry->sendHeartbeat();

			mavlink_message_t hb_msg;
			mavlink_msg_heartbeat_pack(
				kSystemId,
				kComponentId,
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
				kSystemId,
				kComponentId,
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

