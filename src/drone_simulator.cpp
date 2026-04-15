#include <drone_simulator.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

DroneTelemetrySimulator::DroneTelemetrySimulator(const DroneConfig& config)
	: motion_settings_{
		config.max_velocity_mps,
		config.climb_rate_mps,
		config.land_rate_mps,
		config.arm_target_altitude_m,
		config.horizontal_arrival_tolerance_m,
		std::chrono::milliseconds(config.manual_control_timeout_ms),
	  },
	  geofence_corners_(app_config::sort_geofence_corners(config.geofence_corners_m))
{
	const XYPoint launch_position = app_config::compute_polygon_centroid(geofence_corners_);
	state_.x = launch_position.x;
	state_.y = launch_position.y;
	state_.hold_target = launch_position;
	last_update_time_ = std::chrono::steady_clock::now();
}

float DroneTelemetrySimulator::getX() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.x;
}

float DroneTelemetrySimulator::getY() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.y;
}

float DroneTelemetrySimulator::getAltitude() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.altitude;
}

MAVMode DroneTelemetrySimulator::getMode() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.mode;
}

void DroneTelemetrySimulator::setMode(MAVMode mode)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	if (mode == MAVMode::GUIDED_ARMED && state_.mode != MAVMode::GUIDED_ARMED) {
		state_.target_altitude = -motion_settings_.arm_target_altitude_m;
		state_.hold_target = XYPoint{state_.x, state_.y};
		state_.vx = 0.0f;
		state_.vy = 0.0f;
		std::printf("\nDrone: ARMED - climbing to %.1fm altitude\n", motion_settings_.arm_target_altitude_m);
	} else if (mode == MAVMode::LAND && state_.mode != MAVMode::GUIDED_DISARMED) {
		state_.target_altitude = 0.0f;
		state_.hold_target = XYPoint{state_.x, state_.y};
		state_.vx = 0.0f;
		state_.vy = 0.0f;
		std::printf("\nDrone: LAND mode engaged\n");
	} else if (mode == MAVMode::GUIDED_DISARMED) {
		if (state_.altitude < -0.5f) {
			state_.mode = MAVMode::LAND;
			state_.target_altitude = 0.0f;
			state_.hold_target = XYPoint{state_.x, state_.y};
			std::printf("\nDrone: DISARM requested while airborne - entering LAND mode\n");
			return;
		}

		state_.target_altitude = 0.0f;
		state_.altitude = 0.0f;
		state_.vx = 0.0f;
		state_.vy = 0.0f;
		state_.vz = 0.0f;
		state_.hold_target = XYPoint{state_.x, state_.y};
		std::printf("\nDrone: DISARMED\n");
	} else {
		std::printf("\nDrone: Unsupported mode requested -> forcing DISARM\n");

		state_.target_altitude = 0.0f;
		state_.vx = 0.0f;
		state_.vy = 0.0f;
		state_.vz = 0.0f;
		state_.hold_target = XYPoint{state_.x, state_.y};

		resetManualControlState();

		state_.mode = MAVMode::GUIDED_DISARMED;
		return;
	}

	if (mode != MAVMode::GUIDED_ARMED) {
		resetManualControlState();
	}

	state_.mode = mode;
}

bool DroneTelemetrySimulator::setGuidedHoldTarget(float x, float y, float altitude)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	if (state_.mode != MAVMode::GUIDED_ARMED) {
		return false;
	}

	const XYPoint target{x, y};
	if (!app_config::is_point_inside_polygon(target, geofence_corners_)) {
		return false;
	}

	state_.hold_target = target;
	state_.target_altitude = altitude;
	std::printf(
		"\nDrone: received MAV_CMD_OVERRIDE_GOTO -> X=%.2f Y=%.2f Z=%.2f\n",
		state_.hold_target.x,
		state_.hold_target.y,
		state_.target_altitude);
	return true;
}

void DroneTelemetrySimulator::holdCurrentPosition()
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	resetManualControlState();
	state_.hold_target = XYPoint{state_.x, state_.y};
	state_.target_altitude = state_.altitude;
}

bool DroneTelemetrySimulator::setManualControl(float x_input, float y_input, float z_input)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	if (state_.mode != MAVMode::GUIDED_ARMED) {
		return false;
	}

	manual_control_.active = true;
	manual_control_.x_input = std::clamp(x_input, -1.0f, 1.0f);
	manual_control_.y_input = std::clamp(y_input, -1.0f, 1.0f);
	manual_control_.z_input = std::clamp(z_input, -1.0f, 1.0f);
	manual_control_.last_command_time = std::chrono::steady_clock::now();
	return true;
}

void DroneTelemetrySimulator::setTargetAltitude(float altitude)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	state_.target_altitude = altitude;
}

float DroneTelemetrySimulator::getTargetAltitude() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.target_altitude;
}

void DroneTelemetrySimulator::startTelemetryStream()
{
	std::printf("Telemetry stream started\n");
}

void DroneTelemetrySimulator::stopTelemetryStream()
{
	std::printf("Telemetry stream stopped\n");
}

void DroneTelemetrySimulator::sendHeartbeat()
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	static bool hb_tick = false;
	hb_tick = !hb_tick;
	std::printf("\rHEARTBEAT: [%c] System armed=%s   ",
		hb_tick ? '*' : ' ',
		state_.mode == MAVMode::GUIDED_DISARMED ? "false" : "true");
	std::fflush(stdout);
}

void DroneTelemetrySimulator::sendLocalPositionNED()
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	static bool pos_tick = false;
	pos_tick = !pos_tick;
	std::printf("\rLOCAL_POSITION_NED: [%c] X=%.2f Y=%.2f Z=%.2f VX=%.2f VY=%.2f VZ=%.2f   ",
		pos_tick ? '*' : ' ',
		state_.x,
		state_.y,
		state_.altitude,
		state_.vx,
		state_.vy,
		state_.vz);
	std::fflush(stdout);
}

void DroneTelemetrySimulator::update()
{
	std::lock_guard<std::mutex> lock(state_mutex_);

	const auto now = std::chrono::steady_clock::now();
	float dt = std::chrono::duration<float>(now - last_update_time_).count();
	if (dt <= 0.0f) {
		dt = 0.01f;
	}
	last_update_time_ = now;

	const bool manual_control_active = manual_control_.active
		&& manual_control_.last_command_time.time_since_epoch().count() != 0
		&& now - manual_control_.last_command_time <= motion_settings_.manual_control_timeout;
	if (!manual_control_active) {
		resetManualControlState();
	}

	if (state_.mode == MAVMode::GUIDED_ARMED && manual_control_active) {
		const float requested_vx = manual_control_.x_input * motion_settings_.max_velocity_mps;
		const float requested_vy = manual_control_.y_input * motion_settings_.max_velocity_mps;
		const float next_x = state_.x + requested_vx * dt;
		const float next_y = state_.y + requested_vy * dt;
		if (app_config::is_point_inside_polygon(XYPoint{next_x, next_y}, geofence_corners_)) {
			state_.x = next_x;
			state_.y = next_y;
			state_.vx = requested_vx;
			state_.vy = requested_vy;
		} else {
			state_.vx = 0.0f;
			state_.vy = 0.0f;
		}

		const float requested_vz = -manual_control_.z_input * motion_settings_.climb_rate_mps;
		const float previous_altitude = state_.altitude;
		state_.altitude = std::min(state_.altitude + requested_vz * dt, 0.0f);
		state_.vz = (state_.altitude - previous_altitude) / dt;
		state_.hold_target = XYPoint{state_.x, state_.y};
		state_.target_altitude = state_.altitude;
	} else if (state_.mode == MAVMode::GUIDED_ARMED) {
		const float alt_diff = state_.target_altitude - state_.altitude;
		const float max_step = motion_settings_.climb_rate_mps * dt;
		const float alt_step = std::clamp(alt_diff, -max_step, max_step);
		state_.altitude += alt_step;
		state_.vz = alt_step / dt;
	} else if (state_.mode == MAVMode::LAND) {
		const float alt_diff = 0.0f - state_.altitude;
		const float alt_step = std::clamp(alt_diff, 0.0f, motion_settings_.land_rate_mps * dt);
		state_.altitude += alt_step;
		state_.vz = alt_step / dt;

		if (state_.altitude >= -0.5f) {
			state_.altitude = 0.0f;
			state_.mode = MAVMode::GUIDED_DISARMED;
			state_.vx = 0.0f;
			state_.vy = 0.0f;
			state_.vz = 0.0f;
			std::printf("\nDrone: Landed and disarmed\n");
		}
	} else {
		state_.vz = 0.0f;
	}

	if (state_.mode == MAVMode::GUIDED_ARMED && !manual_control_active) {
		const float delta_x = state_.hold_target.x - state_.x;
		const float delta_y = state_.hold_target.y - state_.y;
		const float remaining_distance = std::hypot(delta_x, delta_y);
		if (remaining_distance <= motion_settings_.horizontal_arrival_tolerance_m) {
			state_.x = state_.hold_target.x;
			state_.y = state_.hold_target.y;
			state_.vx = 0.0f;
			state_.vy = 0.0f;
		} else {
			const float max_step = motion_settings_.max_velocity_mps * dt;
			const float step = std::min(remaining_distance, max_step);
			const float direction_x = delta_x / remaining_distance;
			const float direction_y = delta_y / remaining_distance;
			const float step_x = direction_x * step;
			const float step_y = direction_y * step;
			state_.x += step_x;
			state_.y += step_y;
			state_.vx = step_x / dt;
			state_.vy = step_y / dt;
		}
	} else if (state_.mode != MAVMode::GUIDED_ARMED) {
		state_.vx = 0.0f;
		state_.vy = 0.0f;
		state_.hold_target = XYPoint{state_.x, state_.y};
	}
}

float DroneTelemetrySimulator::getVx() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.vx;
}

float DroneTelemetrySimulator::getVy() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.vy;
}

float DroneTelemetrySimulator::getVz() const
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	return state_.vz;
}

void DroneTelemetrySimulator::resetManualControlState()
{
	manual_control_.active = false;
	manual_control_.x_input = 0.0f;
	manual_control_.y_input = 0.0f;
	manual_control_.z_input = 0.0f;
}
