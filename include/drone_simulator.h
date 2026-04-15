#ifndef DRONE_SIMULATOR_H
#define DRONE_SIMULATOR_H

#include <app_config.h>
#include <drone_telemtry_abc.h>

#include <array>
#include <chrono>
#include <mutex>

/**
 * Tunable motion settings used by the drone simulator.
 */
struct DroneMotionSettings {
	float max_velocity_mps = 0.0f;
	float manual_horizontal_velocity_mps = 0.0f;
	float climb_rate_mps = 0.0f;
	float manual_vertical_velocity_mps = 0.0f;
	float land_rate_mps = 0.0f;
	float arm_target_altitude_m = 0.0f;
	float horizontal_arrival_tolerance_m = 0.75f;
	std::chrono::milliseconds manual_control_timeout{250};
};

/**
 * Mutable vehicle state tracked by the simulator.
 */
struct DroneState {
	float x = 0.0f;
	float y = 0.0f;
	float altitude = 0.0f;
	float target_altitude = 0.0f;
	float vx = 0.0f;
	float vy = 0.0f;
	float vz = 0.0f;
	XYPoint hold_target{};
	MAVMode mode = MAVMode::GUIDED_DISARMED;
};

/**
 * Latest manual control input received from the GCS.
 */
struct ManualControlState {
	bool active = false;
	float x_input = 0.0f;
	float y_input = 0.0f;
	float z_input = 0.0f;
	std::chrono::steady_clock::time_point last_command_time{};
};

/**
 * Thread-safe telemetry simulator that emulates a small MAVLink-enabled drone.
 */
class DroneTelemetrySimulator final : public DroneTelemetryABC {
public:
	explicit DroneTelemetrySimulator(const DroneConfig& config);

	float getX() const override;
	float getY() const override;
	float getAltitude() const override;
	MAVMode getMode() const override;
	void setMode(MAVMode mode) override;

	void setTargetAltitude(float altitude) override;
	float getTargetAltitude() const override;

	void startTelemetryStream() override;
	void stopTelemetryStream() override;
	void sendHeartbeat() override;
	void sendLocalPositionNED() override;
	void update() override;

	bool setGuidedHoldTarget(float x, float y, float altitude);
	void holdCurrentPosition();
	bool setManualControl(float x_input, float y_input, float z_input);

	float getVx() const;
	float getVy() const;
	float getVz() const;

private:
	void resetManualControlState();

	mutable std::mutex state_mutex_;
	DroneState state_{};
	ManualControlState manual_control_{};
	DroneMotionSettings motion_settings_{};
	std::array<XYPoint, 4> geofence_corners_{};
	std::chrono::steady_clock::time_point last_update_time_{};
};

#endif // DRONE_SIMULATOR_H
