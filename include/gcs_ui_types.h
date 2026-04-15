#ifndef GCS_UI_TYPES_H
#define GCS_UI_TYPES_H

#include <imgui.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>

namespace gcs_ui {

/** Snapshot of the currently known geofence polygon. */
struct GeofenceSnapshot {
	std::array<ImVec2, 4> vertices{};
	std::array<bool, 4> received{};
	uint8_t expected_count = 0;
	uint8_t received_count = 0;
};

/** A single altitude history sample for plotting. */
struct AltitudeSample {
	uint32_t time_boot_ms = 0;
	float altitude_m = 0.0f;
};

/** Thread-safe telemetry data copied from the MAVLink worker. */
struct TelemetrySnapshot {
	uint8_t system_id = 0;
	uint8_t component_id = 0;
	uint8_t vehicle_type = 0;
	uint8_t base_mode = 0;
	uint32_t custom_mode = 0;
	uint8_t system_status = 0;
	uint32_t time_boot_ms = 0;
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	float vx = 0.0f;
	float vy = 0.0f;
	float vz = 0.0f;
	bool has_heartbeat = false;
	bool has_position = false;
	std::chrono::steady_clock::time_point last_message_time{};
	std::deque<AltitudeSample> altitude_history;
	GeofenceSnapshot geofence;
};

/** Immutable UI state consumed by the dashboard for a single frame. */
struct DashboardState {
	const char* host = "";
	uint16_t gcs_port = 0;
	uint16_t drone_port = 0;
	float arm_target_altitude_m = 0.0f;
	float manual_horizontal_velocity_mps = 0.0f;
	float manual_vertical_velocity_mps = 0.0f;
	int telemetry_stale_timeout_ms = 1500;
	bool is_running = false;
	bool teleop_enabled = false;
	TelemetrySnapshot telemetry;
};

/** Callback bundle used by the dashboard to request user actions. */
struct DashboardActions {
	std::function<void()> on_arm;
	std::function<void()> on_land;
	std::function<void()> on_disarm;
	std::function<bool(float, float, float)> on_send_goto;
	std::function<void(bool)> on_set_teleop_enabled;
	std::function<void(float, float, float)> on_update_teleop_axes;
};

} // namespace gcs_ui

#endif // GCS_UI_TYPES_H
