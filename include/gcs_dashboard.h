#ifndef GCS_DASHBOARD_H
#define GCS_DASHBOARD_H

#include <app_config.h>
#include <gcs_ui_types.h>

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace gcs_ui {
namespace detail {

constexpr ImVec2 kMinPlotSize{320.0f, 320.0f};
constexpr ImVec2 kMinAltitudePlotSize{320.0f, 220.0f};

inline size_t geofence_vertex_target_count(const GeofenceSnapshot& geofence)
{
	if (geofence.expected_count == 0) {
		return geofence.received_count == 0 ? 0 : geofence.vertices.size();
	}

	return std::min<size_t>(geofence.expected_count, geofence.vertices.size());
}

inline bool has_complete_geofence(const GeofenceSnapshot& geofence)
{
	const size_t target_count = geofence_vertex_target_count(geofence);
	return target_count >= 3 && geofence.received_count >= target_count;
}

inline std::array<XYPoint, 4> geofence_vertices_as_xy(const GeofenceSnapshot& geofence)
{
	std::array<XYPoint, 4> polygon{};
	for (size_t index = 0; index < polygon.size(); ++index) {
		polygon[index] = XYPoint{geofence.vertices[index].x, geofence.vertices[index].y};
	}
	return app_config::sort_geofence_corners(polygon);
}

inline bool is_point_inside_geofence(const GeofenceSnapshot& geofence, float x, float y)
{
	if (!has_complete_geofence(geofence)) {
		return false;
	}

	return app_config::is_point_inside_polygon(XYPoint{x, y}, geofence_vertices_as_xy(geofence));
}

inline float compute_plot_extent(const TelemetrySnapshot& telemetry)
{
	float max_abs = std::max(std::abs(telemetry.x), std::abs(telemetry.y));

	const size_t fence_points = geofence_vertex_target_count(telemetry.geofence);
	for (size_t index = 0; index < fence_points; ++index) {
		if (!telemetry.geofence.received[index]) {
			continue;
		}

		const ImVec2& point = telemetry.geofence.vertices[index];
		max_abs = std::max(max_abs, std::abs(point.x));
		max_abs = std::max(max_abs, std::abs(point.y));
	}

	return std::max(5.0f, max_abs * 1.15f + 0.5f);
}

inline ImVec2 world_to_canvas(float x, float y, const ImVec2& origin, const ImVec2& size, float extent)
{
	const float normalized_x = (x + extent) / (2.0f * extent);
	const float normalized_y = (y + extent) / (2.0f * extent);
	return ImVec2(
		origin.x + normalized_x * size.x,
		origin.y + (1.0f - normalized_y) * size.y);
}

inline ImVec2 altitude_to_canvas(
	float time_s,
	float altitude_m,
	const ImVec2& origin,
	const ImVec2& size,
	float min_time_s,
	float max_time_s,
	float min_altitude_m,
	float max_altitude_m)
{
	const float time_span = std::max(max_time_s - min_time_s, 1.0f);
	const float altitude_span = std::max(max_altitude_m - min_altitude_m, 1.0f);
	const float normalized_x = (time_s - min_time_s) / time_span;
	const float normalized_y = (altitude_m - min_altitude_m) / altitude_span;
	return ImVec2(
		origin.x + normalized_x * size.x,
		origin.y + (1.0f - normalized_y) * size.y);
}

inline bool has_recent_data(const TelemetrySnapshot& telemetry)
{
	const auto now = std::chrono::steady_clock::now();
	return telemetry.last_message_time.time_since_epoch().count() != 0
		&& std::chrono::duration_cast<std::chrono::milliseconds>(now - telemetry.last_message_time).count() < 1500;
}

enum class VehicleMode {
	Disconnected,
	GuidedDisarmed,
	GuidedArmed,
	Land,
};

inline VehicleMode vehicle_mode(const TelemetrySnapshot& telemetry)
{
	if (!telemetry.has_heartbeat || !has_recent_data(telemetry)) {
		return VehicleMode::Disconnected;
	}

	switch (telemetry.custom_mode) {
	case 2:
		return VehicleMode::Land;
	case 1:
		return VehicleMode::GuidedArmed;
	case 0:
	default:
		return VehicleMode::GuidedDisarmed;
	}
}

inline void draw_geofence_overlay(const TelemetrySnapshot& telemetry, const ImVec2& canvas_pos, const ImVec2& canvas_size, float extent)
{
	if (!has_complete_geofence(telemetry.geofence)) {
		return;
	}

	std::vector<ImVec2> points;
	points.reserve(telemetry.geofence.vertices.size());
	for (size_t index = 0; index < geofence_vertex_target_count(telemetry.geofence); ++index) {
		const ImVec2& point = telemetry.geofence.vertices[index];
		points.push_back(world_to_canvas(point.x, point.y, canvas_pos, canvas_size, extent));
	}

	if (points.size() < 3) {
		return;
	}

	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddConvexPolyFilled(points.data(), static_cast<int>(points.size()), IM_COL32(255, 120, 90, 24));
	for (size_t index = 0; index < points.size(); ++index) {
		const ImVec2& current = points[index];
		const ImVec2& next = points[(index + 1) % points.size()];
		draw_list->AddLine(current, next, IM_COL32(255, 120, 90, 255), 2.0f);
		draw_list->AddCircleFilled(current, 4.0f, IM_COL32(255, 160, 120, 255));
	}
}

inline void draw_position_plot(const TelemetrySnapshot& telemetry, float requested_height = 420.0f)
{
	ImGui::TextUnformatted("2D Position (X/Y)");
	ImVec2 canvas_size = ImGui::GetContentRegionAvail();
	canvas_size.x = std::max(canvas_size.x, kMinPlotSize.x);
	canvas_size.y = std::min(canvas_size.y, std::max(requested_height, 0.0f));
	if (canvas_size.y <= 0.0f) {
		return;
	}

	const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	const ImVec2 canvas_end(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);

	draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(15, 18, 24, 255), 8.0f);
	draw_list->AddRect(canvas_pos, canvas_end, IM_COL32(82, 96, 122, 255), 8.0f, 0, 1.5f);

	const float extent = compute_plot_extent(telemetry);
	const ImU32 grid_color = IM_COL32(65, 76, 96, 255);
	const ImU32 axis_color = IM_COL32(124, 170, 255, 255);
	const ImU32 point_color = IM_COL32(255, 200, 90, 255);

	for (int step = 1; step < 4; ++step) {
		const float t = static_cast<float>(step) / 4.0f;
		const float x = canvas_pos.x + t * canvas_size.x;
		const float y = canvas_pos.y + t * canvas_size.y;
		draw_list->AddLine(ImVec2(x, canvas_pos.y), ImVec2(x, canvas_end.y), grid_color, 1.0f);
		draw_list->AddLine(ImVec2(canvas_pos.x, y), ImVec2(canvas_end.x, y), grid_color, 1.0f);
	}

	const ImVec2 origin = world_to_canvas(0.0f, 0.0f, canvas_pos, canvas_size, extent);
	draw_list->AddLine(ImVec2(canvas_pos.x, origin.y), ImVec2(canvas_end.x, origin.y), axis_color, 1.5f);
	draw_list->AddLine(ImVec2(origin.x, canvas_pos.y), ImVec2(origin.x, canvas_end.y), axis_color, 1.5f);

	draw_geofence_overlay(telemetry, canvas_pos, canvas_size, extent);

	if (telemetry.has_position) {
		const ImVec2 drone_pos = world_to_canvas(telemetry.x, telemetry.y, canvas_pos, canvas_size, extent);
		draw_list->AddCircleFilled(drone_pos, 5.0f, point_color);
		draw_list->AddCircle(drone_pos, 8.0f, IM_COL32(255, 255, 255, 180), 0, 1.5f);
	}

	char label[64];
	std::snprintf(label, sizeof(label), "+/- %.1f m", extent);
	draw_list->AddText(ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 10.0f), IM_COL32_WHITE, label);

	ImGui::InvisibleButton("position_plot", canvas_size);
}

inline void draw_altitude_plot(const TelemetrySnapshot& telemetry, float requested_height = 260.0f)
{
	ImGui::Spacing();
	ImGui::TextUnformatted("Altitude Over Time");
	ImVec2 canvas_size = ImGui::GetContentRegionAvail();
	canvas_size.x = std::max(canvas_size.x, kMinAltitudePlotSize.x);
	canvas_size.y = std::min(canvas_size.y, std::max(requested_height, 0.0f));
	if (canvas_size.y <= 0.0f) {
		return;
	}

	const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	const ImVec2 canvas_end(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);

	draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(15, 18, 24, 255), 8.0f);
	draw_list->AddRect(canvas_pos, canvas_end, IM_COL32(82, 96, 122, 255), 8.0f, 0, 1.5f);

	const ImU32 grid_color = IM_COL32(65, 76, 96, 255);
	const ImU32 axis_color = IM_COL32(124, 170, 255, 255);
	const ImU32 line_color = IM_COL32(90, 210, 255, 255);
	const ImU32 point_color = IM_COL32(255, 200, 90, 255);

	for (int step = 1; step < 4; ++step) {
		const float t = static_cast<float>(step) / 4.0f;
		const float x = canvas_pos.x + t * canvas_size.x;
		const float y = canvas_pos.y + t * canvas_size.y;
		draw_list->AddLine(ImVec2(x, canvas_pos.y), ImVec2(x, canvas_end.y), grid_color, 1.0f);
		draw_list->AddLine(ImVec2(canvas_pos.x, y), ImVec2(canvas_end.x, y), grid_color, 1.0f);
	}

	if (telemetry.altitude_history.empty()) {
		draw_list->AddText(
			ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 12.0f),
			IM_COL32(210, 216, 230, 255),
			"Waiting for altitude telemetry");
		ImGui::InvisibleButton("altitude_plot", canvas_size);
		return;
	}

	const float start_time_s = static_cast<float>(telemetry.altitude_history.front().time_boot_ms) / 1000.0f;
	const float end_time_s = static_cast<float>(telemetry.altitude_history.back().time_boot_ms) / 1000.0f;
	float min_altitude_m = telemetry.altitude_history.front().altitude_m;
	float max_altitude_m = telemetry.altitude_history.front().altitude_m;
	for (const AltitudeSample& sample : telemetry.altitude_history) {
		min_altitude_m = std::min(min_altitude_m, sample.altitude_m);
		max_altitude_m = std::max(max_altitude_m, sample.altitude_m);
	}

	const float altitude_padding = std::max((max_altitude_m - min_altitude_m) * 0.1f, 1.0f);
	min_altitude_m -= altitude_padding;
	max_altitude_m += altitude_padding;

	std::vector<ImVec2> points;
	points.reserve(telemetry.altitude_history.size());
	for (const AltitudeSample& sample : telemetry.altitude_history) {
		const float time_s = static_cast<float>(sample.time_boot_ms) / 1000.0f;
		points.push_back(
			altitude_to_canvas(
				time_s,
				sample.altitude_m,
				canvas_pos,
				canvas_size,
				start_time_s,
				end_time_s,
				min_altitude_m,
				max_altitude_m));
	}

	const ImVec2 bottom_left(canvas_pos.x, canvas_end.y - 1.0f);
	const ImVec2 bottom_right(canvas_end.x, canvas_end.y - 1.0f);
	draw_list->AddLine(bottom_left, bottom_right, axis_color, 1.5f);
	if (min_altitude_m <= 0.0f && max_altitude_m >= 0.0f) {
		const ImVec2 zero_left = altitude_to_canvas(
			start_time_s,
			0.0f,
			canvas_pos,
			canvas_size,
			start_time_s,
			end_time_s,
			min_altitude_m,
			max_altitude_m);
		const ImVec2 zero_right = altitude_to_canvas(
			end_time_s,
			0.0f,
			canvas_pos,
			canvas_size,
			start_time_s,
			end_time_s,
			min_altitude_m,
			max_altitude_m);
		draw_list->AddLine(zero_left, zero_right, axis_color, 1.5f);
	}

	if (points.size() > 1) {
		draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), line_color, 0, 2.0f);
	}
	const ImVec2& latest_point = points.back();
	draw_list->AddCircleFilled(latest_point, 4.5f, point_color);
	draw_list->AddCircle(latest_point, 7.0f, IM_COL32(255, 255, 255, 180), 0, 1.25f);

	char label[96];
	std::snprintf(label, sizeof(label), "%.1f to %.1f m", min_altitude_m, max_altitude_m);
	draw_list->AddText(ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 10.0f), IM_COL32_WHITE, label);
	std::snprintf(label, sizeof(label), "%.1f s window", std::max(end_time_s - start_time_s, 0.0f));
	draw_list->AddText(ImVec2(canvas_pos.x + 12.0f, canvas_end.y - 24.0f), IM_COL32(210, 216, 230, 255), label);
	std::snprintf(label, sizeof(label), "Current %.1f m", telemetry.altitude_history.back().altitude_m);
	draw_list->AddText(ImVec2(canvas_end.x - 150.0f, canvas_pos.y + 10.0f), point_color, label);

	ImGui::InvisibleButton("altitude_plot", canvas_size);
}

} // namespace detail

inline void render_dashboard(const DashboardState& state, const DashboardActions& actions)
{
	static float goto_x = 0.0f;
	static float goto_y = 0.0f;
	static float goto_altitude_m = 20.0f;
	static bool goto_initialized = false;
	static std::string goto_status = "Waiting for geofence";
	static bool goto_status_is_error = false;

	if (!goto_initialized && state.telemetry.has_position) {
		goto_x = state.telemetry.x;
		goto_y = state.telemetry.y;
		goto_altitude_m = std::max(state.arm_target_altitude_m, -state.telemetry.z);
		goto_initialized = true;
	}

	const ImVec2 panel_margin(24.0f, 24.0f);
	const ImVec2 desired_panel_size(1400.0f, 920.0f);
	const ImGuiViewport* viewport = ImGui::GetMainViewport();
	const ImVec2 panel_pos(
		viewport->WorkPos.x + panel_margin.x,
		viewport->WorkPos.y + panel_margin.y);
	const ImVec2 max_panel_size(
		std::max(0.0f, viewport->WorkSize.x - panel_margin.x * 2.0f),
		std::max(0.0f, viewport->WorkSize.y - panel_margin.y * 2.0f));
	const ImVec2 panel_size(
		std::min(desired_panel_size.x, max_panel_size.x),
		std::min(desired_panel_size.y, max_panel_size.y));

	ImGui::SetNextWindowPos(panel_pos, ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(panel_size, ImGuiCond_Appearing);
	ImGui::Begin("Ground Control Station", nullptr, ImGuiWindowFlags_NoCollapse);

	const bool is_connected = detail::has_recent_data(state.telemetry);
	const detail::VehicleMode current_mode = detail::vehicle_mode(state.telemetry);
	const bool is_disarmed = current_mode == detail::VehicleMode::GuidedDisarmed;
	const bool is_armed = current_mode == detail::VehicleMode::GuidedArmed || current_mode == detail::VehicleMode::Land;
	const float button_width = 120.0f;
	const ImVec4 heartbeat_receiving_color(0.36f, 0.82f, 0.42f, 1.0f);
	const ImVec4 heartbeat_stale_color(0.74f, 0.20f, 0.20f, 1.0f);
	const ImVec4 arm_button_color(0.20f, 0.62f, 0.26f, 1.0f);	
	const ImVec4 arm_button_hovered(0.25f, 0.72f, 0.31f, 1.0f);
	const ImVec4 arm_button_active(0.17f, 0.52f, 0.22f, 1.0f);
	const ImVec4 disarm_button_color(0.74f, 0.20f, 0.20f, 1.0f);
	const ImVec4 disarm_button_hovered(0.82f, 0.25f, 0.25f, 1.0f);
	const ImVec4 disarm_button_active(0.64f, 0.16f, 0.16f, 1.0f);

	ImGui::TextUnformatted("Heartbeat:");
	ImGui::SameLine();
	ImGui::TextColored(
		is_connected ? heartbeat_receiving_color : heartbeat_stale_color,
		"%s",
		is_connected ? "RECEIVING" : "STALE");
	ImGui::SameLine();
	ImGui::Text("Host %s  GCS %u  Drone %u", state.host, state.gcs_port, state.drone_port);
	ImGui::Separator();
	const char* arm_label = is_armed ? "ARMED" : "ARM";
	const char* disarm_label = is_disarmed ? "DISARMED" : "DISARM";

	if (is_armed) {
		ImGui::PushStyleColor(ImGuiCol_Button, arm_button_color);
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, arm_button_hovered);
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, arm_button_active);
	}
	ImGui::BeginDisabled(!state.is_running || is_armed);
	if (ImGui::Button(arm_label, ImVec2(button_width, 0.0f)) && actions.on_arm) {
		actions.on_arm();
	}
	ImGui::EndDisabled();
	if (is_armed) {
		ImGui::PopStyleColor(3);
	}
	ImGui::SameLine();
	if (is_disarmed) {
		ImGui::PushStyleColor(ImGuiCol_Button, disarm_button_color);
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, disarm_button_hovered);
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, disarm_button_active);
	}
	ImGui::BeginDisabled(!state.is_running || is_disarmed);
	if (ImGui::Button(disarm_label, ImVec2(button_width, 0.0f)) && actions.on_disarm) {
		actions.on_disarm();
	}
	ImGui::EndDisabled();
	if (is_disarmed) {
		ImGui::PopStyleColor(3);
	}
	ImGui::SameLine();
	ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - button_width));
	ImGui::BeginDisabled(!state.is_running);
	if (ImGui::Button("LAND", ImVec2(button_width, 0.0f)) && actions.on_land) {
		actions.on_land();
	}
	ImGui::EndDisabled();

	ImGui::Text("Arm target altitude: %.1f m", state.arm_target_altitude_m);
	const bool geofence_ready = detail::has_complete_geofence(state.telemetry.geofence);
	const bool goto_inside_geofence = detail::is_point_inside_geofence(state.telemetry.geofence, goto_x, goto_y);
	const bool goto_altitude_valid = goto_altitude_m >= 0.0f;
	ImGui::Separator();
	ImGui::TextUnformatted("Override Goto Command");
	ImGui::SetNextItemWidth(140.0f);
	ImGui::InputFloat("Target X", &goto_x, 0.5f, 2.0f, "%.2f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(140.0f);
	ImGui::InputFloat("Target Y", &goto_y, 0.5f, 2.0f, "%.2f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(140.0f);
	ImGui::InputFloat("Target Alt (m)", &goto_altitude_m, 1.0f, 5.0f, "%.1f");
	ImGui::SameLine();
	ImGui::BeginDisabled(!state.is_running || !geofence_ready || !goto_inside_geofence || !goto_altitude_valid);
	if (ImGui::Button("Send Goto CMD", ImVec2(160.0f, 0.0f)) && actions.on_send_goto) {
		if (actions.on_send_goto(goto_x, goto_y, goto_altitude_m)) {
			char buffer[128];
			std::snprintf(
				buffer,
				sizeof(buffer),
				"Queued MAV_CMD_OVERRIDE_GOTO to X %.2f Y %.2f Alt %.1f m",
				goto_x,
				goto_y,
				goto_altitude_m);
			goto_status = buffer;
			goto_status_is_error = false;
		} else {
			goto_status = "Goto command rejected by GCS";
			goto_status_is_error = true;
		}
	}
	ImGui::EndDisabled();
	if (geofence_ready && goto_inside_geofence && goto_altitude_valid) {
		goto_status = "Geofence received - ready to send Goto command";
		goto_status_is_error = false;
	} else if (!geofence_ready) {
		goto_status = "Goto command disabled until full geofence is received";
		goto_status_is_error = true;
	} else if (!goto_inside_geofence) {
		goto_status = "Target is outside the geofence";
		goto_status_is_error = true;
	} else if (!goto_altitude_valid) {
		goto_status = "Target altitude must be zero or higher";
		goto_status_is_error = true;
	} else {
		goto_status = "Unexpected state";
		goto_status_is_error = true;
	}
	ImGui::TextColored(
		goto_status_is_error ? ImVec4(0.95f, 0.45f, 0.35f, 1.0f) : ImVec4(0.45f, 0.85f, 0.60f, 1.0f),
		"%s",
		goto_status.c_str());
	ImGui::Separator();

	ImGui::Columns(1, "telemetry_columns", false);
	ImGui::Text("time_boot_ms: %u", state.telemetry.time_boot_ms);
	ImGui::Text("X: %.2f m", state.telemetry.x);
	ImGui::Text("Y: %.2f m", state.telemetry.y);
	ImGui::Text("Alt: %.2f m", state.telemetry.z);
	ImGui::Text("VX: %.2f m/s", state.telemetry.vx);
	ImGui::Text("VY: %.2f m/s", state.telemetry.vy);
	ImGui::Text("VAlt: %.2f m/s", state.telemetry.vz);

	ImGui::Separator();

	const float plot_spacing = ImGui::GetStyle().ItemSpacing.y + ImGui::GetTextLineHeightWithSpacing();
	const float available_plot_height = std::max(ImGui::GetContentRegionAvail().y, 0.0f);
	const float preferred_position_height = 420.0f;
	const float preferred_altitude_height = 260.0f;
	const float preferred_total_height = preferred_position_height + preferred_altitude_height + plot_spacing;
	const float plot_scale = preferred_total_height > 0.0f
		? std::min(1.0f, available_plot_height / preferred_total_height)
		: 1.0f;
	const float position_plot_height = preferred_position_height * plot_scale;
	const float altitude_plot_height = preferred_altitude_height * plot_scale;

	detail::draw_position_plot(state.telemetry, position_plot_height);
	detail::draw_altitude_plot(state.telemetry, altitude_plot_height);

	ImGui::End();
}

} // namespace gcs_ui

#endif // GCS_DASHBOARD_H
