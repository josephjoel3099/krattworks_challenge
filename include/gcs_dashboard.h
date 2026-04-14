#ifndef GCS_DASHBOARD_H
#define GCS_DASHBOARD_H

#include <gcs_ui_types.h>

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace gcs_ui {
namespace detail {

constexpr ImVec2 kMinPlotSize{320.0f, 320.0f};

inline float compute_plot_extent(const std::deque<ImVec2>& history, float current_x, float current_y)
{
	float max_abs = std::max(std::abs(current_x), std::abs(current_y));
	for (const ImVec2& point : history) {
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

inline bool has_recent_data(const TelemetrySnapshot& telemetry)
{
	const auto now = std::chrono::steady_clock::now();
	return telemetry.last_message_time.time_since_epoch().count() != 0
		&& std::chrono::duration_cast<std::chrono::milliseconds>(now - telemetry.last_message_time).count() < 1500;
}

inline void draw_position_plot(const TelemetrySnapshot& telemetry)
{
	ImGui::TextUnformatted("2D Position (X/Y)");
	ImVec2 canvas_size = ImGui::GetContentRegionAvail();
	canvas_size.x = std::max(canvas_size.x, kMinPlotSize.x);
	canvas_size.y = std::max(std::min(canvas_size.y, 420.0f), kMinPlotSize.y);

	const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	const ImVec2 canvas_end(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);

	draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(15, 18, 24, 255), 8.0f);
	draw_list->AddRect(canvas_pos, canvas_end, IM_COL32(82, 96, 122, 255), 8.0f, 0, 1.5f);

	const float extent = compute_plot_extent(telemetry.position_history, telemetry.x, telemetry.y);
	const ImU32 grid_color = IM_COL32(65, 76, 96, 255);
	const ImU32 axis_color = IM_COL32(124, 170, 255, 255);
	const ImU32 path_color = IM_COL32(110, 220, 170, 255);
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

	if (telemetry.position_history.size() > 1) {
		std::vector<ImVec2> points;
		points.reserve(telemetry.position_history.size());
		for (const ImVec2& point : telemetry.position_history) {
			points.push_back(world_to_canvas(point.x, point.y, canvas_pos, canvas_size, extent));
		}
		draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), path_color, 0, 2.0f);
	}

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

} // namespace detail

inline void render_dashboard(const DashboardState& state, const DashboardActions& actions)
{
	ImGui::SetNextWindowPos(ImVec2(24.0f, 24.0f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(1100.0f, 700.0f), ImGuiCond_Once);
	ImGui::Begin("Ground Control Station", nullptr, ImGuiWindowFlags_NoCollapse);

	ImGui::Text("Link: %s", detail::has_recent_data(state.telemetry) ? "connected" : "waiting for telemetry");
	ImGui::SameLine();
	ImGui::Text("Host %s  GCS %u  Drone %u", state.host, state.gcs_port, state.drone_port);
	ImGui::Separator();

	ImGui::BeginDisabled(!state.is_running);
	if (ImGui::Button("ARM", ImVec2(120.0f, 0.0f)) && actions.on_arm) {
		actions.on_arm();
	}
	ImGui::SameLine();
	if (ImGui::Button("LAND", ImVec2(120.0f, 0.0f)) && actions.on_land) {
		actions.on_land();
	}
	ImGui::SameLine();
	if (ImGui::Button("DISARM", ImVec2(120.0f, 0.0f)) && actions.on_disarm) {
		actions.on_disarm();
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear Track", ImVec2(120.0f, 0.0f)) && actions.on_clear_track) {
		actions.on_clear_track();
	}
	ImGui::EndDisabled();

	ImGui::Text("Arm target altitude: %.1f m", state.arm_target_altitude_m);
	ImGui::Separator();

	ImGui::Columns(2, "telemetry_columns", false);
	ImGui::Text("Heartbeat received: %s", state.telemetry.has_heartbeat ? "yes" : "no");
	ImGui::Text("Position received: %s", state.telemetry.has_position ? "yes" : "no");
	ImGui::Text("System/Component: %u / %u", state.telemetry.system_id, state.telemetry.component_id);
	ImGui::Text("Vehicle type: %u", state.telemetry.vehicle_type);
	ImGui::Text("Base mode: 0x%02X", state.telemetry.base_mode);
	ImGui::Text("Custom mode: %u", state.telemetry.custom_mode);
	ImGui::Text("System status: %u", state.telemetry.system_status);

	ImGui::NextColumn();
	ImGui::Text("time_boot_ms: %u", state.telemetry.time_boot_ms);
	ImGui::Text("X: %.2f m", state.telemetry.x);
	ImGui::Text("Y: %.2f m", state.telemetry.y);
	ImGui::Text("Z: %.2f m", state.telemetry.z);
	ImGui::Text("VX: %.2f m/s", state.telemetry.vx);
	ImGui::Text("VY: %.2f m/s", state.telemetry.vy);
	ImGui::Text("VZ: %.2f m/s", state.telemetry.vz);
	ImGui::Columns(1);
	ImGui::Separator();

	detail::draw_position_plot(state.telemetry);

	ImGui::End();
}

} // namespace gcs_ui

#endif // GCS_DASHBOARD_H
