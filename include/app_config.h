#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

/**
 * Network settings shared by the drone and the GCS.
 */
struct SharedConfig {
	std::string host;
	uint16_t gcs_port = 0;
	uint16_t drone_port = 0;
};

/**
 * Runtime configuration for the ground control station.
 */
struct GcsConfig {
	int poll_timeout_ms = 0;
};

/**
 * Simple 2D point expressed in local meters.
 */
struct XYPoint {
	float x = 0.0f;
	float y = 0.0f;
};

/**
 * User-configurable drone simulation settings.
 */
struct DroneConfig {
	float max_velocity_mps = 0.0f;
	float manual_horizontal_velocity_mps = 0.0f;
	float update_rate_hz = 0.0f;
	float heartbeat_rate_hz = 0.0f;
	float position_rate_hz = 0.0f;
	float climb_rate_mps = 0.0f;
	float manual_vertical_velocity_mps = 0.0f;
	float land_rate_mps = 0.0f;
	float arm_target_altitude_m = 0.0f;
	float horizontal_arrival_tolerance_m = 0.75f;
	int manual_control_timeout_ms = 250;
	int rx_poll_timeout_ms = 0;
	uint16_t drone_system_id = 1;
	uint16_t drone_component_id = 1;
	uint16_t gcs_system_id = 255;
	uint16_t gcs_component_id = 250;
	std::array<XYPoint, 4> geofence_corners_m{};
};

namespace app_config {

inline std::array<XYPoint, 4> sort_geofence_corners(std::array<XYPoint, 4> corners)
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

inline XYPoint compute_polygon_centroid(const std::array<XYPoint, 4>& polygon)
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

inline bool is_point_inside_polygon(const XYPoint& point, const std::array<XYPoint, 4>& polygon)
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

inline std::optional<std::filesystem::path> find_config_path(const std::string& file_name)
{
	std::filesystem::path config_path;

	const auto repo_root_from_header = std::filesystem::path(__FILE__).parent_path().parent_path();
	config_path = repo_root_from_header / "config" / file_name;

	if (std::filesystem::exists(config_path) && std::filesystem::is_regular_file(config_path)) {
		return config_path;
	}

	return std::nullopt;
}

inline std::optional<std::string> read_config_text(const std::string& file_name)
{
	const auto path = find_config_path(file_name);
	if (!path.has_value()) {
		return std::nullopt;
	}

	std::ifstream in(*path);
	if (!in.is_open()) {
		return std::nullopt;
	}

	std::ostringstream ss;
	ss << in.rdbuf();
	return ss.str();
}

inline std::optional<double> get_json_number(const std::string& json, const std::string& key)
{
	const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
	std::smatch match;
	if (!std::regex_search(json, match, pattern)) {
		return std::nullopt;
	}

	try {
		return std::stod(match[1].str());
	} catch (...) {
		return std::nullopt;
	}
}

inline std::optional<std::string> get_json_string(const std::string& json, const std::string& key)
{
	const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
	std::smatch match;
	if (!std::regex_search(json, match, pattern)) {
		return std::nullopt;
	}

	return match[1].str();
}

inline std::optional<std::array<XYPoint, 4>> get_json_xy_points(
	const std::string& json,
	const std::string& key)
{
	const std::regex array_pattern("\"" + key + "\"\\s*:\\s*\\[([\\s\\S]*?)\\]");
	std::smatch array_match;
	if (!std::regex_search(json, array_match, array_pattern)) {
		return std::nullopt;
	}

	const std::string points_json = array_match[1].str();
	const std::regex point_pattern(
		"\\{\\s*\"x\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*,\\s*\"y\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)\\s*\\}");

	std::array<XYPoint, 4> points{};
	size_t index = 0;
	for (std::sregex_iterator it(points_json.begin(), points_json.end(), point_pattern), end;
		 it != end;
		 ++it) {
		if (index >= points.size()) {
			return std::nullopt;
		}

		try {
			points[index++] = XYPoint{
				static_cast<float>(std::stof((*it)[1].str())),
				static_cast<float>(std::stof((*it)[2].str())),
			};
		} catch (...) {
			return std::nullopt;
		}
	}

	if (index != points.size()) {
		return std::nullopt;
	}

	return points;
}

inline float polygon_area(std::array<XYPoint, 4> points)
{
	points = sort_geofence_corners(points);

	float twice_area = 0.0f;
	for (size_t i = 0; i < points.size(); ++i) {
		const auto& current = points[i];
		const auto& next = points[(i + 1) % points.size()];
		twice_area += (current.x * next.y) - (next.x * current.y);
	}

	return std::abs(twice_area) * 0.5f;
}

template <typename T>
inline void set_if_present_from_json_number(const std::string& json, const std::string& key, T& out_value)
{
	const auto value = get_json_number(json, key);
	if (value.has_value()) {
		out_value = static_cast<T>(*value);
	}
}

inline SharedConfig load_shared_config()
{
	SharedConfig cfg;
	const auto json = read_config_text("shared_config.json");
	if (json.has_value()) {
		const auto host = get_json_string(*json, "host");
		if (host.has_value() && !host->empty()) {
			cfg.host = *host;
		}
		set_if_present_from_json_number(*json, "gcs_port", cfg.gcs_port);
		set_if_present_from_json_number(*json, "drone_port", cfg.drone_port);
	}

	if (cfg.gcs_port == 0 || cfg.drone_port == 0) {
		if (const auto gcs_json = read_config_text("gcs_config.json"); gcs_json.has_value()) {
			if (cfg.gcs_port == 0) {
				set_if_present_from_json_number(*gcs_json, "gcs_port", cfg.gcs_port);
			}
			if (cfg.drone_port == 0) {
				set_if_present_from_json_number(*gcs_json, "drone_port", cfg.drone_port);
			}
		}
		if (const auto drone_json = read_config_text("drone_config.json"); drone_json.has_value()) {
			if (cfg.drone_port == 0) {
				set_if_present_from_json_number(*drone_json, "drone_port", cfg.drone_port);
			}
			if (cfg.gcs_port == 0) {
				set_if_present_from_json_number(*drone_json, "gcs_port", cfg.gcs_port);
			}
		}
	}

	return cfg;
}

inline GcsConfig load_gcs_config()
{
	GcsConfig cfg;
	const auto json = read_config_text("gcs_config.json");
	if (!json.has_value()) {
		return cfg;
	}

	set_if_present_from_json_number(*json, "poll_timeout_ms", cfg.poll_timeout_ms);
	return cfg;
}

inline DroneConfig load_drone_config()
{
	DroneConfig cfg;
	const auto json = read_config_text("drone_config.json");
	if (!json.has_value()) {
		return cfg;
	}

	set_if_present_from_json_number(*json, "max_velocity_mps", cfg.max_velocity_mps);
	set_if_present_from_json_number(*json, "manual_horizontal_velocity_mps", cfg.manual_horizontal_velocity_mps);
	set_if_present_from_json_number(*json, "update_rate_hz", cfg.update_rate_hz);
	set_if_present_from_json_number(*json, "heartbeat_rate_hz", cfg.heartbeat_rate_hz);
	set_if_present_from_json_number(*json, "position_rate_hz", cfg.position_rate_hz);
	set_if_present_from_json_number(*json, "climb_rate_mps", cfg.climb_rate_mps);
	set_if_present_from_json_number(*json, "manual_vertical_velocity_mps", cfg.manual_vertical_velocity_mps);
	set_if_present_from_json_number(*json, "land_rate_mps", cfg.land_rate_mps);
	set_if_present_from_json_number(*json, "arm_target_altitude_m", cfg.arm_target_altitude_m);
	set_if_present_from_json_number(*json, "horizontal_arrival_tolerance_m", cfg.horizontal_arrival_tolerance_m);
	set_if_present_from_json_number(*json, "manual_control_timeout_ms", cfg.manual_control_timeout_ms);
	set_if_present_from_json_number(*json, "rx_poll_timeout_ms", cfg.rx_poll_timeout_ms);
	set_if_present_from_json_number(*json, "drone_system_id", cfg.drone_system_id);
	set_if_present_from_json_number(*json, "drone_component_id", cfg.drone_component_id);
	set_if_present_from_json_number(*json, "gcs_system_id", cfg.gcs_system_id);
	set_if_present_from_json_number(*json, "gcs_component_id", cfg.gcs_component_id);
	if (const auto geofence_corners = get_json_xy_points(*json, "geofence_corners_m"); geofence_corners.has_value()) {
		cfg.geofence_corners_m = *geofence_corners;
	}

	if (cfg.manual_horizontal_velocity_mps <= 0.0f) {
		cfg.manual_horizontal_velocity_mps = cfg.max_velocity_mps;
	}
	if (cfg.manual_vertical_velocity_mps <= 0.0f) {
		cfg.manual_vertical_velocity_mps = cfg.climb_rate_mps;
	}
	return cfg;
}

inline bool is_valid(const SharedConfig& cfg)
{
	return !cfg.host.empty() && cfg.gcs_port != 0 && cfg.drone_port != 0;
}

inline bool is_valid(const GcsConfig& cfg)
{
	return cfg.poll_timeout_ms > 0;
}

inline bool is_valid(const DroneConfig& cfg)
{
	return cfg.max_velocity_mps > 0.0f
		&& cfg.manual_horizontal_velocity_mps > 0.0f
		&& cfg.update_rate_hz > 0.0f
		&& cfg.heartbeat_rate_hz > 0.0f
		&& cfg.position_rate_hz > 0.0f
		&& cfg.climb_rate_mps > 0.0f
		&& cfg.manual_vertical_velocity_mps > 0.0f
		&& cfg.land_rate_mps > 0.0f
		&& cfg.arm_target_altitude_m > 0.0f
		&& cfg.horizontal_arrival_tolerance_m > 0.0f
		&& cfg.manual_control_timeout_ms > 0
		&& cfg.rx_poll_timeout_ms > 0
		&& cfg.drone_system_id > 0
		&& cfg.drone_component_id > 0
		&& cfg.gcs_system_id > 0
		&& cfg.gcs_component_id > 0
		&& polygon_area(cfg.geofence_corners_m) > 0.01f;
}

} // namespace app_config

#endif // APP_CONFIG_H