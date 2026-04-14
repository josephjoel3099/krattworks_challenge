#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

struct SharedConfig {
	std::string host = "127.0.0.1";
};

struct GcsConfig {
	uint16_t gcs_port = 14550;
	uint16_t drone_port = 14551;
	int poll_timeout_ms = 100;
};

struct DroneConfig {
	uint16_t drone_port = 14551;
	uint16_t gcs_port = 14550;
	float max_velocity_mps = 10.0f;
	float update_rate_hz = 100.0f;
	float heartbeat_rate_hz = 1.0f;
	float position_rate_hz = 10.0f;
	float climb_rate_mps = 5.0f;
	float land_rate_mps = 1.0f;
	float arm_target_altitude_m = 20.0f;
	int rx_poll_timeout_ms = 5;
};

namespace app_config {

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
	if (!json.has_value()) {
		return cfg;
	}

	const auto host = get_json_string(*json, "host");
	if (host.has_value() && !host->empty()) {
		cfg.host = *host;
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

	set_if_present_from_json_number(*json, "gcs_port", cfg.gcs_port);
	set_if_present_from_json_number(*json, "drone_port", cfg.drone_port);
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

	set_if_present_from_json_number(*json, "drone_port", cfg.drone_port);
	set_if_present_from_json_number(*json, "gcs_port", cfg.gcs_port);
	set_if_present_from_json_number(*json, "max_velocity_mps", cfg.max_velocity_mps);
	set_if_present_from_json_number(*json, "update_rate_hz", cfg.update_rate_hz);
	set_if_present_from_json_number(*json, "heartbeat_rate_hz", cfg.heartbeat_rate_hz);
	set_if_present_from_json_number(*json, "position_rate_hz", cfg.position_rate_hz);
	set_if_present_from_json_number(*json, "climb_rate_mps", cfg.climb_rate_mps);
	set_if_present_from_json_number(*json, "land_rate_mps", cfg.land_rate_mps);
	set_if_present_from_json_number(*json, "arm_target_altitude_m", cfg.arm_target_altitude_m);
	set_if_present_from_json_number(*json, "rx_poll_timeout_ms", cfg.rx_poll_timeout_ms);
	return cfg;
}

} // namespace app_config

#endif // APP_CONFIG_H