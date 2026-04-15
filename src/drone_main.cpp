#include <app_config.h>
#include <drone_mavlink_node.h>
#include <drone_simulator.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <thread>

namespace {

std::atomic_bool g_running{true};

/**
 * Runtime objects required by the drone process.
 */
struct DroneRuntimeContext {
	SharedConfig shared_config;
	DroneConfig drone_config;
	std::unique_ptr<DroneTelemetrySimulator> telemetry;
};

void signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		g_running = false;
	}
}

bool load_runtime_context(DroneRuntimeContext& context)
{
	context.shared_config = app_config::load_shared_config();
	context.drone_config = app_config::load_drone_config();

	if (!app_config::find_config_path("shared_config.json").has_value() || !app_config::is_valid(context.shared_config)) {
		std::fprintf(stderr, "Drone: shared_config.json is missing or invalid\n");
		return false;
	}

	if (!app_config::find_config_path("drone_config.json").has_value() || !app_config::is_valid(context.drone_config)) {
		std::fprintf(stderr, "Drone: drone_config.json is missing or invalid\n");
		return false;
	}

	context.telemetry = std::make_unique<DroneTelemetrySimulator>(context.drone_config);
	return true;
}

void log_loaded_config(const DroneConfig& config)
{
	if (const auto cfg_path = app_config::find_config_path("drone_config.json"); cfg_path.has_value()) {
		std::printf(
			"Drone: loaded config from %s (climb_rate_mps=%.2f, land_rate_mps=%.2f, arm_target_altitude_m=%.2f)\n",
			cfg_path->string().c_str(),
			config.climb_rate_mps,
			config.land_rate_mps,
			config.arm_target_altitude_m);
		for (size_t i = 0; i < config.geofence_corners_m.size(); ++i) {
			const auto& corner = config.geofence_corners_m[i];
			std::printf("Drone: geofence corner %zu -> X=%.2f Y=%.2f\n", i + 1, corner.x, corner.y);
		}
		return;
	}

	std::printf(
		"Drone: drone_config.json not found; using defaults (climb_rate_mps=%.2f, land_rate_mps=%.2f, arm_target_altitude_m=%.2f)\n",
		config.climb_rate_mps,
		config.land_rate_mps,
		config.arm_target_altitude_m);
}

void simulation_worker(DroneTelemetrySimulator& telemetry, const DroneConfig& config)
{
	const double update_rate_hz = std::max(1.0, static_cast<double>(config.update_rate_hz));
	const auto update_interval = std::chrono::duration<double>(1.0 / update_rate_hz);

	while (g_running) {
		telemetry.update();
		std::this_thread::sleep_for(update_interval);
	}
}

} // namespace

int main()
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	DroneRuntimeContext context;
	if (!load_runtime_context(context)) {
		return 1;
	}

	log_loaded_config(context.drone_config);

	DroneMavlinkNode mavlink_node(*context.telemetry, context.shared_config, context.drone_config, g_running);

	std::thread sim_thread([&context] {
		simulation_worker(*context.telemetry, context.drone_config);
	});
	std::thread mavlink_thread(&DroneMavlinkNode::run, &mavlink_node);

	sim_thread.join();
	mavlink_thread.join();

	std::printf("Drone: stopped\n");
	return 0;
}

