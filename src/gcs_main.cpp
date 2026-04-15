/**
 * @file gcs_main.cpp
 * @brief Entry point for the ground control station application.
 *
 * Validates configuration, starts the MAVLink worker, and continuously renders
 * the dashboard until the operator exits or a shutdown signal is received.
 */
#include <app_config.h>
#include <gcs_gui_host.h>
#include <gcs_mavlink_node.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <thread>

namespace {

/** Groups the runtime dependencies needed by the GCS application. */
struct GcsRuntimeContext {
	SharedConfig shared_config;
	GcsConfig gcs_config;
	DroneConfig drone_config;
	GuiConfig gui_config;
};

std::atomic_bool* g_running_flag = nullptr;

/** Stops the application loop on process termination signals. */
void signal_handler(int signum)
{
	if ((signum == SIGINT || signum == SIGTERM) && g_running_flag != nullptr) {
		*g_running_flag = false;
	}
}

template <typename Config>
bool require_valid_config(const char* file_name, const Config& config, const char* app_name)
{
	if (!app_config::find_config_path(file_name).has_value() || !app_config::is_valid(config)) {
		std::fprintf(stderr, "%s: %s is missing or invalid\n", app_name, file_name);
		return false;
	}
	return true;
}

bool load_runtime_context(GcsRuntimeContext& context)
{
	context.shared_config = app_config::load_shared_config();
	context.gcs_config = app_config::load_gcs_config();
	context.drone_config = app_config::load_drone_config();
	context.gui_config = app_config::load_gui_config();

	return require_valid_config("shared_config.json", context.shared_config, "GCS")
		&& require_valid_config("gcs_config.json", context.gcs_config, "GCS")
		&& require_valid_config("drone_config.json", context.drone_config, "GCS")
		&& require_valid_config("gui_config.json", context.gui_config, "GCS");
}

} // namespace

int main()
{
	std::atomic_bool running{true};
	g_running_flag = &running;
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	GcsRuntimeContext context;
	if (!load_runtime_context(context)) {
		return 1;
	}

	GcsMavlinkNode mavlink_node(
		context.shared_config,
		context.gcs_config,
		context.drone_config,
		running);

	gcs_ui::GuiHost gui(context.gui_config);
	if (!gui.initialize()) {
		return 1;
	}

	std::thread mavlink_thread([&mavlink_node] { mavlink_node.run(); });

	// Keep rendering on the main thread while the worker thread handles all UDP
	// communication and telemetry decoding.
	while (running && !gui.should_close()) {
		gui.begin_frame();
		gui.render_dashboard(mavlink_node.makeDashboardState(), mavlink_node.makeDashboardActions());
		gui.end_frame();
	}

	running = false;
	if (mavlink_thread.joinable()) {
		mavlink_thread.join();
	}

	std::printf("GCS: stopped\n");
	return 0;
}
