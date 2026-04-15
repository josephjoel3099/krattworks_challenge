#ifndef GCS_GUI_HOST_H
#define GCS_GUI_HOST_H

#include <app_config.h>
#include <gcs_dashboard.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdio>

namespace gcs_ui {

class GuiHost {
public:
	GuiHost() = default;
	explicit GuiHost(const GuiConfig& config)
		: config_(config)
	{
	}

	GuiHost(const GuiHost&) = delete;
	GuiHost& operator=(const GuiHost&) = delete;

	~GuiHost()
	{
		shutdown();
	}

	/** Initializes GLFW, ImGui, and the main application window. */
	bool initialize(const char* title = nullptr)
	{
		if (!glfwInit()) {
			std::fprintf(stderr, "GCS: failed to initialize GLFW\n");
			return false;
		}

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

		const char* requested_title = (title != nullptr && title[0] != '\0') ? title : config_.window_title.c_str();
		window_ = glfwCreateWindow(config_.window_width, config_.window_height, requested_title, nullptr, nullptr);
		if (!window_) {
			std::fprintf(stderr, "GCS: failed to create GLFW window\n");
			glfwTerminate();
			return false;
		}

		glfwMakeContextCurrent(window_);
		glfwSwapInterval(1);

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
		ImGui::StyleColorsDark();

		if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
			std::fprintf(stderr, "GCS: failed to initialize ImGui GLFW backend\n");
			ImGui::DestroyContext();
			glfwDestroyWindow(window_);
			window_ = nullptr;
			glfwTerminate();
			return false;
		}

		if (!ImGui_ImplOpenGL3_Init(glsl_version_)) {
			std::fprintf(stderr, "GCS: failed to initialize ImGui OpenGL backend\n");
			ImGui_ImplGlfw_Shutdown();
			ImGui::DestroyContext();
			glfwDestroyWindow(window_);
			window_ = nullptr;
			glfwTerminate();
			return false;
		}

		is_initialized_ = true;
		return true;
	}

	/** Releases ImGui and GLFW resources. */
	void shutdown()
	{
		if (!is_initialized_) {
			return;
		}

		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
		if (window_) {
			glfwDestroyWindow(window_);
			window_ = nullptr;
		}
		glfwTerminate();
		is_initialized_ = false;
	}

	/** Returns true when the window has been closed by the user. */
	bool should_close() const
	{
		return !window_ || glfwWindowShouldClose(window_);
	}

	/** Begins a new ImGui frame. */
	void begin_frame()
	{
		glfwPollEvents();
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
	}

	/** Renders the ground control dashboard for the current frame. */
	void render_dashboard(const DashboardState& state, const DashboardActions& actions)
	{
		gcs_ui::render_dashboard(state, actions);
	}

	/** Finalizes rendering and presents the frame to the display. */
	void end_frame()
	{
		ImGui::Render();
		int display_width = 0;
		int display_height = 0;
		glfwGetFramebufferSize(window_, &display_width, &display_height);
		glViewport(0, 0, display_width, display_height);
		glClearColor(config_.clear_color_r, config_.clear_color_g, config_.clear_color_b, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window_);
	}

private:
	GuiConfig config_{};
	GLFWwindow* window_ = nullptr;
	bool is_initialized_ = false;
	const char* glsl_version_ = "#version 130";
};

} // namespace gcs_ui

#endif // GCS_GUI_HOST_H
