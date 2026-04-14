#ifndef GCS_GUI_HOST_H
#define GCS_GUI_HOST_H

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
	GuiHost(const GuiHost&) = delete;
	GuiHost& operator=(const GuiHost&) = delete;

	~GuiHost()
	{
		shutdown();
	}

	bool initialize(const char* title = "Krattworks GCS")
	{
		if (!glfwInit()) {
			std::fprintf(stderr, "GCS: failed to initialize GLFW\n");
			return false;
		}

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

		window_ = glfwCreateWindow(kWindowWidth, kWindowHeight, title, nullptr, nullptr);
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

	bool should_close() const
	{
		return !window_ || glfwWindowShouldClose(window_);
	}

	void begin_frame()
	{
		glfwPollEvents();
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
	}

	void render_dashboard(const DashboardState& state, const DashboardActions& actions)
	{
		gcs_ui::render_dashboard(state, actions);
	}

	void end_frame()
	{
		ImGui::Render();
		int display_width = 0;
		int display_height = 0;
		glfwGetFramebufferSize(window_, &display_width, &display_height);
		glViewport(0, 0, display_width, display_height);
		glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window_);
	}

private:
	static constexpr int kWindowWidth = 1280;
	static constexpr int kWindowHeight = 800;

	GLFWwindow* window_ = nullptr;
	bool is_initialized_ = false;
	const char* glsl_version_ = "#version 130";
};

} // namespace gcs_ui

#endif // GCS_GUI_HOST_H
