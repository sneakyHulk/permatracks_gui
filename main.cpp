#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <cmath>
#include <filesystem>
#include <ranges>

#include "App.h"
#include "SerialConnection.h"

int main(int, char**) {
	App app;

	HelloImGui::RunnerParams params;

	params.appWindowParams.windowTitle = "PERMATRACKS";

	params.appWindowParams.windowGeometry.size = {1024, 720};
	params.appWindowParams.windowGeometry.positionMode = HelloImGui::WindowPositionMode::MonitorCenter;
	params.appWindowParams.windowGeometry.monitorIdx = 0;

	params.fpsIdling.enableIdling = false;

	params.callbacks.LoadAdditionalFonts = []() {
		ImGuiIO& io = ImGui::GetIO();
		io.FontDefault = io.Fonts->AddFontFromFileTTF((std::filesystem::path(CMAKE_SOURCE_DIR) / "fonts" / "RobotoMono-VariableFont_wght.ttf").c_str(), 24.0f);
	};

	params.appWindowParams.restorePreviousGeometry = true;

	params.platformBackendType = HelloImGui::PlatformBackendType::Glfw;

	params.imGuiWindowParams.showMenuBar = true;
	params.imGuiWindowParams.showMenu_App = true;   // shows "App" menu
	params.imGuiWindowParams.showMenu_View = true;  // shows "View" menu

	params.callbacks.ShowGui = [&app]() { app.render(); };

	params.callbacks.PostInit = []() {
		ImPlot::CreateContext();
		ImPlot3D::CreateContext();
	};

	params.callbacks.BeforeExit = []() {
		ImPlot3D::DestroyContext();
		ImPlot::DestroyContext();
	};

	HelloImGui::Run(params);

	return 0;
}