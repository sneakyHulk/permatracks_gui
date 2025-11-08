#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <cmath>
#include <ranges>

#include "App.h"
#include "SerialConnection.h"

void Demo_ImPlot3D_Scatter() {
	if (ImPlot3D::BeginPlot("Calibration", {-1, -1})) {
		// Generate random or computed 3D points
		static std::vector<double> xs, ys, zs;
		if (xs.empty()) {
			int count = 200;
			xs.resize(count);
			ys.resize(count);
			zs.resize(count);
			for (int i = 0; i < count; ++i) {
				xs[i] = 2.0 * (rand() / (double)RAND_MAX - 0.5);
				ys[i] = 2.0 * (rand() / (double)RAND_MAX - 0.5);
				zs[i] = std::sin(xs[i] * 3.0) * std::cos(ys[i] * 3.0);
			}
		}

		ImPlot3D::PlotScatter("Points", xs.data(), ys.data(), zs.data(), (int)xs.size());
		ImPlot3D::EndPlot();
	}
}

int main(int, char**) {
	boost::asio::io_context io;
	SerialConnection connection;

	App app(connection);

	HelloImGui::RunnerParams params;

	params.appWindowParams.windowTitle = "PERMATRACKS";

	params.appWindowParams.windowGeometry.size = {1024, 720};
	params.appWindowParams.windowGeometry.positionMode = HelloImGui::WindowPositionMode::MonitorCenter;
	params.appWindowParams.windowGeometry.monitorIdx = 0;

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