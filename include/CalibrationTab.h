#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <string>

#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"
#include "MiMedMagnetometerArraySerialConnectionBinary.h"
#include "SerialConnection.h"

class CalibrationTab {
	SerialConnection& connection;
	MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>> array;

   public:
	explicit CalibrationTab(SerialConnection& connection) : connection(connection), array(connection) {}

	void plot() {
		if (auto const point = array.push(); point.has_value()) {
			if (ImPlot::BeginSubplots("41 scatter plots", 9, 5, ImVec2(-1, -1), ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_LinkAllY)) {
				for (auto e : point.value()) {
					if (ImPlot::BeginPlot("Sensor")) {
						ImPlot::PlotScatter("Graph", &e.x, &e.y, 1);
						ImPlot::EndPlot();
					}
				}

				ImPlot::EndSubplots();
			}
		} else {
			ImGui::Text("No Sensor Found");
		}
	}

	void render() {
		ImGui::BeginChild("Calibration Child", ImVec2(-1, -1), true);

		if (connection.connected()) {
			plot();
		} else {
			ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
			ImGui::Begin("No Connection", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

			ImGui::TextWrapped("Connect to a device first!");
			ImGui::Spacing();
			ImGui::Separator();

			if (ImGui::Button("Go to Connection Tab", {500, 0})) {
				set_active_tab("Connection");
			}

			ImGui::End();
		}

		ImGui::EndChild();
	}

	virtual void set_active_tab(std::string tab_name) = 0;
};