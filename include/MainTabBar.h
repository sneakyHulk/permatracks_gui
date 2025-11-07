#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>
#include <boost/asio.hpp>
#include <string>

#include "CalibrationTab.h"
#include "ConnectionTab.h"

class MainTabBar : ConnectionTab, CalibrationTab {
	std::string active_tab = "Connection";

public:
	MainTabBar(SerialConnection& connection) : ConnectionTab(connection), CalibrationTab(connection) {
	}

	void render() {
		ImGui::BeginTabBar("Main Tab Bar");

		if (ImGui::BeginTabItem("Connection", nullptr, active_tab == "Connection" ? ImGuiTabItemFlags_SetSelected : 0)) {
			ConnectionTab::render();
		}

		if (ImGui::BeginTabItem("Calibration", nullptr, ImGuiTabItemFlags_SetSelected)) {
			CalibrationTab::render();
		}

		if (ImGui::BeginTabItem("Tracking Solution")) {
			ImGui::BeginChild("Tracking Solution Child", ImVec2(-1, -1), true);

			ImGui::Text("Tracking");
			ImGui::Separator();

			ImGui::EndChild();
			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	void set_active_tab(std::string tab_name) final {
		active_tab = tab_name;
		ImGui::SetTabItemClosed(tab_name.c_str());

	}
};