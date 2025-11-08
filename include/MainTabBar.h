#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <boost/asio.hpp>
#include <queue>
#include <string>

#include "CalibrationTab.h"
#include "ConnectionTab.h"

class MainTabBar : ConnectionTab, CalibrationTab {
	std::string previous_active_tab = "Connection";
	std::string active_tab = "Connection";
	std::string new_active_tab = "Connection";

   public:
	MainTabBar(SerialConnection& connection) : ConnectionTab(connection), CalibrationTab(connection) {}

	void render() {
		ImGui::BeginTabBar("Main Tab Bar");

		if (active_tab != previous_active_tab) {
			active_tab = previous_active_tab;
		}

		if (previous_active_tab != new_active_tab) {
			previous_active_tab = new_active_tab;
		}

		if (ImGui::BeginTabItem("Connection", nullptr, previous_active_tab == "Connection" ? ImGuiTabItemFlags_SetSelected : 0)) {
			if (active_tab != "Connection" && active_tab == previous_active_tab) {
				new_active_tab = "Connection";
			} else {
				ConnectionTab::render();
			}
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Calibration", nullptr, previous_active_tab == "Calibration" ? ImGuiTabItemFlags_SetSelected : 0)) {
			if (active_tab != "Calibration" && active_tab == previous_active_tab) {
				new_active_tab = "Calibration";
			} else {
				CalibrationTab::render();
			}

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Tracking Solution", nullptr, previous_active_tab == "Tracking Solution" ? ImGuiTabItemFlags_SetSelected : 0)) {
			if (active_tab != "Tracking Solution") {
				new_active_tab = "Tracking Solution";
			} else {
				ImGui::BeginChild("Tracking Solution Child", ImVec2(-1, -1), true);

				ImGui::Text("Tracking");
				ImGui::Separator();

				ImGui::EndChild();
			}
			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	void set_active_tab(std::string const tab_name) final { new_active_tab = tab_name; }
};