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
#include "MagnetSelectionTab.h"
#include "TrackingTab.h"
#include "ZeroingTab.h"

class MainTabBar : virtual protected SerialConnection, protected ConnectionTab, protected CalibrationTab, protected ZeroingTab, protected MagnetSelectionTab, protected TrackingTab {
	std::string previous_active_tab = "Connection";
	std::string active_tab = "Connection";
	std::string new_active_tab = "Connection";

   public:
	MainTabBar() {}

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
		} else {
			CalibrationTab::stop_thread();
		}

		if (ImGui::BeginTabItem("Zeroing", nullptr, previous_active_tab == "Zeroing" ? ImGuiTabItemFlags_SetSelected : 0)) {
			if (active_tab != "Zeroing" && active_tab == previous_active_tab) {
				new_active_tab = "Zeroing";
			} else {
				ZeroingTab::render();
			}

			ImGui::EndTabItem();
		} else {
			ZeroingTab::stop_thread();
		}

		if (ImGui::BeginTabItem("Magnet Selection", nullptr, previous_active_tab == "Magnet Selection" ? ImGuiTabItemFlags_SetSelected : 0)) {
			if (active_tab != "Magnet Selection" && active_tab == previous_active_tab) {
				new_active_tab = "Magnet Selection";
			} else {
				MagnetSelectionTab::render();
			}

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Tracking", nullptr, previous_active_tab == "Tracking" ? ImGuiTabItemFlags_SetSelected : 0)) {
			if (active_tab != "Tracking" && active_tab == previous_active_tab) {
				new_active_tab = "Tracking";
			} else {
				TrackingTab::render();
			}

			ImGui::EndTabItem();
		} else {
			TrackingTab::stop_thread();
		}

		ImGui::EndTabBar();
	}

	void set_active_tab(std::string const tab_name) final { new_active_tab = tab_name; }
};