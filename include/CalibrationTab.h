#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>
#include <string>

#include "SerialConnection.h"

class CalibrationTab {
	SerialConnection& connection;

public:
	explicit CalibrationTab(SerialConnection& connection) : connection(connection) {
	}

	void render() {
		ImGui::BeginChild("Calibration Child", ImVec2(-1, -1), true);

		if (connection.connected()) {
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
		ImGui::EndTabItem();
	}

	virtual void set_active_tab(std::string tab_name) = 0;
};