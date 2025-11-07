#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <string>

#include "SerialConnection.h"
#include "list_serial_devices.h"

class ConnectionTab {
	int selected_index = -1;
	std::vector<SerialDeviceInfo> devices = list_serial_devices();

	std::vector<char> buffer;
	SerialConnection& connection;
	std::string error_message;

public:
	explicit ConnectionTab(SerialConnection& connection) : connection(connection) {
	}

	void render() {
		ImGui::BeginChild("Connection Tab Child", ImVec2(-1, -1), true);

		ImGui::Text("Connect to the magnetic tracking device by selecting the right serial port and clicking connect");

		ImGui::Separator();

		refresh_connect_tap_serial_device();

		ImGui::Separator();

		ImGui::BeginDisabled(connection.connected());
		if (ImGui::BeginTable("SerialPortsTable", 6,
			ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
			ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp)) {
			ImGui::TableSetupScrollFreeze(0, 1); // freeze header
			ImGui::TableSetupColumn("Device");
			ImGui::TableSetupColumn("Product");
			ImGui::TableSetupColumn("Manufacturer");
			ImGui::TableSetupColumn("VID:PID");
			ImGui::TableSetupColumn("Serial #");
			ImGui::TableSetupColumn("Location");
			ImGui::TableHeadersRow();

			for (auto index = 0; auto& [device, manufacturer, product, serialNumber, location, vid, pid] : devices) {
				ImGui::TableNextRow();

				ImGui::TableSetColumnIndex(0);
				if (ImGui::Selectable(device.c_str(), index == selected_index, ImGuiSelectableFlags_SpanAllColumns)) {
					selected_index = index;
				}

				ImGui::TableSetColumnIndex(1);
				ImGui::TextUnformatted(product.empty() ? "-" : product.c_str());

				ImGui::TableSetColumnIndex(2);
				ImGui::TextUnformatted(manufacturer.empty() ? "-" : manufacturer.c_str());

				ImGui::TableSetColumnIndex(3);
				ImGui::Text("VID:0x%04X PID:0x%04X", vid, pid);

				ImGui::TableSetColumnIndex(4);
				ImGui::TextUnformatted(serialNumber.empty() ? "-" : serialNumber.c_str());

				ImGui::TableSetColumnIndex(5);
				ImGui::TextUnformatted(location.empty() ? "-" : location.c_str());

				++index;
			}

			ImGui::EndTable();
		}
		ImGui::EndDisabled();

		if (!error_message.empty()) {
			ImGui::OpenPopup("ERROR");
		}

		ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
		if (ImGui::BeginPopupModal("ERROR", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse)) {
			ImGui::TextWrapped("%s", error_message.c_str());
			ImGui::Spacing();
			ImGui::Separator();
			if (ImGui::Button("OK", {500, 0})) {
				error_message.clear();
				ImGui::CloseCurrentPopup();
			}
			ImGui::EndPopup();
		}

		ImGui::EndChild();

		if (connection.connected()) {
			ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
			ImGui::Begin("Connection", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

			ImGui::TextWrapped("Connected to %s with baud rate %u.", devices[selected_index].device.c_str(), connection.baud());
			ImGui::Spacing();
			ImGui::Separator();

			if (ImGui::Button("Disconnect", {500, 0})) { stop_connecting(); }

			ImGui::End();
		}

		ImGui::EndTabItem();
	}

private:
	void refresh() {
		devices = list_serial_devices();
		selected_index = -1;
	}

	void start_tapping() {
		if (auto err = connection.open_serial_port(devices[selected_index].device)) {
			ImGui::OpenPopup("Serial Output");
		} else {
			error_message = err.error().message + " (" + std::to_string(err.error().code) + ")";
		}
	}

	void start_connecting() {
		if (auto err = connection.open_serial_port(devices[selected_index].device)) {
		} else {
			error_message = err.error().message + " (" + std::to_string(err.error().code) + ")";
		}
	}

	void stop_connecting() {
		connection.close_serial_port();
	}

	void stop_tapping() {
		buffer.clear();
		connection.close_serial_port();
	}

	void refresh_connect_tap_serial_device() {
		ImGui::BeginDisabled(connection.connected());
		if (ImGui::Button("Refresh Serial Device List")) {
			refresh();
		}

		ImGui::SameLine();
		ImGui::Text("Found %zu serial devices", devices.size());

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::GetStyle().FramePadding.x * 2);
		if (ImGui::Button("Connect")) {
			if (selected_index < 0) {
				ImGui::OpenPopup("Select Serial Device");
			} else {
				start_connecting();
			}
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::CalcTextSize("Tap").x - ImGui::GetStyle().FramePadding.x * 5);
		if (ImGui::Button("Tap")) {
			if (selected_index < 0) {
				ImGui::OpenPopup("Select Serial Device");
			} else {
				start_tapping();
			}
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::CalcTextSize("Tap").x - 150 - ImGui::GetStyle().FramePadding.x * 6);

		ImGui::SetNextItemWidth(150);
		if (ImGui::BeginCombo("##combo", std::to_string(connection.baud()).c_str())) {
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD9600).c_str(), connection.baud() == Baudrate::BAUD9600)) { connection.baud() = Baudrate::BAUD9600; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD19200).c_str(), connection.baud() == Baudrate::BAUD19200)) { connection.baud() = Baudrate::BAUD19200; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD38400).c_str(), connection.baud() == Baudrate::BAUD38400)) { connection.baud() = Baudrate::BAUD38400; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD57600).c_str(), connection.baud() == Baudrate::BAUD57600)) { connection.baud() = Baudrate::BAUD57600; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD115200).c_str(), connection.baud() == Baudrate::BAUD115200)) { connection.baud() = Baudrate::BAUD115200; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD230400).c_str(), connection.baud() == Baudrate::BAUD230400)) { connection.baud() = Baudrate::BAUD230400; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD460800).c_str(), connection.baud() == Baudrate::BAUD460800)) { connection.baud() = Baudrate::BAUD460800; }
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD921600).c_str(), connection.baud() == Baudrate::BAUD921600)) { connection.baud() = Baudrate::BAUD921600; }
			ImGui::EndCombo();
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::CalcTextSize("Tap").x - 150 - ImGui::CalcTextSize("Select Baud Rate:").x - ImGui::GetStyle().FramePadding.x * 8);
		ImGui::Text("Select Baud Rate:");
		ImGui::EndDisabled();

		ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
		if (ImGui::BeginPopupModal("Select Serial Device", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse)) {
			ImGui::Text("Please select a serial device first.");
			ImGui::Spacing();

			if (ImGui::Button("OK", {-1, 0})) {
				ImGui::CloseCurrentPopup();
			}

			ImGui::EndPopup();
		}

		ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
		if (ImGui::BeginPopupModal("Serial Output", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse)) {
			ImGui::Text("Serial data stream:");
			ImGui::Separator();

			ImGui::BeginChild("SerialScroll", ImVec2(500, 300), true);

			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));

			if (auto reading = connection.read_some()) {
				buffer.insert(buffer.end(), reading.value().begin(), reading.value().end());
			} else {
				ImGui::CloseCurrentPopup();

				error_message = reading.error().message + " (" + std::to_string(reading.error().code) + ")";
				stop_tapping();
			}

			ImGui::TextUnformatted(buffer.data(), buffer.data() + buffer.size());
			ImGui::PopStyleVar();

			// Auto-scroll to bottom
			if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
				ImGui::SetScrollHereY(1.0f);

			ImGui::EndChild();

			ImGui::Spacing();
			if (ImGui::Button("Close", ImVec2(-1, 0))) {
				ImGui::CloseCurrentPopup();

				stop_tapping();
			}

			ImGui::EndPopup();
		}
	}
};