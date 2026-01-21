#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

#include <atomic>
#include <string>

#include "SerialConnection.h"
#include "list_serial_devices.h"

class ConnectionTab : virtual protected SerialConnection {
	std::atomic_bool error = false;

	enum class ConnectionTabState {
		NONE,
		TAPPING,
	};
	std::atomic<ConnectionTabState> state = ConnectionTabState::NONE;

	// ConnectionTab Data
	std::string error_message;

	int selected_index = -1;
	std::vector<SerialDeviceInfo> devices = list_serial_devices();

	std::vector<char> buffer;

   public:
	ConnectionTab() = default;
	~ConnectionTab() = default;

	void start_connection() {
		if (auto err = SerialConnection::open_serial_port(devices[selected_index].device)) {
		} else {
			error_message = err.error().what();

			error.store(true);
		}
	}

	void stop_connection() { SerialConnection::close_serial_port(); }

	void refresh() {
		devices = list_serial_devices();
		selected_index = -1;
	}

	void start_tapping() {
		if (auto err = SerialConnection::open_serial_port(devices[selected_index].device)) {
			state.store(ConnectionTabState::TAPPING);
		} else {
			error_message = err.error().what();
			error.store(true);
		}
	}

	void stop_tapping() {
		state.store(ConnectionTabState::NONE);

		buffer.clear();
		SerialConnection::close_serial_port();
	}

	void render() {
		auto const error_ = error.load();
		auto const connected_ = SerialConnection::connected();

		if (ImGui::BeginChild("Connection Child", ImVec2(-1, -1), true)) {
			ImGui::TextWrapped(
			    "Choose the correct serial port corresponding to your magnetic tracking device and click Connect to establish communication. You can tap the connected device to verify its identity and ensure that the correct port has been "
			    "selected.");

			ImGui::Separator();

			if (error_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("ERROR", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("%s", error_message.c_str());
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("OK", {500, 0})) {
					error.store(false);
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			switch (auto const current_state = state.load(); current_state) {
				case ConnectionTabState::NONE: {
					ImGui::BeginDisabled(connected_);
					render_table();
					ImGui::EndDisabled();

					break;
				}
				case ConnectionTabState::TAPPING: {
					ImGui::BeginDisabled(true);
					render_table();
					ImGui::EndDisabled();

					ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
					ImGui::Begin("Serial data stream", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

					ImGui::BeginChild("SerialScroll", ImVec2(500, 300), true);

					ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));

					if (auto reading = SerialConnection::read_some()) {
						buffer.insert(buffer.end(), reading.value().data.begin(), reading.value().data.end());
					} else {
						stop_tapping();

						error_message = reading.error().what();
						error.store(true);
					}

					ImGui::TextUnformatted(buffer.data(), buffer.data() + buffer.size());
					ImGui::PopStyleVar();

					// Auto-scroll to bottom
					if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);

					ImGui::EndChild();

					ImGui::Spacing();
					if (ImGui::Button("Close", ImVec2(-1, 0))) {
						stop_tapping();
					}

					ImGui::End();
					ImGui::EndChild();
					return;
				}
			}
		}

		if (connected_) {
			ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
			ImGui::Begin("Connection", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

			ImGui::TextWrapped("Connected to %s with baud rate %u.", devices[selected_index].device.c_str(), baud);
			ImGui::Spacing();
			ImGui::Separator();

			if (ImGui::Button("Disconnect", {500, 0})) {
				stop_connection();
			}

			ImGui::End();
			ImGui::EndChild();
			return;
		}

		ImGui::EndChild();
	}

   private:
	void render_table() {
		if (ImGui::Button("Refresh")) {
			refresh();
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::GetStyle().FramePadding.x * 2);
		if (ImGui::Button("Connect")) {
			if (selected_index < 0) {
				error_message = "Please select a serial device first";
				error.store(true);
			} else {
				start_connection();
			}
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::CalcTextSize("Tap").x - ImGui::GetStyle().FramePadding.x * 5);
		if (ImGui::Button("Tap")) {
			if (selected_index < 0) {
				error_message = "Please select a serial device first";
				error.store(true);
			} else {
				start_tapping();
			}
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::CalcTextSize("Tap").x - 150 - ImGui::GetStyle().FramePadding.x * 6);

		ImGui::SetNextItemWidth(150);
		if (ImGui::BeginCombo("##combo", std::to_string(baud).c_str())) {
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD9600).c_str(), baud == Baudrate::BAUD9600)) {
				baud = Baudrate::BAUD9600;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD19200).c_str(), baud == Baudrate::BAUD19200)) {
				baud = Baudrate::BAUD19200;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD38400).c_str(), baud == Baudrate::BAUD38400)) {
				baud = Baudrate::BAUD38400;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD57600).c_str(), baud == Baudrate::BAUD57600)) {
				baud = Baudrate::BAUD57600;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD115200).c_str(), baud == Baudrate::BAUD115200)) {
				baud = Baudrate::BAUD115200;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD230400).c_str(), baud == Baudrate::BAUD230400)) {
				baud = Baudrate::BAUD230400;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD460800).c_str(), baud == Baudrate::BAUD460800)) {
				baud = Baudrate::BAUD460800;
			}
			if (ImGui::Selectable(std::to_string(Baudrate::BAUD921600).c_str(), baud == Baudrate::BAUD921600)) {
				baud = Baudrate::BAUD921600;
			}
			ImGui::EndCombo();
		}

		ImGui::SameLine();
		ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("Connect").x - ImGui::CalcTextSize("Tap").x - 150 - ImGui::CalcTextSize("Baud Rate:").x - ImGui::GetStyle().FramePadding.x * 8);
		ImGui::Text("Baud Rate:");

		if (ImGui::BeginTable("SerialPortsTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp)) {
			ImGui::TableSetupScrollFreeze(0, 1);  // freeze header
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
	}
};