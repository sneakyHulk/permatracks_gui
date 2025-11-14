#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <expected>

#include "Calibration.h"
#include "CeresOptimizerDirectionVector.h"
#include "MagnetSelection.h"
#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"
#include "MiMedMagnetometerArraySerialConnectionBinary.h"
#include "SerialConnection.h"
#include "Zeroing.h"

class TrackingTab : virtual protected SerialConnection,
                    virtual protected MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>,
                    virtual protected Calibration<41>,
                    virtual protected Zeroing<41>,
                    virtual protected MagnetSelection,
                    virtual protected CeresOptimizerDirectionVector<41> {
	std::atomic_bool error = false;

	enum class TrackingTabState {
		NONE,
		TRACKING,
	};
	std::atomic<TrackingTabState> state = TrackingTabState::NONE;

	std::thread thread;

	// TrackingTab Data
	std::string error_message;

	std::shared_ptr<std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>>> tracking_solution =
	    std::make_shared<std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>>>();

   public:
	TrackingTab() = default;
	~TrackingTab() = default;

	void start_thread() {
		if (auto expected = TrackingTabState::NONE; state.compare_exchange_strong(expected, TrackingTabState::TRACKING)) {
			thread = std::thread([this]() {
				std::cout << "Tracking Thread started" << std::endl;

				std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>> current_tracking_solution(MagnetSelection::_magnets.size());

				while (true) {
					auto current_state = state.load();

					if (auto points = MiMedMagnetometerArraySerialConnectionBinary::push([&current_state]() { return current_state == TrackingTabState::TRACKING; }); points.has_value()) {
						if (current_state == TrackingTabState::TRACKING) {
							auto result = CeresOptimizerDirectionVector::process(points.value());

							auto& [x, y, z] = current_tracking_solution[0];

							std::move_backward(x.begin(), x.end() - 1, x.end());
							std::move_backward(y.begin(), y.end() - 1, y.end());
							std::move_backward(z.begin(), z.end() - 1, z.end());

							x.front() = result.x * 1e3;
							y.front() = result.y * 1e3;
							z.front() = result.z * 1e3;

							std::atomic_store(&tracking_solution, std::make_shared<decltype(current_tracking_solution)>(current_tracking_solution));

							continue;
						}
					}
					break;
				}

				std::cout << "Tracking Thread finished" << std::endl;
			});
		}
	}

	void stop_thread() {
		state.store(TrackingTabState::NONE);
		if (thread.joinable()) {
			thread.join();
		}
	}

	void render() {
		auto const error_ = error.load();
		auto const connected_ = SerialConnection::connected();
		auto const calibrated_ = Calibration::calibrated();
		auto const zeroed_ = Zeroing::zeroed();
		auto const magnets_selected_ = MagnetSelection::magnets_selected();

		if (ImGui::BeginChild("Tracking Child", ImVec2(-1, -1), true)) {
			if (error_) {
				stop_thread();

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

			if (!connected_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("No Connection", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("Connect to a device first!");
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("Go to Connection Tab", {500, 0})) {
					set_active_tab("Connection");
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			if (!calibrated_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("No Calibration", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("Calibrate the device first!");
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("Go to Calibration Tab", {500, 0})) {
					set_active_tab("Calibration");
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			if (!zeroed_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("No Zeroing", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("Zero the device first!");
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("Go to Zeroing Tab", {500, 0})) {
					set_active_tab("Zeroing");
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			if (!magnets_selected_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("No Magnet(s) Selected", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("Select a Magnet first!");
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("Go to Magnet Selection Tab", {500, 0})) {
					set_active_tab("Magnet Selection");
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			switch (state) {
				case TrackingTabState::NONE: {
					if (thread.joinable()) {
						thread.join();
					}
					start_thread();
					break;
				}
				case TrackingTabState::TRACKING: {
					ImPlot3D::PushStyleVar(ImPlot3DStyleVar_MarkerSize, 1.0f);
					ImPlot3D::PushStyleVar(ImPlot3DStyleVar_LabelPadding, 1.1f);

					if (std::string title = "Tracking Solution"; ImPlot3D::BeginPlot(title.c_str(), {-1, -1}, ImPlot3DFlags_NoTitle | ImPlot3DFlags_Equal)) {
						ImPlot3D::SetupAxisLimits(ImAxis3D_X, -50, 200, ImGuiCond_Always);
						ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -50, 200, ImGuiCond_Always);
						ImPlot3D::SetupAxisLimits(ImAxis3D_Z, 0, 300, ImGuiCond_Always);

						static auto ticklabel = [](float const value, char* buff, int const size, void* user_data) { return snprintf(buff, size, "%ld mm", std::lroundf(value)); };

						ImPlot3D::SetupAxisFormat(ImAxis3D_X, ticklabel, nullptr);
						ImPlot3D::SetupAxisFormat(ImAxis3D_Y, ticklabel, nullptr);
						ImPlot3D::SetupAxisFormat(ImAxis3D_Z, ticklabel, nullptr);

						// ImPlot3D::SetupAxis(ImAxis3D_X, "Magnetic Flux Density [T]");
						// ImPlot3D::SetupAxis(ImAxis3D_Y, "Magnetic Flux Density [T]");
						// ImPlot3D::SetupAxis(ImAxis3D_Z, "Magnetic Flux Density [T]");
						for (auto i = 0; auto const& [x, y, z] : *std::atomic_load(&tracking_solution)) {
							ImPlot3D::PlotScatter((std::string("Magnet ") + std::to_string(i)).c_str(), x.data(), y.data(), z.data(), x.size());
						}

						ImPlot3D::EndPlot();
					}

					break;
				}
			}

			ImGui::EndChild();
		}
	}
	virtual void set_active_tab(std::string tab_name) = 0;
};