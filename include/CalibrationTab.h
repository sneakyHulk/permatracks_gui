#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <queue>
#include <string>

#include "Calibration.h"
#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"
#include "MiMedMagnetometerArraySerialConnectionBinary.h"
#include "SerialConnection.h"

class CalibrationTab : virtual protected SerialConnection,
                       virtual protected MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>,
                       virtual protected Calibration<41> {
	std::atomic_bool error = false;

	enum class CalibrationTabState {
		NONE,
		PLOTTING,
		CALIBRATING,
	};
	std::atomic<CalibrationTabState> state = CalibrationTabState::NONE;

	std::thread thread;

	// CalibrationTab Data
	std::string error_message;

	std::shared_ptr<std::array<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>, OutputSize>> plot_data =
	    std::make_shared<std::array<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>, OutputSize>>();
	std::shared_ptr<std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, OutputSize>> calibration_data =
	    std::make_shared<std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, OutputSize>>();

   public:
	CalibrationTab() {}
	~CalibrationTab() { stop_thread(); }

	void start_thread() {
		if (auto expected = CalibrationTabState::NONE; state.compare_exchange_strong(expected, CalibrationTabState::PLOTTING)) {
			thread = std::thread([this]() {
				std::cout << "Calibration Thread started" << std::endl;

				std::array<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>, OutputSize> current_plotting_data{};
				std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, OutputSize> current_calibration_data{};

				while (true) {
					auto current_state = state.load();

					if (auto points = push([&current_state]() { return current_state == CalibrationTabState::CALIBRATING or current_state == CalibrationTabState::PLOTTING; }); points.has_value()) {
						if (current_state == CalibrationTabState::PLOTTING) {
							for (auto const& [point, xyz] : std::ranges::views::zip(points.value(), current_plotting_data)) {
								auto& [x, y, z] = xyz;

								std::move_backward(x.begin(), x.end() - 1, x.end());
								std::move_backward(y.begin(), y.end() - 1, y.end());
								std::move_backward(z.begin(), z.end() - 1, z.end());

								x.front() = point.x;
								y.front() = point.y;
								z.front() = point.z;
							}

							std::atomic_store(&plot_data, std::make_shared<decltype(current_plotting_data)>(current_plotting_data));
							continue;
						}
						if (current_state == CalibrationTabState::CALIBRATING) {
							for (auto const& [point, xyz] : std::ranges::views::zip(points.value(), current_calibration_data)) {
								auto& [x, y, z] = xyz;

								x.push_back(point.x);
								y.push_back(point.y);
								z.push_back(point.z);
							}

							std::atomic_store(&calibration_data, std::make_shared<decltype(current_calibration_data)>(current_calibration_data));

							continue;
						}
					} else {
						if (SerialConnection::connected()) {
							error_message = points.error().what();
							SerialConnection::close_serial_port();

							error.store(true);
						} else {
							state.store(CalibrationTabState::NONE);
						}
					}

					break;
				}

				std::cout << "Calibration Thread finished" << std::endl;
			});
		}
	}

	void stop_thread() {
		state.store(CalibrationTabState::NONE);
		if (thread.joinable()) {
			thread.join();
		}
	}

	void render() {
		auto const error_ = error.load();
		auto const connected_ = SerialConnection::connected();
		auto const calibrated_ = Calibration::calibrated();

		if (ImGui::BeginChild("Calibration Child", ImVec2(-1, -1), true)) {
			ImGui::TextWrapped(
			    "Click Start Calibration, then slowly rotate and tilt the device in all possible orientations to ensure the sensor experiences a full range of magnetic field directions. When the plotted data points form a complete sphere, "
			    "click Calculate to derive the calibration coefficients from the collected samples.");

			ImGui::Separator();

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

			if (calibrated_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("Calibrated", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("The device is calibrated!");
				ImGui::Spacing();
				ImGui::Separator();

				constexpr int total = 41;
				int const cols = static_cast<int>(std::ceil(std::sqrt(total)));
				int const rows = static_cast<int>(std::ceil(static_cast<float>(total) / cols));

				if (ImGui::BeginTabBar("Transformations and Centers Tabbar", ImGuiTabBarFlags_None)) {
					if (ImGui::BeginTabItem("Transformations")) {
						if (ImGui::BeginTable("Grid", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
							for (auto row = 0, index = 0; row < rows; ++row) {
								ImGui::TableNextRow();
								for (int col = 0; col < cols; ++col) {
									ImGui::TableSetColumnIndex(col);
									if (index < total) {
										auto& [transformation, center] = _calibrations[index++];  // assume p is callable or has operator()(i)
										ImGui::BeginGroup();
										for (int i = 0; i < 3; ++i) {
											ImGui::Text("%.2f  %.2f  %.2f", transformation(i, 0), transformation(i, 1), transformation(i, 2));
										}
										ImGui::EndGroup();
									} else {
										ImGui::TextDisabled("-");
									}
								}
							}
							ImGui::EndTable();
						}
						ImGui::EndTabItem();
					}

					if (ImGui::BeginTabItem("Centers")) {
						if (ImGui::BeginTable("Grid", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
							for (auto row = 0, index = 0; row < rows; ++row) {
								ImGui::TableNextRow();
								for (int col = 0; col < cols; ++col) {
									ImGui::TableSetColumnIndex(col);
									if (index < total) {
										auto& [transformation, center] = _calibrations[index++];  // assume p is callable or has operator()(i)
										ImGui::Text("x: %.2e", center(0));
										ImGui::Text("y: %.2e", center(1));
										ImGui::Text("z: %.2e", center(2));
									} else {
										ImGui::TextDisabled("-");
									}
								}
							}
							ImGui::EndTable();
						}
						ImGui::EndTabItem();
					}

					ImGui::EndTabBar();
				}

				if (ImGui::Button("Reset Calibration", {-1, 0})) {
					reset_calibration();
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			switch (state.load()) {
				case CalibrationTabState::NONE: {
					if (thread.joinable()) {
						thread.join();
					}
					start_thread();

					break;
				}
				case CalibrationTabState::PLOTTING: {
					if (ImGui::Button("Start Hard- and Soft-Iron Calibration", {-1, 0})) {
						state.store(CalibrationTabState::CALIBRATING);
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						// if (ImPlot::BeginSubplots("41 scatter plots", 41, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_LinkAllY)) {
						// ImPlot::EndSubplots();
						// }
						for (auto i = 0; auto const& [x, y, z] : *std::atomic_load(&plot_data)) {
							if (std::string sensor_tab = std::string("S") + std::to_string(i); ImGui::BeginTabItem(sensor_tab.c_str())) {
								ImPlot3D::PushStyleVar(ImPlot3DStyleVar_MarkerSize, 1.0f);
								// ImPlot3D::PushStyleVar(ImPlot3DStyleVar_PlotPadding, {-50.0f, -50.0f});
								// ImPlot3D::PushStyleVar(ImPlot3DStyleVar_LegendPadding, {100.0f, 100.0f});
								ImPlot3D::PushStyleVar(ImPlot3DStyleVar_LabelPadding, 1.1f);

								if (std::string sensor = std::string("Sensor ") + std::to_string(i); ImPlot3D::BeginPlot(sensor.c_str(), {-1, -1}, ImPlot3DFlags_NoTitle | ImPlot3DFlags_Equal)) {
									ImPlot3D::SetupAxisLimits(ImAxis3D_X, -100e-6, 100e-6, ImGuiCond_Always);
									ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -100e-6, 100e-6, ImGuiCond_Always);
									ImPlot3D::SetupAxisLimits(ImAxis3D_Z, -100e-6, 100e-6, ImGuiCond_Always);

									static auto ticklabel = [](double const value, char* buff, int const size, void* user_data) { return snprintf(buff, size, "%.2e", value); };

									ImPlot3D::SetupAxisFormat(ImAxis3D_X, ticklabel, nullptr);
									ImPlot3D::SetupAxisFormat(ImAxis3D_Y, ticklabel, nullptr);
									ImPlot3D::SetupAxisFormat(ImAxis3D_Z, ticklabel, nullptr);

									// ImPlot3D::SetupAxis(ImAxis3D_X, "Magnetic Flux Density [T]");
									// ImPlot3D::SetupAxis(ImAxis3D_Y, "Magnetic Flux Density [T]");
									// ImPlot3D::SetupAxis(ImAxis3D_Z, "Magnetic Flux Density [T]");

									ImPlot3D::PlotScatter("MagData", x.data(), y.data(), z.data(), x.size());
									ImPlot3D::EndPlot();
								}
								ImGui::EndTabItem();
							}
							++i;
						}

						ImGui::EndTabBar();
					}
					break;
				}
				case CalibrationTabState::CALIBRATING: {
					if (ImGui::Button("Calculate Hard- and Soft-Iron Calibration", {-1, 0})) {
						stop_thread();

						calibrate(*std::atomic_load(&calibration_data));
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						// if (ImPlot::BeginSubplots("41 scatter plots", 41, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_LinkAllY)) {
						// ImPlot::EndSubplots();
						// }
						for (auto i = 0; auto const& [x, y, z] : *std::atomic_load(&calibration_data)) {
							if (std::string sensor_tab = std::string("S") + std::to_string(i); ImGui::BeginTabItem(sensor_tab.c_str())) {
								ImPlot3D::PushStyleVar(ImPlot3DStyleVar_MarkerSize, 1.0f);
								// ImPlot3D::PushStyleVar(ImPlot3DStyleVar_PlotPadding, {-50.0f, -50.0f});
								// ImPlot3D::PushStyleVar(ImPlot3DStyleVar_LegendPadding, {100.0f, 100.0f});
								ImPlot3D::PushStyleVar(ImPlot3DStyleVar_LabelPadding, 1.1f);

								if (std::string sensor = std::string("Sensor ") + std::to_string(i); ImPlot3D::BeginPlot(sensor.c_str(), {-1, -1}, ImPlot3DFlags_NoTitle | ImPlot3DFlags_Equal)) {
									ImPlot3D::SetupAxisLimits(ImAxis3D_X, -100e-6, 100e-6, ImGuiCond_Always);
									ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -100e-6, 100e-6, ImGuiCond_Always);
									ImPlot3D::SetupAxisLimits(ImAxis3D_Z, -100e-6, 100e-6, ImGuiCond_Always);

									static auto ticklabel = [](double const value, char* buff, int const size, void* user_data) { return snprintf(buff, size, "%.2e", value); };

									ImPlot3D::SetupAxisFormat(ImAxis3D_X, ticklabel, nullptr);
									ImPlot3D::SetupAxisFormat(ImAxis3D_Y, ticklabel, nullptr);
									ImPlot3D::SetupAxisFormat(ImAxis3D_Z, ticklabel, nullptr);

									// ImPlot3D::SetupAxis(ImAxis3D_X, "Magnetic Flux Density [T]");
									// ImPlot3D::SetupAxis(ImAxis3D_Y, "Magnetic Flux Density [T]");
									// ImPlot3D::SetupAxis(ImAxis3D_Z, "Magnetic Flux Density [T]");

									ImPlot3D::PlotScatter("MagData", x.data(), y.data(), z.data(), x.size());
									ImPlot3D::EndPlot();
								}
								ImGui::EndTabItem();
							}
							++i;
						}

						ImGui::EndTabBar();
					}
					break;
				}
			}

			ImGui::EndChild();
		}
	}

	virtual void set_active_tab(std::string tab_name) = 0;
};