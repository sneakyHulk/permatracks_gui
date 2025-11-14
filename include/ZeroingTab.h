#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include "Calibration.h"
#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"
#include "MiMedMagnetometerArraySerialConnectionBinary.h"
#include "SerialConnection.h"
#include "Zeroing.h"

class ZeroingTab : virtual protected SerialConnection,
                   virtual protected MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>,
                   virtual protected Calibration<41>,
                   virtual protected Zeroing<41> {
	std::atomic_bool error = false;

	enum class ZeroingTabState {
		NONE,
		PLOTTING,
		ZEROING,
	};
	std::atomic<ZeroingTabState> state = ZeroingTabState::NONE;

	std::thread thread;

	// ZeroingTab Data
	std::string error_message;

	std::shared_ptr<std::array<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>, OutputSize>> plot_data =
	    std::make_shared<std::array<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>, OutputSize>>();
	std::shared_ptr<std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, OutputSize>> zeroing_data =
	    std::make_shared<std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, OutputSize>>();

   public:
	ZeroingTab() {}
	~ZeroingTab() { stop_thread(); }

	void start_thread() {
		if (auto expected = ZeroingTabState::NONE; state.compare_exchange_strong(expected, ZeroingTabState::PLOTTING)) {
			thread = std::thread([this]() {
				std::cout << "Zeroing Thread started" << std::endl;

				std::array<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>, OutputSize> current_plotting_data{};
				std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, OutputSize> current_zeroing_data{};

				while (true) {
					auto current_state = state.load();

					if (auto points = push([&current_state]() { return current_state == ZeroingTabState::PLOTTING or current_state == ZeroingTabState::ZEROING; }); points.has_value()) {
						if (current_state == ZeroingTabState::PLOTTING) {
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
						if (current_state == ZeroingTabState::ZEROING) {
							for (auto const& [point, xyz] : std::ranges::views::zip(points.value(), current_zeroing_data)) {
								auto& [x, y, z] = xyz;

								x.push_back(point.x);
								y.push_back(point.y);
								z.push_back(point.z);
							}

							std::atomic_store(&zeroing_data, std::make_shared<decltype(current_zeroing_data)>(current_zeroing_data));

							continue;
						}
					} else {
						if (SerialConnection::connected()) {
							error_message = points.error().what();
							SerialConnection::close_serial_port();

							error.store(true);
						} else {
							state.store(ZeroingTabState::NONE);
						}
					}

					break;
				}

				std::cout << "Zeroing Thread finished" << std::endl;
			});
		}
	}

	void stop_thread() {
		state.store(ZeroingTabState::NONE);
		if (thread.joinable()) {
			thread.join();
		}
	}

	void render() {
		auto const error_ = error.load();
		auto const connected_ = SerialConnection::connected();
		auto const calibrated_ = Calibration::calibrated();
		auto const zeroed_ = Zeroing::zeroed();

		if (ImGui::BeginChild("Zeroing Child", ImVec2(-1, -1), true)) {
			ImGui::TextWrapped(
			    "Position the device in the intended operating environment and initiate the zeroing procedure by clicking Start Zeroing. Wait a few moments for the readings to stabilize, then click Calculate to determine the zero-offset "
			    "values.");

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

			if (zeroed_) {
				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("Zeroed", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("The device is zeroed!");
				ImGui::Spacing();
				ImGui::Separator();

				constexpr int total = 41;
				int const cols = static_cast<int>(std::ceil(std::sqrt(total)));
				int const rows = static_cast<int>(std::ceil(static_cast<float>(total) / cols));

				if (ImGui::BeginTable("Grid", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
					for (auto row = 0, index = 0; row < rows; ++row) {
						ImGui::TableNextRow();
						for (int col = 0; col < cols; ++col) {
							ImGui::TableSetColumnIndex(col);
							if (index < total) {
								auto& p = _zeroings[index++];  // assume p is callable or has operator()(i)
								ImGui::Text("x: %.2e", p(0));
								ImGui::Text("y: %.2e", p(1));
								ImGui::Text("z: %.2e", p(2));
							} else {
								ImGui::TextDisabled("-");
							}
						}
					}
					ImGui::EndTable();
				}

				if (ImGui::Button("Reset Zeroing", {-1, 0})) {
					reset_zeroing();
				}

				ImGui::End();
				ImGui::EndChild();
				return;
			}

			switch (state.load()) {
				case ZeroingTabState::NONE: {
					if (thread.joinable()) {
						thread.join();
					}
					start_thread();
					break;
				}
				case ZeroingTabState::PLOTTING: {
					if (ImGui::Button("Start Zeroing", {-1, 0})) {
						state.store(ZeroingTabState::ZEROING);
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						for (auto i = 0; auto const& [x, y, z] : *std::atomic_load(&plot_data)) {
							if (std::string sensor_tab = std::string("S") + std::to_string(i); ImGui::BeginTabItem(sensor_tab.c_str())) {
								ImVec2 avail = ImGui::GetContentRegionAvail();

								ImPlot::SetNextAxisLimits(ImAxis_X1, -100e-6, 100e-6, ImGuiCond_Always);
								ImPlot::SetNextAxisLimits(ImAxis_Y1, -100e-6, 100e-6, ImGuiCond_Always);
								if (std::string sensor = std::string("Sensor ") + std::to_string(i); ImPlot::BeginPlot(sensor.c_str(), {-1, -1})) {
									ImPlotStyle const& style = ImPlot::GetStyle();
									auto const title_height = style.LabelPadding.y + ImGui::GetFontSize();
									auto const x_ticks_height = style.LabelPadding.y + ImGui::GetFontSize() + style.LabelPadding.y + ImGui::GetFontSize();
									auto const y_ticks_width = style.LabelPadding.x + ImGui::GetFontSize() + style.LabelPadding.x + ImGui::CalcTextSize("-1.40e-05").x;

									auto const side = std::min(avail.x, avail.y);

									static auto ticklabel = [](double const value, char* buff, int const size, void* user_data) { return snprintf(buff, size, "%.2e", value); };

									ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding,
									    ImVec2((avail.x - side + std::max(x_ticks_height + title_height - y_ticks_width, 0.f)) * 0.5f, (avail.y - side + std::max(y_ticks_width - x_ticks_height - title_height, 0.f)) * 0.5f));

									ImPlot::SetupAxisFormat(ImAxis_X1, ticklabel, nullptr);
									ImPlot::SetupAxisFormat(ImAxis_Y1, ticklabel, nullptr);

									ImPlot::SetupAxis(ImAxis_X1, "Magnetic Flux Density [T]");
									ImPlot::SetupAxis(ImAxis_Y1, "Magnetic Flux Density [T]");

									ImPlot::PlotScatter("X-Y Projection", x.data(), y.data(), x.size());
									ImPlot::PlotScatter("Y-Z Projection", y.data(), z.data(), y.size());
									ImPlot::PlotScatter("Z-X Projection", z.data(), x.data(), z.size());
									ImPlot::EndPlot();
								}
								ImGui::EndTabItem();
							}
							++i;
						}

						ImGui::EndTabBar();
					}
					break;
				}
				case ZeroingTabState::ZEROING: {
					if (ImGui::Button("Calculate Zeroing", {-1, 0})) {
						stop_thread();

						zero(*std::atomic_load(&zeroing_data));
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						for (auto i = 0; auto const& [x, y, z] : *std::atomic_load(&zeroing_data)) {
							if (std::string sensor_tab = std::string("S") + std::to_string(i); ImGui::BeginTabItem(sensor_tab.c_str())) {
								ImVec2 avail = ImGui::GetContentRegionAvail();

								ImPlot::SetNextAxisLimits(ImAxis_X1, -100e-6, 100e-6, ImGuiCond_Always);
								ImPlot::SetNextAxisLimits(ImAxis_Y1, -100e-6, 100e-6, ImGuiCond_Always);
								if (std::string sensor = std::string("Sensor ") + std::to_string(i); ImPlot::BeginPlot(sensor.c_str(), {-1, -1})) {
									ImPlotStyle const& style = ImPlot::GetStyle();
									auto const title_height = style.LabelPadding.y + ImGui::GetFontSize();
									auto const x_ticks_height = style.LabelPadding.y + ImGui::GetFontSize() + style.LabelPadding.y + ImGui::GetFontSize();
									auto const y_ticks_width = style.LabelPadding.x + ImGui::GetFontSize() + style.LabelPadding.x + ImGui::CalcTextSize("-1.40e-05").x;

									auto const side = std::min(avail.x, avail.y);

									static auto ticklabel = [](double const value, char* buff, int const size, void* user_data) { return snprintf(buff, size, "%.2e", value); };

									ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding,
									    ImVec2((avail.x - side + std::max(x_ticks_height + title_height - y_ticks_width, 0.f)) * 0.5f, (avail.y - side + std::max(y_ticks_width - x_ticks_height - title_height, 0.f)) * 0.5f));

									ImPlot::SetupAxisFormat(ImAxis_X1, ticklabel, nullptr);
									ImPlot::SetupAxisFormat(ImAxis_Y1, ticklabel, nullptr);

									ImPlot::SetupAxis(ImAxis_X1, "Magnetic Flux Density [T]");
									ImPlot::SetupAxis(ImAxis_Y1, "Magnetic Flux Density [T]");

									ImPlot::PlotScatter("X-Y Projection", x.data(), y.data(), x.size());
									ImPlot::PlotScatter("Y-Z Projection", y.data(), z.data(), y.size());
									ImPlot::PlotScatter("Z-X Projection", z.data(), x.data(), z.size());
									ImPlot::EndPlot();
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
