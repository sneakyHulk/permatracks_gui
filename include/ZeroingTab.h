#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <iomanip>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include "Calibration.h"
#include "CalibrationTab.h"
#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"
#include "MiMedMagnetometerArraySerialConnectionBinary.h"
#include "SerialConnection.h"
#include "Zeroing.h"

class ZeroingTab : virtual protected SerialConnection,
                   protected MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>,
                   virtual protected Calibration<41>,
                   virtual protected Zeroing<41> {
	enum class ZeroingTabState {
		NONE,
		PLOTTING,
		ZEROING,
	};
	std::atomic<ZeroingTabState> state = ZeroingTabState::NONE;

	std::thread thread;

	// ZeroingTab Data
	std::shared_ptr<ERR> latest_error = nullptr;

	std::list<std::shared_ptr<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>> magnetic_flux_density_messages;

   public:
	ZeroingTab() {}
	~ZeroingTab() { stop_thread(); }

	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override {
		for (auto const& [calibration, magnetometer_datapoint] : std::ranges::views::zip(Calibration::_calibrations, magnetic_flux_density_message)) {
			Eigen::Vector<double, 3> tmp;
			tmp << magnetometer_datapoint.x, magnetometer_datapoint.y, magnetometer_datapoint.z;

			tmp = calibration.transformation * (tmp - calibration.center);
			magnetometer_datapoint.x = tmp.x();
			magnetometer_datapoint.y = tmp.y();
			magnetometer_datapoint.z = tmp.z();
		}

		magnetic_flux_density_messages.push_front(std::make_shared<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>(magnetic_flux_density_message));
	}

	void start_thread() {
		if (auto expected = ZeroingTabState::NONE; state.compare_exchange_strong(expected, ZeroingTabState::PLOTTING)) {
			thread = std::thread([this]() {
				std::cout << "Zeroing Thread started" << std::endl;

				while (state.load() != ZeroingTabState::NONE) {
					if (auto serial_data = read_some(); serial_data.has_value()) {
						parse(serial_data.value());
					} else {
						if (connected()) {
							std::atomic_store(&latest_error, std::make_shared<ERR>(serial_data.error()));
							close_serial_port();
						} else {
							state.store(ZeroingTabState::NONE);
						}
					}
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
		// if (state.exchange(ZeroingTabState::NONE) != ZeroingTabState::NONE) stop_reading();
	}

	void render() {
		auto const error_ = std::atomic_load(&latest_error);
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
				close_serial_port();

				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("ERROR", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("%s", error_->message.c_str());
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("OK", {500, 0})) {
					std::atomic_store(&latest_error, std::shared_ptr<ERR>{nullptr});
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

				int const cols = static_cast<int>(std::ceil(std::sqrt(total_mag_sensors)));
				int const rows = static_cast<int>(std::ceil(static_cast<float>(total_mag_sensors) / cols));

				if (ImGui::BeginTable("Grid", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
					for (auto row = 0, index = 0; row < rows; ++row) {
						ImGui::TableNextRow();
						for (int col = 0; col < cols; ++col) {
							ImGui::TableSetColumnIndex(col);
							if (index < total_mag_sensors) {
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

#if not SHOWCASE
				if (ImGui::Button("Save Zeroing", {-1, 0})) {
					nlohmann::json current_zeroing_json;

					for (auto i = 0; auto const& zeroing : _zeroings) {
						std::stringstream name;

						name << std::setw(2) << std::setfill('0') << i++;
						current_zeroing_json[name.str()]["zeroing"] = zeroing;
					}

					auto [ymd, hms] = get_year_month_day_hh_mm_ss();  // from previous message

					std::ostringstream oss;
					oss << "zeroing_" << ymd << "_" << std::setfill('0') << std::setw(2) << hms.hours().count() << "-" << std::setw(2) << hms.minutes().count() << "-" << std::setw(2) << hms.seconds().count() << ".json";

					std::ofstream out(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "zeroings" / oss.str());
					out << current_zeroing_json.dump();
				}
#endif

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
						magnetic_flux_density_messages.clear();

						state.store(ZeroingTabState::ZEROING);
					}

					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						auto current_head = magnetic_flux_density_messages.begin();
						for (auto i = 0; i < total_mag_sensors; ++i) {
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

									std::array<double, 10> xs{};
									std::array<double, 10> ys{};
									std::array<double, 10> zs{};
									for (auto const& [x, y, z] : std::ranges::views::zip(xs, ys, zs)) {
										if (current_head != magnetic_flux_density_messages.end()) {
											x = (**current_head)[i].x;
											y = (**current_head)[i].y;
											z = (**current_head)[i].z;

											++current_head;
										} else {
											break;
										}
									}

									magnetic_flux_density_messages.erase(current_head, magnetic_flux_density_messages.end());

									ImPlot::PlotScatter("X-Y Projection", xs.data(), ys.data(), xs.size());
									ImPlot::PlotScatter("Y-Z Projection", ys.data(), zs.data(), ys.size());
									ImPlot::PlotScatter("Z-X Projection", zs.data(), xs.data(), zs.size());
									ImPlot::EndPlot();
								}
								ImGui::EndTabItem();
							}
						}

						ImGui::EndTabBar();
					}
					break;
				}
				case ZeroingTabState::ZEROING: {
					if (ImGui::Button("Calculate Zeroing", {-1, 0})) {
						stop_thread();

						auto current_head = magnetic_flux_density_messages.begin();
						std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, total_mag_sensors> output_zeroing_data;

						for (; current_head != magnetic_flux_density_messages.end(); ++current_head) {
							for (auto const& [magnetic_flux_density_datapoint, output_zeroing_datapoint] : std::ranges::views::zip(**current_head, output_zeroing_data)) {
								auto& [xs, ys, zs] = output_zeroing_datapoint;

								xs.push_back(magnetic_flux_density_datapoint.x);
								ys.push_back(magnetic_flux_density_datapoint.y);
								zs.push_back(magnetic_flux_density_datapoint.z);
							}
						}

						zero(output_zeroing_data);
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						auto current_head = magnetic_flux_density_messages.begin();
						for (auto i = 0; i < total_mag_sensors; ++i) {
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

									std::vector<double> xs{};
									std::vector<double> ys{};
									std::vector<double> zs{};

									for (; current_head != magnetic_flux_density_messages.end(); ++current_head) {
										xs.push_back((**current_head)[i].x);
										ys.push_back((**current_head)[i].y);
										zs.push_back((**current_head)[i].z);
									}

									ImPlot::PlotScatter("X-Y Projection", xs.data(), ys.data(), xs.size());
									ImPlot::PlotScatter("Y-Z Projection", ys.data(), zs.data(), ys.size());
									ImPlot::PlotScatter("Z-X Projection", zs.data(), xs.data(), zs.size());
									ImPlot::EndPlot();
								}
								ImGui::EndTabItem();
							}
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
