#pragma once

#include <MagArrayParser.h>
#include <MagneticFluxDensityDataRawLIS3MDL.h>
#include <MagneticFluxDensityDataRawMMC5983MA.h>
#include <MagneticFluxDensityDataRawAK09940A.h>
#include <SerialConnection.h>
#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <common_error.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <queue>
#include <string>

#include "Calibration.h"
#include "EigenJsonUtils.h"
#include "ImGuiFileDialog.h"

inline std::tuple<std::chrono::year_month_day, std::chrono::hh_mm_ss<std::chrono::seconds>> get_year_month_day_hh_mm_ss(std::chrono::system_clock::time_point const& t = std::chrono::system_clock::now()) {
	auto const day = std::chrono::floor<std::chrono::days>(t);
	auto const second = std::chrono::floor<std::chrono::seconds>(t - day);
	std::chrono::year_month_day const ymd{day};
	std::chrono::hh_mm_ss const hms{second};

	return {ymd, hms};
}
#if BOARD_VERSION == 1
class CalibrationTab : virtual protected SerialConnection, protected MagArrayParser<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>, virtual protected Calibration<41> {
#elif BOARD_VERSION == 2
class CalibrationTab : virtual protected SerialConnection, protected MagArrayParser<SENSOR_TYPE<MagneticFluxDensityDataRawAK09940A, 0, 111>>, virtual protected Calibration<111> {
#endif
	enum class CalibrationTabState {
		NONE,
		PLOTTING,
		CALIBRATING,
	};
	std::atomic<CalibrationTabState> state = CalibrationTabState::NONE;

	std::thread thread;

	// CalibrationTab Data
	std::shared_ptr<common::Error> latest_error = nullptr;

	std::list<std::shared_ptr<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>> magnetic_flux_density_messages;

   public:
	CalibrationTab() {}
	~CalibrationTab() { stop_thread(); }

	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override {
		magnetic_flux_density_messages.push_front(std::make_shared<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>(magnetic_flux_density_message));
	}

	void start_thread() {
		if (auto expected = CalibrationTabState::NONE; state.compare_exchange_strong(expected, CalibrationTabState::PLOTTING)) {
			thread = std::thread([this]() {
				std::cout << "Calibration Thread started" << std::endl;

				while (state.load() != CalibrationTabState::NONE) {
					if (auto serial_data = read_some(); serial_data.has_value()) {
						parse(serial_data.value());
					} else {
						if (connected()) {
							std::atomic_store(&latest_error, std::make_shared<common::Error>(serial_data.error()));
							close_serial_port();
						}

						state.store(CalibrationTabState::NONE);
					}
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
		// if (state.exchange(CalibrationTabState::NONE) != CalibrationTabState::NONE) stop_reading();
	}

	void render() {
		auto const error_ = std::atomic_load(&latest_error);
		auto const connected_ = SerialConnection::connected();
		auto const calibrated_ = Calibration::calibrated();

		if (ImGui::BeginChild("Calibration Child", ImVec2(-1, -1), true)) {
			ImGui::TextWrapped("Select a calibration file to load calibration.");

			ImGui::Separator();

			if (ImGui::Button("Select Calibration", {-1, 0})) {
				IGFD::FileDialogConfig config;
				config.path = CMAKE_SOURCE_DIR;
				ImGuiFileDialog::Instance()->OpenDialog("Select Calibration", "Choose File", ".json", config);
				ImGui::SetNextWindowSize({800, 500}, ImGuiCond_Always);
			}
			// display
			ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
			if (ImGuiFileDialog::Instance()->Display("Select Calibration")) {
				if (ImGuiFileDialog::Instance()->IsOk()) {  // action if OK
					std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
					std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
					// action

					std::ifstream stream(std::filesystem::path(filePath) / filePathName);
					nlohmann::json json = nlohmann::json::parse(stream);
					std::vector transformations =
					    json | std::ranges::views::transform([](nlohmann::basic_json<> const& value) { return value["transformation"].template get<Eigen::Matrix<double, 4, 4>>(); }) | std::ranges::to<std::vector>();

					for (auto const& [transformation, calibration] : std::ranges::views::zip(transformations, _calibrations)) {
						calibration.transformation = transformation.block<3, 3>(0, 0);
						calibration.center = -transformation.block<3, 1>(0, 3);
					}
					_calibrated = true;
					state.store(CalibrationTabState::CALIBRATING);
				}

				// close
				ImGuiFileDialog::Instance()->Close();
			}
#if not SHOWCASE
			ImGui::TextWrapped(
			    "Click Start Calibration, then slowly rotate and tilt the device in all possible orientations to ensure the sensor experiences a full range of magnetic field directions. When the plotted data points form a complete sphere, "
			    "click Calculate to derive the calibration coefficients from the collected samples.");
#endif

			ImGui::Separator();

			if (error_) {
				stop_thread();

				ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, {0.5f, 0.5f});
				ImGui::Begin("ERROR", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

				ImGui::TextWrapped("%s", error_->message.c_str());
				ImGui::Spacing();
				ImGui::Separator();

				if (ImGui::Button("OK", {500, 0})) {
					std::atomic_store(&latest_error, std::shared_ptr<common::Error>{nullptr});
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

#if not SHOWCASE
				if (ImGui::Button("Save Cailbration", {-1, 0})) {
					nlohmann::json current_calibration_json;

					for (auto i = 0; auto const& calibration : _calibrations) {
						Eigen::Matrix<double, 4, 4> out = Eigen::Matrix<double, 4, 4>::Identity();
						out.block<3, 3>(0, 0) = calibration.transformation;
						out.block<3, 1>(0, 3) = -calibration.center;

						std::stringstream name;

						name << std::setw(2) << std::setfill('0') << i++;
						current_calibration_json[name.str()]["transformation"] = out;
					}

					auto [ymd, hms] = get_year_month_day_hh_mm_ss();  // from previous message

					std::ostringstream oss;
					oss << "calibration_" << ymd << "_" << std::setfill('0') << std::setw(2) << hms.hours().count() << "-" << std::setw(2) << hms.minutes().count() << "-" << std::setw(2) << hms.seconds().count() << ".json";

					std::ofstream out(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "calibrations" / oss.str());
					out << current_calibration_json.dump();
				}
#endif

				ImGui::End();
				ImGui::EndChild();
				return;
			}

#if not SHOWCASE
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
						magnetic_flux_density_messages.clear();

						state.store(CalibrationTabState::CALIBRATING);
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						auto current_head = magnetic_flux_density_messages.begin();
						for (auto i = 0; i < total_mag_sensors; ++i) {
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

									ImPlot3D::PlotScatter("MagData", xs.data(), ys.data(), zs.data(), xs.size());
									ImPlot3D::EndPlot();
								}
								ImGui::EndTabItem();
							}
						}

						ImGui::EndTabBar();
					}
					break;
				}
				case CalibrationTabState::CALIBRATING: {
					if (ImGui::Button("Calculate Hard- and Soft-Iron Calibration", {-1, 0})) {
						stop_thread();

						auto current_head = magnetic_flux_density_messages.begin();
						std::array<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, total_mag_sensors> output_calibration_data;

						for (; current_head != magnetic_flux_density_messages.end(); ++current_head) {
							for (auto const& [magnetic_flux_density_datapoint, output_calibration_datapoint] : std::ranges::views::zip(**current_head, output_calibration_data)) {
								auto& [xs, ys, zs] = output_calibration_datapoint;

								xs.push_back(magnetic_flux_density_datapoint.x);
								ys.push_back(magnetic_flux_density_datapoint.y);
								zs.push_back(magnetic_flux_density_datapoint.z);
							}
						}

						calibrate(output_calibration_data);
					}
					if (ImGui::BeginTabBar("Sensor Tabbar")) {
						auto current_head = magnetic_flux_density_messages.begin();
						for (auto i = 0; i < total_mag_sensors; ++i) {
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

									std::vector<double> xs{};
									std::vector<double> ys{};
									std::vector<double> zs{};

									for (; current_head != magnetic_flux_density_messages.end(); ++current_head) {
										xs.push_back((**current_head)[i].x);
										ys.push_back((**current_head)[i].y);
										zs.push_back((**current_head)[i].z);
									}

									ImPlot3D::PlotScatter("MagData", xs.data(), ys.data(), zs.data(), xs.size());
									ImPlot3D::EndPlot();
								}
								ImGui::EndTabItem();
							}
						}

						ImGui::EndTabBar();
					}
					break;
				}
			}
#endif
			ImGui::EndChild();
		}
	}

	virtual void set_active_tab(std::string tab_name) = 0;
};