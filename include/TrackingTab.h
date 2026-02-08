#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

// Above includes must come before
#include <ImGuizmo.h>
#include <MagArrayParser.h>
#include <MagneticFluxDensityDataRawAK09940A.h>
#include <MagneticFluxDensityDataRawLIS3MDL.h>
#include <MagneticFluxDensityDataRawMMC5983MA.h>
#include <SerialConnection.h>
#include <glad/glad.h>
#include <implot.h>
#include <implot3d.h>

#include <chrono>
#include <expected>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <numbers>

#include "Calibration.h"
#include "CeresOptimizerDirectionVector.h"
#include "MLOptimizer.h"
#include "MagnetSelection.h"
#include "Zeroing.h"

using namespace std::chrono_literals;

enum class ProjectError {
	BehindCamera,
	OutsideFrustum,
	ZeroW,
};

std::expected<ImVec2, ProjectError> ProjectPoint(const glm::vec3& p, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize);
void DrawDirectionArrow(const glm::mat4& view, const glm::vec3& camPos, const glm::mat4& proj, const glm::vec3& origin, const glm::vec3& direction, const ImVec2& rectPos, const ImVec2& rectSize);
ImU32 ShadeFace(const glm::mat4& view, const glm::vec3& normal, ImU32 baseColor);
void DrawShadedFace(std::array<glm::vec3, 4>&& corners, const glm::vec3& normal, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor, ImU32 edgeColor = IM_COL32(255, 255, 255, 180),
    float edgeThickness = 1.5f);
void DrawCylinderFaces(const glm::mat4& view, glm::vec3 const& camPos, const glm::mat4& proj, const glm::vec3& center, const glm::vec3& axis, float radius, float height, const ImVec2& rectPos, const ImVec2& rectSize);
void DrawSphere(const glm::mat4& view, const glm::vec3& camPos, const glm::mat4& proj, const glm::vec3& center, float radius, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor = IM_COL32(255, 50, 50, 40),  // very transparent
    int rings = 16, int sectors = 32);

inline glm::vec3 to_imgui(glm::vec3 const& u) { return {-u.y, u.z, -u.x}; }

#if BOARD_VERSION == 1
class TrackingTab : virtual protected SerialConnection,
                    protected MagArrayParser<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>,
                    virtual protected Calibration<41>,
                    virtual protected Zeroing<41>,
                    virtual protected MagnetSelection,
                    virtual protected CeresOptimizerDirectionVector<25>,
                    virtual protected MLOptimizer<25> {
#elif BOARD_VERSION == 2
class TrackingTab : virtual protected SerialConnection,
                    protected MagArrayParser<SENSOR_TYPE<MagneticFluxDensityDataRawAK09940A, 0, 111>>,
                    virtual protected Calibration<111>,
                    virtual protected Zeroing<111>,
                    virtual protected MagnetSelection,
                    virtual protected CeresOptimizerDirectionVector<111>,
                    virtual protected MLOptimizer<25> {
#endif
	enum class TrackingTabState {
		NONE,
		TRACKING,
	};
	std::atomic<TrackingTabState> state = TrackingTabState::NONE;

	enum class TrackingMethod {
		NONE,
		OPTIMIZATION_PROBLEM,
		RESNET18,
		ENCODER_DECODER,
	};
	std::string to_string(TrackingMethod const method) const {
		switch (method) {
			case TrackingMethod::NONE: return "NONE";
			case TrackingMethod::OPTIMIZATION_PROBLEM: return "OPTIMIZATION_PROBLEM";
			case TrackingMethod::RESNET18: return "RESNET18";
			case TrackingMethod::ENCODER_DECODER: return "ENCODER_DECODER";
		}
		return {};
	}
	std::atomic<TrackingMethod> method = TrackingMethod::NONE;

	std::thread thread;

	// TrackingTab Data
	std::shared_ptr<common::Error> latest_error = nullptr;

	std::list<std::shared_ptr<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>> magnetic_flux_density_messages;

	std::vector<std::list<std::shared_ptr<Message<Pack<Position, DirectionVector>> const>>> tracking_solutions;

   public:
	TrackingTab() = default;
	~TrackingTab() { stop_thread(); }

	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override {
		for (auto const& [calibration, zeroing, magnetometer_datapoint] : std::ranges::views::zip(Calibration::_calibrations, Zeroing::_zeroings, magnetic_flux_density_message)) {
			Eigen::Vector<double, 3> tmp;
			tmp << magnetometer_datapoint.x, magnetometer_datapoint.y, magnetometer_datapoint.z;

			tmp = calibration.transformation * (tmp - calibration.center) - zeroing;
			magnetometer_datapoint.x = tmp.x();
			magnetometer_datapoint.y = tmp.y();
			magnetometer_datapoint.z = tmp.z();
		}

		magnetic_flux_density_messages.push_front(std::make_shared<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>(magnetic_flux_density_message));
	}

	void start_thread() {
		if (auto expected = TrackingTabState::NONE; state.compare_exchange_strong(expected, TrackingTabState::TRACKING)) {
			tracking_solutions = std::vector<std::list<std::shared_ptr<Message<Pack<Position, DirectionVector>> const>>>(MagnetSelection::_magnets.size());

			thread = std::thread([this]() {
				std::cout << "Tracking Thread started" << std::endl;

				while (state.load() != TrackingTabState::NONE) {
					if (auto serial_data = read_some(); serial_data.has_value()) {
						parse(serial_data.value());
					} else {
						if (connected()) {
							std::atomic_store(&latest_error, std::make_shared<common::Error>(serial_data.error()));
							close_serial_port();
						} else {
							state.store(TrackingTabState::NONE);
						}
					}

					if (auto const current_head = magnetic_flux_density_messages.begin(); current_head != magnetic_flux_density_messages.end()) {
						auto const current_method = method.load();
#if not SHOWCASE
						if (current_method == TrackingMethod::OPTIMIZATION_PROBLEM) {
#if BOARD_VERSION == 1
							Message<Array<MagneticFluxDensityData, 25>> value;
							value.timestamp = (**current_head).timestamp;
							value.src = (**current_head).src;

							for (auto const& [s25, s41] : std::ranges::views::zip(value, **current_head) | std::ranges::views::take(25)) {
								s25 = s41;
							}

							auto const result = CeresOptimizerDirectionVector::process(value);
#elif BOARD_VERSION == 2
							auto const result = CeresOptimizerDirectionVector::process(**current_head);
#endif

							tracking_solutions[0].push_front(std::make_shared<Message<Pack<Position, DirectionVector>>>(result));
							continue;
						}
#endif
						// if (current_method == TrackingMethod::RESNET18) {
						// auto const result = MLOptimizer::process(magnetometer_data.value());
						//
						// std::atomic_store_explicit(&tracking_solutions[0], std::make_shared<TrackingSolutionNode>(tracking_solutions[0], result), std::memory_order_release);
						// continue;
						//
						//	if (current_method == TrackingMethod::ENCODER_DECODER) {
						//	auto const result = MLOptimizer::process(magnetometer_data.value());
						//
						//	std::atomic_store_explicit(&tracking_solutions[0], std::make_shared<TrackingSolutionNode>(tracking_solutions[0], result), std::memory_order_release);
						//	continue;
						//}
						//
						// if (current_method == TrackingMethod::NONE) {
						//	auto const result = Message<Pack<Position, DirectionVector>>{};
						//
						//	std::atomic_store_explicit(&tracking_solutions[0], std::make_shared<TrackingSolutionNode>(tracking_solutions[0], result), std::memory_order_release);
						//
						//	continue;
					}
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
		auto const error_ = std::atomic_load(&latest_error);
		auto const connected_ = SerialConnection::connected();
		auto const calibrated_ = Calibration::calibrated();
		auto const zeroed_ = Zeroing::zeroed();
		auto const magnets_selected_ = MagnetSelection::magnets_selected();

		if (ImGui::BeginChild("Tracking Child", ImVec2(-1, -1), true)) {
			static auto debug = true;
			{  // Display Magnet
				static constexpr glm::vec3 up(0.0f, 1.0f, 0.0f);
				static constexpr float speed = 5.f;
				static constexpr float orbit_speed = 0.01f;

				static glm::vec3 camPos(4.0f, 10.0f, 5.0f);
				static float yaw = glm::radians(-135.0f);
				static float pitch = glm::radians(-45.0f);

				ImVec2 const winPos = ImGui::GetCursorScreenPos();
				ImVec2 const winSize = ImGui::GetContentRegionAvail();
				ImGuiIO const& io = ImGui::GetIO();
				auto const dt = io.DeltaTime;

				if (ImGui::IsWindowHovered()) {
					if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
						yaw += io.MouseDelta.x * orbit_speed;
						pitch -= io.MouseDelta.y * orbit_speed;
						pitch = glm::clamp(pitch, -1.57f, 1.57f);
					}

					if (ImGui::IsKeyDown(ImGuiKey_E)) {
						yaw -= speed * dt;
					}
					if (ImGui::IsKeyDown(ImGuiKey_Q)) {
						yaw += speed * dt;
					}
				}

				glm::vec3 const forward = glm::normalize(glm::vec3{std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw)});

				if (ImGui::IsWindowHovered()) {
					glm::vec3 const right = glm::normalize(glm::cross(forward, up));

					if (io.MouseWheel != 0.0f) camPos += forward * io.MouseWheel * 0.5f;

					if (ImGui::IsKeyDown(ImGuiKey_W)) camPos += forward * speed * dt;
					if (ImGui::IsKeyDown(ImGuiKey_S)) camPos -= forward * speed * dt;
					if (ImGui::IsKeyDown(ImGuiKey_A)) camPos -= right * speed * dt;
					if (ImGui::IsKeyDown(ImGuiKey_D)) camPos += right * speed * dt;
					if (ImGui::IsKeyDown(ImGuiKey_Space)) camPos.y += speed * dt;
					if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl)) camPos.y -= speed * dt;
				}

				glm::mat4 view = glm::lookAt(camPos, camPos + forward, up);
				glm::mat4 proj = glm::perspective(glm::radians(90.0f), winSize.x / winSize.y, 0.1f, 100.0f);

				// Tell ImGuizmo where to draw
				ImGuizmo::BeginFrame();
				ImGuizmo::SetDrawlist();
				ImGuizmo::SetRect(winPos.x, winPos.y, winSize.x, winSize.y);

				glm::mat4 model(1.0f);
				ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), glm::value_ptr(model), 10.0f);

				for (auto const& [solutions, magnet] : std::ranges::views::zip(tracking_solutions, _magnets)) {
					auto current_head = solutions.begin();
					auto current_time = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());

					// Conversion to cm:
					float const radius = magnet.R * 100.f;
					float const height = magnet.H * 100.f;

					if (debug) {
						glm::vec3 position = {0.f, 0.f, 0.f};
						glm::vec3 direction = {0.f, 0.f, 0.f};
						if (current_head != solutions.end()) {
							position = glm::vec3{(**current_head).x, (**current_head).y, (**current_head).z};
							direction = glm::vec3{(**current_head).mx, (**current_head).my, (**current_head).mz};
						}

						// Conversion to cm:
						position = position * 100.f;

						if (direction == glm::vec3{0.f, 0.f, 0.f}) {
							direction = glm::vec3{0.f, 0.f, 1.f};
						} else {
							DrawDirectionArrow(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), to_imgui(direction), winPos, winSize);
						}

						DrawCylinderFaces(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), to_imgui(direction), radius, height, winPos, winSize);
					} else {
						glm::vec3 position = {0.f, 0.f, 0.f};
						glm::vec3 direction = {0.f, 0.f, 0.f};
						{
							auto n_samples = 0;
							for (auto tmp_current_head = current_head; tmp_current_head != solutions.end(); ++tmp_current_head) {
								if (auto const timestamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>{std::chrono::nanoseconds{(**tmp_current_head).timestamp}}; timestamp + 250ms > current_time) {
									position += glm::vec3{(**tmp_current_head).x, (**tmp_current_head).y, (**tmp_current_head).z};
									direction += glm::vec3{(**tmp_current_head).mx, (**tmp_current_head).my, (**tmp_current_head).mz};
									++n_samples;
								} else {
									std::cout << std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>{std::chrono::nanoseconds{(**tmp_current_head).timestamp}} << std::endl;
									std::cout << current_time << std::endl;
									std::cout << (**tmp_current_head).timestamp << std::endl;
									std::cout << timestamp - current_time << std::endl;
									std::cout << std::chrono::time_point_cast<std::chrono::nanoseconds>(current_time).time_since_epoch().count() << std::endl;
									std::cout << std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>{std::chrono::nanoseconds{(**tmp_current_head).timestamp}})
									                 .time_since_epoch()
									                 .count()
									          << std::endl;
									break;
								}
							}
							if (n_samples != 0) {
								position /= n_samples;
								direction /= n_samples;
							}
						}

						auto stddev = 0.f;
						{
							auto n_samples = 0;
							for (auto tmp_current_head = current_head; tmp_current_head != solutions.end(); ++tmp_current_head) {
								if (auto const timestamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>{std::chrono::nanoseconds{(**tmp_current_head).timestamp}}; timestamp + 250ms > current_time) {
									stddev += ((**tmp_current_head).x - position.x) * ((**tmp_current_head).x - position.x) + ((**tmp_current_head).y - position.y) * ((**tmp_current_head).y - position.y) +
									          ((**tmp_current_head).z - position.z) * ((**tmp_current_head).z - position.z);
									++n_samples;
								} else {
									solutions.erase(tmp_current_head, solutions.end());
									break;
								}
							}
							if (n_samples != 0) {
								stddev /= n_samples;
								stddev = std::sqrt(stddev);
							}

							std::cout << stddev * 1000.f << "mm" << std::endl;
						}

						// Conversion to cm:
						position = position * 100.f;

						DrawDirectionArrow(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), to_imgui(direction), winPos, winSize);
						DrawCylinderFaces(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), to_imgui(direction), radius, height, winPos, winSize);
						DrawSphere(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), stddev * 100.f, winPos, winSize);
					}
				}

				ImGui::SetCursorPosY(ImGui::GetStyle().WindowPadding.y);
				ImGui::SetCursorPosX(ImGui::GetWindowWidth() - 130.f - ImGui::GetStyle().WindowPadding.x);
				if (ImGui::BeginChild("CameraWidget", ImVec2(130.f, 130.f), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
					ImGuizmo::BeginFrame();

					ImGuizmo::SetDrawlist();
					ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, ImGui::GetWindowWidth(), ImGui::GetWindowHeight());

					ImGuizmo::ViewManipulate(glm::value_ptr(view), 10, ImVec2(ImGui::GetWindowPos().x + 130 / 2 - 128 / 2, ImGui::GetWindowPos().y + 130 / 2 - 128 / 2), ImVec2(128, 128), 0x10101010);

					if (ImGui::IsWindowHovered()) {
						glm::mat4 inv = glm::inverse(view);
						glm::vec3 const forward2 = glm::normalize(-glm::vec3(inv[2]));

						yaw = std::atan2(forward2.z, forward2.x);
						pitch = glm::clamp(std::asin(forward2.y), -1.57f, 1.57f);

						camPos = glm::vec3(inv[3]);
					}

					ImGui::EndChild();
				}
			}

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
					ImGui::SetCursorPosY(ImGui::GetStyle().WindowPadding.y);
					ImGui::SetCursorPosX(ImGui::GetStyle().WindowPadding.x);
					ImGui::SetNextItemWidth(300.f);

					if (auto const current_method = method.load(); ImGui::BeginCombo("##combo", to_string(method).c_str())) {
#if not SHOWCASE
						if (ImGui::Selectable(to_string(TrackingMethod::OPTIMIZATION_PROBLEM).c_str(), current_method == TrackingMethod::OPTIMIZATION_PROBLEM)) {
							method.store(TrackingMethod::OPTIMIZATION_PROBLEM);
						}
#endif
						if (ImGui::Selectable(to_string(TrackingMethod::RESNET18).c_str(), current_method == TrackingMethod::RESNET18)) {
#if not SHOWCASE
							MLOptimizer::set_model(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "models" / "inverse_biot_savart_net_resnet18_2000000.pt");
#else
							MLOptimizer::set_model(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "models" / "inverse_biot_savart_net_resnet18_1000000.pt");
#endif

							method.store(TrackingMethod::RESNET18);
						}

						if (ImGui::Selectable(to_string(TrackingMethod::ENCODER_DECODER).c_str(), current_method == TrackingMethod::RESNET18)) {
							MLOptimizer::set_model(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "models" / "inverse_biot_savart_net_encoder_decoder_100000.pt");
							method.store(TrackingMethod::ENCODER_DECODER);
						}
						ImGui::EndCombo();
					}

#if not SHOWCASE
					ImGui::SameLine();
					ImGui::Checkbox("Debug", &debug);
#endif
				}
			}

			ImGui::EndChild();
		}
	}
	virtual void set_active_tab(std::string tab_name) = 0;
};