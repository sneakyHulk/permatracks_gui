#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

// Above includes must come before
#include <ImGuizmo.h>
#include <glad/glad.h>
#include <implot.h>
#include <implot3d.h>

#include <expected>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <numbers>

#include "Calibration.h"
#include "CeresOptimizerDirectionVector.h"
#include "MLOptimizer.h"
#include "MagnetSelection.h"
#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"
#include "MiMedMagnetometerArraySerialConnectionBinary.h"
#include "SerialConnection.h"
#include "Zeroing.h"

enum class ProjectError {
	BehindCamera,
	OutsideFrustum,
	ZeroW,
};

std::expected<ImVec2, ProjectError> ProjectPoint(const glm::vec3& p, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize);
void DrawDirectionArrow(const glm::mat4& view, const glm::vec3& camPos, const glm::mat4& proj, const glm::vec3& origin, const glm::vec3& direction, const ImVec2& rectPos, const ImVec2& rectSize);
ImU32 ShadeFace(const glm::mat4& view, const glm::vec3& normal, ImU32 baseColor);
void DrawShadedFace(std::array<glm::vec3, 4>&& corners, const glm::vec3& normal, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor, ImU32 edgeColor = IM_COL32(255, 255, 255, 180),
    float const edgeThickness = 1.5f);
void DrawCylinderFaces(const glm::mat4& view, glm::vec3 const& camPos, const glm::mat4& proj, const glm::vec3& center, const glm::vec3& axis, float radius, float height, const ImVec2& rectPos, const ImVec2& rectSize);
inline glm::vec3 to_imgui(glm::vec3 const& u) { return {-u.y, u.z, -u.x}; }

class TrackingTab : virtual protected SerialConnection,
                    virtual protected MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>>,
                    virtual protected Calibration<41>,
                    virtual protected Zeroing<41>,
                    virtual protected MagnetSelection,
                    virtual protected CeresOptimizerDirectionVector<41>,
                    virtual protected MLOptimizer<41> {
	std::atomic_bool error = false;

	enum class TrackingTabState {
		NONE,
		TRACKING,
	};
	std::atomic<TrackingTabState> state = TrackingTabState::NONE;

	enum class TrackingMethod {
		OPTIMIZATION_PROBLEM,
		RESNET18,
	};
	std::string to_string(TrackingMethod const method) {
		switch (method) {
			case TrackingMethod::OPTIMIZATION_PROBLEM: return "OPTIMIZATION_PROBLEM";
			case TrackingMethod::RESNET18: return "RESNET18";
		}
	}
	std::atomic<TrackingMethod> method = TrackingMethod::OPTIMIZATION_PROBLEM;

	std::thread thread;

	// TrackingTab Data
	std::string error_message;

	std::shared_ptr<std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>>> tracking_solution =
	    std::make_shared<std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>>>();

   public:
	TrackingTab() = default;
	~TrackingTab() { stop_thread(); }

	void start_thread() {
		if (auto expected = TrackingTabState::NONE; state.compare_exchange_strong(expected, TrackingTabState::TRACKING)) {
			thread = std::thread([this]() {
				std::cout << "Tracking Thread started" << std::endl;

				std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>> current_tracking_solution(
				    MagnetSelection::_magnets.size());

				while (true) {
					auto current_state = state.load();

					if (auto magnetometer_data = MiMedMagnetometerArraySerialConnectionBinary::push([&current_state]() { return current_state == TrackingTabState::TRACKING; }); magnetometer_data.has_value()) {
						for (auto const& [calibration, zeroing, magnetometer_datapoint] : std::ranges::views::zip(Calibration::_calibrations, Zeroing::_zeroings, magnetometer_data.value())) {
							Eigen::Vector<double, 3> tmp;
							tmp << magnetometer_datapoint.x, magnetometer_datapoint.y, magnetometer_datapoint.z;

							tmp = calibration.transformation * (tmp - calibration.center) - zeroing;
							magnetometer_datapoint.x = tmp.x();
							magnetometer_datapoint.y = tmp.y();
							magnetometer_datapoint.z = tmp.z();
						}

						if (current_state == TrackingTabState::TRACKING) {
							auto current_method = method.load();

							if (current_method == TrackingMethod::OPTIMIZATION_PROBLEM) {
								auto const result = CeresOptimizerDirectionVector::process(magnetometer_data.value());

								auto& [x, y, z, mx, my, mz] = current_tracking_solution[0];

								std::move_backward(x.begin(), x.end() - 1, x.end());
								std::move_backward(y.begin(), y.end() - 1, y.end());
								std::move_backward(z.begin(), z.end() - 1, z.end());

								std::move_backward(mx.begin(), mx.end() - 1, mx.end());
								std::move_backward(my.begin(), my.end() - 1, my.end());
								std::move_backward(mz.begin(), mz.end() - 1, mz.end());

								x.front() = result.x;
								y.front() = result.y;
								z.front() = result.z;

								mx.front() = result.mx;
								my.front() = result.my;
								mz.front() = result.mz;

								std::atomic_store(&tracking_solution, std::make_shared<decltype(current_tracking_solution)>(current_tracking_solution));

								continue;
							}
							if (current_method == TrackingMethod::RESNET18) {
								auto const result = MLOptimizer::process(magnetometer_data.value());

								auto& [x, y, z, mx, my, mz] = current_tracking_solution[0];

								std::move_backward(x.begin(), x.end() - 1, x.end());
								std::move_backward(y.begin(), y.end() - 1, y.end());
								std::move_backward(z.begin(), z.end() - 1, z.end());

								std::move_backward(mx.begin(), mx.end() - 1, mx.end());
								std::move_backward(my.begin(), my.end() - 1, my.end());
								std::move_backward(mz.begin(), mz.end() - 1, mz.end());

								x.front() = result.x;
								y.front() = result.y;
								z.front() = result.z;

								mx.front() = result.mx;
								my.front() = result.my;
								mz.front() = result.mz;

								std::atomic_store(&tracking_solution, std::make_shared<decltype(current_tracking_solution)>(current_tracking_solution));

								continue;
							}
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
			{  // Display Magnet
				static constexpr glm::vec3 up(0.0f, 1.0f, 0.0f);
				static constexpr float speed = 5.f;
				static constexpr float orbit_speed = 0.01f;

				static glm::vec3 camPos(4.0f, 10.0f, 5.0f);
				static float yaw = glm::radians(-135.0f);
				static float pitch = glm::radians(-45.0f);
				static glm::vec3 position = {0.0f, 0.0f, 0.0f};
				static glm::vec3 direction = {0.0f, 0.0f, 1.0f};

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

				for (auto const& [solutions, magnet] : std::ranges::views::zip(*std::atomic_load(&tracking_solution), _magnets)) {
					auto const& [x, y, z, mx, my, mz] = solutions;

					position = glm::vec3{x.front(), y.front(), z.front()};
					direction = glm::vec3{mx.front(), my.front(), mz.front()};

					// Conversion to cm:
					position = position * 100.f;
					float const radius = magnet.R * 100.f;
					float const height = magnet.H * 100.f;

					DrawDirectionArrow(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), to_imgui(direction), winPos, winSize);
					DrawCylinderFaces(view, camPos, proj, to_imgui(position - glm::vec3{17.f / 2.f, 20.f / 2.f, 0.f}), to_imgui(direction), radius, height, winPos, winSize);
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
					ImGui::SetCursorPosY(ImGui::GetStyle().WindowPadding.y);
					ImGui::SetCursorPosX(ImGui::GetStyle().WindowPadding.x);
					ImGui::SetNextItemWidth(300.f);

					if (auto const current_method = method.load(); ImGui::BeginCombo("##combo", to_string(method).c_str())) {
						if (ImGui::Selectable(to_string(TrackingMethod::OPTIMIZATION_PROBLEM).c_str(), current_method == TrackingMethod::OPTIMIZATION_PROBLEM)) {
							method.store(TrackingMethod::OPTIMIZATION_PROBLEM);
						}
						if (ImGui::Selectable(to_string(TrackingMethod::RESNET18).c_str(), current_method == TrackingMethod::RESNET18)) {
							MLOptimizer::set_model(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "models" / "inverse_biot_savart_net_resnet18_2000000.pt");
							method.store(TrackingMethod::RESNET18);
						}
						ImGui::EndCombo();
					}
				}
			}

			ImGui::EndChild();
		}
	}
	virtual void set_active_tab(std::string tab_name) = 0;
};