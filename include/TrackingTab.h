#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

// Above includes must come before
#include <ImGuizmo.h>
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
#include "imgui_internal.h"

struct Vec3 {
	float x, y, z;

	Vec3() : x(0), y(0), z(0) {}
	Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
};

void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16);
void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16);
void Cross(const float* a, const float* b, float* r);
float Dot(const float* a, const float* b);
void Normalize(const float* a, float* r);
void MultiplyMatrix(const float* a, const float* b, float* result);
void LookAt(const float* eye, const float* at, const float* up, float* m16);
bool InvertMatrix(const float m[16], float invOut[16]);
void ExtractDirectionFromViewMatrix(const float* view, Vec3& forward, Vec3& right, Vec3& up);
inline Vec3 ExtractPositionFromViewMatrix(float const* view) {
	float inv[16];
	InvertMatrix(view, inv);
	return Vec3(inv[12], inv[13], inv[14]);
}

inline Vec3 ExtractForwardFromViewMatrix(float const* view) {
	// forward is the -Z axis of the camera
	Vec3 f(-view[2], -view[6], -view[10]);
	Normalize(&f.x, &f.x);
	return f;
}

inline inline std::tuple<float, float> ExtractAnglesFromForward(Vec3 const& forward) {
	// yAngle = rotation around Y
	float yaw = atan2(forward.x, forward.z);

	// xAngle = rotation around X: pitch
	float pitch = -asin(forward.y);

	return {pitch, yaw};
}
inline std::tuple<float, float> ExtractAnglesFromForwardSafe(Vec3 const& forward) {
	// pitch: clamp because asin(±1) is valid, but later tan() or cos() becomes unstable
	float pitch = -asin(std::clamp(forward.y, -0.9999f, 0.9999f));

	float yaw = atan2(forward.x, forward.z);

	return {pitch, yaw};
}

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

	float cameraProjection[16];
	float cameraView[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	static constexpr float identityMatrix[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
	float objectMatrix[4][16] = {{1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 2.f, 0.f, 0.f, 1.f},
	    {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 2.f, 0.f, 2.f, 1.f}, {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 2.f, 1.f}};
	float fov = 27.f;
	// float camYAngle = 165.f / 180.f * 3.14159f;
	// float camXAngle = 32.f / 180.f * 3.14159f;
	float camDistance = 8.f;
	// Vec3 camPos = Vec3(0, 2, 8);
	Vec3 camTarget = Vec3(0, 0, 0);

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
			{
				ImGuizmo::BeginFrame();

				ImGuiIO& io = ImGui::GetIO();
				bool hovered = ImGui::IsWindowHovered();
				if (hovered) {
					auto camPos = ExtractPositionFromViewMatrix(cameraView);

					Vec3 forward, right, up;
					ExtractDirectionFromViewMatrix(cameraView, forward, right, up);
					Normalize(&forward.x, &forward.x);
					Normalize(&right.x, &right.x);
					Normalize(&up.x, &up.x);

					auto [camXAngle, camYAngle] = ExtractAnglesFromForwardSafe(forward);

					// -------------------------------------
					//         MOUSE WHEEL ZOOM
					// -------------------------------------
					if (io.MouseWheel != 0.0f) {
						camPos.x += forward.x * io.MouseWheel * 0.5f;
						camPos.y += forward.y * io.MouseWheel * 0.5f;
						camPos.z += forward.z * io.MouseWheel * 0.5f;
					}

					// -------------------------------------
					//        ORBIT CAMERA (Middle)
					// -------------------------------------
					if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
						camYAngle -= io.MouseDelta.x * 0.003f;
						camXAngle += io.MouseDelta.y * 0.003f;

						float const limit = 1.55f;
						camXAngle = std::clamp(camXAngle, -limit, limit);

						forward = Vec3(sin(camYAngle) * cos(camXAngle), -sinf(camXAngle), cos(camYAngle) * cos(camXAngle));
						Normalize(&forward.x, &forward.x);

						// FIX: use WORLD UP to avoid tilt
						Vec3 worldUp(0.f, 1.f, 0.f);

						// right = forward × worldUp
						Cross(&forward.x, &worldUp.x, &right.x);
						Normalize(&right.x, &right.x);

						// up = right × forward
						Cross(&right.x, &forward.x, &up.x);
						Normalize(&up.x, &up.x);
					}

					// -------------------------------------
					//        REBUILD TARGET (orbit mode)
					// -------------------------------------

					camTarget.x = camPos.x + forward.x * camDistance;
					camTarget.y = camPos.y + forward.y * camDistance;
					camTarget.z = camPos.z + forward.z * camDistance;

					// -------------------------------------
					//        WASD + Q/E MOVEMENT
					// -------------------------------------
					float const dt = io.DeltaTime;
					float const speed = 5.0f * dt;

					if (ImGui::IsKeyDown(ImGuiKey_W)) {
						camPos.x += forward.x * speed;
						camPos.y += forward.y * speed;
						camPos.z += forward.z * speed;
					}
					if (ImGui::IsKeyDown(ImGuiKey_S)) {
						camPos.x -= forward.x * speed;
						camPos.y -= forward.y * speed;
						camPos.z -= forward.z * speed;
					}
					if (ImGui::IsKeyDown(ImGuiKey_A)) {
						camPos.x -= right.x * speed;
						camPos.y -= right.y * speed;
						camPos.z -= right.z * speed;
					}
					if (ImGui::IsKeyDown(ImGuiKey_D)) {
						camPos.x += right.x * speed;
						camPos.y += right.y * speed;
						camPos.z += right.z * speed;
					}
					if (ImGui::IsKeyDown(ImGuiKey_Space)) camPos.y += speed;
					if (ImGui::IsKeyDown(ImGuiKey_LeftShift)) camPos.y -= speed;

					camTarget.x = camPos.x + forward.x * camDistance;
					camTarget.y = camPos.y + forward.y * camDistance;
					camTarget.z = camPos.z + forward.z * camDistance;

					// -------------------------------------
					//     BUILD cameraView with LookAt
					// -------------------------------------
					float eye[3] = {camPos.x, camPos.y, camPos.z};
					float at[3] = {camTarget.x, camTarget.y, camTarget.z};
					float upz[3] = {up.x, up.y, up.z};

					LookAt(eye, at, upz, cameraView);
				}

				ImGui::Checkbox("Using ViewManipulate", &hovered);

				// -------------------------------------
				//      PROJECTION
				// -------------------------------------
				Perspective(fov, ImGui::GetWindowWidth() / ImGui::GetWindowHeight(), 0.1f, 89.9f, cameraProjection);

				// -------------------------------------
				//      GUZMO DRAW
				// -------------------------------------
				ImGuizmo::SetDrawlist();
				ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, ImGui::GetWindowWidth(), ImGui::GetWindowHeight());

				ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix, 100.f);

				// -------------------------------------
				//      VIEW MANIPULATOR
				// -------------------------------------

				ImGui::BeginChild("CameraWidget", ImVec2(150, 150), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
				ImGuizmo::BeginFrame();

				ImGuizmo::SetDrawlist();
				ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, ImGui::GetWindowWidth(), ImGui::GetWindowHeight());

				ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - 128, ImGui::GetWindowPos().y), ImVec2(128, 128), 0x10101010);

				if (!hovered) {

				}

				ImGui::EndChild();

				ImGui::EndChild();

				return;
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