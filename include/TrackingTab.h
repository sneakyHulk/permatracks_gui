#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

// Above includes must come before
#include <ImGuizmo.h>
#include <glad/glad.h>
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

struct Vec3 {
	float x, y, z;

	Vec3() : x(0), y(0), z(0) {}
	Vec3(float const X, float const Y, float const Z) : x(X), y(Y), z(Z) {}
};

struct CylinderFace {
	float z;
	std::vector<ImVec2> pts;
	ImU32 color;
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
Vec3 ExtractPositionFromViewMatrix(float const* view);
Vec3 ExtractForwardFromViewMatrix(float const* view);
std::tuple<float, float> ExtractAnglesFromForwardSafe(Vec3 const& forward);
void TransformPoint(const float* m, const float in[3], float out[4]);
// Apply: proj * view * model * localPos -> screen pixel coords
ImVec2 ProjectToScreen(const float* view, const float* proj, const float* model, const float local[3], const ImVec2& viewportMin, const ImVec2& viewportSize);

static inline Vec3 NormalizeV(const Vec3& v) {
	float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	if (len < 1e-6f) return Vec3(0, 1, 0);
	return Vec3(v.x / len, v.y / len, v.z / len);
}

static inline Vec3 CrossV(const Vec3& a, const Vec3& b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

static inline float DotV(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static void BuildBasisFromDirection(const Vec3& direction, Vec3& outRight, Vec3& outUp, Vec3& outForward) {
	outUp = NormalizeV(direction);

	// pick world-up to build right vector
	Vec3 worldUp(0, 1, 0);
	if (fabsf(DotV(outUp, worldUp)) > 0.99f) worldUp = Vec3(1, 0, 0);  // fallback

	outRight = NormalizeV(CrossV(worldUp, outUp));
	outForward = CrossV(outUp, outRight);
}

static std::array<float, 16> MatrixFromPosDirScale(const Vec3& pos, const Vec3& direction, float radius, float lengthScale = 1.0f) {
	Vec3 right, up, forward;
	BuildBasisFromDirection(direction, right, up, forward);

	float h = lengthScale;  // length of cylinder along direction

	return {right.x * radius, right.y * radius, right.z * radius, 0, up.x * h, up.y * h, up.z * h, 0, forward.x * radius, forward.y * radius, forward.z * radius, 0, pos.x, pos.y, pos.z, 1};
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

	std::shared_ptr<std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>>> tracking_solution =
	    std::make_shared<std::vector<std::tuple<std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>, std::array<double, 10>>>>();

	float cameraProjection[16];
	float cameraView[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	static constexpr float identityMatrix[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};

	std::vector<std::array<float, 16>> object_matrices = {{1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f}};

	float fov = 27.f;
	float camDistance = 8.f;

	static void DrawCylindersFilled(const float* view, const float* projection, const float* matrices, int matrixCount, float radius, float height, int segments = 32) {
		ImDrawList* dl = ImGui::GetWindowDrawList();
		if (!dl || matrixCount <= 0) return;

		ImVec2 const vpMin = ImGui::GetWindowPos();
		ImVec2 const vpSize(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());

		const float halfH = height * 0.5f;

		std::vector<CylinderFace> faces;
		faces.reserve(matrixCount * (segments + 2));  // top + bottom + strips

		for (int i = 0; i < matrixCount; ++i) {
			const float* model = &matrices[i * 16];

			// precompute all points in world+screen
			std::vector<ImVec2> topScr(segments);
			std::vector<ImVec2> botScr(segments);
			std::vector<float> zTop(segments), zBot(segments);

			for (auto s = 0; s < segments; s++) {
				float const a = static_cast<float>(s) / segments * 2.f * std::numbers::pi_v<float>;
				float const x = cosf(a) * radius;
				float const z = sinf(a) * radius;

				float const topLocal[3] = {x, halfH, z};
				float const botLocal[3] = {x, -halfH, z};

				float w0[4], w1[4];
				float v0[4], v1[4];
				float c0[4], c1[4];

				// top vertex -> world
				TransformPoint(model, topLocal, w0);
				TransformPoint(view, w0, v0);
				TransformPoint(projection, v0, c0);

				// bottom vertex -> world
				TransformPoint(model, botLocal, w1);
				TransformPoint(view, w1, v1);
				TransformPoint(projection, v1, c1);

				auto toScreen = [&](float clip[4]) {
					float const w = (clip[3] != 0.f ? clip[3] : 1e-6f);
					float const ndcX = clip[0] / w;
					float const ndcY = clip[1] / w;
					float const sx = vpMin.x + (ndcX * 0.5f + 0.5f) * vpSize.x;
					float const sy = vpMin.y + (-ndcY * 0.5f + 0.5f) * vpSize.y;
					return ImVec2(sx, sy);
				};

				topScr[s] = toScreen(c0);
				botScr[s] = toScreen(c1);

				zTop[s] = c0[2] / c0[3];
				zBot[s] = c1[2] / c1[3];
			}

			// -------------------------
			// TOP DISK
			// -------------------------
			{
				CylinderFace f;
				f.color = IM_COL32(120, 180, 255, 255);
				f.z = 0;

				for (auto s = 0; s < segments; ++s) {
					f.pts.push_back(topScr[s]);
					f.z += zTop[s];
				}
				f.z /= segments;

				faces.push_back(f);
			}

			// -------------------------
			// BOTTOM DISK
			// -------------------------
			{
				CylinderFace f;
				f.color = IM_COL32(120, 120, 200, 255);
				f.z = 0;

				for (auto s = 0; s < segments; ++s) {
					f.pts.push_back(botScr[s]);
					f.z += zBot[s];
				}
				f.z /= segments;

				faces.push_back(f);
			}

			// -------------------------
			// SIDE STRIPS
			// -------------------------
			for (int s = 0; s < segments; s++) {
				int const next = (s + 1) % segments;

				CylinderFace f;
				f.color = IM_COL32(160, 160, 255, 255);

				f.pts.push_back(topScr[s]);
				f.pts.push_back(topScr[next]);
				f.pts.push_back(botScr[next]);
				f.pts.push_back(botScr[s]);

				f.z = (zTop[s] + zTop[next] + zBot[s] + zBot[next]) * 0.25f;

				faces.push_back(f);
			}
		}

		// ----------------------------------------
		// depth sort back → front (painter's algorithm)
		// ----------------------------------------
		std::ranges::sort(faces, [](const CylinderFace& a, const CylinderFace& b) { return a.z > b.z; });

		// ----------------------------------------
		// draw all faces
		// ----------------------------------------
		for (auto& f : faces) {
			dl->AddConvexPolyFilled(f.pts.data(), f.pts.size(), f.color);
		}
	}
	static void DrawCylindersWire(const float* view, const float* projection, const float* matrices, int matrixCount, float radius, float height, int segments = 32) {
		ImDrawList* dl = ImGui::GetWindowDrawList();
		if (!dl || matrixCount <= 0) return;

		ImVec2 const vpMin = ImGui::GetWindowPos();
		ImVec2 const vpSize(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());

		const float halfH = height * 0.5f;

		std::vector<ImVec2> top(segments + 1);
		std::vector<ImVec2> bottom(segments + 1);

		ImU32 const col = IM_COL32(200, 200, 255, 255);
		float const thickness = 1.5f;

		for (int i = 0; i < matrixCount; i++) {
			const float* model = &matrices[i * 16];

			// Build top & bottom circles
			for (int s = 0; s <= segments; s++) {
				float const a = (float)s / (float)segments * 2.0f * 3.14159265f;
				float const x = cosf(a) * radius * 2.0f;
				float const z = sinf(a) * radius * 2.0f;

				float const topLocal[3] = {x, halfH, z};
				float const bottomLocal[3] = {x, -halfH, z};

				top[s] = ProjectToScreen(view, projection, model, topLocal, vpMin, vpSize);
				bottom[s] = ProjectToScreen(view, projection, model, bottomLocal, vpMin, vpSize);
			}

			// draw top circle
			dl->AddPolyline(top.data(), segments + 1, col, false, thickness);

			// draw bottom circle
			dl->AddPolyline(bottom.data(), segments + 1, col, false, thickness);

			// connect vertical edges
			for (int s = 0; s < segments; s++) dl->AddLine(top[s], bottom[s], col, thickness);
		}
	}

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

					if (auto points = MiMedMagnetometerArraySerialConnectionBinary::push([&current_state]() { return current_state == TrackingTabState::TRACKING; }); points.has_value()) {
						if (current_state == TrackingTabState::TRACKING) {
							auto const result = CeresOptimizerDirectionVector::process(points.value());

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

							mx.front() = result.x;
							my.front() = result.y;
							mz.front() = result.z;

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

				ImGuiIO const& io = ImGui::GetIO();
				bool hovered = ImGui::IsWindowHovered();
				if (hovered) {
					auto camPos = ExtractPositionFromViewMatrix(cameraView);
					auto camTarget = Vec3(0, 0, 0);

					Vec3 forward, right, up;
					ExtractDirectionFromViewMatrix(cameraView, forward, right, up);
					Normalize(&forward.x, &forward.x);
					Normalize(&right.x, &right.x);
					Normalize(&up.x, &up.x);

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
						auto [camXAngle, camYAngle] = ExtractAnglesFromForwardSafe(forward);

						camYAngle -= io.MouseDelta.x * 0.003f;
						camXAngle += io.MouseDelta.y * 0.003f;

						float const limit = 1.55f;
						camXAngle = std::clamp(camXAngle, -limit, limit);

						forward = Vec3(std::sin(camYAngle) * std::cos(camXAngle), -std::sinf(camXAngle), std::cos(camYAngle) * std::cos(camXAngle));
						Normalize(&forward.x, &forward.x);

						// FIX: use WORLD UP to avoid tilt
						Vec3 const worldUp(0.f, 1.f, 0.f);

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
					if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl)) camPos.y -= speed;

					camTarget.x = camPos.x + forward.x * camDistance;
					camTarget.y = camPos.y + forward.y * camDistance;
					camTarget.z = camPos.z + forward.z * camDistance;

					// -------------------------------------
					//     BUILD cameraView with LookAt
					// -------------------------------------
					float const eye[3] = {camPos.x, camPos.y, camPos.z};
					float const at[3] = {camTarget.x, camTarget.y, camTarget.z};
					float const upz[3] = {up.x, up.y, up.z};

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

				for (auto magnet : *std::atomic_load(&tracking_solution)) {
					auto const& [x, y, z, mx, my, mz] = magnet;

					auto const pos = Vec3{static_cast<float>(x[0] * 1e2), static_cast<float>(z[0] * 1e2), -static_cast<float>(y[0] * 1e2)};
					auto const dir = Vec3{static_cast<float>(mx[0] * 1e2), static_cast<float>(mz[0] * 1e2), -static_cast<float>(my[0] * 1e2)};

					auto matrix = MatrixFromPosDirScale(pos, dir, 0.4f);

					DrawCylindersWire(cameraView, cameraProjection, matrix.data(), 1,
					    0.4f,  // radius 0.4cm
					    0.5f,  // height 0.5cm
					    64     // segments
					);
				}

				// -------------------------------------
				//      VIEW MANIPULATOR
				// -------------------------------------

				ImGui::BeginChild("CameraWidget", ImVec2(150, 150), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
				ImGuizmo::BeginFrame();

				ImGuizmo::SetDrawlist();
				ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, ImGui::GetWindowWidth(), ImGui::GetWindowHeight());

				ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - 128, ImGui::GetWindowPos().y), ImVec2(128, 128), 0x10101010);

				if (!hovered) {
					// Here code to do after view manipulation
				}

				ImGui::EndChild();
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
					// ImPlot3D::PushStyleVar(ImPlot3DStyleVar_MarkerSize, 1.0f);
					// ImPlot3D::PushStyleVar(ImPlot3DStyleVar_LabelPadding, 1.1f);
					//
					// if (std::string title = "Tracking Solution"; ImPlot3D::BeginPlot(title.c_str(), {-1, -1}, ImPlot3DFlags_NoTitle | ImPlot3DFlags_Equal)) {
					//	ImPlot3D::SetupAxisLimits(ImAxis3D_X, -50, 200, ImGuiCond_Always);
					//	ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -50, 200, ImGuiCond_Always);
					//	ImPlot3D::SetupAxisLimits(ImAxis3D_Z, 0, 300, ImGuiCond_Always);
					//
					//	static auto ticklabel = [](float const value, char* buff, int const size, void* user_data) { return snprintf(buff, size, "%ld mm", std::lroundf(value)); };
					//
					//	ImPlot3D::SetupAxisFormat(ImAxis3D_X, ticklabel, nullptr);
					//	ImPlot3D::SetupAxisFormat(ImAxis3D_Y, ticklabel, nullptr);
					//	ImPlot3D::SetupAxisFormat(ImAxis3D_Z, ticklabel, nullptr);
					//
					//	// ImPlot3D::SetupAxis(ImAxis3D_X, "Magnetic Flux Density [T]");
					//	// ImPlot3D::SetupAxis(ImAxis3D_Y, "Magnetic Flux Density [T]");
					//	// ImPlot3D::SetupAxis(ImAxis3D_Z, "Magnetic Flux Density [T]");
					//	for (auto i = 0; auto const& [x, y, z] : *std::atomic_load(&tracking_solution)) {
					//		ImPlot3D::PlotScatter((std::string("Magnet ") + std::to_string(i)).c_str(), x.data(), y.data(), z.data(), x.size());
					//	}
					//
					//	ImPlot3D::EndPlot();
					//}
					//
					// break;
				}
			}

			ImGui::EndChild();
		}
	}
	virtual void set_active_tab(std::string tab_name) = 0;
};