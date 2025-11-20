#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

// This is necessary

#include <ImGuizmo.h>
#include <__filesystem/filesystem_error.h>

#include <expected>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <numbers>
#include <ranges>

enum class ProjectError {
	BehindCamera,
	OutsideFrustum,
	ZeroW,
};

std::expected<ImVec2, ProjectError> ProjectPoint(const glm::vec3& p, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize) {
	glm::vec4 clip = proj * view * glm::vec4(p, 1.0f);

	if (clip.w == 0.0f) return std::unexpected(ProjectError::ZeroW);
	if (clip.w < 0.0f) return std::unexpected(ProjectError::BehindCamera);

	clip /= clip.w;

	if (clip.x < -1.0f || clip.x > 1.0f || clip.y < -1.0f || clip.y > 1.0f || clip.z < -1.0f || clip.z > 1.0f) {
		return std::unexpected(ProjectError::OutsideFrustum);
	}

	float const x = rectPos.x + (clip.x * 0.5f + 0.5f) * rectSize.x;
	float const y = rectPos.y + (clip.y * -0.5f + 0.5f) * rectSize.y;

	return ImVec2(x, y);
}

void DrawDirectionArrow(const glm::mat4& view, const glm::vec3& camPos, const glm::mat4& proj, const glm::vec3& origin, const glm::vec3& direction, const ImVec2& rectPos, const ImVec2& rectSize) {
	ImDrawList* dl = ImGui::GetWindowDrawList();

	glm::vec3 const p0 = origin;
	glm::vec3 const p1 = origin + glm::normalize(direction) * 1.5f;

	auto const a = ProjectPoint(p0, view, proj, rectPos, rectSize);
	auto const b = ProjectPoint(p1, view, proj, rectPos, rectSize);

	if (a.has_value() && b.has_value()) dl->AddLine(a.value(), b.value(), IM_COL32(255, 50, 50, 255), 3.0f);

	// Arrow tip
	glm::vec3 const f = glm::normalize(direction);
	glm::vec3 const up(0, 1, 0);
	glm::vec3 const side = glm::normalize(glm::cross(f, up));
	glm::vec3 const tipL = p1 - f * 0.2f + side * 0.15f;
	glm::vec3 const tipR = p1 - f * 0.2f - side * 0.15f;

	auto const sL = ProjectPoint(tipL, view, proj, rectPos, rectSize);
	auto const sR = ProjectPoint(tipR, view, proj, rectPos, rectSize);

	if (b.has_value() && sL.has_value()) dl->AddLine(b.value(), sL.value(), IM_COL32(255, 50, 50, 255), 3.0f);
	if (b.has_value() && sR.has_value()) dl->AddLine(b.value(), sR.value(), IM_COL32(255, 50, 50, 255), 3.0f);
}

ImU32 ShadeFace(const glm::mat4& view, const glm::vec3& normal, ImU32 baseColor) {
	glm::vec3 const lightWorld = glm::normalize(glm::vec3(0.0f, -1.0f, 0.0f));

	float d = glm::dot(glm::normalize(normal), lightWorld);
	d = glm::clamp(d * 0.5f + 0.5f, 0.1f, 1.0f);  // restrict contrast

	ImVec4 c = ImGui::ColorConvertU32ToFloat4(baseColor);
	c.x *= d;
	c.y *= d;
	c.z *= d;

	return ImGui::ColorConvertFloat4ToU32(c);
}

void DrawShadedFace(std::array<glm::vec3, 4>&& corners, const glm::vec3& normal, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor, ImU32 edgeColor = IM_COL32(255, 255, 255, 180),
    float const edgeThickness = 1.5f) {
	ImDrawList* dl = ImGui::GetWindowDrawList();

	ImU32 const shadedColor = ShadeFace(view, normal, baseColor);

	std::array<ImVec2, 4> pts;
	for (auto const& [pt, corner] : std::ranges::views::zip(pts, corners)) {
		if (auto p = ProjectPoint(corner, view, proj, rectPos, rectSize)) {
			pt = p.value();
		} else {
			return;
		}
	}

	// Fill
	dl->AddConvexPolyFilled(pts.data(), 4, shadedColor);

	// Outline
	for (int i = 0; i < 4; i++) dl->AddLine(pts[i], pts[(i + 1) % 4], edgeColor, edgeThickness);
}

void DrawCylinderFaces(const glm::mat4& view, glm::vec3 const& camPos, const glm::mat4& proj, const glm::vec3& center, const glm::vec3& axis, float radius, float height, const ImVec2& rectPos, const ImVec2& rectSize) {
	constexpr int segments = 32;
	constexpr ImU32 topColor = IM_COL32(200, 200, 255, 255);
	constexpr ImU32 bottomColor = IM_COL32(200, 255, 200, 255);
	constexpr ImU32 sideColor = IM_COL32(255, 200, 200, 255);

	glm::vec3 const up = normalize(axis);
	glm::vec3 const tmp = (fabs(up.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0));

	glm::vec3 const right = normalize(cross(tmp, up));
	glm::vec3 const forward = normalize(cross(up, right));

	float const h = height * 0.5f;

	glm::vec3 const topCenter = center + up * h;
	glm::vec3 const bottomCenter = center - up * h;

	for (int i = 0; i < segments; i++) {
		float const a0 = static_cast<float>(i) / segments * 2.f * std::numbers::pi_v<float>;
		float const a1 = static_cast<float>(i + 1) / segments * 2.f * std::numbers::pi_v<float>;

		glm::vec3 const t0 = topCenter + radius * (right * std::cos(a0) + forward * std::sin(a0));
		glm::vec3 const t1 = topCenter + radius * (right * std::cos(a1) + forward * std::sin(a1));
		glm::vec3 const b0 = bottomCenter + radius * (right * std::cos(a0) + forward * std::sin(a0));
		glm::vec3 const b1 = bottomCenter + radius * (right * std::cos(a1) + forward * std::sin(a1));

		// SIDE FACES:
		glm::vec3 const normal = normalize(cross(t1 - t0, b0 - t0));
		glm::vec3 const faceCenter = (t0 + t1 + b0 + b1) * 0.25f;
		if (glm::vec3 const viewDir = glm::normalize(camPos - faceCenter); glm::dot(normal, -viewDir) > 0.0f) {
			DrawShadedFace({b0, b1, t1, t0}, normal, view, proj, rectPos, rectSize, sideColor);
		}

		// TOP FACE:
		if (glm::vec3 const viewDir = glm::normalize(camPos - topCenter); glm::dot(up, viewDir) > 0.0f) {
			DrawShadedFace({topCenter, t0, t1, topCenter}, -up, view, proj, rectPos, rectSize, topColor);
		}

		// BOTTOM:
		if (glm::vec3 const viewDir = glm::normalize(camPos - bottomCenter); glm::dot(-up, viewDir) > 0.0f) {
			DrawShadedFace({bottomCenter, b1, b0, bottomCenter}, up, view, proj, rectPos, rectSize, bottomColor);
		}
	}
}

#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <assimp/Importer.hpp>

struct StlTriangle {
	glm::vec3 v0;
	glm::vec3 v1;
	glm::vec3 v2;
	glm::vec3 normal;
};

void DrawStlTriangle(const StlTriangle& tri, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor, ImU32 edgeColor = IM_COL32(0, 0, 0, 255), float edgeThickness = 1.0f) {
	auto s0 = ProjectPoint(tri.v0, view, proj, rectPos, rectSize);
	auto s1 = ProjectPoint(tri.v1, view, proj, rectPos, rectSize);
	auto s2 = ProjectPoint(tri.v2, view, proj, rectPos, rectSize);

	if (!s0 || !s1 || !s2) return;  // at least one vertex not visible / behind camera

	ImDrawList* dl = ImGui::GetWindowDrawList();

	ImVec2 pts[3] = {s0.value(), s1.value(), s2.value()};

	// Normal: use file normal if non-zero, else recalc
	glm::vec3 n = tri.normal;
	if (glm::length(n) < 1e-6f)
		n = glm::normalize(glm::cross(tri.v1 - tri.v0, tri.v2 - tri.v0));
	else
		n = glm::normalize(n);

	ImU32 shaded = ShadeFace(view, n, baseColor);

	dl->AddConvexPolyFilled(pts, 3, shaded);
	dl->AddPolyline(pts, 3, edgeColor, ImDrawFlags_Closed, edgeThickness);
}

void DrawStlMesh(const std::vector<StlTriangle>& tris, const glm::mat4& view, const glm::mat4& proj, const glm::vec3& camPos, const ImVec2& rectPos, const ImVec2& rectSize) {
	for (auto const& t : tris) {
		DrawStlTriangle(t, view, proj, rectPos, rectSize, IM_COL32(200, 200, 255, 255), IM_COL32(20, 20, 20, 255), 1.0f);
	}
}

glm::vec3 ComputeMeshCenter(const aiMesh* mesh) {
	glm::vec3 minv(FLT_MAX);
	glm::vec3 maxv(-FLT_MAX);

	for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
		aiVector3D v = mesh->mVertices[i];
		minv.x = std::min(minv.x, v.x);
		minv.y = std::min(minv.y, v.y);
		minv.z = std::min(minv.z, v.z);

		maxv.x = std::max(maxv.x, v.x);
		maxv.y = std::max(maxv.y, v.y);
		maxv.z = std::max(maxv.z, v.z);
	}

	return (minv + maxv) * 0.5f;
}

void CenterMesh(aiMesh* mesh) {
	glm::vec3 center = ComputeMeshCenter(mesh);

	for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
		mesh->mVertices[i].x -= center.x;
		mesh->mVertices[i].y -= center.y;
		mesh->mVertices[i].z -= center.z;
	}
}

std::vector<StlTriangle> LoadStlAssimp(const std::string& filename) {
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);

	if (!scene || !scene->HasMeshes()) return {};

	std::vector<StlTriangle> tris;

	for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
		aiMesh* mesh = scene->mMeshes[m];

		// 🟢 STL zentrieren
		CenterMesh(mesh);

		for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
			const aiFace& face = mesh->mFaces[f];
			if (face.mNumIndices != 3) continue;

			aiVector3D a = mesh->mVertices[face.mIndices[0]];
			aiVector3D b = mesh->mVertices[face.mIndices[1]];
			aiVector3D c = mesh->mVertices[face.mIndices[2]];

			aiVector3D n = mesh->mNormals[face.mIndices[0]];

			tris.push_back({glm::vec3(a.x, a.y, a.z), glm::vec3(b.x, b.y, b.z), glm::vec3(c.x, c.y, c.z), glm::normalize(glm::vec3(n.x, n.y, n.z))});
		}
	}

	return tris;
}

inline glm::vec3 to_imgui(glm::vec3 const& u) { return {-u.y, u.z, -u.x}; }

void AppGui() {
	if (ImGui::BeginTabBar("MainTabs")) {
		if (ImGui::BeginTabItem("Viewport")) {
			if (ImGui::BeginChild("Tracking Child", ImVec2(-1, -1), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
				static constexpr glm::vec3 up(0.0f, 1.0f, 0.0f);
				static constexpr float speed = 5.f;
				static constexpr float orbit_speed = 0.01f;

				static auto stl = LoadStlAssimp(std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "OBJ_PCB_MAGNETOMETER_ARRAY_V1.obj");

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
				// DrawStlMesh(stl, view, proj, camPos, winPos, winSize);

				ImGuizmo::DrawGrid(glm::value_ptr(view), glm::value_ptr(proj), glm::value_ptr(model), 10.0f);
				DrawDirectionArrow(view, camPos, proj, to_imgui(position), to_imgui(direction), winPos, winSize);
				// DrawCylinder(view, proj, position, direction, 0.4f, 0.5f, winPos, winSize);
				DrawCylinderFaces(view, camPos, proj, to_imgui(position), to_imgui(direction), 0.4f, 0.5f, winPos, winSize);

				// glm::vec3 f = glm::normalize(direction);
				// glm::vec3 u = glm::vec3(0, 1, 0);
				// glm::vec3 r = glm::normalize(glm::cross(u, f));
				// u = glm::cross(f, r);
				//
				// glm::mat4 rot(1.0f);
				// rot[0] = glm::vec4(r, 0.0f);
				// rot[1] = glm::vec4(u, 0.0f);
				// rot[2] = glm::vec4(f, 0.0f);
				//
				// glm::mat4 object(1.0f);
				// object = glm::translate(object, position);
				// object *= rot;
				// ImGuizmo::DrawCubes(glm::value_ptr(view), glm::value_ptr(proj), glm::value_ptr(object), 1);

				if (ImGui::BeginChild("CameraWidget", ImVec2(130, 130), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
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

				if (ImGui::BeginChild("Inspector", ImVec2(300, 60), true)) {
					ImGui::DragFloat3("Position", glm::value_ptr(position), 0.1f);
					ImGui::DragFloat3("Rotation", glm::value_ptr(direction), 0.05f);

					direction = glm::normalize(direction);
					ImGui::EndChild();
				}

				ImGui::EndChild();
			}

			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}
}

int main(int, char**) {
	HelloImGui::RunnerParams params;
	params.appWindowParams.windowTitle = "test_magnet";
	params.appWindowParams.windowGeometry.size = {1280, 720};
	params.imGuiWindowParams.showMenuBar = false;

	params.platformBackendType = HelloImGui::PlatformBackendType::Glfw;

	params.appWindowParams.windowGeometry.positionMode = HelloImGui::WindowPositionMode::MonitorCenter;
	params.appWindowParams.windowGeometry.windowSizeMeasureMode = HelloImGui::WindowSizeMeasureMode::RelativeTo96Ppi;

	params.callbacks.ShowGui = AppGui;

	HelloImGui::Run(params);
	return 0;
}