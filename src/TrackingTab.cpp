#include "TrackingTab.h"

#include <numbers>
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
void DrawShadedFace(std::array<glm::vec3, 4>&& corners, const glm::vec3& normal, const glm::mat4& view, const glm::mat4& proj, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor, ImU32 edgeColor, float const edgeThickness) {
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
void DrawSphere(const glm::mat4& view, const glm::vec3& camPos, const glm::mat4& proj, const glm::vec3& center, float radius, const ImVec2& rectPos, const ImVec2& rectSize, ImU32 baseColor, int rings, int sectors) {
	for (int i = 0; i < rings; i++) {
		float theta0 = std::numbers::pi_v<float> * static_cast<float>(i) / rings;
		float theta1 = std::numbers::pi_v<float> * static_cast<float>(i + 1) / rings;

		for (int j = 0; j < sectors; j++) {
			float phi0 = 2.f * std::numbers::pi_v<float> * static_cast<float>(j) / sectors;
			float phi1 = 2.f * std::numbers::pi_v<float> * static_cast<float>(j + 1) / sectors;

			glm::vec3 const p00 = center + radius * glm::vec3(std::sin(theta0) * std::cos(phi0), std::cos(theta0), std::sin(theta0) * std::sin(phi0));
			glm::vec3 const p01 = center + radius * glm::vec3(std::sin(theta0) * std::cos(phi1), std::cos(theta0), std::sin(theta0) * std::sin(phi1));
			glm::vec3 const p10 = center + radius * glm::vec3(std::sin(theta1) * std::cos(phi0), std::cos(theta1), std::sin(theta1) * std::sin(phi0));
			glm::vec3 const p11 = center + radius * glm::vec3(std::sin(theta1) * std::cos(phi1), std::cos(theta1), std::sin(theta1) * std::sin(phi1));

			// Face normal (approx)
			glm::vec3 const n = glm::normalize(p00 + p01 + p10 + p11 - 4.0f * center);

			// Only draw if front-facing relative to camera
			glm::vec3 const faceCenter = (p00 + p01 + p10 + p11) * 0.25f;
			glm::vec3 const viewDir = glm::normalize(camPos - faceCenter);
			if (glm::dot(n, -viewDir) <= 0.0f) continue;

			DrawShadedFace({p00, p01, p11, p10}, n, view, proj, rectPos, rectSize, baseColor, IM_COL32(200, 200, 255, 30),  // faint outline
			    0.8f);
		}
	}
}