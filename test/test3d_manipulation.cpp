// Minimal ImGuizmo demo using Hello ImGui,
// preserving the original "Gizmo" window behavior and view cube.

#define IMGUI_DEFINE_MATH_OPERATORS

#include "hello_imgui/hello_imgui.h"

// Both includes must come before

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

#include "ImGuizmo.h"
#include "imgui.h"
#include "imgui_internal.h"

// -----------------------------------------------------------------------------
// Original global state from the full example
// -----------------------------------------------------------------------------

float camDistance = 8.f;
static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);
static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::WORLD);

float objectMatrix[4][16] = {{1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 2.f, 0.f, 0.f, 1.f},
    {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 2.f, 0.f, 2.f, 1.f}, {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 2.f, 1.f}};

static constexpr float identityMatrix[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};

// Camera matrices
float cameraView[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};

float cameraProjection[16];

// Camera params
float fov = 27.f;
float viewWidth = 10.f;  // for orthographic
float camYAngle = 165.f / 180.f * 3.14159f;
float camXAngle = 32.f / 180.f * 3.14159f;
bool firstFrame = true;
int lastUsing = 0;

// -----------------------------------------------------------------------------
// Original math helpers
// -----------------------------------------------------------------------------
void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16) {
	float temp, temp2, temp3, temp4;
	temp = 2.0f * znear;
	temp2 = right - left;
	temp3 = top - bottom;
	temp4 = zfar - znear;
	m16[0] = temp / temp2;
	m16[1] = 0.0f;
	m16[2] = 0.0f;
	m16[3] = 0.0f;
	m16[4] = 0.0f;
	m16[5] = temp / temp3;
	m16[6] = 0.0f;
	m16[7] = 0.0f;
	m16[8] = (right + left) / temp2;
	m16[9] = (top + bottom) / temp3;
	m16[10] = (-zfar - znear) / temp4;
	m16[11] = -1.0f;
	m16[12] = 0.0;
	m16[13] = 0.0;
	m16[14] = (-temp * zfar) / temp4;
	m16[15] = 0.0;
}

void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16) {
	float ymax, xmax;
	ymax = znear * tanf(fovyInDegrees * 3.141592f / 180.0f);
	xmax = ymax * aspectRatio;
	Frustum(-xmax, xmax, -ymax, ymax, znear, zfar, m16);
}

void Cross(const float* a, const float* b, float* r) {
	r[0] = a[1] * b[2] - a[2] * b[1];
	r[1] = a[2] * b[0] - a[0] * b[2];
	r[2] = a[0] * b[1] - a[1] * b[0];
}

float Dot(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

void Normalize(const float* a, float* r) {
	float il = 1.f / (sqrtf(Dot(a, a)) + FLT_EPSILON);
	r[0] = a[0] * il;
	r[1] = a[1] * il;
	r[2] = a[2] * il;
}

void MultiplyMatrix(const float* a, const float* b, float* result) {
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			result[col + row * 4] = a[row * 4 + 0] * b[col + 0] + a[row * 4 + 1] * b[col + 4] + a[row * 4 + 2] * b[col + 8] + a[row * 4 + 3] * b[col + 12];
		}
	}
}

void LookAt(const float* eye, const float* at, const float* up, float* m16) {
	float X[3], Y[3], Z[3], tmp[3];

	tmp[0] = eye[0] - at[0];
	tmp[1] = eye[1] - at[1];
	tmp[2] = eye[2] - at[2];
	Normalize(tmp, Z);
	Normalize(up, Y);

	Cross(Y, Z, tmp);
	Normalize(tmp, X);

	Cross(Z, X, tmp);
	Normalize(tmp, Y);

	m16[0] = X[0];
	m16[1] = Y[0];
	m16[2] = Z[0];
	m16[3] = 0.0f;
	m16[4] = X[1];
	m16[5] = Y[1];
	m16[6] = Z[1];
	m16[7] = 0.0f;
	m16[8] = X[2];
	m16[9] = Y[2];
	m16[10] = Z[2];
	m16[11] = 0.0f;
	m16[12] = -Dot(X, eye);
	m16[13] = -Dot(Y, eye);
	m16[14] = -Dot(Z, eye);
	m16[15] = 1.0f;
}

void OrthoGraphic(const float l, float r, const float b, const float t, float zn, const float zf, float* m16) {
	m16[0] = 2 / (r - l);
	m16[1] = 0.0f;
	m16[2] = 0.0f;
	m16[3] = 0.0f;
	m16[4] = 0.0f;
	m16[5] = 2 / (t - b);
	m16[6] = 0.0f;
	m16[7] = 0.0f;
	m16[8] = 0.0f;
	m16[9] = 0.0f;
	m16[10] = 1.0f / (zf - zn);
	m16[11] = 0.0f;
	m16[12] = (l + r) / (l - r);
	m16[13] = (t + b) / (b - t);
	m16[14] = zn / (zn - zf);
	m16[15] = 1.0f;
}

// -----------------------------------------------------------------------------
// Original TransformStart / TransformEnd / EditTransform
// -----------------------------------------------------------------------------
void TransformStart(float* cameraView, float* cameraProjection, float* matrix) {
	static float bounds[] = {-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};
	static bool boundSizing = false;

	if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE)) mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
	ImGui::SameLine();
	if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE)) mCurrentGizmoOperation = ImGuizmo::ROTATE;
	ImGui::SameLine();
	if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE)) mCurrentGizmoOperation = ImGuizmo::SCALE;

	float matrixTranslation[3], matrixRotation[3], matrixScale[3];
	ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale);
	ImGui::InputFloat3("Tr", matrixTranslation);
	ImGui::InputFloat3("Rt", matrixRotation);
	ImGui::InputFloat3("Sc", matrixScale);
	ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix);

	if (mCurrentGizmoOperation != ImGuizmo::SCALE) {
		if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL)) mCurrentGizmoMode = ImGuizmo::LOCAL;
		ImGui::SameLine();
		if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD)) mCurrentGizmoMode = ImGuizmo::WORLD;
	}

	static ImGuiWindowFlags gizmoWindowFlags = 0;

	// This is the original "Gizmo" window
	// ImGui::SetNextWindowSize(ImVec2(800, 400), ImGuiCond_Appearing);
	// ImGui::SetNextWindowPos(ImVec2(400, 20), ImGuiCond_Appearing);

	ImGuiViewport* viewport = ImGui::GetMainViewport();

	ImGui::SetNextWindowPos(viewport->WorkPos);
	ImGui::SetNextWindowSize(viewport->WorkSize);
	ImGui::SetNextWindowViewport(viewport->ID);

	ImGui::PushStyleColor(ImGuiCol_WindowBg, (ImVec4)ImColor(0.35f, 0.3f, 0.3f));

	ImGui::Begin("Gizmo", nullptr, gizmoWindowFlags);
	ImGuizmo::SetDrawlist();  // default foreground / background (same as original)

	auto const windowWidth = ImGui::GetWindowWidth();
	auto const windowHeight = ImGui::GetWindowHeight();

	Perspective(fov, windowWidth / windowHeight, 0.1f, 89.9f, cameraProjection);

	ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, windowWidth, windowHeight);
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	gizmoWindowFlags = ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(window->InnerRect.Min, window->InnerRect.Max) ? ImGuiWindowFlags_NoMove : 0;

	ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix, 100.f);
	ImGuizmo::DrawCubes(cameraView, cameraProjection, &objectMatrix[0][0], 1);

	ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(ImGui::GetWindowPos().x + windowWidth - 128, ImGui::GetWindowPos().y), ImVec2(128, 128), 0x10101010);

	ImGui::PopStyleColor(1);
	ImGui::End();
}

// -----------------------------------------------------------------------------
// Hello ImGui frame: this replaces the old while(!imApp.Done()) loop
// -----------------------------------------------------------------------------
void AppGui() {
	ImGuizmo::BeginFrame();

	// "Editor" window from original demo
	ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(320, 340), ImGuiCond_Appearing);
	ImGui::Begin("Editor");

	ImGui::Text("Camera");
	bool viewDirty = false;

	ImGui::SliderFloat("Fov", &fov, 20.f, 110.f);

	viewDirty |= ImGui::SliderFloat("Distance", &camDistance, 1.f, 10.f);

	if (viewDirty || firstFrame) {
		float eye[] = {cosf(camYAngle) * cosf(camXAngle) * camDistance, sinf(camXAngle) * camDistance, sinf(camYAngle) * cosf(camXAngle) * camDistance};
		float at[] = {0.f, 0.f, 0.f};
		float up[] = {0.f, 1.f, 0.f};
		LookAt(eye, at, up, cameraView);
		firstFrame = false;
	}

	// Same transform UI & gizmo loop as original
	TransformStart(cameraView, cameraProjection, objectMatrix[lastUsing]);
	ImGui::End();
}

int main(int, char**) {
	HelloImGui::RunnerParams params;
	params.appWindowParams.windowTitle = "ImGuizmo + Hello ImGui (like original)";
	params.appWindowParams.windowGeometry.size = {1280, 720};
	params.imGuiWindowParams.backgroundColor = ImVec4(0.45f, 0.4f, 0.4f, 1.f);
	params.imGuiWindowParams.showMenuBar = false;

	params.callbacks.ShowGui = AppGui;

	HelloImGui::Run(params);
	return 0;
}