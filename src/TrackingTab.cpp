#include "TrackingTab.h"
#include <numbers>

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
bool InvertMatrix(const float m[16], float invOut[16]) {
	float inv[16];
	inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
	inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
	inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
	inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
	inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
	inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
	inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
	inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
	inv[3] = -m[1] * m[6] * m[15] + m[1] * m[7] * m[14] + m[5] * m[2] * m[15] - m[5] * m[3] * m[14] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
	inv[7] = m[0] * m[6] * m[15] - m[0] * m[7] * m[14] - m[4] * m[2] * m[15] + m[4] * m[3] * m[14] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
	inv[11] = -m[0] * m[5] * m[15] + m[0] * m[7] * m[13] + m[4] * m[1] * m[15] - m[4] * m[3] * m[13] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
	inv[15] = m[0] * m[5] * m[14] - m[0] * m[6] * m[13] - m[4] * m[1] * m[14] + m[4] * m[2] * m[13] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
	float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
	if (det == 0) return false;
	det = 1.0f / det;
	for (auto i = 0; i < 16; i++) invOut[i] = inv[i] * det;
	return true;
}
void ExtractDirectionFromViewMatrix(const float* view, Vec3& forward, Vec3& right, Vec3& up) {
	right = Vec3(view[0], view[4], view[8]);
	up = Vec3(view[1], view[5], view[9]);
	forward = Vec3(-view[2], -view[6], -view[10]);  // negative Z
}
Vec3 ExtractPositionFromViewMatrix(float const* view) {
	float inv[16];
	InvertMatrix(view, inv);
	return Vec3(inv[12], inv[13], inv[14]);
}
Vec3 ExtractForwardFromViewMatrix(float const* view) {
	// forward is the -Z axis of the camera
	Vec3 f(-view[2], -view[6], -view[10]);
	Normalize(&f.x, &f.x);
	return f;
}
std::tuple<float, float> ExtractAnglesFromForwardSafe(Vec3 const& forward) {
	// pitch: clamp because asin(±1) is valid, but later tan() or cos() becomes unstable
	float pitch = -asin(std::clamp(forward.y, -0.9999f, 0.9999f));

	float yaw = atan2(forward.x, forward.z);

	return {pitch, yaw};
}
void TransformPoint(const float* m, const float in[3], float out[4]) {
	const float x = in[0];
	const float y = in[1];
	const float z = in[2];

	out[0] = m[0] * x + m[4] * y + m[8] * z + m[12];
	out[1] = m[1] * x + m[5] * y + m[9] * z + m[13];
	out[2] = m[2] * x + m[6] * y + m[10] * z + m[14];
	out[3] = m[3] * x + m[7] * y + m[11] * z + m[15];
}
ImVec2 ProjectToScreen(const float* view, const float* proj, const float* model, const float local[3], const ImVec2& viewportMin, const ImVec2& viewportSize) {
	float world[4];
	TransformPoint(model, local, world);

	float viewPos[4];
	TransformPoint(view, world, viewPos);

	float clip[4];
	TransformPoint(proj, viewPos, clip);

	if (clip[3] == 0.0f) clip[3] = 1e-6f;

	float const ndcX = clip[0] / clip[3];
	float const ndcY = clip[1] / clip[3];

	float const sx = viewportMin.x + (ndcX * 0.5f + 0.5f) * viewportSize.x;
	float const sy = viewportMin.y + (-ndcY * 0.5f + 0.5f) * viewportSize.y;  // flip Y

	return {sx, sy};
}