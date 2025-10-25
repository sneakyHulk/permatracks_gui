#include "hello_imgui/hello_imgui.h"
#include "imgui.h"

// this must come after

#include "ImGuizmo.h"

#include <glad/glad.h> // or use glad, depending on your setup
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Simple camera and model
static glm::mat4 cameraView;
static glm::mat4 cameraProjection;
static glm::mat4 modelMatrix = glm::mat4(1.0f);

void Render3DScene() {
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw simple XYZ axes for reference
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0); // X axis
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0); // Y axis
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1); // Z axis
  glEnd();
}

void Show3DViewer() {
  ImGui::Begin("3D Viewer");

  ImVec2 region = ImVec2(500, 500);
  ImGui::BeginChild("RenderRegion", region, true,
                    ImGuiWindowFlags_NoScrollWithMouse);

  // Get window info for ImGuizmo
  ImVec2 windowPos = ImGui::GetWindowPos();
  ImVec2 windowSize = ImGui::GetWindowSize();

  // Setup camera
  cameraView =
      glm::lookAt(glm::vec3(2, 2, 2), glm::vec3(0, 0, 0), glm::vec3(0, 0, 1));
  cameraProjection =
      glm::perspective(glm::radians(45.0f), region.x / region.y, 0.1f, 100.0f);

  // Render our simple OpenGL scene
  Render3DScene();

  // Setup and draw ImGuizmo
  ImGuizmo::BeginFrame();
  ImGuizmo::SetOrthographic(false);
  ImGuizmo::SetDrawlist();
  ImGuizmo::SetRect(windowPos.x, windowPos.y, windowSize.x, windowSize.y);

  static ImGuizmo::OPERATION gizmoOperation = ImGuizmo::TRANSLATE;
  if (ImGui::RadioButton("Translate", gizmoOperation == ImGuizmo::TRANSLATE))
    gizmoOperation = ImGuizmo::TRANSLATE;
  ImGui::SameLine();
  if (ImGui::RadioButton("Rotate", gizmoOperation == ImGuizmo::ROTATE))
    gizmoOperation = ImGuizmo::ROTATE;

  ImGuizmo::Manipulate(glm::value_ptr(cameraView),
                       glm::value_ptr(cameraProjection), gizmoOperation,
                       ImGuizmo::WORLD, glm::value_ptr(modelMatrix));

  ImGui::EndChild();
  ImGui::End();
}

void Gui() {
  ImGui::Begin("Controls");
  ImGui::Text("Use the gizmo in the 3D Viewer window.");
  ImGui::End();

  Show3DViewer();
}

int main() {
  HelloImGui::RunnerParams params;
  params.appWindowParams.windowTitle = "ImGuizmo 3D Manipulator Example";
  params.appWindowParams.windowGeometry.size = {1024, 720};
  params.rendererBackendType = HelloImGui::RendererBackendType::OpenGL3;
  params.callbacks.ShowGui = Gui;

  HelloImGui::Run(params);
}