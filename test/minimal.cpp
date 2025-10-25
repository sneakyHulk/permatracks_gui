#include "hello_imgui/hello_imgui.h"
#include "imgui.h"

void Gui() {
  ImGui::Text("HelloImGui windowGeometry demo!");
  ImGui::Text("Window position and size are set before startup.");
}

int main(int, char **) {
  HelloImGui::RunnerParams params;

  params.appWindowParams.windowTitle = "HelloImGui windowGeometry Demo";

  params.appWindowParams.windowGeometry.size = {1024, 720};
  params.appWindowParams.windowGeometry.position = {100, 100};
  params.appWindowParams.windowGeometry.positionMode =
      HelloImGui::WindowPositionMode::OsDefault;
  params.appWindowParams.windowGeometry.monitorIdx = 0;

  params.appWindowParams.restorePreviousGeometry = true;

  params.callbacks.ShowGui = Gui;

  params.platformBackendType = HelloImGui::PlatformBackendType::Glfw;
  HelloImGui::Run(params);
}