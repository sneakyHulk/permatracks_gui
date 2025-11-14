#pragma once
#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <atomic>

#include "MagnetSelection.h"
#include "linspace.h"

class MagnetSelectionTab : virtual protected MagnetSelection {
	std::atomic_bool error = false;

	enum class MagnetSelectionTabState {
		NONE,
	};
	std::atomic<MagnetSelectionTabState> state = MagnetSelectionTabState::NONE;

	// MagnetSelectionTab Data
	std::string error_message;

	double height = 5.;
	double radius = 4.;
	double remanence = 1.35;

	int selected_index = -1;

   public:
	MagnetSelectionTab() {}
	~MagnetSelectionTab() {}

	void render() {
		auto const error_ = error.load();

		if (ImGui::BeginChild("Magnet Selection Child", ImVec2(-1, -1), true)) {
			if (error_) {
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

			switch (state) {
				case MagnetSelectionTabState::NONE: {
					if (ImGui::InputDouble("mm Height", &height, 0.1, 1, "%.3f")) {
						if (height < 0.0) height = 0.0;
					}
					if (ImGui::InputDouble("mm Radius", &radius, 0.1, 1, "%.3f")) {
						if (radius < 0.0) radius = 0.0;
					}
					if (ImGui::InputDouble("T Remanence", &remanence, 0.1, 1, "%.3f")) {
						if (remanence < 0.0) remanence = 0.0;
					}

					if (ImGui::Button("Add Magnet")) {
						MagnetSelection::add_magnet({height * 1e-3, radius * 1e-3, remanence});
					}

					if (ImGui::BeginTable("Magnets", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchSame, {-1, 100})) {
						ImGui::TableSetupScrollFreeze(0, 1);
						ImGui::TableSetupColumn("Magnet");
						ImGui::TableSetupColumn("Height [mm]");
						ImGui::TableSetupColumn("Radius [mm]");
						ImGui::TableSetupColumn("Remanence [T]");
						ImGui::TableHeadersRow();

						for (auto index = 0; auto magnet : _magnets) {
							ImGui::TableNextRow();

							ImGui::TableSetColumnIndex(0);
							if (ImGui::Selectable(std::to_string(index + 1).c_str(), index == selected_index, ImGuiSelectableFlags_SpanAllColumns)) {
								selected_index = index;
							}

							ImGui::TableSetColumnIndex(1);
							ImGui::Text("%s", std::to_string(height).c_str());

							ImGui::TableSetColumnIndex(2);
							ImGui::Text("%s", std::to_string(radius).c_str());

							ImGui::TableSetColumnIndex(3);
							ImGui::Text("%s", std::to_string(remanence).c_str());

							++index;
						}

						ImGui::EndTable();
					}

					if (selected_index > -1) {
						Magnet const& magnet = _magnets[selected_index];
						if (ImGui::BeginTabBar("Magnet Position Accuracy")) {
							if (ImGui::BeginTabItem("X/Y")) {
								ImPlot::SetNextAxisLimits(ImAxis_X1, 0, 300, ImGuiCond_Always);
								ImPlot::SetNextAxisLimits(ImAxis_Y1, 0, 10, ImGuiCond_Always);
								if (std::string const title = std::string("Maximal Magnet X/Y-Position Accuracy of Magnet (H:") + std::to_string(magnet.H * 1e3) + std::string("mm, R:") + std::to_string(magnet.R * 1e3) +
								                              std::string("mm, Br:") + std::to_string(magnet.Br) + std::string("T)");
								    ImPlot::BeginPlot(title.c_str(), {-1, -1})) {
									std::array<double, 100> x = linspace<100>(0, 300);
									std::array<double, 100> y = linspace<100>(0, 300);

									ImPlot::PlotLine("X-Y Projection", x.data(), y.data(), x.size());
									ImPlot::EndPlot();
								}
								ImGui::EndTabItem();
							}

							if (ImGui::BeginTabItem("Z")) {
								ImPlot::SetNextAxisLimits(ImAxis_X1, 0, 300, ImGuiCond_Always);
								ImPlot::SetNextAxisLimits(ImAxis_Y1, 0, 10, ImGuiCond_Always);
								if (std::string const title = std::string("Maximal Magnet Z-Position Accuracy of Magnet (H:") + std::to_string(magnet.H * 1e3) + std::string("mm, R:") + std::to_string(magnet.R * 1e3) +
								                              std::string("mm, Br:") + std::to_string(magnet.Br) + std::string("T)");
								    ImPlot::BeginPlot(title.c_str(), {-1, -1})) {
									std::array<double, 100> x = linspace<100>(0, 300);
									std::array<double, 100> y = linspace<100>(0, 300);

									ImPlot::PlotLine("X-Y Projection", x.data(), y.data(), x.size());
									ImPlot::EndPlot();
								}
								ImGui::EndTabItem();
							}
							ImGui::EndTabBar();
						}
					}

					break;
				}
			}

			ImGui::EndChild();
		}
	}
};