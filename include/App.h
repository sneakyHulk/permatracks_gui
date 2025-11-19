#pragma once

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>
#include <implot.h>
#include <implot3d.h>

#include <boost/asio.hpp>
#include <string>

#include "MainTabBar.h"

class App final : protected MainTabBar {
   public:
	App() = default;
	void render() { MainTabBar::render(); }

	~App() = default;
};