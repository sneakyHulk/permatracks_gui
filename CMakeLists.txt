cmake_minimum_required(VERSION 3.22)
project(permatracks)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Assimp REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(tbb REQUIRED)

### HELLOIMGUI
set(HELLOIMGUI_USE_GLFW3 ON CACHE BOOL "" FORCE)
set(HELLOIMGUI_HAS_OPENGL3 ON CACHE BOOL "" FORCE)
set(HELLOIMGUI_MACOS_NO_BUNDLE ON CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/helloimgui)

### ImGuizmo
project(imguizmo)

add_library(${PROJECT_NAME} thirdparty/imguizmo/ImGuizmo.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui)
target_include_directories(${PROJECT_NAME} PUBLIC thirdparty/imguizmo)

### ImPlot
project(implot)

add_library(${PROJECT_NAME} thirdparty/implot/implot.cpp thirdparty/implot/implot_items.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui)
target_include_directories(${PROJECT_NAME} PUBLIC thirdparty/implot)

### ImPlot3d
project(implot3d)

add_library(${PROJECT_NAME} thirdparty/implot3d/implot3d.cpp thirdparty/implot3d/implot3d_items.cpp thirdparty/implot3d/implot3d_meshes.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui)
target_include_directories(${PROJECT_NAME} PUBLIC thirdparty/implot3d)

### add local libraries
add_subdirectory(list_serial_devices)
add_subdirectory(ring_buffer)

### GUI
project(gui)

add_executable(${PROJECT_NAME} main.cpp src/App.cpp src/MainTabBar.cpp src/SerialConnection.cpp src/ConnectionTab.cpp src/CalibrationTab.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC implot)
target_link_libraries(${PROJECT_NAME} PUBLIC implot3d)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC list_serial_devices)
target_link_libraries(${PROJECT_NAME} PUBLIC ring_buffer)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

project(minimal)

add_executable(${PROJECT_NAME} test/minimal.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

#project(minimal_3d)
#add_executable(${PROJECT_NAME} test/minimal_3d.cpp)
#target_include_directories(${PROJECT_NAME} PUBLIC include)
#target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
#target_link_libraries(${PROJECT_NAME} PUBLIC ImGuizmo)
#target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL::GL)
#target_link_libraries(${PROJECT_NAME} PUBLIC GLEW::GLEW)
#target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
#target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
#
#project(minimal_3d_load)
#add_executable(${PROJECT_NAME} test/minimal_3d_load.cpp)
#target_include_directories(${PROJECT_NAME} PUBLIC include)
#target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
#target_link_libraries(${PROJECT_NAME} PUBLIC ImGuizmo)
#target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL::GL)
#target_link_libraries(${PROJECT_NAME} PUBLIC GLEW::GLEW)
#target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
#target_link_libraries(${PROJECT_NAME} PUBLIC assimp::assimp)
#target_link_libraries(${PROJECT_NAME} PUBLIC glm::glm)
#
#target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")