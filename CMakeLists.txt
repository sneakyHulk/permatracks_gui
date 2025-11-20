cmake_minimum_required(VERSION 3.22)
project(permatracks)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 23)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)

### Ceres
set(USE_CUDA OFF)
set(PROVIDE_UNINSTALL_TARGET OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_TESTING OFF)
set(BUILD_BENCHMARKS OFF)
set(BUILD_SHARED_LIBS ON)
set(MINIGLOG ON)
set(MINIGLOG_MAX_LOG_LEVEL 0)
add_subdirectory(thirdparty/ceres)

### HELLOIMGUI
set(HELLOIMGUI_USE_GLFW3 ON CACHE BOOL "" FORCE)
set(HELLOIMGUI_HAS_OPENGL3 ON CACHE BOOL "" FORCE)
set(HELLOIMGUI_MACOS_NO_BUNDLE ON CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/helloimgui)

### ImGuizmo
project(imguizmo)

add_library(${PROJECT_NAME} thirdparty/imguizmo/ImGuizmo.cpp thirdparty/imguizmo/GraphEditor.cpp thirdparty/imguizmo/ImCurveEdit.cpp thirdparty/imguizmo/ImGradient.cpp thirdparty/imguizmo/ImSequencer.cpp)
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

### Sophus
project(sophus)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} INTERFACE thirdparty/sophus/sophus)
target_link_libraries(${PROJECT_NAME} INTERFACE ceres)

set(SOPHUS_HEADER_FILES
        sophus/average.hpp
        sophus/cartesian.hpp
        sophus/ceres_manifold.hpp
        sophus/ceres_typetraits.hpp
        sophus/common.hpp
        sophus/geometry.hpp
        sophus/interpolate.hpp
        sophus/interpolate_details.hpp
        sophus/num_diff.hpp
        sophus/rotation_matrix.hpp
        sophus/rxso2.hpp
        sophus/rxso3.hpp
        sophus/se2.hpp
        sophus/se3.hpp
        sophus/sim2.hpp
        sophus/sim3.hpp
        sophus/sim_details.hpp
        sophus/so2.hpp
        sophus/so3.hpp
        sophus/spline.hpp
        sophus/types.hpp)


### add local libraries
add_subdirectory(list_serial_devices)
add_subdirectory(ring_buffer)
add_subdirectory(msg)
add_subdirectory(calibration)

### GUI
project(gui)

add_executable(${PROJECT_NAME} main.cpp
        src/App.cpp src/MainTabBar.cpp
        src/SerialConnection.cpp src/ConnectionTab.cpp
        src/CalibrationTab.cpp src/Calibration.cpp
        src/ZeroingTab.cpp src/Zeroing.cpp
        src/MagnetSelectionTab.cpp src/MagnetSelection.cpp
        src/TrackingTab.cpp src/CeresOptimizerDirectionVector.cpp src/dipol_model_with_direction_vector.cpp
        src/MiMedMagnetometerArraySerialConnectionBinary.cpp src/ERR.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC implot)
target_link_libraries(${PROJECT_NAME} PUBLIC implot3d)
target_link_libraries(${PROJECT_NAME} PUBLIC imguizmo)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC list_serial_devices)
target_link_libraries(${PROJECT_NAME} PUBLIC ring_buffer)
target_link_libraries(${PROJECT_NAME} PUBLIC msg)
target_link_libraries(${PROJECT_NAME} PUBLIC TBB::tbb)
target_link_libraries(${PROJECT_NAME} PUBLIC calibration)
target_link_libraries(${PROJECT_NAME} PUBLIC ceres)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PUBLIC sophus)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

project(test_magnet)

add_executable(${PROJECT_NAME} test/test_magnet.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC imguizmo)
target_link_libraries(${PROJECT_NAME} PUBLIC glm::glm)
target_link_libraries(${PROJECT_NAME} PUBLIC GLEW::GLEW)
#target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL::GL)
target_link_libraries(${PROJECT_NAME} PUBLIC assimp::assimp)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")