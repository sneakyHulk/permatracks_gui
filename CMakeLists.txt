cmake_minimum_required(VERSION 3.22)
project(permatracks)

include(FetchContent)

set(CMAKE_CXX_STANDARD 23)


set(ENV{PYTORCH_ROCM_ARCH} "gfx1030")
message(STATUS "Forcing ROCm arch: ${PYTORCH_ROCM_ARCH}")


### add local libraries
FetchContent_Declare(permatracks_data_collection SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/data_collection)
FetchContent_MakeAvailable(permatracks_data_collection)
add_library(permatracks_data_collection_${PROJECT_NAME} ALIAS permatracks_data_collection)
set_target_properties(permatracks_data_collection PROPERTIES EXCLUDE_FROM_ALL TRUE)

FetchContent_Declare(permatracks_msg SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/msg)
FetchContent_MakeAvailable(permatracks_msg)
add_library(permatracks_msg_${PROJECT_NAME} ALIAS permatracks_msg)
set_target_properties(permatracks_msg PROPERTIES EXCLUDE_FROM_ALL TRUE)

FetchContent_Declare(common SOURCE_DIR ${PROJECT_SOURCE_DIR}/external/common)
FetchContent_MakeAvailable(common)
add_library(common_${PROJECT_NAME} ALIAS common)
set_target_properties(common PROPERTIES EXCLUDE_FROM_ALL TRUE)


set(SHOWCASE false)
message(SHOWCASE: ${SHOWCASE})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(assimp REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)
find_package(GLOG REQUIRED)
find_package(nlohmann_json REQUIRED)

### Ceres
# set(USE_CUDA OFF CACHE BOOL "" FORCE)
# set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
# set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
# set(MINIGLOG ON CACHE BOOL "" FORCE)
# set(MINIGLOG_MAX_LOG_LEVEL 0 CACHE BOOL "" FORCE)
# set(CERES_NO_GLOG ON CACHE BOOL "" FORCE)
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

### ImGuiFileDialog
project(imfiledialog)
add_library(${PROJECT_NAME} thirdparty/imguifiledialog/ImGuiFileDialog.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC thirdparty/imguifiledialog)
target_link_libraries(${PROJECT_NAME} PUBLIC imgui)

### Sophus
project(sophus)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} INTERFACE thirdparty/sophus/sophus)
target_link_libraries(${PROJECT_NAME} INTERFACE Ceres::ceres)

### LibTorch
project(ml LANGUAGES C CXX)

find_package(Torch REQUIRED)

add_library(${PROJECT_NAME} src/MLOptimizer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${TORCH_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${TORCH_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC permatracks_msg)
target_compile_options(${PROJECT_NAME} PRIVATE ${TORCH_CXX_FLAGS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE glog::glog)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

project(test_torch)

add_executable(${PROJECT_NAME} test/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ml)
target_link_libraries(${PROJECT_NAME} PRIVATE glog::glog)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")


add_subdirectory(list_serial_devices)
add_subdirectory(calibration)
add_subdirectory(utils)

### GUI
project(gui)

add_executable(${PROJECT_NAME} main.cpp
        src/App.cpp
        src/MainTabBar.cpp
        src/ConnectionTab.cpp
        src/CalibrationTab.cpp
        src/Calibration.cpp
        src/ZeroingTab.cpp
        src/Zeroing.cpp
        src/MagnetSelectionTab.cpp
        src/MagnetSelection.cpp
        src/TrackingTab.cpp
        src/CeresOptimizerDirectionVector.cpp
        src/dipol_model_with_direction_vector_25.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC implot)
target_link_libraries(${PROJECT_NAME} PUBLIC implot3d)
target_link_libraries(${PROJECT_NAME} PUBLIC imguizmo)
target_link_libraries(${PROJECT_NAME} PUBLIC imfiledialog)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC list_serial_devices)
target_link_libraries(${PROJECT_NAME} PUBLIC permatracks_msg)
target_link_libraries(${PROJECT_NAME} PUBLIC permatracks_data_collection)
target_link_libraries(${PROJECT_NAME} PUBLIC TBB::tbb)
target_link_libraries(${PROJECT_NAME} PUBLIC calibration)
target_link_libraries(${PROJECT_NAME} PUBLIC ceres)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PUBLIC sophus)
target_link_libraries(${PROJECT_NAME} PRIVATE ml)
target_link_libraries(${PROJECT_NAME} PRIVATE glog::glog)
target_link_libraries(${PROJECT_NAME} PUBLIC utils)
target_link_libraries(${PROJECT_NAME} PUBLIC common)
target_link_libraries(${PROJECT_NAME} PUBLIC eigen_utils)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}" SHOWCASE=${SHOWCASE})
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

project(test_single_producer_single_consumer)

add_executable(${PROJECT_NAME} test/test_single_producer_single_consumer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")