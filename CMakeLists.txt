cmake_minimum_required(VERSION 3.22)
project(permatracks)

set(CMAKE_CXX_STANDARD 23)

project(gui)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
#find_package(Assimp REQUIRED)
find_package(OpenCASCADE REQUIRED)

set(HELLOIMGUI_USE_GLFW3 ON CACHE BOOL "" FORCE)
set(HELLOIMGUI_HAS_OPENGL3 ON CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/helloimgui)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

project(minimal)

add_executable(${PROJECT_NAME} test/minimal.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_library(ImGuizmo thirdparty/imguizmo/ImGuizmo.cpp)
target_link_libraries(ImGuizmo PUBLIC imgui)
target_include_directories(ImGuizmo PUBLIC thirdparty/imguizmo)

project(minimal_3d)
add_executable(${PROJECT_NAME} test/minimal_3d.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC ImGuizmo)
target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL::GL)
target_link_libraries(${PROJECT_NAME} PUBLIC GLEW::GLEW)
target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

project(minimal_3d_load)
add_executable(${PROJECT_NAME} test/minimal_3d_load.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC hello_imgui)
target_link_libraries(${PROJECT_NAME} PUBLIC ImGuizmo)
target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL::GL)
target_link_libraries(${PROJECT_NAME} PUBLIC GLEW::GLEW)
target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
#target_link_libraries(${PROJECT_NAME} PUBLIC assimp::assimp)
target_link_libraries(${PROJECT_NAME} PUBLIC TKernel TKXSBase TKService TKV3d TKOpenGl TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKTopAlgo TKPrim TKDE TKDESTEP)

target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")