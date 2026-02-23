project(utils)

add_library(${PROJECT_NAME} SHARED src/AfterReturnTimeMeasure.cpp src/utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC common)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")