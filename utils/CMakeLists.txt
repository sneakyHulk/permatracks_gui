project(utils)

add_library(${PROJECT_NAME} SHARED src/AfterReturnTimeMeasure.cpp src/linspace.cpp src/utils.cpp src/ERROR.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC common)
target_compile_definitions(${PROJECT_NAME} PUBLIC CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

project(concurra_utils)

add_library(${PROJECT_NAME} src/Averager.cpp src/MessagePrinter.cpp src/MagnetometerArrayMagneticFluxDensityDataAverager.cpp src/UpdaterMagnetometerArrayMagneticFluxDensityDataToMagnetometerArrayMagneticFluxDensityDataWithTruePositionDirection.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC msg)
target_link_libraries(${PROJECT_NAME} PUBLIC concurra)
target_link_libraries(${PROJECT_NAME} PUBLIC common)

project(eigen_utils)

add_library(${PROJECT_NAME} src/EigenJsonUtils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

project(test_eigen_utils)

add_executable(${PROJECT_NAME} test/test_eigen_utils.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC eigen_utils)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

project(test_averager)

add_executable(${PROJECT_NAME} test/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC concurra_utils)
target_compile_definitions(${PROJECT_NAME} PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
