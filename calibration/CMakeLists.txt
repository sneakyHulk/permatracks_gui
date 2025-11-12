project(calibration)

add_library(${PROJECT_NAME} src/ellipsoid_fit.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC include)