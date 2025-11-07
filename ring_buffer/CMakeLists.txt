project(ring_buffer)

add_library(${PROJECT_NAME} src/ring_buffer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)