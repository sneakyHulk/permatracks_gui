project(serial_connection)

if (APPLE)
    message(STATUS "Configuring for macOS")

    add_library(${PROJECT_NAME} src/SerialConnectionUnix.cpp)
elseif (WIN32)
    message(STATUS "Configuring for Windows")

    add_library(${PROJECT_NAME} src/SerialConnectionWindows.cpp)
elseif (UNIX)
    message(STATUS "Configuring for Linux")
    add_library(${PROJECT_NAME} src/SerialConnectionUnix.cpp)
else ()
    message(FATAL_ERROR "Unsupported platform!")
endif ()
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC utils)

add_executable(test_${PROJECT_NAME} test/main.cpp)
target_link_libraries(test_${PROJECT_NAME} PUBLIC ${PROJECT_NAME})

project(serial_connection_boost)

add_library(${PROJECT_NAME} src/SerialConnectionBoost.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC utils)