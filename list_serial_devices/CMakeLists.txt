project(list_serial_devices)

find_library(IOKIT_LIBRARY IOKit REQUIRED)
find_library(FOUNDATION_LIBRARY Foundation REQUIRED)

if (APPLE)
    message(STATUS "Configuring for macOS")
    add_library(${PROJECT_NAME} src/list_serial_devices_macos.cpp)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif (UNIX)
    message(STATUS "Configuring for Linux")
    add_library(${PROJECT_NAME} src/list_serial_devices_linux.cpp)
else ()
    message(FATAL_ERROR "Unsupported platform!")
endif ()
target_include_directories(${PROJECT_NAME} PUBLIC include)

add_executable(test_${PROJECT_NAME} test/main.cpp)
target_link_libraries(test_${PROJECT_NAME} PUBLIC ${PROJECT_NAME})
