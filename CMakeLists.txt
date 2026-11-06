cmake_minimum_required(VERSION 3.16)
project(rm_autoaim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d dnn)

add_library(autoaim_core STATIC
    src/config.cpp
    src/detector.cpp
    src/pose.cpp
    src/tracker.cpp
    src/target.cpp
    src/ballistics.cpp
    src/aimer.cpp
    src/sentry.cpp
    src/serial_protocol.cpp
    src/evaluation.cpp
    src/draw.cpp)
target_include_directories(autoaim_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(autoaim_core PUBLIC ${OpenCV_LIBS})
target_compile_definitions(autoaim_core PUBLIC AUTOAIM_ROOT="${CMAKE_CURRENT_SOURCE_DIR}" _USE_MATH_DEFINES
                           _CRT_SECURE_NO_WARNINGS)
if (UNIX AND NOT APPLE)
    target_sources(autoaim_core PRIVATE src/io/serial_port.cpp)
endif()
if (MSVC)
    target_compile_options(autoaim_core PUBLIC /utf-8 /W3)
else()
    target_compile_options(autoaim_core PUBLIC -Wall -Wextra)
endif()

add_executable(autoaim_video apps/autoaim_video.cpp)
target_link_libraries(autoaim_video PRIVATE autoaim_core)

add_executable(autoaim_sim apps/autoaim_sim.cpp)
target_link_libraries(autoaim_sim PRIVATE autoaim_core)

add_executable(autoaim_lights apps/autoaim_lights.cpp)
target_link_libraries(autoaim_lights PRIVATE autoaim_core)

add_executable(sentry_offline apps/sentry_offline.cpp)
target_link_libraries(sentry_offline PRIVATE autoaim_core)

add_executable(detector_bench apps/detector_bench.cpp)
target_link_libraries(detector_bench PRIVATE autoaim_core)

if (UNIX AND NOT APPLE)
    add_executable(sentry_robot apps/sentry_robot.cpp)
    target_link_libraries(sentry_robot PRIVATE autoaim_core)
endif()

add_executable(autoaim_tests tests/test_main.cpp)
target_link_libraries(autoaim_tests PRIVATE autoaim_core)

add_executable(sentry_tests tests/test_sentry.cpp)
target_link_libraries(sentry_tests PRIVATE autoaim_core)

enable_testing()
add_test(NAME autoaim_tests COMMAND autoaim_tests)
add_test(NAME sentry_tests COMMAND sentry_tests)
