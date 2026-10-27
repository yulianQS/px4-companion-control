cmake_minimum_required(VERSION 3.16)
project(drone_control LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
add_executable(drone_service
    main.cpp px4.cpp flight_log.cpp controller.cpp
    control/commands.cpp control/safety.cpp control/joystick.cpp
    control/trajectory.cpp control/update.cpp control/takeoff.cpp control/vision.cpp control/mission.cpp
)
target_include_directories(drone_service PRIVATE vendor ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(drone_service PRIVATE -Wall -Wextra -Wno-address-of-packed-member)
target_link_libraries(drone_service PRIVATE Threads::Threads)
enable_testing()
add_test(NAME controller_math COMMAND drone_service --self-test)
