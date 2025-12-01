cmake_minimum_required(VERSION 3.5)
project(motor_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Build executable
add_executable(motor_test
    src/main.cpp
    src/motor_control.cpp
)

# For Jetson Nano (needed if linking raw sockets)
target_link_libraries(motor_test pthread)
