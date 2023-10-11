cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(gz-sensors-noise-demo)

# Find the Gazebo Libraries used directly by the example
find_package(gz-sensors9 REQUIRED)

add_executable(sensor_noise main.cc)
target_link_libraries(sensor_noise PUBLIC gz-sensors9)
