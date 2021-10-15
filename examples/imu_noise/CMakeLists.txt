cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ignition-sensors-noise-demo)

# Find the Ignition Libraries used directly by the example
find_package(ignition-sensors7 REQUIRED)

add_executable(sensor_noise main.cc)
target_link_libraries(sensor_noise PUBLIC ignition-sensors7)
