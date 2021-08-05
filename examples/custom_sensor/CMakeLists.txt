cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(double_sensor)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-sensors6 REQUIRED)

add_library(${PROJECT_NAME} SHARED DoubleSensor.cc)
target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-sensors6::ignition-sensors6)
