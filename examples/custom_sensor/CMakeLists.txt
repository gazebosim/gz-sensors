cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(custom_sensor)

find_package(ignition-sensors6 REQUIRED)

add_library(${PROJECT_NAME} SHARED CustomSensor.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-sensors6::ignition-sensors6)
