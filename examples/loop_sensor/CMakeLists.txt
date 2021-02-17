cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(loop_sensor)

find_package(ignition-sensors5 REQUIRED)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ignition-sensors5::ignition-sensors5)
