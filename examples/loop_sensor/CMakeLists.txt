cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(loop_sensor)

find_package(ignition-sensors6 REQUIRED
  # Find built-in sensors
  COMPONENTS
    altimeter
)

# A simple example of how to find custom sensors
add_subdirectory(../custom_sensor odometer)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ignition-sensors6::ignition-sensors6

  # Link to custom sensors
  odometer

  # Link to built-in sensors
  ignition-sensors6::altimeter)
