cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(loop_sensor)

find_package(gz-sensors9 REQUIRED
  # Find built-in sensors
  COMPONENTS
    altimeter
)

# A simple example of how to find custom sensors
add_subdirectory(../custom_sensor odometer)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC
  gz-sensors9::gz-sensors9

  # Link to custom sensors
  odometer

  # Link to built-in sensors
  gz-sensors9::altimeter)
