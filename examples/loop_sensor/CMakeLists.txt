cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(loop_sensor)

find_package(gz-sensors7 REQUIRED
  # Find built-in sensors
  COMPONENTS
    altimeter
)

# A simple example of how to find custom sensors
if (NOT TARGET odometer)
  add_subdirectory(../custom_sensor odometer)
endif()

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC
  gz-sensors7::gz-sensors7

  # Link to custom sensors
  odometer

  # Link to built-in sensors
  gz-sensors7::altimeter)
