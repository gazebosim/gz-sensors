cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition Libraries used directly by the example
find_package(ignition-sensors0 REQUIRED)

add_executable(save_image main.cc)
target_include_directories(save_image PUBLIC
  ${IGNITION-SENSORS_INCLUDE_DIRS})
target_link_libraries(save_image PUBLIC
  ${IGNITION-SENSORS_LIBRARIES})
