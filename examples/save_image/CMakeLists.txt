cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition Libraries used directly by the example
find_package(ignition-common0 REQUIRED)
find_package(ignition-rendering0 REQUIRED)
find_package(ignition-sensors0 REQUIRED)

add_executable(save_image main.cc)
target_include_directories(save_image PUBLIC
  ${IGNITION-SENSORS_INCLUDE_DIRS}
  ${IGNITION-RENDERING_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS})
target_link_libraries(save_image PUBLIC
  ${IGNITION-SENSORS_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES})
