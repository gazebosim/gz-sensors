cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

# Must follow the pattern "ignition-sensors<MAJOR_VERSION>-<SENSOR_TYPE>"
project(ignition-sensors6-custom_sensor)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-sensors6 REQUIRED)

add_library(${PROJECT_NAME} SHARED CustomSensor.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-plugin1::ignition-plugin1
  PRIVATE ignition-sensors6::ignition-sensors6)
