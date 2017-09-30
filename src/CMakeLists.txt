
set (sources
  CameraConfig.cc
  Manager.cc
  Sensor.cc
)

set (gtest_sources
  Manager_TEST.cc
)

ign_create_main_library(${sources})

set_target_properties(${PROJECT_LIBRARY_TARGET_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQURED YES)

# Generate a header with info about the build
configure_file (build_config.hh.in ${PROJECT_BINARY_DIR}/build_config.hh)

# Headers that are only used internally (non-installed headers or .cc files)
target_include_directories(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE
  ${PROJECT_BINARY_DIR}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS})

# Headers that are used in installed headers
target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>
         ${IGNITION-MATH_INCLUDE_DIRS}
         ${IGNITION-RENDERING_INCLUDE_DIRS}
         ${IGNITION-MSGS_INCLUDE_DIRS}
         ${SDFormat_INCLUDE_DIRS})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC
  ignition-common0::ignition-common0
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${SDFormat_LIBRARIES})


add_subdirectory(camera)

ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})
