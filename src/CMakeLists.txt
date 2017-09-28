include (${project_cmake_dir}/Utils.cmake)
add_subdirectory(camera)

set (sources
  CameraConfig.cc
  Manager.cc
  Sensor.cc
)

set (gtest_sources
  Manager_TEST.cc
)

ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})

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
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${SDFormat_LIBRARIES})

# Link against dummy target to get generated header files
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE ${PROJECT_LIBRARY_TARGET_NAME}_headers)

ign_build_tests(${gtest_sources})
ign_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})
