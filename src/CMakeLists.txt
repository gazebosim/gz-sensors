set (sources
  CameraConfig.cc
  Events.cc
  Manager.cc
  Sensor.cc
  DepthCameraSensor.cc
  Noise.cc
  GaussianNoiseModel.cc
  SensorTypes.cc
)

set (gtest_sources
  Manager_TEST.cc
  Noise_TEST.cc
)

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)

# Include the interface directories that we always need.
#ign_target_interface_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
#  ignition-math${IGN_MATH_VER})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ${ignition-math${IGN_MATH_VER}_LIBRARIES}
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES}
    ${IGNITION-RENDERING_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${SDFormat_LIBRARIES})

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

add_library(${PROJECT_LIBRARY_TARGET_NAME}-camera SHARED CameraSensor.cc)
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}-camera PUBLIC
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PROJECT_LIBRARY_TARGET_NAME})
install(TARGETS ${PROJECT_LIBRARY_TARGET_NAME}-camera
  DESTINATION ${IGN_SENSORS_PLUGIN_PATH})

add_library(${PROJECT_LIBRARY_TARGET_NAME}-depth-camera SHARED DepthCameraSensor.cc)
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}-depth-camera PUBLIC
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PROJECT_LIBRARY_TARGET_NAME})
install(TARGETS ${PROJECT_LIBRARY_TARGET_NAME}-depth-camera
  DESTINATION ${IGN_SENSORS_PLUGIN_PATH})
