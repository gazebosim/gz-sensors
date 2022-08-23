set(TEST_TYPE "INTEGRATION")

set(dri_tests
  boundingbox_camera.cc
  camera.cc
  depth_camera.cc
  distortion_camera.cc
  gpu_lidar_sensor.cc
  rgbd_camera.cc
  segmentation_camera.cc
  thermal_camera.cc
  triggered_camera.cc
  wide_angle_camera.cc
)

set(tests
  air_pressure.cc
  altimeter.cc
  force_torque.cc
  logical_camera.cc
  magnetometer.cc
  navsat.cc
  imu.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

include_directories(${PROJECT_SOURCE_DIR}/src)

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)
endif()

if (DRI_TESTS)
  gz_build_tests(TYPE INTEGRATION
    SOURCES
      ${dri_tests}
    LIB_DEPS
      ${GZ-TRANSPORT_LIBRARIES}
      ${PROJECT_LIBRARY_TARGET_NAME}-boundingbox_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-depth_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-camera
      ${PROJECT_LIBRARY_TARGET_NAME}-lidar
      ${PROJECT_LIBRARY_TARGET_NAME}-gpu_lidar
      ${PROJECT_LIBRARY_TARGET_NAME}-rgbd_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-thermal_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-segmentation_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-wide_angle_camera
  )
endif()

gz_build_tests(TYPE INTEGRATION
  SOURCES
    ${tests}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}-air_pressure
    ${PROJECT_LIBRARY_TARGET_NAME}-altimeter
    ${PROJECT_LIBRARY_TARGET_NAME}-logical_camera
    ${PROJECT_LIBRARY_TARGET_NAME}-magnetometer
    ${PROJECT_LIBRARY_TARGET_NAME}-imu
    ${PROJECT_LIBRARY_TARGET_NAME}-force_torque
    ${PROJECT_LIBRARY_TARGET_NAME}-navsat
)

