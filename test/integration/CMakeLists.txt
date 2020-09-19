set(TEST_TYPE "INTEGRATION")

set(dri_tests
  camera_plugin.cc
  depth_camera_plugin.cc
  gpu_lidar_sensor_plugin.cc
  rgbd_camera_plugin.cc
  thermal_camera_plugin.cc
)

set(tests
  air_pressure_plugin.cc
  altimeter_plugin.cc
  logical_camera_plugin.cc
  magnetometer_plugin.cc
  imu_plugin.cc
)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(
      ${tests}
      ${dri_tests}
    COMPILE_FLAGS "/wd4251 /wd4146"
  )
endif()

link_directories(${PROJECT_BINARY_DIR}/test)

include_directories(${PROJECT_SOURCE_DIR}/src)

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)
endif()

if (DRI_TESTS)
  ign_build_tests(TYPE INTEGRATION
    SOURCES
      ${dri_tests}
    LIB_DEPS
      ${IGNITION-TRANSPORT_LIBRARIES}
      ${PROJECT_LIBRARY_TARGET_NAME}-depth_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-camera
      ${PROJECT_LIBRARY_TARGET_NAME}-lidar
      ${PROJECT_LIBRARY_TARGET_NAME}-gpu_lidar
      ${PROJECT_LIBRARY_TARGET_NAME}-rgbd_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-thermal_camera
  )
endif()

ign_build_tests(TYPE INTEGRATION
  SOURCES
    ${tests}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}-air_pressure
    ${PROJECT_LIBRARY_TARGET_NAME}-altimeter
    ${PROJECT_LIBRARY_TARGET_NAME}-logical_camera
    ${PROJECT_LIBRARY_TARGET_NAME}-magnetometer
    ${PROJECT_LIBRARY_TARGET_NAME}-imu
)
