set(TEST_TYPE "INTEGRATION")

set(dri_tests
  camera_plugin.cc
  depth_camera_plugin.cc
  gpu_lidar_sensor_plugin.cc
  logical_camera_plugin.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

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
      ${PROJECT_LIBRARY_TARGET_NAME}-depth_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-camera
      ${PROJECT_LIBRARY_TARGET_NAME}-lidar
      ${PROJECT_LIBRARY_TARGET_NAME}-logical_camera
      ${PROJECT_LIBRARY_TARGET_NAME}-gpu_lidar
  )
endif()

# ign_build_tests(TYPE INTEGRATION
#   SOURCES
#     camera_plugin.cc
#   LIB_DEPS
#      ${IGNITION-TRANSPORT_LIBRARIES}
# #     ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
# #     ${camera_target}
#      ${PROJECT_LIBRARY_TARGET_NAME}-camera
# )
#
# ign_build_tests(TYPE INTEGRATION
#   SOURCES
#     depth_camera_plugin.cc
#   LIB_DEPS
#      ${IGNITION-TRANSPORT_LIBRARIES}
#      ${PROJECT_LIBRARY_TARGET_NAME}-depth_camera
# )
#
# ign_build_tests(TYPE INTEGRATION
#   SOURCES
#     gpu_lidar_sensor_plugin.cc
#   LIB_DEPS
#      ${IGNITION-TRANSPORT_LIBRARIES}
#      ${PROJECT_LIBRARY_TARGET_NAME}-lidar
#      ${PROJECT_LIBRARY_TARGET_NAME}-gpu_lidar
# )
