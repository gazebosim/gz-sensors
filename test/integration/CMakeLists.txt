set(TEST_TYPE "INTEGRATION")

set(dri_tests
  camera_plugin.cc
  depth_camera_plugin.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

if (DRI_TESTS)
  ign_build_tests(TYPE INTEGRATION SOURCES ${dri_tests})

  target_link_libraries(INTEGRATION_camera_plugin
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${PROJECT_LIBRARY_TARGET_NAME}-camera)

  target_link_libraries(INTEGRATION_depth_camera_plugin
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${PROJECT_LIBRARY_TARGET_NAME}-camera
    ${PROJECT_LIBRARY_TARGET_NAME}-depth_camera)
endif()
