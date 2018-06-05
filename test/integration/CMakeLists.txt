set(TEST_TYPE "INTEGRATION")

set(dri_tests
  camera_plugin.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

if (DRI_TESTS)
  ign_build_tests(TYPE INTEGRATION SOURCES ${dri_tests})
  target_link_libraries(INTEGRATION_camera_plugin
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${PROJECT_LIBRARY_TARGET_NAME}-camera)
endif()
