set(TEST_TYPE "INTEGRATION")

set(tests
  camera_plugin.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
target_link_libraries(INTEGRATION_camera_plugin
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PROJECT_LIBRARY_TARGET_NAME}-camera)

