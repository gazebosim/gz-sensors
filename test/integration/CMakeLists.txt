set(TEST_TYPE "INTEGRATION")

set(tests
  camera_plugin.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(${tests})

target_link_libraries(INTEGRATION_camera_plugin ${IGNITION-TRANSPORT_LIBRARIES})
set_tests_properties(INTEGRATION_camera_plugin PROPERTIES TIMEOUT 60)
