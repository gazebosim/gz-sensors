set(TEST_TYPE "INTEGRATION")

set(dri_tests
  camera_plugin.cc
  depth_camera_plugin.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)
endif()

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
