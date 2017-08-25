include (${project_cmake_dir}/Utils.cmake)
add_subdirectory(camera)

set (sources
  CameraUtils.cc
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

# Allow including other files in src
target_include_directories(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE .)

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>
         ${IGNITION-MATH_INCLUDE_DIRS}
         ${SDFormat_INCLUDE_DIRS})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${SDFormat_LIBRARIES})

# Link against header target to get generated header files
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE ${PROJECT_LIBRARY_TARGET_NAME}_headers)

ign_build_tests(${gtest_sources})
ign_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})
