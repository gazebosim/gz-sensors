set (sources
  Manager.cc
  Sensor.cc
  CameraConfig.cc
  Noise.cc
  GaussianNoiseModel.cc
  SensorFactory.cc
  SensorTypes.cc
)

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${SDFormat_LIBRARIES}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::loader
  )

set(rendering_sensor_sources RenderingSensor.cc RenderingEvents.cc)
ign_add_component(rendering_sensor SOURCES ${rendering_sensor_sources} GET_TARGET_NAME rendering_sensor_target)
target_link_libraries(${rendering_sensor_target}
  PUBLIC
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    )


set(camera_sources CameraSensor.cc)
ign_add_component(camera SOURCES ${camera_sources} GET_TARGET_NAME camera_target)
# custom compile definitions since the one provided automatically is versioned and will
# make the code need to change with every major version
target_link_libraries(${camera_target}
  PUBLIC
    ${rendering_sensor_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-plugin${IGN_PLUGIN_VER}::register
    )

set(depth_camera_sources DepthCameraSensor.cc)
ign_add_component(depth_camera SOURCES ${depth_camera_sources} GET_TARGET_NAME depth_camera_target)
target_link_libraries(${depth_camera_target}
  PRIVATE
    ${camera_target}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-plugin${IGN_PLUGIN_VER}::register
    )

set(lidar_sources Lidar.cc)
ign_add_component(lidar SOURCES ${lidar_sources} GET_TARGET_NAME lidar_target)
target_link_libraries(${lidar_target}
  PUBLIC
    ${rendering_sensor_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-plugin${IGN_PLUGIN_VER}::register
    )

set(gpu_lidar_sources GpuLidarSensor.cc)
ign_add_component(gpu_lidar SOURCES ${gpu_lidar_sources} GET_TARGET_NAME gpu_lidar_target)
target_link_libraries(${gpu_lidar_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ${lidar_target}
    ignition-plugin${IGN_PLUGIN_VER}::register
    )

set(logical_camera_sources LogicalCameraSensor.cc)
ign_add_component(logical_camera SOURCES ${logical_camera_sources} GET_TARGET_NAME logical_camera_target)
target_link_libraries(${logical_camera_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-plugin${IGN_PLUGIN_VER}::register
    )

set (gtest_sources
#  Manager_TEST.cc
  Noise_TEST.cc
)

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Build the unit tests that depend on components.
ign_build_tests(TYPE UNIT SOURCES Lidar_TEST.cc LIB_DEPS ${lidar_target})

#ign_build_tests(TYPE UNIT SOURCES Camera_TEST.cc LIB_DEPS ${camera_target})
#
#add_executable(abc abc.test.cc)
#target_link_libraries(abc ${camera_target} ${PROJECT_LIBRARY_TARGET_NAME} gtest gtest_main)
#
#    target_include_directories(abc
#      PRIVATE
#        ${PROJECT_SOURCE_DIR}
#        ${PROJECT_BINARY_DIR}
#    )
#
#
#    ign_build_executables(
#      PREFIX "my_"
#      SOURCES abc.test.cc
#      LIB_DEPS gtest gtest_main ${camera_target}
##      INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/test/gtest/include ${ign_build_tests_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/build
#      EXEC_LIST test_list)

