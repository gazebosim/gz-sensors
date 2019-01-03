set (sources
  Events.cc
  Manager.cc
  Sensor.cc
  Lidar.cc
  CameraConfig.cc
  Noise.cc
  GaussianNoiseModel.cc
  SensorTypes.cc
)

set (gtest_sources
  Manager_TEST.cc
  Lidar_TEST.cc
  Noise_TEST.cc
)

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${SDFormat_LIBRARIES})

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

set(camera_sources CameraSensor.cc)
ign_add_component(camera SOURCES ${camera_sources} GET_TARGET_NAME camera_target)
target_link_libraries(${camera_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(depth_camera_sources DepthCameraSensor.cc)
ign_add_component(depth_camera SOURCES ${depth_camera_sources} GET_TARGET_NAME depth_camera_target)
target_link_libraries(${depth_camera_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(lidar_sources Lidar.cc)
ign_add_component(lidar SOURCES ${lidar_sources} GET_TARGET_NAME lidar_target)
target_link_libraries(${lidar_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(gpu_lidar_sources GpuLidarSensor.cc)
ign_add_component(gpu_lidar SOURCES ${gpu_lidar_sources} GET_TARGET_NAME gpu_lidar_target)
target_link_libraries(${gpu_lidar_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${lidar_target}
    )
