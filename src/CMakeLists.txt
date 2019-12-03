set (sources
  Manager.cc
  Sensor.cc
  Noise.cc
  GaussianNoiseModel.cc
  PointCloudUtil.cc
  SensorFactory.cc
  SensorTypes.cc
)

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 14)

# todo(anyone) dependency on rendering has been moved to its own component
# and relevant functions are deprecated in ign-sensors 2. We can remove linking
# against ign-rendering from the core lib in ign-sensors 3
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${SDFormat_LIBRARIES}
  PRIVATE
    ignition-common${IGN_COMMON_VER}::profiler
)

set(rendering_sources
  RenderingSensor.cc
  RenderingEvents.cc
  ImageGaussianNoiseModel.cc
  ImageNoise.cc
)

ign_add_component(rendering SOURCES ${rendering_sources} GET_TARGET_NAME rendering_target)
target_link_libraries(${rendering_target}
  PUBLIC
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    )


set(camera_sources CameraSensor.cc)
ign_add_component(camera SOURCES ${camera_sources} GET_TARGET_NAME camera_target)
# custom compile definitions since the one provided automatically is versioned and will
# make the code need to change with every major version
target_compile_definitions(${camera_target} PUBLIC CameraSensor_EXPORTS)
target_link_libraries(${camera_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(depth_camera_sources DepthCameraSensor.cc)
ign_add_component(depth_camera SOURCES ${depth_camera_sources} GET_TARGET_NAME depth_camera_target)
target_compile_definitions(${depth_camera_target} PUBLIC DepthCameraSensor_EXPORTS)
target_link_libraries(${depth_camera_target}
  PRIVATE
    ${camera_target}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(lidar_sources Lidar.cc)
ign_add_component(lidar SOURCES ${lidar_sources} GET_TARGET_NAME lidar_target)
target_compile_definitions(${lidar_target} PUBLIC Lidar_EXPORTS)
target_link_libraries(${lidar_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(gpu_lidar_sources GpuLidarSensor.cc)
ign_add_component(gpu_lidar SOURCES ${gpu_lidar_sources} GET_TARGET_NAME gpu_lidar_target)
target_compile_definitions(${gpu_lidar_target} PUBLIC GpuLidarSensor_EXPORTS)
target_link_libraries(${gpu_lidar_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${lidar_target}
    )

set(logical_camera_sources LogicalCameraSensor.cc)
ign_add_component(logical_camera SOURCES ${logical_camera_sources} GET_TARGET_NAME logical_camera_target)
target_compile_definitions(${logical_camera_target} PUBLIC LogicalCameraSensor_EXPORTS)
target_link_libraries(${logical_camera_target}
  PRIVATE
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(magnetometer_sources MagnetometerSensor.cc)
ign_add_component(magnetometer SOURCES ${magnetometer_sources} GET_TARGET_NAME magnetometer_target)

set(imu_sources ImuSensor.cc)
ign_add_component(imu SOURCES ${imu_sources} GET_TARGET_NAME imu_target)

set(altimeter_sources AltimeterSensor.cc)
ign_add_component(altimeter SOURCES ${altimeter_sources} GET_TARGET_NAME altimeter_target)

set(air_pressure_sources AirPressureSensor.cc)
ign_add_component(air_pressure SOURCES ${air_pressure_sources} GET_TARGET_NAME air_pressure_target)

set(rgbd_camera_sources RgbdCameraSensor.cc)
ign_add_component(rgbd_camera SOURCES ${rgbd_camera_sources} GET_TARGET_NAME rgbd_camera_target)
target_compile_definitions(${rgbd_camera_target} PUBLIC RgbdCameraSensor_EXPORTS)
target_link_libraries(${rgbd_camera_target}
  PRIVATE
    ${camera_target}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )

set(thermal_camera_sources ThermalCameraSensor.cc)
ign_add_component(thermal_camera SOURCES ${thermal_camera_sources} GET_TARGET_NAME thermal_camera_target)
target_link_libraries(${thermal_camera_target}
  PRIVATE
    ${camera_target}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )


set (gtest_sources
  Manager_TEST.cc
  Noise_TEST.cc
  Sensor_TEST.cc
)

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources} LIB_DEPS ${rendering_target})

# Build the unit tests that depend on components.
ign_build_tests(TYPE UNIT SOURCES Lidar_TEST.cc LIB_DEPS ${lidar_target})
ign_build_tests(TYPE UNIT SOURCES Camera_TEST.cc LIB_DEPS ${camera_target})
ign_build_tests(TYPE UNIT SOURCES ImuSensor_TEST.cc LIB_DEPS ${imu_target})
