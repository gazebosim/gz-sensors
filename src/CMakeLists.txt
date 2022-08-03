set (sources
  Manager.cc
  Sensor.cc
  Noise.cc
  GaussianNoiseModel.cc
  Distortion.cc
  BrownDistortionModel.cc
  PointCloudUtil.cc
  SensorFactory.cc
  SensorTypes.cc
  Util.cc
)

set(rendering_sources
  RenderingSensor.cc
  RenderingEvents.cc
  ImageGaussianNoiseModel.cc
  ImageNoise.cc
  ImageBrownDistortionModel.cc
  ImageDistortion.cc
)

set (gtest_sources
  Manager_TEST.cc
  Noise_TEST.cc
  Sensor_TEST.cc
  Util_TEST.cc
)

if (MSVC)
  # TODO(louise) Remove this once warnings are suppressed in gz-rendering
  set_source_files_properties(
      ${rendering_sources}
      SegmentationCameraSensor.cc
    COMPILE_FLAGS "/wd4251"
  )
endif()

# Create the library target.

gz_create_core_library(SOURCES ${sources})
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    sdformat${SDF_VER}::sdformat${SDF_VER}
  PRIVATE
    gz-common${GZ_COMMON_VER}::profiler
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)
target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC DepthPoints_EXPORTS)

gz_add_component(rendering SOURCES ${rendering_sources} GET_TARGET_NAME rendering_target)
target_link_libraries(${rendering_target}
  PUBLIC
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
)

set(camera_sources CameraSensor.cc)
gz_add_component(camera SOURCES ${camera_sources} GET_TARGET_NAME camera_target)
# custom compile definitions since the one provided automatically is versioned and will
# make the code need to change with every major version
target_compile_definitions(${camera_target} PUBLIC CameraSensor_EXPORTS)
target_link_libraries(${camera_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(depth_camera_sources DepthCameraSensor.cc)
gz_add_component(depth_camera SOURCES ${depth_camera_sources} GET_TARGET_NAME depth_camera_target)
target_compile_definitions(${depth_camera_target} PUBLIC DepthCameraSensor_EXPORTS)
target_link_libraries(${depth_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(lidar_sources Lidar.cc)
gz_add_component(lidar SOURCES ${lidar_sources} GET_TARGET_NAME lidar_target)
target_compile_definitions(${lidar_target} PUBLIC Lidar_EXPORTS)
target_link_libraries(${lidar_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(gpu_lidar_sources GpuLidarSensor.cc)
gz_add_component(gpu_lidar SOURCES ${gpu_lidar_sources} GET_TARGET_NAME gpu_lidar_target)
target_compile_definitions(${gpu_lidar_target} PUBLIC GpuLidarSensor_EXPORTS)
target_link_libraries(${gpu_lidar_target}
  PRIVATE
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    ${lidar_target}
)

set(logical_camera_sources LogicalCameraSensor.cc)
gz_add_component(logical_camera SOURCES ${logical_camera_sources} GET_TARGET_NAME logical_camera_target)
target_compile_definitions(${logical_camera_target} PUBLIC LogicalCameraSensor_EXPORTS)
target_link_libraries(${logical_camera_target}
  PRIVATE
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(magnetometer_sources MagnetometerSensor.cc)
gz_add_component(magnetometer SOURCES ${magnetometer_sources} GET_TARGET_NAME magnetometer_target)

set(imu_sources ImuSensor.cc)
gz_add_component(imu SOURCES ${imu_sources} GET_TARGET_NAME imu_target)

set(force_torque_sources ForceTorqueSensor.cc)
gz_add_component(force_torque SOURCES ${force_torque_sources} GET_TARGET_NAME force_torque_target)

set(altimeter_sources AltimeterSensor.cc)
gz_add_component(altimeter SOURCES ${altimeter_sources} GET_TARGET_NAME altimeter_target)

set(air_pressure_sources AirPressureSensor.cc)
gz_add_component(air_pressure SOURCES ${air_pressure_sources} GET_TARGET_NAME air_pressure_target)

set(navsat_sources NavSatSensor.cc)
gz_add_component(navsat SOURCES ${navsat_sources} GET_TARGET_NAME navsat_target)

set(rgbd_camera_sources RgbdCameraSensor.cc)
gz_add_component(rgbd_camera SOURCES ${rgbd_camera_sources} GET_TARGET_NAME rgbd_camera_target)
target_compile_definitions(${rgbd_camera_target} PUBLIC RgbdCameraSensor_EXPORTS)
target_link_libraries(${rgbd_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(thermal_camera_sources ThermalCameraSensor.cc)
gz_add_component(thermal_camera SOURCES ${thermal_camera_sources} GET_TARGET_NAME thermal_camera_target)
target_link_libraries(${thermal_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(boundingbox_camera_sources BoundingBoxCameraSensor.cc)
ign_add_component(boundingbox_camera SOURCES ${boundingbox_camera_sources} GET_TARGET_NAME boundingbox_camera_target)
target_link_libraries(${boundingbox_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(segmentation_camera_sources SegmentationCameraSensor.cc)
gz_add_component(segmentation_camera SOURCES ${segmentation_camera_sources} GET_TARGET_NAME segmentation_camera_target)
target_compile_definitions(${segmentation_camera_target} PUBLIC SegmentationCameraSensor_EXPORTS)
target_link_libraries(${segmentation_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

set(wide_angle_camera_sources WideAngleCameraSensor.cc)
gz_add_component(wide_angle_camera SOURCES ${wide_angle_camera_sources} GET_TARGET_NAME wide_angle_camera_target)
target_link_libraries(${wide_angle_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

# Build the unit tests.
gz_build_tests(TYPE UNIT SOURCES ${gtest_sources} LIB_DEPS ${rendering_target})

# Build the unit tests that depend on components.
gz_build_tests(TYPE UNIT SOURCES Lidar_TEST.cc LIB_DEPS ${lidar_target})
gz_build_tests(TYPE UNIT SOURCES Camera_TEST.cc LIB_DEPS ${camera_target})
gz_build_tests(TYPE UNIT SOURCES ImuSensor_TEST.cc LIB_DEPS ${imu_target})

