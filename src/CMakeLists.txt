set (sources
  BrownDistortionModel.cc
  Distortion.cc
  EnvironmentalData.cc
  GaussianNoiseModel.cc
  Manager.cc
  Noise.cc
  PointCloudUtil.cc
  Sensor.cc
  SensorFactory.cc
  SensorTypes.cc
  Util.cc
)

set(rendering_sources
  RenderingSensor.cc
  RenderingEvents.cc
  ImageBrownDistortionModel.cc
  ImageDistortion.cc
  ImageGaussianNoiseModel.cc
  ImageNoise.cc
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

gz_create_core_library(SOURCES ${sources} CXX_STANDARD ${CMAKE_CXX_STANDARD})
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-math::gz-math
    gz-common::gz-common
    gz-transport::gz-transport
    sdformat::sdformat
  PRIVATE
    gz-common::profiler
    gz-msgs::gz-msgs
)
target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC DepthPoints_EXPORTS)

gz_add_component(rendering SOURCES ${rendering_sources} GET_TARGET_NAME rendering_target)
target_link_libraries(${rendering_target}
  PUBLIC
    gz-rendering::gz-rendering
)

set(camera_sources CameraSensor.cc CameraSensorUtil.cc)
gz_add_component(camera
  SOURCES ${camera_sources}
  DEPENDS_ON_COMPONENTS rendering
  GET_TARGET_NAME camera_target
)
# custom compile definitions since the one provided automatically is versioned and will
# make the code need to change with every major version
target_compile_definitions(${camera_target} PUBLIC CameraSensor_EXPORTS)
target_link_libraries(${camera_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(depth_camera_sources DepthCameraSensor.cc CameraSensorUtil.cc)
gz_add_component(depth_camera
  SOURCES ${depth_camera_sources}
  DEPENDS_ON_COMPONENTS camera
  GET_TARGET_NAME depth_camera_target
)
target_compile_definitions(${depth_camera_target} PUBLIC DepthCameraSensor_EXPORTS)
target_link_libraries(${depth_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(dvl_sources DopplerVelocityLog.cc)
gz_add_component(dvl
  SOURCES ${dvl_sources}
  DEPENDS_ON_COMPONENTS rendering
  GET_TARGET_NAME dvl_target)
target_compile_definitions(${dvl_target} PUBLIC DopplerVelocityLog_EXPORTS)
target_link_libraries(${dvl_target}
  PUBLIC
    ${rendering_target}
    Eigen3::Eigen
  PRIVATE
    gz-common::gz-common
    gz-math::gz-math
    gz-msgs::gz-msgs
    gz-transport::gz-transport)

set(lidar_sources Lidar.cc)
gz_add_component(lidar
  SOURCES ${lidar_sources}
  DEPENDS_ON_COMPONENTS rendering
  GET_TARGET_NAME lidar_target
)
target_compile_definitions(${lidar_target} PUBLIC Lidar_EXPORTS)
target_link_libraries(${lidar_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(gpu_lidar_sources GpuLidarSensor.cc)
gz_add_component(gpu_lidar
  DEPENDS_ON_COMPONENTS lidar
  SOURCES ${gpu_lidar_sources}
  GET_TARGET_NAME gpu_lidar_target
)
target_compile_definitions(${gpu_lidar_target} PUBLIC GpuLidarSensor_EXPORTS)
target_link_libraries(${gpu_lidar_target}
  PRIVATE
    gz-msgs::gz-msgs
    gz-transport::gz-transport
    ${lidar_target}
)

# Marine radar.
#
# The vendored marine_radar core library (scan generation, echo model,
# target extraction and PPI rendering) is engine agnostic and is also linked
# into its unit tests, so it is built once as a static library.
set(marine_radar_core_sources
  marine_radar/RadarConfig.cc
  marine_radar/ScanGeometry.cc
  marine_radar/EchoModel.cc
  marine_radar/TargetExtraction.cc
  marine_radar/PpiRenderer.cc
  marine_radar/RadarProcessor.cc
)
add_library(marine_radar_core STATIC ${marine_radar_core_sources})
set_target_properties(marine_radar_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(marine_radar_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

set(marine_radar_sources MarineRadarSensor.cc marine_radar/RadarGzMessages.cc)
gz_add_component(marine_radar
  SOURCES ${marine_radar_sources}
  DEPENDS_ON_COMPONENTS rendering
  GET_TARGET_NAME marine_radar_target
)
target_compile_definitions(${marine_radar_target} PUBLIC MarineRadarSensor_EXPORTS)
target_link_libraries(${marine_radar_target}
  PUBLIC
    ${rendering_target}
  PRIVATE
    marine_radar_core
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(logical_camera_sources LogicalCameraSensor.cc)
gz_add_component(logical_camera SOURCES ${logical_camera_sources} GET_TARGET_NAME logical_camera_target)
target_compile_definitions(${logical_camera_target} PUBLIC LogicalCameraSensor_EXPORTS)
target_link_libraries(${logical_camera_target}
  PRIVATE
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(magnetometer_sources MagnetometerSensor.cc)
gz_add_component(magnetometer SOURCES ${magnetometer_sources} GET_TARGET_NAME magnetometer_target)

set(imu_sources ImuSensor.cc)
gz_add_component(imu SOURCES ${imu_sources} GET_TARGET_NAME imu_target)

set(cpu_lidar_sources CpuLidarSensor.cc)
gz_add_component(cpu_lidar SOURCES ${cpu_lidar_sources} GET_TARGET_NAME cpu_lidar_target)
target_link_libraries(${cpu_lidar_target}
  PRIVATE
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(altimeter_sources AltimeterSensor.cc)
gz_add_component(altimeter SOURCES ${altimeter_sources} GET_TARGET_NAME altimeter_target)

set(air_pressure_sources AirPressureSensor.cc)
gz_add_component(air_pressure SOURCES ${air_pressure_sources} GET_TARGET_NAME air_pressure_target)

set(air_speed_sources AirSpeedSensor.cc)
gz_add_component(air_speed SOURCES ${air_speed_sources} GET_TARGET_NAME air_speed_target)

set(force_torque_sources ForceTorqueSensor.cc)
gz_add_component(force_torque SOURCES ${force_torque_sources} GET_TARGET_NAME force_torque_target)

set(navsat_sources NavSatSensor.cc)
gz_add_component(navsat SOURCES ${navsat_sources} GET_TARGET_NAME navsat_target)

set(rgbd_camera_sources RgbdCameraSensor.cc CameraSensorUtil.cc)
gz_add_component(rgbd_camera
  SOURCES ${rgbd_camera_sources}
  DEPENDS_ON_COMPONENTS camera
  GET_TARGET_NAME rgbd_camera_target
)
target_compile_definitions(${rgbd_camera_target} PUBLIC RgbdCameraSensor_EXPORTS)
target_link_libraries(${rgbd_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(thermal_camera_sources ThermalCameraSensor.cc)
gz_add_component(thermal_camera
  SOURCES ${thermal_camera_sources}
  DEPENDS_ON_COMPONENTS camera
  GET_TARGET_NAME thermal_camera_target
)
target_link_libraries(${thermal_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(boundingbox_camera_sources BoundingBoxCameraSensor.cc)
gz_add_component(boundingbox_camera SOURCES ${boundingbox_camera_sources} GET_TARGET_NAME boundingbox_camera_target)
target_link_libraries(${boundingbox_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(segmentation_camera_sources SegmentationCameraSensor.cc)
gz_add_component(segmentation_camera SOURCES ${segmentation_camera_sources} GET_TARGET_NAME segmentation_camera_target)
target_compile_definitions(${segmentation_camera_target} PUBLIC SegmentationCameraSensor_EXPORTS)
target_link_libraries(${segmentation_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set(wide_angle_camera_sources WideAngleCameraSensor.cc)
gz_add_component(wide_angle_camera SOURCES ${wide_angle_camera_sources} GET_TARGET_NAME wide_angle_camera_target)
target_link_libraries(${wide_angle_camera_target}
  PRIVATE
    ${camera_target}
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

# Build the unit tests.
gz_build_tests(TYPE UNIT SOURCES ${gtest_sources} LIB_DEPS ${rendering_target})

# Build the unit tests that depend on components.
gz_build_tests(TYPE UNIT SOURCES Lidar_TEST.cc LIB_DEPS ${lidar_target})
gz_build_tests(TYPE UNIT SOURCES Camera_TEST.cc LIB_DEPS ${camera_target})
gz_build_tests(TYPE UNIT SOURCES ImuSensor_TEST.cc LIB_DEPS ${imu_target})
gz_build_tests(TYPE UNIT SOURCES MarineRadarSensor_TEST.cc
  LIB_DEPS ${marine_radar_target} marine_radar_core)

# Build the unit tests of the vendored marine radar core library.
gz_build_tests(TYPE UNIT
  SOURCES
    marine_radar/RadarConfig_TEST.cc
    marine_radar/ScanGeometry_TEST.cc
    marine_radar/EchoModel_TEST.cc
    marine_radar/TargetExtraction_TEST.cc
    marine_radar/PpiRenderer_TEST.cc
    marine_radar/RadarProcessor_TEST.cc
  LIB_DEPS marine_radar_core
)
gz_build_tests(TYPE UNIT SOURCES marine_radar/RadarGzMessages_TEST.cc
  LIB_DEPS marine_radar_core ${marine_radar_target} gz-msgs::gz-msgs)
