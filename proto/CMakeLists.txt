set(proto_files
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/air_speed_sensor.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/altimeter.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/annotated_axis_aligned_2d_box.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/annotated_axis_aligned_2d_box_v.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/annotated_oriented_3d_box.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/annotated_oriented_3d_box_v.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/axis_aligned_2d_box.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/axis_aligned_box.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/camera_info.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/dvl_beam_state.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/dvl_kinematic_estimate.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/dvl_range_estimate.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/dvl_tracking_target.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/dvl_velocity_tracking.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/fluid_pressure.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/imu.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/laserscan.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/logical_camera_image.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/magnetometer.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/navsat.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/oriented_3d_box.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/performance_sensor_metrics.proto
  ${PROJECT_SOURCE_DIR}/proto/gz/msgs/sensor_noise.proto
)

gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "gz.msgs"
  MSGS_PATH ${PROJECT_SOURCE_DIR}/proto
  MSGS_PROTOS ${proto_files}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}-msgs
)

install(
  DIRECTORY gz
  DESTINATION share/protos
  COMPONENT proto
  FILES_MATCHING PATTERN "*.proto")
