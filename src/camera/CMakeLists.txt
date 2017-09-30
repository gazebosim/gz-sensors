
ign_add_component(
  camera
  SOURCES
    CameraSensor.cc
    ImageSaver.cc
  GET_TARGET_NAME camera_target)

target_include_directories(${camera_target} PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS})

target_include_directories(${camera_target} PUBLIC
  ${IGNITION-MSGS_INCLUDE_DIRS})

target_link_libraries(${camera_target} PUBLIC
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PROJECT_LIBRARY_TARGET_NAME})
