add_library(ignition-sensors-camera SHARED CameraSensor.cc)

target_link_libraries(ignition-sensors-camera PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME})

target_link_libraries(ignition-sensors-camera PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}_headers)

ign_install_library(ignition-sensors-camera ${PROJECT_EXPORT_NAME}-camera)
