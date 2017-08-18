add_library(ignition-sensors-camera CameraSensorPlugin.cc)

target_link_libraries(ignition-sensors-camera PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME})

target_link_libraries(ignition-sensors-camera PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}_headers)

target_include_directories(ignition-sensors-camera PRIVATE .)

ign_install_library(ignition-sensors-camera ${PROJECT_EXPORT_NAME}-camera)
