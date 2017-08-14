include (${project_cmake_dir}/Utils.cmake)

set (headers
  Manager.hh
  Sensor.hh
  System.hh
)


set (ign_headers "" CACHE INTERNAL "Ignition Sensors headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Ignition Sensors headers" "#include <ignition/sensors/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sensors.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)

include (GenerateExportHeader)
generate_export_header(${PROJECT_LIBRARY_TARGET_NAME}
  BASE_NAME ign_sensors)

# Add an Interface library for headers to make propagating properties easier
add_library(${PROJECT_LIBRARY_TARGET_NAME}_headers INTERFACE)
# This dummy target adds generated headers to include dirs in a way that
# targets depending on this one will get those include_dirs, but targets
# afterwards won't get those include directories.
target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}_headers INTERFACE
  ${PROJECT_BINARY_DIR}/include)

ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition" ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)
ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition/${IGN_PROJECT_NAME}"
  ${headers}
  ${CMAKE_CURRENT_BINARY_DIR}/ign_sensors_export.h)
