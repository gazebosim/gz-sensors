include (${project_cmake_dir}/Utils.cmake)

set (headers
  System.hh
)

set (ign_headers "" CACHE INTERNAL "Ignition Sensors headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Ignition Sensors headers" "#include <ignition/sensors/${hdr}>\n")
endforeach()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sensors.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)

ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition" ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)
ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition/${IGN_PROJECT_NAME}" ${headers})
