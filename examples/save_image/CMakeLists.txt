cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ignition-sensors-camera-demo)

# Find the Ignition Libraries used directly by the example
find_package(ignition-sensors0 REQUIRED)

#--------------------------------------
# Find OGRE
ign_find_package(IgnOGRE VERSION 1.8.0
  COMPONENTS ${ign_ogre_components}
  REQUIRED_BY ogre
  PRIVATE_FOR ogre
  PKGCONFIG_IGNORE)

if (OGRE_FOUND)
  add_definitions(-DWITH_OGRE)
endif()

#--------------------------------------
# Find OGRE2
ign_find_package(OGRE2 VERSION 2.1.0
    COMPONENTS HlmsPbs HlmsUnlit Overlay
    REQUIRED_BY ogre2
    PRIVATE_FOR ogre2
    PKGCONFIG_IGNORE)

if (OGRE2_FOUND)
  add_definitions(-DWITH_OGRE2)
endif()

add_executable(save_image main.cc)
target_include_directories(save_image PUBLIC
  ${IGNITION-SENSORS_INCLUDE_DIRS}
  ignition-sensors0-camera)

target_link_libraries(save_image PUBLIC
  ${IGNITION-SENSORS_LIBRARIES}
  ignition-sensors0-camera)
