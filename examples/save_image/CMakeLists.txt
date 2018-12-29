cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ignition-sensors-camera-demo)

# Find the Ignition Libraries used directly by the example
find_package(ignition-rendering1 REQUIRED)
find_package(ignition-sensors0 REQUIRED)

#--------------------------------------
# Find OGRE
ign_find_package(IgnOGRE VERSION 1.8.0
  PKGCONFIG_IGNORE)

if (OGRE_FOUND)
  add_definitions(-DWITH_OGRE)
endif()

#--------------------------------------
# Find OGRE2
ign_find_package(IgnOGRE2 VERSION 2.1.0
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
