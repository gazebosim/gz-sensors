cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ignition-sensors-camera-demo)

# Find the Ignition Libraries used directly by the example
find_package(ignition-rendering2 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
find_package(ignition-sensors2 REQUIRED COMPONENTS rendering camera)

if (TARGET ignition-rendering2::ogre)
   add_definitions(-DWITH_OGRE)
endif()
if (TARGET ignition-rendering2::ogre2)
   add_definitions(-DWITH_OGRE2)
endif()

add_executable(save_image main.cc)
target_link_libraries(save_image PUBLIC
  ignition-sensors2::camera)
