cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(gz-sensors-camera-demo)

# Find the Gazebo Libraries used directly by the example
find_package(gz-rendering9 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
find_package(gz-sensors REQUIRED COMPONENTS rendering camera)

if (TARGET gz-rendering9::ogre)
   add_definitions(-DWITH_OGRE)
endif()
if (TARGET gz-rendering9::ogre2)
   add_definitions(-DWITH_OGRE2)
endif()

add_executable(save_image main.cc)
target_link_libraries(save_image PUBLIC
  gz-sensors::camera)
