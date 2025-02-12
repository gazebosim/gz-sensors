cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-sensors9 VERSION 9.1.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake4 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX)

#============================================================================
# Set project-specific options
#============================================================================
set (DRI_TESTS TRUE CACHE BOOL "True to enable DRI tests")

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DGZ_PROFILER_ENABLE=1")
else()
  add_definitions("-DGZ_PROFILER_ENABLE=0")
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find Protobuf
gz_find_package(GzProtobuf
                REQUIRED
                PRETTY Protobuf)

#--------------------------------------
# Find gz-math
gz_find_package(gz-math8 REQUIRED)
set(GZ_MATH_VER ${gz-math8_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
gz_find_package(gz-common6
                 COMPONENTS profiler
                 REQUIRED)
set(GZ_COMMON_VER ${gz-common6_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport14 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport14_VERSION_MAJOR})

#--------------------------------------
# Find gz-rendering
gz_find_package(gz-rendering9 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
set(GZ_RENDERING_VER ${gz-rendering9_VERSION_MAJOR})

if (TARGET gz-rendering${GZ_RENDERING_VER}::ogre)
  set(HAVE_OGRE TRUE)
  add_definitions(-DWITH_OGRE)
endif()

if (TARGET gz-rendering${GZ_RENDERING_VER}::ogre2)
  set(HAVE_OGRE2 TRUE)
  add_definitions(-DWITH_OGRE2)
endif()

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs11 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs11_VERSION_MAJOR})

#--------------------------------------
# Find SDFormat
gz_find_package(sdformat15 REQUIRED)
set(SDF_VER ${sdformat15_VERSION_MAJOR})

#--------------------------------------
# Find Eigen3
find_package(Eigen3 REQUIRED)

#============================================================================
# Configure the build
#============================================================================
gz_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# Create package information
#============================================================================
gz_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

gz_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
  IMAGE_PATH_DIRS "${CMAKE_SOURCE_DIR}/tutorials/files")

if(TARGET doc)
  file(COPY ${CMAKE_SOURCE_DIR}/tutorials/files/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/files/)
endif()
