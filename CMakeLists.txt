cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-sensors0 VERSION 0.1.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(VERSION_SUFFIX pre1)

#============================================================================
# Set project-specific options
#============================================================================
set (DRI_TESTS TRUE CACHE BOOL "True to enable DRI tests")

#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find Protobuf
set(REQ_PROTOBUF_VER 3)
ign_find_package(IgnProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 PRETTY Protobuf)

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common3 REQUIRED)
set(IGN_COMMON_VER ${ignition-common3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport6 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-rendering
ign_find_package(ignition-rendering1 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs3 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs3_VERSION_MAJOR})

#--------------------------------------
# Find SDFormat
ign_find_package(sdformat8 REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

set(IGN_SENSORS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

#--------------------------------------
# Find OGRE
ign_find_package(IgnOGRE VERSION 1.8.0
  PKGCONFIG_IGNORE)

if (OGRE_FOUND)
  set(HAVE_OGRE TRUE)
  add_definitions(-DWITH_OGRE)
endif()

#--------------------------------------
# Find OGRE2
ign_find_package(OGRE2 VERSION 2.1.0
    PKGCONFIG_IGNORE)

if (OGRE2_FOUND)
  set(HAVE_OGRE2 TRUE)
  add_definitions(-DWITH_OGRE2)
endif()

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md")
