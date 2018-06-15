cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-sensors0 VERSION 0.1.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake1 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()

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
if(MSVC)
  # On Windows we require version 3+ in order to have proper support for
  # shared libraries.
  set(REQ_PROTOBUF_VER 3)
  set(PROTOBUF_PURPOSE "Windows requires version 3+ to ensure proper support for DLLs")
else()
  set(REQ_PROTOBUF_VER 2.3.0)
  set(PROTOBUF_PURPOSE)
endif()
ign_find_package(IgnProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 PRETTY Protobuf
                 PURPOSE ${PROTOBUF_PURPOSE})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math5 REQUIRED)
set(IGN_MATH_VER ${ignition-math5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common2 REQUIRED)
set(IGN_COMMON_VER ${ignition-common2_VERSION_MAJOR})

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport5 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-rendering
ign_find_package(ignition-rendering0 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering0_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs2 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs2_VERSION_MAJOR})

#--------------------------------------
# Find ignition-tools
ign_find_package(ignition-tools QUIET)
if (ignition-tools_FOUND)
  set (HAVE_IGN_TOOLS TRUE)
endif()

####### TEMP
# Find OGRE
ign_find_package(OGRE VERSION 1.9.0
    REQUIRED_BY ogre
    PRIVATE_FOR ogre )

include_directories(
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging )
#######

#--------------------------------------
# Find SDFormat
find_package(SDFormat REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

set(IGN_SENSORS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS
    COMPONENTS ogre )

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

