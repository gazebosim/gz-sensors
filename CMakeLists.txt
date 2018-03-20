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
set(IGN_MATH_VER 4)
ign_find_package(ignition-math${IGN_MATH_VER} REQUIRED)

#--------------------------------------
# Find ignition-common
set(IGN_COMMON_VER 1)
ign_find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

#--------------------------------------
# Find ignition-transport
set(IGN_TRANSPORT_VER 4)
ign_find_package(ignition-transport${IGN_TRANSPORT_VER} REQUIRED)

#--------------------------------------
# Find ignition-rendering
find_package(ignition-rendering0 QUIET)
if (NOT ignition-rendering0_FOUND)
  ign_build_error("Missing: Ignition Rendering libignition-rendering0-dev.")
else()
  message (STATUS "Found Ignition Rendering 0")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-RENDERING_CXX_FLAGS}")
  include_directories(${IGNITION-RENDERING_INCLUDE_DIRS})
  link_directories(${IGNITION-RENDERING_LIBRARY_DIRS})
endif()

#--------------------------------------
# Find ignition-msgs
set(IGN_MSGS_VER 1)
ign_find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

#--------------------------------------
# Find ignition-tools
ign_find_package(ignition-tools QUIET)
if (ignition-tools_FOUND)
  set (HAVE_IGN_TOOLS TRUE)
endif()

#--------------------------------------
# Find SDFormat
find_package(SDFormat REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

set(IGN_SENSORS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

