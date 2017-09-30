cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake0 REQUIRED)


#============================================================================
# Set up the project
#============================================================================
ign_configure_project(sensors 0.0.1)


#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find ignition-math
set(IGN_MATH_VER 4)
ign_find_package(ignition-math${IGN_MATH_VER} REQUIRED)

#--------------------------------------
# Find ignition-transport
set(IGN_TRANSPORT_VER 3)
ign_find_package(ignition-transport${IGN_TRANSPORT_VER} REQUIRED)

#--------------------------------------
# Find ignition-rendering
set(IGN_RENDERING_VER 0)
ign_find_package(ignition-rendering${IGN_RENDERING_VER} REQUIRED)

#--------------------------------------
# Find ignition-common
set(IGN_COMMON_VER 0)
ign_find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

#--------------------------------------
# Find SDFormat
ign_find_package(SDFormat 6.0.0 REQUIRED)

message(STATUS "-------------------------------------------\n")


#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)


#============================================================================
# Create package information
#============================================================================
ign_create_packages()
