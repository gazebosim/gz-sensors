include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

################################################################################
# Ignition msgs
find_package(ignition-msgs1 QUIET)
if (NOT ignition-msgs1_FOUND)
  BUILD_ERROR ("Missing: Ignition msgs (libignition-msgs1-dev)")
else()
  message (STATUS "Found Ignition msgs")
endif()

################################################################################
# Ignition math
find_package(ignition-math4 QUIET)
if (NOT ignition-math4_FOUND)
  BUILD_ERROR ("Missing: Ignition math (libignition-math4-dev)")
else()
  message (STATUS "Found Ignition Math")
endif()

################################################################################
# Ignition transport
find_package(ignition-transport3 QUIET)
if (NOT ignition-transport3_FOUND)
  BUILD_ERROR ("Missing: Ignition transport (libignition-transport3-dev)")
else()
  message (STATUS "Found Ignition transport")
endif()

################################################################################
# Ignition rendering
find_package(ignition-rendering0 QUIET)
if (NOT ignition-rendering0_FOUND)
  BUILD_ERROR ("Missing: Ignition Rendering (libignition-rendering0-dev)")
else()
  message (STATUS "Found Ignition Rendering")
endif()

################################################################################
# Ignition common
find_package(ignition-common0 QUIET)
if (NOT ignition-common0_FOUND)
  BUILD_ERROR ("Missing: Ignition Common (libignition-common0-dev)")
else()
  message (STATUS "Found Ignition Common")
endif()

################################################################################
# Find SDFormat
set (SDFormat_MIN_VERSION 6.0.0)
find_package(SDFormat ${SDFormat_MIN_VERSION})

if (NOT SDFormat_FOUND)
  message (STATUS "Looking for SDFormat - not found")
  BUILD_ERROR ("Missing: SDF version >=${SDFormat_MIN_VERSION}. Required for reading and writing SDF files.")
else()
  message (STATUS "Looking for SDFormat - found")
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
