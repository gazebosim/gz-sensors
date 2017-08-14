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
# Ignition common
find_package(ignition-common0 QUIET)
if (NOT ignition-common0_FOUND)
  BUILD_ERROR ("Missing: Ignition Common (libignition-common0-dev)")
else()
  message (STATUS "Found Ignition Common")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-COMMON_CXX_FLAGS}")
  include_directories(${IGNITION-COMMON_INCLUDE_DIRS})
  link_directories(${IGNITION-COMMON_LIBRARY_DIRS})
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
