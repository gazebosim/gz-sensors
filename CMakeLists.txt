cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-sensors7 VERSION 7.3.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)

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
gz_find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
gz_find_package(gz-common5
                 COMPONENTS profiler
                 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

#--------------------------------------
# Find gz-rendering

# See CMP0072 for more details (cmake --help-policy CMP0072)
if ((NOT ${CMAKE_VERSION} VERSION_LESS 3.11) AND (NOT OpenGL_GL_PREFERENCE))
  set(OpenGL_GL_PREFERENCE "GLVND")
endif()

gz_find_package(gz-rendering7 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

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
gz_find_package(gz-msgs9 REQUIRED VERSION 9.3)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

#--------------------------------------
# Find SDFormat
gz_find_package(sdformat13 VERSION 13.3 REQUIRED)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

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
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md")

if(TARGET doc)
  file(COPY ${CMAKE_SOURCE_DIR}/tutorials/files/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/files/)
endif()
