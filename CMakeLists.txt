SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(ubitrack_device_camera_kinect4azure)

cmake_minimum_required(VERSION 3.4.0)

include(CheckIncludeFile)

include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)

# External Libraries (could this be solved with conan dependencies ??)

# OpenGL
SET(OpenGL_GL_PREFERENCE "GLVND")
SET(HAVE_OPENGL 0)
find_package(OpenGL)
IF(OpenGL_FOUND)
  SET(HAVE_OPENGL 1)
  MESSAGE(STATUS "Found OpenGL: ${OPENGL_INCLUDE_DIR} - ${OPENGL_LIBRARIES}")
ENDIF(OpenGL_FOUND)

# OpenCL
SET(HAVE_OPENCL 0)
find_package(OpenCL)
IF(OpenCL_FOUND)
  include_directories(${OpenCL_INCLUDE_DIR})
  SET(HAVE_OPENCL 1)
  MESSAGE(STATUS "Found OpenCL: ${OpenCL_INCLUDE_DIR} - ${OpenCL_LIBRARY}")
ENDIF(OpenCL_FOUND)

set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS})

set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_KINECT4AZURE")

add_subdirectory(src/kinect4azure)
add_subdirectory(src/kinect4azure_io)

ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
