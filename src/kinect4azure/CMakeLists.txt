# All cpp files for kinect4azure2 camera capture
set(UBITRACK_LIBRARY_NAME "kinect4azurecamera")

FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

FILE(GLOB UBITRACK_HDRS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        )

add_library(${UBITRACK_LIBRARY_NAME} SHARED ${UBITRACK_SRCS} ${UBITRACK_HDRS})



target_link_libraries(${UBITRACK_LIBRARY_NAME}
    CONAN_PKG::ubitrack_core
    CONAN_PKG::ubitrack_vision
    CONAN_PKG::ubitrack_dataflow
    CONAN_PKG::ubitrack_dataflow
    CONAN_PKG::kinect-azure-sensor-sdk
    OpenGL::GL
    OpenGL::GLU
    OpenCL::OpenCL
    ${UBITRACK_EXTRA_LIBS}
    )

target_include_directories(${UBITRACK_LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# do not create symlinks to versioned names of component libraries
set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
ubitrack_install_component(${UBITRACK_LIBRARY_NAME})
