cmake_minimum_required(VERSION 3.2)
project(PoMiDAQ
    DESCRIPTION "Portable Miniscope data acquisition"
    VERSION 0.2.0
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/contrib/cmake/")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(OpenGL_GL_PREFERENCE GLVND)

set(LIBSOVERSION 0)

#
# Options
#
option(MAINTAINER "Enable maintainer mode" OFF)
option(GUI        "Build Python module" ON)
option(PYTHON     "Build Python module" ON)


#
# Compiler flags
#
add_definitions("-Wall")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_definitions("-Wextra")
endif()
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions("-Wno-c++98-compat" "-Wno-c++98-compat-pedantic")
endif()

if (MAINTAINER)
    if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        add_definitions("-Werror")
    endif()
endif()

# Windows compat stuff
set(BUILD_SHARED_LIBS ON)
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
    # No shared libs for Windows (yet)
    set(BUILD_SHARED_LIBS OFF)

    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        add_definitions("-Wno-old-style-cast")
    endif()
endif()

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()


#
# Dependencies
#
include(GNUInstallDirs)
find_package(Threads)
find_package(Boost 1.60 REQUIRED COMPONENTS container)
find_package(Qt5Widgets REQUIRED)

find_package(OpenCV 4.1 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(FFmpeg 58.35 REQUIRED COMPONENTS avcodec avutil avformat swscale)

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
    find_package(KF5ConfigWidgets)
    set(KF5_ConfigWidgets KF5::ConfigWidgets)
endif()


#
# Config
#
configure_file(config.h.in ${CMAKE_BINARY_DIR}/config.h)
include_directories(${CMAKE_BINARY_DIR})


#
# Subdirectories
#
add_subdirectory(libminiscope)
add_subdirectory(data)
if (GUI)
  add_subdirectory(src)
endif()
if (PYTHON)
  add_subdirectory(py)
endif()
