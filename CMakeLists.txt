project(PoMiDAQ)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/data/cmake/")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(OpenGL_GL_PREFERENCE GLVND)

set(PROJECT_VERSION 0.1)
set(LIBSOVERSION 0)

#
# Options
#
option(MAINTAINER "Enable maintainer mode" OFF)

#
# Compiler flags
#
add_definitions("-Wall")

if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions("-Wextra")
endif()

if (MAINTAINER)
    add_definitions("-Werror")
endif()

#
# Dependencies
#
include(GNUInstallDirs)
find_package(Threads)
find_package(Boost 1.60 REQUIRED COMPONENTS container)
find_package(Qt5Widgets REQUIRED)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(FFmpeg 58.35 REQUIRED COMPONENTS avcodec avutil avformat)

#
# Compile options
#

# NONE

#
# Subdirectories
#
add_subdirectory(src)
add_subdirectory(libminiscope)
add_subdirectory(data)
