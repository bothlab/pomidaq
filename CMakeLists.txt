project(PoMiDAQ)
cmake_minimum_required(VERSION 3.2)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/data/cmake/")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(OpenGL_GL_PREFERENCE GLVND)

set(PROJECT_VERSION 0.1)
set(LIBSOVERSION 0)

#
# Compiler flags
#
add_definitions("-Wall" "-Wextra" "-Werror")
add_definitions("-Wno-unused-but-set-variable")
add_definitions("-Werror")

#
# Dependencies
#
include(GNUInstallDirs)
find_package(Boost 1.60 REQUIRED COMPONENTS container)
find_package(Qt5Widgets REQUIRED)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(FFmpeg 4.1 REQUIRED COMPONENTS avcodec avutil avformat)

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
