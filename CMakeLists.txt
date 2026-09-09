cmake_minimum_required(VERSION 3.16)
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "Prevented in-tree build. Please create a dedicated build directory and call cmake from there.")
endif()

project(PoMiDAQ
    DESCRIPTION "Portable Miniscope data acquisition"
    VERSION 0.6.2
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/contrib/cmake/")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenGL_GL_PREFERENCE GLVND)

set(LIBSOVERSION 1)

#
# Options
#
option(MAINTAINER "Enable maintainer mode" OFF)
option(GUI        "Build Qt user interface" ON)
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
find_package(Threads REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED)
if (GUI)
    find_package(Qt6 6.8 REQUIRED COMPONENTS Core Widgets Svg OpenGLWidgets)
endif()

# OpenCV's version file only accepts the requested major version,
# so probe for OpenCV 5 first and fall back to OpenCV 4
find_package(OpenCV 5 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4.8 REQUIRED)
endif()
find_package(OpenGL REQUIRED)
find_package(FFmpeg 58.35 REQUIRED COMPONENTS avcodec avutil avformat swscale)

if (GUI AND "${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
    find_package(KF6ConfigWidgets)
    set(KF6_ConfigWidgets KF6::ConfigWidgets)
endif()

if (GUI)
    qt_standard_project_setup()
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
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
