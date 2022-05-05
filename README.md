PoMiDAQ
=======

[![PoMiDAQ Screenshot](contrib/screenshots/v0.2.0_recording_sample.png "PoMiDAQ on Linux")](https://github.com/bothlab/pomidaq/tree/master/contrib/screenshots)

[![Build](https://github.com/bothlab/pomidaq/actions/workflows/build.yml/badge.svg)](https://github.com/bothlab/pomidaq/actions/workflows/build.yml)

PoMiDAQ is a recording software for [UCLA Miniscopes](http://miniscope.org/index.php/Main_Page).
Unlike the [preexisting](https://github.com/daharoni/Miniscope_DAQ_Software) Miniscope software, this program
is written to be portable, so it will run on Windows, Linux and (likely) macOS.
It is designed to work with the latest OpenCV libraries and provides a shared library, so Miniscope recording
features can easily be embedded into other programs and pipelines.
Recorded data is encoded with the [FFV1](https://en.wikipedia.org/wiki/FFV1) codec by default, to allow for
smaller video files at lossless quality that are safe to archive.

It also provides an experimental Python module to access a Miniscope easily from Python. This feature has
currently only been tested on Linux (and may stay Linux-only) and is still considered a bit experimental.

Some features the original Miniscope DAQ software has, like the behavior recording, are intentionally left out.
This software is developed on Linux, but has also been successfully tested on Windows. As of now, no macOS build
has been attempted.

## Users

The CI system provides up-to-date builds for PoMiDAQ for the **Windows** (64-bit), **Debian** 11 and **Ubuntu** 20.04 (amd64) platforms.
You can fetch builds from there or download prebuilt binaries from the [Github releases](https://github.com/bothlab/pomidaq/releases).
Don't hesitate to file issues if you notice anything unusual.

On Linux, you can also just install the software from your software center when Flathub is configured:
<a href="https://flathub.org/apps/details/io.github.bothlab.pomidaq"><img src="https://flathub.org/assets/badges/flathub-badge-en.png" width="140"/></a>

### Notes
#### On Linux
 * The released binaries were built on Ubuntu 20.04 or Debian 11 respectively. They may work on other Debian-based distributions as well,
   but in case they don't, compiling the application from source is pretty easy to do (for short instructions see below).
 * You need at least an OpenGL 3.0-capable graphics driver on your system to run this application.

#### On Windows
 * You will need at least an OpenGL 3.0-capable graphics driver installed, otherwise the application will not start.
 * You will need to trust the executable from an "Unknown Publisher" as it is not signed.
 * Some users reported they can't start PoMiDAQ on Windows because `MF.dll`/`MFPLAT.dll` are missing. In this case,
   you will need to install the [Mediafeaturepack from Microsoft](https://www.microsoft.com/en-us/software-download/mediafeaturepack)
   for your Windows version to resolve the problem. Sometimes enabling this feature via Windows' "Additional Features" selection
   (in the "Apps" page of "Settings") will work better than a manual installation.

## Developers

### Dependencies

 * CMake (>= 3.16)
 * Qt5 (>= 5.12)
 * FFmpeg (>= 4.1)
 * OpenCV (>= 4.1)
 * KF5ConfigWidgets (on Linux)
 * [pyBind11](https://github.com/pybind/pybind11) (optional)

Before attempting to build PoMiDAQ, ensure all dependencies (and their development files) are installed on your system.
You should then be able to build the software after configuring the build with cmake for your platform.

On Debian-based Linux systems, all dependencies can be installed from the package repositories with this command:
```bash
sudo apt install build-essential cmake ninja-build qtbase5-dev libqt5opengl5-dev \
                 libkf5configwidgets-dev libopencv-dev libavcodec-dev libavformat-dev libswscale-dev \
                 pybind11-dev python3-dev python3-numpy
```
The software can then be built like this, for an optimized release build with debug infos included:
```bash
mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
ninja
sudo ninja install
sudo ldconfig
```

Pull-requests are very welcome! (Code should be valid C++14, use 4 spaces for indentation)
