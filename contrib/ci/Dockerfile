#
# Docker file for PoMiDAQ CI
#
FROM ubuntu:focal

# prepare
RUN apt-get update -qq
RUN apt-get install -yq eatmydata

# install build essentials
RUN DEBIAN_FRONTEND=noninteractive eatmydata apt-get install -yq \
    build-essential gcc g++

# install build dependencies
RUN DEBIAN_FRONTEND=noninteractive eatmydata apt-get install -yq --no-install-recommends \
    git-core \
    dpkg-dev \
    devscripts \
    debhelper \
    debspawn \
    cmake \
    qt5-default \
    libqt5opengl5-dev \
    libkf5configwidgets-dev \
    libboost-dev \
    libboost-container-dev \
    libopencv-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libboost-python-dev \
    python3-dev \
    python3-numpy

# finish
RUN mkdir /build
WORKDIR /build
