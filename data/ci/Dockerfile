#
# Docker file for PoMiDAQ CI
#
FROM debian:buster

# prepare
RUN apt-get update -qq
RUN apt-get install -yq eatmydata

# install build essentials
RUN eatmydata apt-get install -yq gcc g++ clang

# install build dependencies
RUN eatmydata apt-get install -yq --no-install-recommends \
    build-essential \
    debspawn \
    cmake \
    qt5-default \
    libqt5opengl5-dev \
    libboost-dev \
    libboost-container-dev \
    libopencv-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

# finish
RUN mkdir /build
WORKDIR /build
