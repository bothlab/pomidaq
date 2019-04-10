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
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

# build a fresh OpenCV version
RUN mkdir /build
ADD data/ci/opencv_quickbuild.sh /build/
RUN cd /build && eatmydata /build/opencv_quickbuild.sh
RUN rm -rf /build

# finish
RUN mkdir /build
WORKDIR /build
