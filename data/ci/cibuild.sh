#!/bin/sh
set -e

echo "C compiler: $CC"
echo "C++ compiler: $CXX"
set -x

#
# This script is supposed to run inside the PoMiDAQ Docker container
# on the CI system.
#

$CC --version

# configure PoMiDAQ build with all flags enabled
mkdir build && cd build
cmake -DMAINTAINER=ON \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    ..

# Build, Test & Install
make -j4
#make test
DESTDIR=/tmp/install_root/ make install
