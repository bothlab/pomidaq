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
      -DPYTHON=ON \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      ..

# Build, Test & Install
make -j4
#make test
DESTDIR=/tmp/install_root/ make install
cd ..
rm -rf build/

#
# Build Debian package
#

git_commit=$(git log --pretty="format:%h" -n1)
git_current_tag=$(git describe --abbrev=0 --tags 2> /dev/null || echo "v0.1")
git_commit_no=$(git rev-list --count HEAD)
upstream_version=$(echo "${git_current_tag}" | sed 's/^v\(.\+\)$/\1/;s/[-]/./g')
upstream_version="$upstream_version+git$git_commit_no"

mv contrib/debian .
dch --distribution "UNRELEASED"	--newversion="${upstream_version}" -b \
    "New automated build from: ${upstream_version} - ${git_commit}"

dpkg-buildpackage
mv ../*.deb .
