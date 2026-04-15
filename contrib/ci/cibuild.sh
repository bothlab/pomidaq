#!/bin/bash
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
cmake -G Ninja \
      -DMAINTAINER=ON \
      -DPYTHON=ON \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      ..

# Build, Test & Install
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test -v
DESTDIR=/tmp/install_root/ ninja install
cd ..
rm -rf build/

#
# Build Debian package
#

git_commit=$(git rev-parse --short HEAD)
git_current_tag=$(git describe --tags --abbrev=0 2>/dev/null || echo v0.0.0)
git_commit_no=$(git rev-list --count "${git_current_tag}..HEAD" 2>/dev/null)
upstream_version=${git_current_tag#v}; upstream_version=${upstream_version//-/.}
if [ "$git_commit_no" -gt 0 ]; then
  upstream_version+="+git$git_commit_no"
fi

cp -dpr contrib/debian .
dch --distribution "UNRELEASED"	--newversion="${upstream_version}~${DEBVER_SLUG}" -b \
    "New automated build from: ${upstream_version} - ${git_commit}"

dpkg-buildpackage
rm -r debian/
mv ../*.deb .
