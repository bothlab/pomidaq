#!/bin/sh
set -e

#
# This script will create the PoMiDAQ Debian package & upload it to the PPA
#

TARGET_SUITE="noble"

set -x
mkdir -p __ppa-pkg-build
cd __ppa-pkg-build

# create clean source copy from current Git tree
mkdir -p pomidaq
git -C "$(git rev-parse --show-toplevel)" archive HEAD | tar -x -C ./pomidaq/
cd pomidaq

#
# Build Debian source package
#

git_commit=$(git rev-parse --short HEAD)
git_current_tag=$(git describe --tags --abbrev=0 2>/dev/null || echo v3.0.2)
git_commit_no=$(git rev-list --count "${git_current_tag}..HEAD" 2>/dev/null)
upstream_version=${git_current_tag#v}; upstream_version=${upstream_version//-/.}
if [ "$git_commit_no" -gt 0 ]; then
  upstream_version+="+git$git_commit_no"
fi

mv contrib/debian .
dch --distribution "$TARGET_SUITE" --newversion="${upstream_version}" -b \
    "New automated build from: ${upstream_version} - ${git_commit}"

# build the source package
debuild -S -sa

# cleanup & upload
cd ..
rm -r pomidaq
dput ppa:ximion/syntalos *.changes
rm *.changes

cd ..
rm -r __ppa-pkg-build
