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

git_commit=$(git log --pretty="format:%h" -n1)
git_current_tag=$(git describe --abbrev=0 --tags 2> /dev/null || echo "v3.0.2")
git_commit_no=$(git rev-list --count HEAD)
upstream_version=$(echo "${git_current_tag}" | sed 's/^v\(.\+\)$/\1/;s/[-]/./g')
upstream_version="$upstream_version+git$git_commit_no"

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
