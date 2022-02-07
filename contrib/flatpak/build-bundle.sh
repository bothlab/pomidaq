#!/bin/sh
set -e

#
# This script is supposed to run inside the PoMiDAQ Docker container
# on the CI system.
#
set -x

#
# Build Flatpak Bundle
#

git_commit=$(git log --pretty="format:%h" -n1)
git_current_tag=$(git describe --abbrev=0 --tags 2> /dev/null || echo "v0.1")
git_commit_no=$(git rev-list --count HEAD)
upstream_version=$(echo "${git_current_tag}" | sed 's/^v\(.\+\)$/\1/;s/[-]/./g')
upstream_version="$upstream_version+git$git_commit_no"

cd contrib/flatpak

flatpak-builder --force-clean \
		--repo=tmp-repo \
		build-dir \
		io.github.bothlab.pomidaq.yaml

flatpak build-bundle \
		--runtime-repo=https://flathub.org/repo/flathub.flatpakrepo \
		tmp-repo/ \
		pomidaq_${upstream_version}_amd64.flatpak \
		io.github.bothlab.pomidaq

rm -rf tmp-repo
cd ../..
