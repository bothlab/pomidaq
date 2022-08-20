#
# Docker file for Flatpak builds
#
FROM debian:testing

# prepare
RUN apt-get update -qq
RUN apt-get install -yq eatmydata

RUN DEBIAN_FRONTEND=noninteractive eatmydata apt-get install -yq \
    flatpak flatpak-builder

RUN flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo

# finish
RUN mkdir /build
WORKDIR /build
