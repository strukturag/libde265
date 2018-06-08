#!/bin/bash
set -eu
#
# H.265 video codec.
# Copyright (c) 2018 struktur AG, Joachim Bauch <bauch@struktur.de>
#
# This file is part of libde265.
#
# libde265 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libde265 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with libde265.  If not, see <http://www.gnu.org/licenses/>.
#

INSTALL_PACKAGES=
UPDATE_APT=

# Output something once per minute to avoid being killed for inactivity.
while true; do echo "Still alive at $(date) ..."; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

if [ -z "$HOST" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        valgrind \
        libsdl-dev \
        libqt4-dev \
        libswscale-dev \
        "
fi

if [ -z "$HOST" ] && [ -z "$DECODESTREAMS" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        devscripts \
        "
fi

if [ ! -z "$WINE" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        wine \
        "
fi
if [ "$WINE" = "wine" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        gcc-mingw-w64-i686 \
        g++-mingw-w64-i686 \
        binutils-mingw-w64-i686 \
        mingw-w64-i686-dev \
        "
elif [ "$WINE" = "wine64" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        gcc-mingw-w64-x86-64 \
        g++-mingw-w64-x86-64 \
        binutils-mingw-w64-x86-64 \
        mingw-w64-x86-64-dev \
        "
fi

if ( echo "$HOST" | grep -q "^arm" ); then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        g++-arm-linux-gnueabihf \
        gcc-arm-linux-gnueabihf \
        qemu-user \
        "
fi

if [ ! -z "$DECODESTREAMS" ]; then
    sudo add-apt-repository -y ppa:strukturag/libde265
    UPDATE_APT=1
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        $DECODESTREAMS \
        "
fi

if [ "$HOST" = "cmake" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        cmake \
        "
fi

if [ ! -z "$UPDATE_APT" ]; then
    echo "Updating package lists ..."
    sudo apt-get update -qq
fi

if [ ! -z "$INSTALL_PACKAGES" ]; then
    echo "Installing packages $INSTALL_PACKAGES ..."
    sudo apt-get install -qq $INSTALL_PACKAGES
fi
