#!/bin/bash
set -e
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

if [ -z "$TARGET_HOST" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        valgrind \
        libsdl2-dev \
        qt5-default \
        libswscale-dev \
        "
fi

if [ -z "$TARGET_HOST" ] && [ -z "$DECODESTREAMS" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        devscripts \
        "
fi

if [ "$WINE" = "wine" ]; then
    sudo dpkg --add-architecture i386
    UPDATE_APT=1

    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        gcc-mingw-w64-i686 \
        g++-mingw-w64-i686 \
        binutils-mingw-w64-i686 \
        mingw-w64-i686-dev \
        wine32 \
        "
elif [ "$WINE" = "wine64" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        gcc-mingw-w64-x86-64 \
        g++-mingw-w64-x86-64 \
        binutils-mingw-w64-x86-64 \
        mingw-w64-x86-64-dev \
        wine64 \
        "
fi

if ( echo "$TARGET_HOST" | grep -q "^arm" ); then
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
        python-is-python2 \
        "
fi

if [ ! -z "$CMAKE" ]; then
    INSTALL_PACKAGES="$INSTALL_PACKAGES \
        cmake \
        "
fi

# Workaround issues where the "azure.archive.ubuntu.com" mirror is not available
# See https://github.com/actions/runner-images/issues/675
sudo sed -i 's/azure\.//' /etc/apt/sources.list
UPDATE_APT=1

if [ ! -z "$UPDATE_APT" ]; then
    echo "Updating package lists ..."
    sudo apt-get update -qq
fi

if [ ! -z "$INSTALL_PACKAGES" ]; then
    echo "Held packages:"
    sudo dpkg --get-selections | { grep hold || true; }   # { || true; } prevents CI from exiting when grep returns '1' (no match)

    echo "Installing packages $INSTALL_PACKAGES"
    sudo apt-get install -qq $INSTALL_PACKAGES
fi

if [ "$WINE" = "wine" ]; then
    sudo update-alternatives --set i686-w64-mingw32-gcc /usr/bin/i686-w64-mingw32-gcc-posix
    sudo update-alternatives --set i686-w64-mingw32-g++ /usr/bin/i686-w64-mingw32-g++-posix
elif [ "$WINE" = "wine64" ]; then
    sudo update-alternatives --set x86_64-w64-mingw32-gcc /usr/bin/x86_64-w64-mingw32-gcc-posix
    sudo update-alternatives --set x86_64-w64-mingw32-g++ /usr/bin/x86_64-w64-mingw32-g++-posix
fi
