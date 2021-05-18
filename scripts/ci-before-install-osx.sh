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

# Output something once per minute to avoid being killed for inactivity.
while true; do echo "Still alive at $(date) ..."; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

INSTALL_PACKAGES="$INSTALL_PACKAGES \
    automake \
    ffmpeg \
    libtool \
    pkg-config \
    qt5 \
    sdl \
    "

if [ ! -z "$INSTALL_PACKAGES" ]; then
    echo "Remove python@2 ..."
    brew unlink python@2 || true

    echo "Installing packages $INSTALL_PACKAGES ..."
    for package in $INSTALL_PACKAGES; do
        brew list $package &>/dev/null || brew install $package
    done
fi
