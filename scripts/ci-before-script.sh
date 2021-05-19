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

CURRENT_OS=$TRAVIS_OS_NAME
if [ -z "$CURRENT_OS" ]; then
    if [ "$(uname)" != "Darwin" ]; then
        CURRENT_OS=linux
    else
        CURRENT_OS=osx
    fi
fi

if [ ! -z "$TARGET_HOST" ]; then
    # Make sure the correct compiler will be used.
    unset CC
    unset CXX
fi

if [ "$CURRENT_OS" = "osx" ]; then
    export PATH="/usr/local/opt/qt@5/bin:$PATH"
    export PKG_CONFIG_PATH="/usr/local/opt/qt@5/lib/pkgconfig"
fi

if [ -z "$CMAKE" ]; then
    ./autogen.sh
    ./configure --host=$TARGET_HOST
else
    cmake .
fi
