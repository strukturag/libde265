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

if [ ! -z "$HOST" ] && [ "$HOST" != "cmake" ]; then
    # Make sure the correct compiler will be used.
    unset CC
    unset CXX
fi

if [ "$TRAVIS_OS_NAME" = "osx" ]; then
    export PATH="/usr/local/opt/qt/bin:$PATH"
    export PKG_CONFIG_PATH=/usr/local/opt/qt/lib/pkgconfig
fi

if [ "$HOST" != "cmake" ]; then
    ./autogen.sh
    ./configure --host=$HOST
fi

if [ "$HOST" = "cmake" ]; then
    cmake .
fi
