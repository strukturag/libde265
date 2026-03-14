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

# Valgrind on Ubuntu 22.04 cannot handle DWARF5 debug info produced by clang.
# Force DWARF4 so valgrind can read the debug info.
if [ "$CURRENT_OS" = "linux" ] && [ "$CC" = "clang" ]; then
    # We need to replicate the autotools defaults (-g -O2) because exporting
    # CFLAGS/CXXFLAGS causes configure to use them verbatim instead of its
    # own defaults. This can be simplified to just appending -gdwarf-4 once
    # we migrate the CI build to cmake.
    export CFLAGS="${CFLAGS:--g -O2} -gdwarf-4"
    export CXXFLAGS="${CXXFLAGS:--g -O2} -gdwarf-4"
fi

if [ -z "$CMAKE" ]; then
    ./autogen.sh
    ./configure --host=$TARGET_HOST
else
    cmake .
fi
