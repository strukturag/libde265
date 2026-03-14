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

CMAKE_OPTS=""

if [ "$CURRENT_OS" = "osx" ]; then
    HOMEBREW_PREFIX="$(brew --prefix)"
    export PATH="$HOMEBREW_PREFIX/opt/qt@5/bin:$PATH"
    export PKG_CONFIG_PATH="$HOMEBREW_PREFIX/opt/qt@5/lib/pkgconfig"
fi

# Valgrind on Ubuntu 22.04 cannot handle DWARF5 debug info produced by clang.
# Force DWARF4 so valgrind can read the debug info.
if [ "$CURRENT_OS" = "linux" ] && [ "$CC" = "clang" ]; then
    CMAKE_OPTS="$CMAKE_OPTS -DCMAKE_C_FLAGS=-gdwarf-4 -DCMAKE_CXX_FLAGS=-gdwarf-4"
fi

# Select toolchain file for cross-compilation
if [ "$WINE" = "wine" ]; then
    CMAKE_OPTS="$CMAKE_OPTS -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/mingw-i686.cmake"
elif [ "$WINE" = "wine64" ]; then
    CMAKE_OPTS="$CMAKE_OPTS -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/mingw-x86_64.cmake"
elif ( echo "$TARGET_HOST" | grep -q "^arm" ); then
    CMAKE_OPTS="$CMAKE_OPTS -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-linux-gnueabihf.cmake"
fi

cmake -B build -S . $CMAKE_OPTS
