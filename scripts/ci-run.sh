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
BUILD_ROOT=$TRAVIS_BUILD_DIR

if [ -z "$HOST" ] && [ -z "$DECODESTREAMS" ] && [ "$TRAVIS_OS_NAME" != "osx" ]; then
    ./scripts/check_licenses.sh
fi

if [ ! -z "$HOST" ] && [ "$HOST" != "cmake" ]; then
    # Make sure the correct compiler will be used.
    unset CC
    unset CXX
fi

make

if [ -z "$HOST" ] && [ -z "$DECODESTREAMS" ]; then
    if [ "$TRAVIS_OS_NAME" != "osx" ]; then
        make dist

        mkdir dist-test
        (cd dist-test && tar xzf ../libde265-*.tar.gz && cd libde265-* && ./configure && make)

        mkdir dist-cmake-test
        (cd dist-cmake-test && tar xzf ../libde265-*.tar.gz && cd libde265-* && cmake . && make)

        VALGRIND="valgrind --tool=memcheck --quiet --error-exitcode=1 --gen-suppressions=all --suppressions=$BUILD_ROOT/valgrind.supp"
        DEC265="$BUILD_ROOT/dec265/.libs/dec265"
    else
        VALGRIND=
        DEC265="$BUILD_ROOT/dec265/dec265"
    fi
    export LD_LIBRARY_PATH=$BUILD_ROOT/libde265/.libs/
    # inter-streams are valgrinded without SSE, because it gives too many false positives in put_hevc_qpel()
    # intra-streams use SSE, because they run fine in valgrind
    $VALGRIND $DEC265 -q -c -f 100 ./libde265-data/IDR-only/paris-352x288-intra.bin
    $VALGRIND $DEC265 -t 4 -q -c -f 100 ./libde265-data/IDR-only/paris-352x288-intra.bin
    $VALGRIND $DEC265 -0 -q -c -f 100 ./libde265-data/RandomAccess/paris-ra-wpp.bin
    $VALGRIND $DEC265 -0 -t 4 -q -c -f 100 ./libde265-data/RandomAccess/paris-ra-wpp.bin
fi

if [ ! -z "$WINE" ]; then
    export WINEPREFIX=$BUILD_ROOT/$WINE
    export WINEPATH="/usr/lib/gcc/$HOST/4.8/;/usr/$HOST/lib"
    $WINE ./dec265/dec265.exe -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    $WINE ./dec265/dec265.exe -t 4 -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    $WINE ./dec265/dec265.exe -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
    $WINE ./dec265/dec265.exe -t 4 -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
fi

if ( echo "$HOST" | grep -q "^arm" ); then
    export LD_LIBRARY_PATH=$BUILD_ROOT/libde265/.libs/
    qemu-arm -L /usr/$HOST ./dec265/.libs/dec265 -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    #qemu-arm -L /usr/$HOST ./dec265/.libs/dec265 -t 4 -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    qemu-arm -L /usr/$HOST ./dec265/.libs/dec265 -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
    #qemu-arm -L /usr/$HOST ./dec265/.libs/dec265 -t 4 -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
fi

if [ ! -z "$DECODESTREAMS" ]; then
    python scripts/decodestreams.py $THREADING /var/lib/libde265-teststreams
fi
