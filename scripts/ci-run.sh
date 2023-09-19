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
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

BUILD_ROOT=$ROOT/..
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

if [ "$CURRENT_OS" != "osx" ]; then
    CONCURRENCY=$(nproc)
else
    CONCURRENCY=$(sysctl -n hw.ncpu)
fi

make -j $CONCURRENCY

if [ -z "$TARGET_HOST" ] && [ -z "$CMAKE" ]  && [ -z "$DECODESTREAMS" ]; then
    if [ "$CURRENT_OS" != "osx" ]; then
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
    export WINEPATH="/usr/lib/gcc/$TARGET_HOST/7.3-posix/;/usr/$TARGET_HOST/lib"
    $WINE ./dec265/dec265.exe -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    $WINE ./dec265/dec265.exe -t 4 -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    $WINE ./dec265/dec265.exe -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
    $WINE ./dec265/dec265.exe -t 4 -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
fi

if ( echo "$TARGET_HOST" | grep -q "^arm" ); then
    export LD_LIBRARY_PATH=$BUILD_ROOT/libde265/.libs/
    qemu-arm -L /usr/$TARGET_HOST ./dec265/.libs/dec265 -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    #qemu-arm -L /usr/$TARGET_HOST ./dec265/.libs/dec265 -t 4 -q -c ./libde265-data/IDR-only/paris-352x288-intra.bin
    qemu-arm -L /usr/$TARGET_HOST ./dec265/.libs/dec265 -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
    #qemu-arm -L /usr/$TARGET_HOST ./dec265/.libs/dec265 -t 4 -q -c ./libde265-data/RandomAccess/paris-ra-wpp.bin
fi

if [ ! -z "$DECODESTREAMS" ]; then
    python scripts/decodestreams.py $THREADING /var/lib/libde265-teststreams

    DECODEHASH=`./dec265/dec265 testdata/girlshy.h265 -q -o - | md5sum - | cut -d " " -f1`
    if [ "$DECODEHASH" != "b81538fa33a67278e5263e231e43ca98" ]; then
	echo "Incorrect decoding result on testdata/girlshy.h265"
	exit 1
    else
	echo "testdata/girlshy.h265 decoded correctly"
    fi
fi
