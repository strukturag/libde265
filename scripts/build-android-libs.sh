#!/bin/bash

#  MIT License
#
#  Copyright (c) 2023, Dirk Farin <dirk.farin@gmail.com>
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.


# This script builds binaries for all Android architectures. They are placed in the directory 'out'.
# The libde265 source has to be unpacked to a directory named "libde265-1.0.12" (replace version number with the value of LIBDE265_VERSION below).

# Set these variables to suit your needs.

NDK_PATH= ... # E.g. .../android-sdk/sdk/ndk/25.1.8937393
TOOLCHAIN=clang
ANDROID_VERSION=15 # the minimum version of Android to support
LIBDE265_VERSION=1.0.12

function build {
    mkdir -p build/$1
    mkdir -p out/$1
    cd build/$1
    cmake -G"Unix Makefiles" \
	  -DANDROID_ABI=$1 \
	  -DANDROID_ARM_MODE=arm \
	  -DANDROID_PLATFORM=android-${ANDROID_VERSION} \
	  -DANDROID_TOOLCHAIN=${TOOLCHAIN} \
	  -DCMAKE_ASM_FLAGS="--target=arm-linux-androideabi${ANDROID_VERSION}" \
	  -DCMAKE_TOOLCHAIN_FILE=${NDK_PATH}/build/cmake/android.toolchain.cmake \
	  -DENABLE_SDL=OFF \
	  -DENABLE_PNG_OUTPUT=OFF \
	  -DDISABLE_SSE=$2 \
	  -DCMAKE_INSTALL_PREFIX=../../out/$1 \
	  -DCMAKE_BUILD_TYPE=Release \
	  -Dld-version-script=OFF \
	  ../../libde265-${LIBDE265_VERSION}
    make VERBOSE=1
    make install
    cd ../..
}

rm -rf build out


## NEON code does not work for ARM64

build armeabi-v7a ON
build arm64-v8a ON
build x86 OFF
build x86_64 OFF

rm -rf build
rm -rf out/*/bin
rm -rf out/*/include/libde265/en265.h
