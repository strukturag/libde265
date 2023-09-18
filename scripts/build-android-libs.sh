#!/bin/bash

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
