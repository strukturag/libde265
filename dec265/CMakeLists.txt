if(NOT CMAKE_BUILD_TYPE)
    # default to Release build for GCC builds
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release."
        FORCE)
endif()

project (dec265)
cmake_minimum_required (VERSION 2.8)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake" "${CMAKE_MODULE_PATH}")

if ("${CMAKE_SIZEOF_VOID_P}" MATCHES 8)
    set(X64 1)
    add_definitions(-DX86_64)
endif()

# Enforce coding standards.  Full warnings and warnings as errors
if(MSVC)
    add_definitions(/W4 /D_CRT_SECURE_NO_WARNINGS)
    add_definitions(/Ob2) # always inline
    add_definitions(/Oi)  # enable intrinsics
    add_definitions(/MP)  # multithreaded build
    add_definitions(/TP)  # all files as cpp

    include_directories(../extra)
endif(MSVC)

if("$ENV{CXX}" STREQUAL "icpc")
    set(GCC 1)
    add_definitions(-Wall -Wextra -Wunused-variable -Wunused-function -Wshadow -no-vec -msse4)
elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(GCC 1)
    add_definitions(-Wall -Wextra -Wunused-variable -Wunused-function -Wshadow -msse4)
endif()

option(USE_ASM "Use of SIMD optimize" ON)
if(USE_ASM)
    add_definitions(-DUSE_ASM=ASM_SSE4)
else(USE_ASM)
    add_definitions(-DUSE_ASM=ASM_NONE)
endif(USE_ASM)

if(UNIX)
    SET(PLATFORM_LIBS pthread rt m)
endif(UNIX)

include_directories(../ ../libde265)

# Main CLI application
if(MSVC)
    add_definitions(/wd4244) # type conversion, possible loss of data
    add_definitions(/wd4100) # unreferenced formal parameter
    add_definitions(/wd4505) # unreferenced local function has been removed
    add_definitions(/wd4701) # potentially uninitialized local variable
    add_definitions(/wd4127) # conditional expression is constant
#    add_definitions(/wd4018) # signed/unsigned mismatch
    add_definitions(/wd4189) # local variable is initialized but not referenced
endif(MSVC)
if(GCC)
    add_definitions(-Wno-sign-compare)
    add_definitions(-Wno-unused-parameter)
endif(GCC)


file(GLOB LIBSRC ../libde265/*.c)
file(GLOB LIBINC ../libde265/*.h)
file(GLOB APPSRC dec265.cc)

source_group(INC  FILES ${LIBINC})
source_group(SRC  FILES ${LIBSRC})
source_group(APP  FILES ${APPSRC})

add_executable(dec265
    dec265.cc ${EXTRAS} ${LIBSRC} ${LIBINC}
)

target_link_libraries(dec265 ${PLATFORM_LIBS})

