set(CMAKE_CONFIGURATION_TYPES "Debug;Release;RelWithDebInfo" CACHE STRING "Configurations" FORCE)

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
#    add_definitions(/MP)  # multithreaded build
#    add_definitions(/TP)  # all files as cpp

    include_directories(../extra)
endif(MSVC)

if("$ENV{CXX}" STREQUAL "icpc")
    set(GCC 1)
    add_definitions(-Wall -Wextra -Wunused-variable -Wunused-function -Wshadow -no-vec -msse4)
elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(GCC 1)
    add_definitions(-Wall -Wextra -Wunused-variable -Wunused-function -Wshadow -msse4)
endif()

option(USE_ASM "Use SSE41 SIMD optimize" ON)
if(USE_ASM)
    add_definitions(-DUSE_ASM=ASM_SSE4)
    add_definitions(-DHAVE_SSE4_1)
else(USE_ASM)
    add_definitions(-DUSE_ASM=ASM_NONE)
endif(USE_ASM)

option(USE_VIDEOGFX "Use of VIDEOGFX" OFF)
if(USE_VIDEOGFX)
    add_definitions(-D HAVE_VIDEOGFX=1)
else(USE_VIDEOGFX)
    add_definitions(-D HAVE_VIDEOGFX=0)
endif(USE_VIDEOGFX)

option(BUILD_STATIC  "Static Lib" ON)
if(BUILD_STATIC)
    add_definitions(-DLIBDE265_STATIC_BUILD)
endif(BUILD_STATIC)

option(DE265_LOG_ERROR  "Enable DE265_LOG_ERROR" OFF)
if(DE265_LOG_ERROR)
    add_definitions(-DDE265_LOG_ERROR)
endif(DE265_LOG_ERROR)

option(DE265_LOG_INFO   "Enable DE265_LOG_INFO"  OFF)
if(DE265_LOG_INFO)
    add_definitions(-DDE265_LOG_INFO)
endif(DE265_LOG_INFO)

option(DE265_LOG_DEBUG  "Enable DE265_LOG_DEBUG" OFF)
if(DE265_LOG_DEBUG)
    add_definitions(-DDE265_LOG_DEBUG)
endif(DE265_LOG_DEBUG)

option(DE265_LOG_TRACE  "Enable DE265_LOG_TRACE" OFF)
if(DE265_LOG_TRACE)
    add_definitions(-DDE265_LOG_TRACE)
endif(DE265_LOG_TRACE)

include_directories(../ ../libde265)

file(GLOB LIBSRC ../libde265/*.c ../extra/*.c)
file(GLOB LIBINC ../libde265/*.h ../extra/*.h)
file(GLOB APPSRC dec265.cc)
file(GLOB ASMSRC0 ../libde265/x86/sse.c ../libde265/x86/sse-dct.c)
file(GLOB ASMSRC1 ../libde265/x86/sse-motion.c)
file(GLOB ASMINC ../libde265/x86/*.h)

source_group(INC  FILES ${LIBINC})
source_group(SRC  FILES ${LIBSRC})
source_group(APP  FILES ${APPSRC})
source_group(ASM  FILES ${ASMSRC0} ${ASMSRC1} ${ASMINC})

if(USE_ASM)
    SET(LIBSRC ${LIBSRC} ${ASMSRC0} ${ASMSRC1})
    SET(LIBINC ${LIBINC} ${ASMINC})

    # disable uninitialize check on VC, because x1 is not assign, remove later
    #if(MSVC)
    #    SET_SOURCE_FILES_PROPERTIES(${ASMSRC1} PROPERTIES COMPILE_FLAGS "/RTCs")
    #endif(MSVC)
endif(USE_ASM)

if(UNIX)
    SET(PLATFORM_LIBS pthread rt m)
    SET_SOURCE_FILES_PROPERTIES(../extra/win32thread.c PROPERTIES HEADER_FILE_ONLY TRUE)
endif(UNIX)

# Main CLI application
if(MSVC)
    add_definitions(/wd4244) # type conversion, possible loss of data
    add_definitions(/wd4100) # unreferenced formal parameter
    add_definitions(/wd4505) # unreferenced local function has been removed
    add_definitions(/wd4701) # potentially uninitialized local variable
    add_definitions(/wd4127) # conditional expression is constant
#    add_definitions(/wd4018) # signed/unsigned mismatch
    add_definitions(/wd4189) # local variable is initialized but not referenced
    add_definitions(/wd4715) # not all control paths return a value

    set_source_files_properties(${LIBSRC} PROPERTIES LANGUAGE CXX)
endif(MSVC)
if(GCC)
    add_definitions(-Wno-sign-compare)
    add_definitions(-Wno-unused-parameter)
endif(GCC)

add_executable(dec265
    ${APPSRC} ${EXTRAS} ${LIBSRC} ${LIBINC}
)

target_link_libraries(dec265 ${PLATFORM_LIBS})

