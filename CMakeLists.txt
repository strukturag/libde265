cmake_minimum_required (VERSION 3.3.2)

project (libde265
    LANGUAGES C CXX
    VERSION 1.0.11
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The version number.
set (NUMERIC_VERSION 0x01001100)
set (PACKAGE_VERSION ${PROJECT_VERSION})

include (${CMAKE_ROOT}/Modules/CheckCCompilerFlag.cmake)
include (${CMAKE_ROOT}/Modules/CheckIncludeFile.cmake)

include(GNUInstallDirs)
include(CheckFunctionExists)

option(ENABLE_SDL "Enable SDL" ON)
option(ENABLE_PNG_OUTPUT "Enable PNG output" ON)

if (ENABLE_SDL)
	find_package(SDL)
endif()

find_package(Threads REQUIRED)

CHECK_INCLUDE_FILE(malloc.h HAVE_MALLOC_H)
CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)

if (HAVE_MALLOC_H)
  add_definitions(-DHAVE_MALLOC_H)
endif()
if (HAVE_POSIX_MEMALIGN)
  add_definitions(-DHAVE_POSIX_MEMALIGN)
endif()

configure_file (libde265/de265-version.h.in libde265/de265-version.h)

if(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  add_definitions(-Wall)
endif()

option(DISABLE_SSE "Disable SSE optimizations" OFF)

option(BUILD_SHARED_LIBS "Build shared library" ON)
if(NOT BUILD_SHARED_LIBS)
  add_definitions(-DLIBDE265_STATIC_BUILD)
endif()

if(MSVC)
  include_directories ("${PROJECT_SOURCE_DIR}/extra")
endif()

option(ENABLE_DECODER "Enable Decoder" ON)
option(ENABLE_ENCODER "Enable Encoder" OFF)

add_subdirectory (libde265)
if (ENABLE_DECODER)
  add_subdirectory (dec265)
endif()
if (ENABLE_ENCODER)
  add_subdirectory (enc265)
endif()
