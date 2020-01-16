cmake_minimum_required (VERSION 3.3.2)

project (libde265
    LANGUAGES C CXX
    VERSION 1.0.5
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The version number.
set (NUMERIC_VERSION 0x01000500)
set (PACKAGE_VERSION ${PROJECT_VERSION})

include (${CMAKE_ROOT}/Modules/CheckCCompilerFlag.cmake)
include (${CMAKE_ROOT}/Modules/CheckIncludeFile.cmake)

include(GNUInstallDirs)
include(CheckFunctionExists)

find_package(SDL)
find_package(Threads REQUIRED)

CHECK_INCLUDE_FILE(malloc.h HAVE_MALLOC_H)
CHECK_INCLUDE_FILE(stdint.h HAVE_STDINT_H)
CHECK_INCLUDE_FILE(stdbool.h HAVE_STDBOOL_H)
CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)

if (HAVE_MALLOC_H)
  add_definitions(-DHAVE_MALLOC_H)
endif()
if (HAVE_STDINT_H)
  add_definitions(-DHAVE_STDINT_H)
endif()
if (HAVE_STDBOOL_H)
  add_definitions(-DHAVE_STDBOOL_H)
endif()
if (HAVE_POSIX_MEMALIGN)
  add_definitions(-DHAVE_POSIX_MEMALIGN)
endif()

configure_file (libde265/de265-version.h.in libde265/de265-version.h)

if(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  add_definitions(-Wall)
endif()

option(DISABLE_SSE "Disable SSE optimizations")
if(NOT ${DISABLE_SSE} EQUAL OFF)
  if(MSVC)
    set(SUPPORTS_SSE4_1 1)
  else()
    CHECK_C_COMPILER_FLAG(-msse4.1 SUPPORTS_SSE4_1)
  endif()
endif()

include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_BINARY_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/libde265")
if(MSVC)
  include_directories ("${PROJECT_SOURCE_DIR}/extra")
  add_definitions(-DHAVE_STDINT_H)
  add_definitions(-DHAVE_STDBOOL_H)
endif()

add_subdirectory (libde265)
add_subdirectory (dec265)
add_subdirectory (enc265)
