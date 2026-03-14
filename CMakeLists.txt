cmake_minimum_required (VERSION 3.16.3)

project (libde265
    LANGUAGES C CXX
    VERSION 1.0.16
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The version number.
set (NUMERIC_VERSION 0x01001600)
set (PACKAGE_VERSION ${PROJECT_VERSION})

include (${CMAKE_ROOT}/Modules/CheckCCompilerFlag.cmake)
include (${CMAKE_ROOT}/Modules/CheckIncludeFile.cmake)

include(GNUInstallDirs)
include(CheckFunctionExists)

option(ENABLE_SDL "Enable SDL" ON)

if (ENABLE_SDL)
	find_package(SDL2)
endif()

CHECK_INCLUDE_FILE(malloc.h HAVE_MALLOC_H)
CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)

include(CheckCSourceCompiles)
check_c_source_compiles(
  "#if !defined(__x86_64) && !defined(__i386__) \
  && !defined(_M_IX86) && !defined(_M_AMD64) \
  || defined(_M_ARM64EC) || defined(_ARM64EC_)
  #error not x86
  #endif
  int main(){return 0;}"
  HAVE_X86)

if(HAVE_X86)
  if (MSVC)
    set(SUPPORTS_SSE2 1)
    set(SUPPORTS_SSSE3 1)
    set(SUPPORTS_SSE4_1 1)
  else()
    check_c_compiler_flag(-msse2 SUPPORTS_SSE2)
    check_c_compiler_flag(-mssse3 SUPPORTS_SSSE3)
    check_c_compiler_flag(-msse4.1 SUPPORTS_SSE4_1)
  endif()
  if(SUPPORTS_SSE4_1)
    set(HAVE_SSE4_1 TRUE)
  endif()
endif()

configure_file (libde265/de265-version.h.in libde265/de265-version.h)
configure_file (cmake/config.h.in config.h)
add_definitions(-DHAVE_CONFIG_H)

if(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  add_definitions(-Wall)
endif()

include(CheckCXXSymbolExists)
check_cxx_symbol_exists(_LIBCPP_VERSION cstdlib HAVE_LIBCPP)
if(HAVE_LIBCPP)
  set(LIBS_PRIVATE "-lc++")
else()
  set(LIBS_PRIVATE "-lstdc++")
endif()

option(BUILD_SHARED_LIBS "Build shared library" ON)
if(NOT BUILD_SHARED_LIBS)
  add_definitions(-DLIBDE265_STATIC_BUILD)
endif()

if(APPLE)
  option(BUILD_FRAMEWORK "Build as Apple Frameworks" OFF)
endif()

include_directories ("${PROJECT_BINARY_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/libde265")
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
