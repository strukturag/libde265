cmake_minimum_required (VERSION 3.16.3)

project (libde265
    LANGUAGES C CXX
    VERSION 1.0.18
)

# Auto-compute BCD-encoded numeric version from project version.
# Each component is BCD-encoded: decimal 16 → 0x16.
# Result: 1.0.16 → 0x01001600
math(EXPR _maj_bcd "(${PROJECT_VERSION_MAJOR}/10)*16 + (${PROJECT_VERSION_MAJOR}%10)")
math(EXPR _min_bcd "(${PROJECT_VERSION_MINOR}/10)*16 + (${PROJECT_VERSION_MINOR}%10)")
math(EXPR _pat_bcd "(${PROJECT_VERSION_PATCH}/10)*16 + (${PROJECT_VERSION_PATCH}%10)")
math(EXPR NUMERIC_VERSION "${_maj_bcd} * 16777216 + ${_min_bcd} * 65536 + ${_pat_bcd} * 256" OUTPUT_FORMAT HEXADECIMAL)
set (PACKAGE_VERSION ${PROJECT_VERSION})

# --- Shared library version (ABI versioning, independent of project version) ---
#
# This controls the shared library filename:
#   libde265.so -> libde265.so.0 -> libde265.so.0.1.9
#                  ^^^^^^^^^^^^^^    ^^^^^^^^^^^^^^^^^^
#                  SOVERSION         VERSION
#
# VERSION is a three-part number: MAJOR.MINOR.PATCH
#   - MAJOR = the ABI major version (same as SOVERSION).
#             Bump when the ABI breaks (exported functions removed or signatures changed).
#             Reset MINOR and PATCH to 0.
#   - MINOR = backward-compatible ABI additions.
#             Bump when new exported functions are added but old ones still work.
#             Reset PATCH to 0.
#   - PATCH = implementation-only changes (bug fixes, performance improvements).
#             Bump when the ABI is unchanged.
#
# SOVERSION must always equal the MAJOR part of VERSION.
# Programs linked against libde265.so.0 will work with any libde265.so.0.x.y.
#
set(DE265_SOVERSION 0)
set(DE265_LIBRARY_VERSION "0.1.11")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(
  "$<$<AND:$<CONFIG:Debug>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-Wall;-Wextra;-Wpedantic;-Wno-unused-parameter>"
  "$<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:MSVC>>:/W4>"
)

option(USE_IWYU "Run include-what-you-use analysis during build" OFF)
if (USE_IWYU)
  find_program(IWYU_PATH include-what-you-use)
  if (IWYU_PATH)
    set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
    set(CMAKE_C_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
  else()
    message(WARNING "include-what-you-use not found, USE_IWYU ignored")
  endif()
endif()

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

check_c_source_compiles(
  "#if !defined(__arm__) && !defined(__aarch64__) \
  && !defined(_M_ARM) && !defined(_M_ARM64)
  #error not ARM
  #endif
  int main(){return 0;}"
  HAVE_ARM)

if(HAVE_ARM)
  enable_language(ASM)
  check_c_source_compiles(
    "#if !defined(__arm__) && !defined(_M_ARM)
    #error not 32-bit ARM
    #endif
    int main(){return 0;}"
    HAVE_ARM32)

  if(HAVE_ARM32)
    check_c_compiler_flag(-mfpu=neon HAVE_NEON)
  endif()
endif()

configure_file (libde265/de265-version.h.in libde265/de265-version.h)
configure_file (cmake/config.h.in config.h)
add_definitions(-DHAVE_CONFIG_H)

if(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  add_definitions(-Wall -Werror=return-type -Werror=unused-result -Werror=reorder)
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

find_package(Threads)

option(ENABLE_DECODER "Enable Decoder" ON)
option(ENABLE_ENCODER "Enable Encoder" OFF)
option(ENABLE_SHERLOCK265 "Build sherlock265 visual inspection tool" OFF)
option(ENABLE_INTERNAL_DEVELOPMENT_TOOLS "Build internal development tools (not for end users)" OFF)
option(WITH_FUZZERS "Build the fuzzers" OFF)
set(FUZZING_SANITIZER_OPTIONS "-fsanitize=address,shift,integer" "-fno-sanitize=unsigned-shift-base" "-fno-sanitize-recover=shift,integer" CACHE STRING "Sanitizer flags for fuzzing builds")

if (WITH_FUZZERS)
  add_compile_options("-fsanitize=fuzzer-no-link" ${FUZZING_SANITIZER_OPTIONS})
  add_link_options(${FUZZING_SANITIZER_OPTIONS})
endif()

add_subdirectory (libde265)
if (ENABLE_DECODER)
  add_subdirectory (dec265)
endif()
if (ENABLE_ENCODER)
  add_subdirectory (enc265)
endif()
if (ENABLE_SHERLOCK265)
  add_subdirectory (sherlock265)
endif()
if (ENABLE_INTERNAL_DEVELOPMENT_TOOLS)
  add_subdirectory (dev-tools)
endif()
if (WITH_FUZZERS)
  add_subdirectory (fuzzing)
endif()

add_custom_target(dist
  COMMAND git archive --format=tar.gz --worktree-attributes
    --prefix=${PROJECT_NAME}-${PROJECT_VERSION}/
    -o ${CMAKE_BINARY_DIR}/${PROJECT_NAME}-${PROJECT_VERSION}.tar.gz HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Creating source tarball ${PROJECT_NAME}-${PROJECT_VERSION}.tar.gz"
)
