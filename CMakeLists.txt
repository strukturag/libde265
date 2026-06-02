cmake_minimum_required (VERSION 3.16.3)

project (libde265
    LANGUAGES C CXX
    VERSION 1.1.0
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
#                  DE265_SOVERSION   DE265_LIBRARY_VERSION
#
# DE265_LIBRARY_VERSION is a three-part number: MAJOR.MINOR.PATCH
#   - MAJOR = the ABI major version (same as DE265_SOVERSION).
#             Bump when the ABI breaks (exported functions removed or signatures changed).
#             Reset MINOR and PATCH to 0.
#   - MINOR = backward-compatible ABI additions.
#             Bump when new exported functions are added but old ones still work.
#             Reset PATCH to 0.
#   - PATCH = implementation-only changes (bug fixes, performance improvements).
#             Bump when the ABI is unchanged.
#
# DE265_SOVERSION must always equal the MAJOR part of DE265_LIBRARY_VERSION.
# Programs linked against libde265.so.0 will work with any libde265.so.0.x.y.
#
set(DE265_SOVERSION 0)
set(DE265_LIBRARY_VERSION "0.2.0")

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

option(ENABLE_SIMD "Enable SIMD acceleration (SSE on x86, NEON on ARM)" ON)
option(ENABLE_AVX2 "Enable AVX2 kernels on x86 (in addition to SSE)" ON)
option(ENABLE_AVX512 "Enable AVX-512 kernels on x86 (in addition to AVX2)" ON)

include(CheckCSourceCompiles)
if(ENABLE_SIMD)
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
      check_c_compiler_flag(-mavx2 SUPPORTS_AVX2)
      check_c_compiler_flag("-mavx512f -mavx512bw" SUPPORTS_AVX512)
    endif()
    if(SUPPORTS_SSE4_1)
      set(HAVE_SSE4_1 TRUE)
    endif()
    # AVX2 kernels back the SSE4.1 ones; only built on 64-bit targets.
    if(ENABLE_AVX2 AND SUPPORTS_AVX2 AND HAVE_SSE4_1 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(HAVE_AVX2 TRUE)
    endif()
    # AVX-512 (F+BW) kernels back the AVX2 ones.
    if(ENABLE_AVX512 AND SUPPORTS_AVX512 AND HAVE_AVX2)
      set(HAVE_AVX512 TRUE)
    endif()
  endif()

  check_c_source_compiles(
    "#if !defined(__arm__) && !defined(_M_ARM)
    #error not 32-bit ARM
    #endif
    int main(){return 0;}"
    HAVE_ARM32)

  check_c_source_compiles(
    "#if !defined(__aarch64__) && !defined(_M_ARM64)
    #error not 64-bit ARM
    #endif
    int main(){return 0;}"
    HAVE_ARM64)

  if(HAVE_ARM32)
    enable_language(ASM)
    check_c_compiler_flag(-mfpu=neon HAVE_NEON)
  endif()
else()
  message(STATUS "SIMD acceleration disabled (ENABLE_SIMD=OFF) — using scalar fallback only")
endif()

configure_file (libde265/de265-version.h.in libde265/de265-version.h)
configure_file (cmake/config.h.in config.h)
add_definitions(-DHAVE_CONFIG_H)

if(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  add_definitions(-Wall -Werror=return-type -Werror=unused-result -Werror=reorder)
endif()

set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include(CheckCXXSymbolExists)
check_cxx_symbol_exists(_LIBCPP_VERSION cstdlib HAVE_LIBCPP)

# Build Libs.private contents for libde265.pc.
set(_libs_private "")
if(NOT MSVC)
  if(HAVE_LIBCPP)
    list(APPEND _libs_private "-lc++")
  else()
    list(APPEND _libs_private "-lstdc++")
  endif()
endif()
# Emit '-lpthread' rather than '-pthread' so consumers using
# `pkg-config --libs --static` get an unambiguous linker flag (see #453).
if(CMAKE_USE_PTHREADS_INIT)
  list(APPEND _libs_private "-lpthread")
endif()
if(UNIX)
  list(APPEND _libs_private "-lm")
endif()
string(JOIN " " LIBS_PRIVATE ${_libs_private})


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
