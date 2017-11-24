set (libde265_sources
  acceleration.h acceleration.cc
  bitstream.cc
  cabac.cc
  de265.cc
  deblock.cc
  decctx.cc
  nal-parser.cc
  nal-parser.h
  dpb.cc
  dpb.h
  frame-dropper.h
  frame-dropper.cc
  frontend-syntax-decoder.h
  frontend-syntax-decoder.cc
  funcs.cc
  image.cc
  intrapred.cc
  md5.cc
  nal.cc
  pps.cc
  transform.cc
  refpic.cc
  sao.cc
  scan.cc
  sei.cc
  slice.cc
  sps.cc
  util.cc
  vps.cc
  bitstream.h
  cabac.h
  deblock.h
  decctx.h
  image.h
  image-unit.h
  image-unit.cc
  intrapred.h
  md5.h
  nal.h
  pps.h
  transform.h
  refpic.h
  sao.h
  scan.h
  sei.h
  slice.h
  sps.h
  util.h
  vps.h
  vui.h vui.cc
  motion.cc motion.h
  threads.cc threads.h
  visualize.cc visualize.h
  fallback.cc fallback.h fallback-motion.cc fallback-motion.h
  fallback-pixelformat.cc fallback-pixelformat.h
  fallback-dct.h fallback-dct.cc fallback-sao.h fallback-sao.cc
  fallback-intra-dc.cc
  quality.cc quality.h
  configparam.cc configparam.h
  image-io.h image-io.cc
  alloc_pool.h alloc_pool.cc
  en265.h en265.cc
  contextmodel.cc
)

# set(CMAKE_VERBOSE_MAKEFILE on)

if(MSVC)
  set (libde265_sources
    ${libde265_sources}
    ../extra/win32cond.c
    ../extra/win32cond.h
  )
endif()

add_definitions(-DLIBDE265_EXPORTS)

add_subdirectory (encoder)

# message(WARNING " CMAKE_SYSTEM_PROCESSOR = ${CMAKE_SYSTEM_PROCESSOR}")

if (# (CMAKE_SYSTEM_PROCESSOR STREQUAL "i686") OR ## when compiling for i686 for Android, compiler says that there is no SSE4.1 support
    (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
    if(SUPPORTS_SSE4_1)
      add_definitions(-DHAVE_SSE4_1)
      add_subdirectory (x86)
    endif()
endif()

if ((CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7-a") OR
    (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64"))
  add_definitions(-DHAVE_AARCH64 -DHAVE_NEON)
  add_subdirectory (arm)
endif()

add_library(${LIBDE265_LIBRARY_NAME} SHARED ${libde265_sources} ${ENCODER_OBJECTS} ${X86_OBJECTS} ${ARM_OBJECTS})
target_link_libraries(${LIBDE265_LIBRARY_NAME} ${CMAKE_THREAD_LIBS_INIT})

check_library_exists(rt clock_gettime "time.h" NEED_LIBRT)
if(NEED_LIBRT)
  target_link_libraries(${LIBDE265_LIBRARY_NAME} rt)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  SET_TARGET_PROPERTIES(${LIBDE265_LIBRARY_NAME} PROPERTIES COMPILE_FLAGS "-fPIC")
endif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
