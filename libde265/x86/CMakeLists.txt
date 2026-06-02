set (x86_sources 
  sse.cc sse.h
)

set (x86_sse_sources
  sse-motion.cc sse-motion.h sse-dct.h sse-dct.cc sse-intrapred.h sse-intrapred.cc
)

set (x86_avx2_sources
  transform-avx2.cc transform-avx2.h transform-dct-tables.h
)

set (x86_avx512_sources
  transform-avx512.cc transform-avx512.h
)

add_library(x86 OBJECT ${x86_sources})

add_library(x86_sse OBJECT ${x86_sse_sources})

set(sse_flags "")

if(NOT MSVC)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(sse_flags "${sse_flags} -msse4.1")
  else(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(sse_flags "${sse_flags} -msse2 -mssse3 -msse4.1")
  endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
endif()

SET_TARGET_PROPERTIES(x86_sse PROPERTIES COMPILE_FLAGS "${sse_flags}")

set(X86_OBJECTS $<TARGET_OBJECTS:x86> $<TARGET_OBJECTS:x86_sse>)

# AVX2 kernels (only compiled where supported). The dispatch/detection code lives
# in the plain x86 object (sse.cc) so it never runs AVX2 instructions before the
# runtime CPU check.
if(HAVE_AVX2)
  add_library(x86_avx2 OBJECT ${x86_avx2_sources})
  if(MSVC)
    SET_TARGET_PROPERTIES(x86_avx2 PROPERTIES COMPILE_FLAGS "/arch:AVX2")
  else()
    SET_TARGET_PROPERTIES(x86_avx2 PROPERTIES COMPILE_FLAGS "-mavx2")
  endif()
  set(X86_OBJECTS ${X86_OBJECTS} $<TARGET_OBJECTS:x86_avx2>)
endif()

if(HAVE_AVX512)
  add_library(x86_avx512 OBJECT ${x86_avx512_sources})
  if(MSVC)
    SET_TARGET_PROPERTIES(x86_avx512 PROPERTIES COMPILE_FLAGS "/arch:AVX512")
  else()
    SET_TARGET_PROPERTIES(x86_avx512 PROPERTIES COMPILE_FLAGS "-mavx512f -mavx512bw")
  endif()
  set(X86_OBJECTS ${X86_OBJECTS} $<TARGET_OBJECTS:x86_avx512>)
endif()

set(X86_OBJECTS ${X86_OBJECTS} PARENT_SCOPE)
