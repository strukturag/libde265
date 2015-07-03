set (x86_sources 
  sse.cc sse.h
)

set (x86_sse_sources 
  sse-motion.cc sse-motion.h sse-dct.h sse-dct.cc
)

add_library(x86 STATIC ${x86_sources})

add_library(x86_sse STATIC ${x86_sse_sources})

set(sse_flags "")

if(NOT MSVC)
  set(sse_flags "${sse_flags} -msse4.1")
endif()

target_link_libraries(x86 x86_sse)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  SET_TARGET_PROPERTIES(x86 PROPERTIES COMPILE_FLAGS "-fPIC")
  SET_TARGET_PROPERTIES(x86_sse PROPERTIES COMPILE_FLAGS "-fPIC ${sse_flags}")
endif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
