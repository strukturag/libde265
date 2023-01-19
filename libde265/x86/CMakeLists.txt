
add_library(x86 OBJECT sse.cc sse.h)

add_library(x86_sse OBJECT sse-motion.cc sse-motion.h sse-dct.h sse-dct.cc)

target_include_directories (x86 PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories (x86_sse PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories (x86 PRIVATE ${PROJECT_BINARY_DIR})
target_include_directories (x86_sse PRIVATE ${PROJECT_BINARY_DIR})

set(sse_flags "")

if(NOT MSVC)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(sse_flags "${sse_flags} -msse4.1")
  else(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(sse_flags "${sse_flags} -msse2 -mssse3 -msse4.1")
  endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
endif()

set(X86_OBJECTS $<TARGET_OBJECTS:x86> $<TARGET_OBJECTS:x86_sse> PARENT_SCOPE)

SET_TARGET_PROPERTIES(x86_sse PROPERTIES COMPILE_FLAGS "${sse_flags}")
