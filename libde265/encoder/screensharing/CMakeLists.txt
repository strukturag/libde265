set (encoder_screensharing_sources
  screensharing-core.cc screensharing-core.h
)

add_library(encoder_screensharing OBJECT ${encoder_screensharing_sources})
set(ENCODER_SCREENSHARING_OBJECTS $<TARGET_OBJECTS:encoder_screensharing> PARENT_SCOPE)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  SET_TARGET_PROPERTIES(encoder_screensharing PROPERTIES COMPILE_FLAGS "-fPIC")
endif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
