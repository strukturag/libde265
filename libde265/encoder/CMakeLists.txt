set (encoder_sources
  encoder-core.cc encoder-core.h
  encoder-types.h encoder-types.cc
  encoder-params.h encoder-params.cc
  encoder-context.h encoder-context.cc
  encoder-syntax.h encoder-syntax.cc
  encoder-intrapred.h encoder-intrapred.cc
  encoder-motion.h encoder-motion.cc
  encpicbuf.h encpicbuf.cc
  sop.h sop.cc
)

add_subdirectory (algo)
add_library(encoder OBJECT ${encoder_sources})
set(ENCODER_OBJECTS $<TARGET_OBJECTS:encoder> ${ALGO_OBJECTS} PARENT_SCOPE)
