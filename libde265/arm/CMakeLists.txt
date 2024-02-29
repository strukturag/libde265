set (arm_sources 
  neon_common.h 
  neon_dct.cc 
  neon_dct.h 
  neon_intrapred.cc 
  neon_intrapred.h 
  arm.cc 
  arm.h
)

add_library(arm OBJECT ${arm_sources})

set(ARM_OBJECTS $<TARGET_OBJECTS:arm>  PARENT_SCOPE)