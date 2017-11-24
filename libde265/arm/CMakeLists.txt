set (arm_sources
  arm.cc arm.h
  arm-dct.cc  arm-dct.h arm-motion.cc  arm-motion.h
)

add_library(arm OBJECT ${arm_sources})

set(arm_flags "")

set(ARM_OBJECTS $<TARGET_OBJECTS:arm> PARENT_SCOPE)
