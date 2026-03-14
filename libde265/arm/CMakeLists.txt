add_library(arm OBJECT arm.cc arm.h)

if(HAVE_NEON)
  add_library(arm_neon OBJECT
    asm.S
    cpudetect.S
    hevcdsp_qpel_neon.S
    neon.S
  )

  target_compile_options(arm_neon PRIVATE
    -mfpu=neon
    -DHAVE_NEON
    -DEXTERN_ASM=
    -DHAVE_AS_FUNC
    -DHAVE_SECTION_DATA_REL_RO
  )

  set(ARM_OBJECTS $<TARGET_OBJECTS:arm> $<TARGET_OBJECTS:arm_neon> PARENT_SCOPE)
else()
  set(ARM_OBJECTS $<TARGET_OBJECTS:arm> PARENT_SCOPE)
endif()
