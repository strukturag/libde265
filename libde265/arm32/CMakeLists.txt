add_library(arm32 OBJECT arm.cc arm.h)

if(HAVE_NEON)
  add_library(arm32_neon OBJECT
    cpudetect.S
    hevcdsp_qpel_neon.S
  )

  # Clang's assembler does not support .func/.endfunc directives.
  if(NOT CMAKE_ASM_COMPILER_ID MATCHES Clang)
    set(AS_FUNC_FLAG -DHAVE_AS_FUNC)
  endif()

  target_compile_options(arm32_neon PRIVATE
    -mfpu=neon
    -DHAVE_NEON
    -DEXTERN_ASM=
    ${AS_FUNC_FLAG}
    -DHAVE_SECTION_DATA_REL_RO
  )

  set(ARM32_OBJECTS $<TARGET_OBJECTS:arm32> $<TARGET_OBJECTS:arm32_neon> PARENT_SCOPE)
else()
  set(ARM32_OBJECTS $<TARGET_OBJECTS:arm32> PARENT_SCOPE)
endif()
