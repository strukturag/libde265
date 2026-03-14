set(tool_targets
  gen-enc-table
  yuv-distortion
  rd-curves
  block-rate-estim
  tests
  bjoentegaard
)

set(gen-enc-table_SOURCES gen-entropy-table.cc)
set(yuv-distortion_SOURCES yuv-distortion.cc)
set(rd-curves_SOURCES rd-curves.cc)
set(block-rate-estim_SOURCES block-rate-estim.cc)
set(tests_SOURCES tests.cc)
set(bjoentegaard_SOURCES bjoentegaard.cc)

foreach(tool ${tool_targets})
  add_executable(${tool} ${${tool}_SOURCES})
  target_link_libraries(${tool} PRIVATE de265)
endforeach()

install(TARGETS ${tool_targets} DESTINATION ${CMAKE_INSTALL_BINDIR})
