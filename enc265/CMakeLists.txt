add_executable (enc265 
  enc265.cc
  image-io-png.cc
)

if(MSVC)
  target_sources(enc265 PRIVATE
    ../extra/getopt.c
    ../extra/getopt_long.c
  )
endif()

target_link_libraries (enc265 PRIVATE de265)

install (TARGETS enc265 DESTINATION ${CMAKE_INSTALL_BINDIR})
