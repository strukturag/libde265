add_executable (dec265 dec265.cc)

target_link_libraries (dec265 PRIVATE ${PROJECT_NAME})

if(SDL_FOUND)
  target_sources(dec265 PRIVATE sdl.cc)
  target_compile_definitions(dec265 PRIVATE HAVE_SDL)
  target_include_directories (dec265 PRIVATE "${SDL_INCLUDE_DIR}")
  target_link_libraries (dec265 PRIVATE ${SDL_LIBRARY})
endif()

if(MSVC)
  target_sources(dec265 PRIVATE 
    ../extra/getopt.c
    ../extra/getopt_long.c
  )
endif()

install (TARGETS dec265 DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT MSVC)
  # hdrcopy uses internal APIs that are not available when compiled for Windows
  add_executable (hdrcopy hdrcopy.cc)

  target_link_libraries (hdrcopy PRIVATE ${PROJECT_NAME})

  install (TARGETS hdrcopy DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
