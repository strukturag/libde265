
# --- dec265

add_executable (dec265 dec265.cc)

target_include_directories (dec265 PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories (dec265 PRIVATE ${PROJECT_BINARY_DIR})

target_link_libraries (dec265 PRIVATE de265)

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

# --- au-dec265

add_executable (au-dec265 au-dec265.cc)

target_include_directories (au-dec265 PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories (au-dec265 PRIVATE ${PROJECT_BINARY_DIR})

target_link_libraries (au-dec265 PRIVATE de265)

if(SDL_FOUND)
  target_sources(au-dec265 PRIVATE sdl.cc)
  target_compile_definitions(au-dec265 PRIVATE HAVE_SDL)
  target_include_directories (au-dec265 PRIVATE "${SDL_INCLUDE_DIR}")
  target_link_libraries (au-dec265 PRIVATE ${SDL_LIBRARY})
endif()

if (ENABLE_PNG_OUTPUT)
    find_package(PNG)
    if (PNG_FOUND)
        add_definitions(-DHAVE_LIBPNG=1)
        target_link_libraries(au-dec265 PRIVATE ${PNG_LIBRARIES})
        target_include_directories(au-dec265 PRIVATE ${PNG_INCLUDE_DIR})
        target_sources(au-dec265 PRIVATE encoder_png.cc encoder_png.h)
    endif ()
endif ()

if(MSVC)
  target_sources(au-dec265 PRIVATE
          ../extra/getopt.c
          ../extra/getopt_long.c
          )
endif()

install (TARGETS au-dec265 DESTINATION ${CMAKE_INSTALL_BINDIR})

# --- hdrcopy

if(NOT MSVC)
  # hdrcopy uses internal APIs that are not available when compiled for Windows
  add_executable (hdrcopy hdrcopy.cc)
  target_include_directories (hdrcopy PRIVATE ${PROJECT_SOURCE_DIR})
  target_include_directories (hdrcopy PRIVATE ${PROJECT_BINARY_DIR})

  target_link_libraries (hdrcopy PRIVATE de265)

  install (TARGETS hdrcopy DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
