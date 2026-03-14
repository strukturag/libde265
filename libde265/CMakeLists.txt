include(CMakePackageConfigHelpers)

set (libde265_sources
  alloc_pool.cc
  bitstream.cc
  cabac.cc
  contextmodel.cc
  de265.cc
  deblock.cc
  decctx.cc
  dpb.cc
  fallback-dct.cc
  fallback-motion.cc 
  fallback.cc
  image-io.cc
  image.cc
  intrapred.cc
  md5.cc
  motion.cc
  nal-parser.cc
  nal.cc
  pps.cc
  quality.cc
  refpic.cc
  sao.cc
  scan.cc
  sei.cc
  slice.cc
  sps.cc
  threads.cc
  transform.cc
  util.cc
  visualize.cc
  vps.cc
  vui.cc
)

set (libde265_headers
  acceleration.h
  alloc_pool.h
  bitstream.h
  cabac.h
  de265-version.h
  contextmodel.h
  de265.h
  deblock.h
  decctx.h
  dpb.h
  fallback-dct.h
  fallback-motion.h
  fallback.h
  image-io.h
  image.h
  intrapred.h
  md5.h
  motion.h
  nal-parser.h
  nal.h
  pps.h
  quality.h
  refpic.h
  sao.h
  scan.h
  sei.h
  slice.h
  sps.h
  threads.h
  transform.h
  util.h
  visualize.h
  vps.h
  vui.h
)

set (libde265_public_headers
  de265.h
  ${CMAKE_CURRENT_BINARY_DIR}/de265-version.h
)

if(MSVC OR MINGW)
  set (libde265_sources
    ${libde265_sources}
    ../extra/win32cond.c
    ../extra/win32cond.h
  )
endif()

add_definitions(-DLIBDE265_EXPORTS)

if (ENABLE_ENCODER)
  add_subdirectory (encoder)
  list(APPEND libde265_sources en265.cc)
  list(APPEND libde265_headers en265.h)
  list(APPEND libde265_public_headers en265.h)
endif()

if(HAVE_X86)
  if(SUPPORTS_SSE4_1 OR (SUPPORTS_SSE2 AND SUPPORTS_SSSE3))
    add_subdirectory (x86)
  endif()
endif()

if(HAVE_ARM)
  add_subdirectory (arm)
endif()

add_library(de265 ${libde265_sources} ${libde265_public_headers} ${ENCODER_OBJECTS} ${X86_OBJECTS} ${ARM_OBJECTS})
target_include_directories(de265 PRIVATE ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR})

write_basic_package_version_file(libde265ConfigVersion.cmake COMPATIBILITY ExactVersion)

# --- debug output

set(LOG_LEVELS error info debug trace)
set(DE265_LOG_LEVEL "error" CACHE STRING "Log level (${LOG_LEVELS})")

set_property(CACHE DE265_LOG_LEVEL PROPERTY STRINGS ${LOG_LEVELS})
if (NOT DE265_LOG_LEVEL IN_LIST LOG_LEVELS)
    message(FATAL_ERROR "DE265_LOG_LEVEL has to be one of: ${LOG_LEVELS}")
endif ()

if (DE265_LOG_LEVEL MATCHES "error")
    target_compile_definitions(de265 PRIVATE DE265_LOG_ERROR)
elseif (DE265_LOG_LEVEL MATCHES "info")
    target_compile_definitions(de265 PRIVATE DE265_LOG_ERROR DE265_LOG_INFO)
elseif (DE265_LOG_LEVEL MATCHES "debug")
    target_compile_definitions(de265 PRIVATE DE265_LOG_ERROR DE265_LOG_INFO DE265_LOG_DEBUG)
elseif (DE265_LOG_LEVEL MATCHES "trace")
    target_compile_definitions(de265 PRIVATE DE265_LOG_ERROR DE265_LOG_INFO DE265_LOG_DEBUG DE265_LOG_TRACE)
endif()


set_target_properties(de265 PROPERTIES
  VERSION "${DE265_LIBRARY_VERSION}"
  SOVERSION "${DE265_SOVERSION}"
)

if (WIN32)
    set_target_properties(de265 PROPERTIES PREFIX "lib")
endif()

if (BUILD_FRAMEWORK) 
  set_target_properties(de265 PROPERTIES
    FRAMEWORK TRUE
    FRAMEWORK_VERSION "${PROJECT_VERSION}"
    PRODUCT_BUNDLE_IDENTIFIER "github.com/strukturag/libde265"
    XCODE_ATTRIBUTE_INSTALL_PATH "@rpath"
    # OUTPUT_NAME "de265"
    XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY ""
    XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED "NO"
    XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED "NO"
    PUBLIC_HEADER "${libde265_public_headers}"
    MACOSX_FRAMEWORK_IDENTIFIER "github.com/strukturag/libde265"
    MACOSX_FRAMEWORK_BUNDLE_VERSION "${PROJECT_VERSION}"
    MACOSX_RPATH TRUE)
endif()

install(TARGETS de265 EXPORT libde265Config
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FRAMEWORK DESTINATION Library/Frameworks COMPONENT runtime OPTIONAL
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libde265
)

install(FILES ${libde265_public_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libde265)
install(EXPORT libde265Config DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/libde265")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libde265ConfigVersion.cmake DESTINATION
    "${CMAKE_INSTALL_LIBDIR}/cmake/libde265")


# --- pkg-config

set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix "\${prefix}")
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
    set(libdir "${CMAKE_INSTALL_LIBDIR}")
else()
    set(libdir "\${exec_prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
    set(includedir "${CMAKE_INSTALL_INCLUDEDIR}")
else()
    set(includedir "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()

set(VERSION ${PROJECT_VERSION}) # so that the replacement in libde265.pc will work with both autotools and CMake
configure_file(../libde265.pc.in ${CMAKE_CURRENT_BINARY_DIR}/libde265.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libde265.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
