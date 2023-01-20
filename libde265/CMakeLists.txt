include(CMakePackageConfigHelpers)

add_definitions(-DLIBDE265_EXPORTS)

add_subdirectory(encoder)

if (NOT DISABLE_SSE)
    if (MSVC)
        set(SUPPORTS_SSE2 1)
        set(SUPPORTS_SSSE3 1)
        set(SUPPORTS_SSE4_1 1)
    else (MSVC)
        check_c_compiler_flag(-msse2 SUPPORTS_SSE2)
        check_c_compiler_flag(-mssse3 SUPPORTS_SSSE3)
        check_c_compiler_flag(-msse4.1 SUPPORTS_SSE4_1)
    endif (MSVC)

    if (SUPPORTS_SSE4_1)
        add_definitions(-DHAVE_SSE4_1)
    endif ()
    if (SUPPORTS_SSE4_1 OR (SUPPORTS_SSE2 AND SUPPORTS_SSSE3))
        add_subdirectory(x86)
    endif ()
endif ()

add_library(de265
        acceleration.h
        alloc_pool.h
        alloc_pool.cc
        bitstream.h
        bitstream.cc
        cabac.h
        cabac.cc
        configparam.h
        configparam.cc
        contextmodel.h
        contextmodel.cc
        de265.h
        de265.cc
        deblock.h
        deblock.cc
        decctx.h
        decctx.cc
        dpb.h
        dpb.cc
        #        en265.h
        #        en265.cc
        fallback-dct.h
        fallback-dct.cc
        fallback-motion.h
        fallback-motion.cc
        fallback.h
        fallback.cc
        image-io.h
        image-io.cc
        image.h
        image.cc
        intrapred.h
        intrapred.cc
        md5.h
        md5.cc
        motion.h
        motion.cc
        nal-parser.h
        nal-parser.cc
        nal.h
        nal.cc
        pps.h
        pps.cc
        quality.h
        quality.cc
        refpic.h
        refpic.cc
        sao.h
        sao.cc
        scan.h
        scan.cc
        sei.h
        sei.cc
        slice.h
        slice.cc
        sps.h
        sps.cc
        threads.h
        threads.cc
        transform.h
        transform.cc
        util.h
        util.cc
        visualize.h
        visualize.cc
        vps.h
        vps.cc
        vui.h
        vui.cc
        #        de265-version.h
        ${X86_OBJECTS} de265-multilayer.h de265-multilayer.cc)

if (MSVC OR MINGW)
    target_sources(de265
            ../extra/win32cond.c
            ../extra/win32cond.h
            )
endif ()


if (ENABLE_ENCODER)
    target_sources(de265 ${ENCODER_OBJECTS})
endif ()

target_include_directories(de265 PRIVATE ${PROJECT_SOURCE_DIR})
#target_include_directories (de265 PRIVATE ${PROJECT_SOURCE_DIR}/libde265)
target_include_directories(de265 PRIVATE ${PROJECT_BINARY_DIR})
if (MSVC)
    target_include_directories(de265 PRIVATE "${PROJECT_SOURCE_DIR}/extra")
endif ()


target_link_libraries(de265 PRIVATE Threads::Threads)
target_include_directories(de265 PRIVATE ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR})

write_basic_package_version_file(libde265ConfigVersion.cmake COMPATIBILITY ExactVersion)

if (WIN32)
    set_target_properties(de265 PROPERTIES PREFIX "lib")
endif()

install(TARGETS de265 EXPORT libde265Config
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

install(FILES de265.h de265-multilayer.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libde265)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/de265-version.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libde265)

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

configure_file(../libde265.pc.in ${CMAKE_CURRENT_BINARY_DIR}/libde265.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libde265.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
