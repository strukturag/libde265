#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([libde265], [0.4], [farin@struktur.de])
AC_CONFIG_SRCDIR([libde265/de265.c])
AC_CONFIG_HEADERS([config.h])

NUMERIC_VERSION=0x00040000 # Numeric representation of the version
AC_SUBST(NUMERIC_VERSION)

# ---------------------------------------------------------------------------

AC_CANONICAL_SYSTEM

LIBDE265_CURRENT=0
LIBDE265_REVISION=3
LIBDE265_AGE=0

AC_SUBST(LIBDE265_CURRENT)
AC_SUBST(LIBDE265_REVISION)
AC_SUBST(LIBDE265_AGE)

dnl Initialize libtool
LT_INIT
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LN_S

# Initialize automake stuff
AM_INIT_AUTOMAKE

CFLAGS+=" -std=c99"
CXXFLAGS+=" -std=c++0x"

dnl Use -Wall if we have gcc.
changequote(,)dnl
if test "x$GCC" = "xyes"; then
  case " $CFLAGS " in
  *[\ \	]-Wall[\ \	]*) ;;
  *) CFLAGS="$CFLAGS -Wall" ;;
  esac
fi
changequote([,])dnl


# Checks for header files.
AC_CHECK_HEADERS([stdint.h stdlib.h string.h malloc.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT8_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_CHECK_FUNCS([malloc memmove memset __malloc_hook memalign posix_memalign __mingw_aligned_malloc __mingw_aligned_free])

AC_SEARCH_LIBS([sqrt], [m])
AC_SEARCH_LIBS([pthread_create], [pthread])

AM_CONDITIONAL(MINGW, expr $host : '.*-mingw' >/dev/null 2>&1)


# --- machine dependent optimizations ---

#AX_EXT

case $target_cpu in
  powerpc*)
    ;;

  i[[3456]]86*|x86_64*|amd64*)

    AX_CHECK_COMPILE_FLAG(-msse4.1, ax_cv_support_sse41_ext=yes, [])
    if test x"$ax_cv_support_sse41_ext" = x"yes"; then
#      SIMD_FLAGS="$SIMD_FLAGS -msse4.1"
      AC_DEFINE(HAVE_SSE4_1,1,[Support SSSE4.1 (Streaming SIMD Extensions 4.1) instructions])
    else
      AC_MSG_WARN([Your compiler does not support SSE4.1 instructions, can you try another compiler?])
    fi
    ;;

esac

AM_CONDITIONAL([ENABLE_SSE_OPT], [test x"$ax_cv_support_sse41_ext" = x"yes"])

# CFLAGS+=$SIMD_FLAGS
# CFLAGS+=" -march=x86-64"


# --- additional logging ---

AC_ARG_ENABLE(log-error,
              [AS_HELP_STRING([--enable-log-error],
                              [turn on logging at error level (default=yes)])],
  [enable_log_error=$enableval],
  [enable_log_error=yes])
if eval "test $enable_log_error = yes"; then
  CFLAGS+=" -DDE265_LOG_ERROR"
fi

AC_ARG_ENABLE(log-info,
              [AS_HELP_STRING([--enable-log-info],
                              [turn on logging at info level (default=no)])],
  [enable_log_info=$enableval],
  [enable_log_info=no])
if eval "test $enable_log_info = yes"; then
  CFLAGS+=" -DDE265_LOG_INFO"
fi

AC_ARG_ENABLE(log-debug,
              [AS_HELP_STRING([--enable-log-debug],
                              [turn on logging at debug level (default=no)])],
  [enable_log_debug=$enableval],
  [enable_log_debug=no])
if eval "test $enable_log_debug = yes"; then
  CFLAGS+=" -DDE265_LOG_DEBUG"
fi

AC_ARG_ENABLE(log-trace,
              [AS_HELP_STRING([--enable-log-trace],
                              [turn on logging at trace level (default=no)])],
  [enable_log_trace=$enableval],
  [enable_log_trace=no])
if eval "test $enable_log_trace = yes"; then
  CFLAGS+=" -DDE265_LOG_TRACE"
fi


# --- enable example programs ---

AC_ARG_ENABLE([dec265], AS_HELP_STRING([--disable-dec265], [Do not build dec265 decoder program.]))
AC_ARG_ENABLE([sherlock265], AS_HELP_STRING([--disable-sherlock265], [Do not build sherlock265 visual inspection program.]))

if eval "test x$enable_dec265 = x"      ; then enable_dec265=yes ; fi
if eval "test x$enable_sherlock265 = x" ; then enable_sherlock265=yes ; fi

if eval "test x$enable_dec265 = xyes" || eval "test x$enable_sherlock265 = xyes" ; then
   PKG_CHECK_MODULES([VIDEOGFX], [libvideogfx],
                     [AC_DEFINE([HAVE_VIDEOGFX], [1], [Whether libvideogfx was found.])
                      AC_SUBST(VIDEOGFX_CFLAGS)
                      AC_SUBST(VIDEOGFX_LIBS)],
                     [enable_sherlock265="no"
                      AC_MSG_WARN([Did not find libvideogfx, compilation of sherlock265 will be disabled.])]
                     )
fi

if eval "test x$enable_dec265 = xyes" || eval "test x$enable_sherlock265 = xyes" ; then
   PKG_CHECK_MODULES([SDL], [sdl],
                     [AC_DEFINE([HAVE_SDL], [1], [Whether libSDL was found.])
                      AC_SUBST(SDL_CFLAGS)
                      AC_SUBST(SDL_LIBS)
                      have_sdl="yes"],
                     [have_sdl="no"
                      AC_MSG_WARN([Did not find libsdl, video output of dec265 will be disabled.])]
                     )
fi

AM_CONDITIONAL([HAVE_SDL], [test "x$have_sdl" != "xno"])

if eval "test $enable_sherlock265 = yes" ; then
  PKG_CHECK_MODULES([QT], [QtCore QtGui])
fi

AM_CONDITIONAL([ENABLE_DEC265], [test "x$enable_dec265" != "xno"])
AM_CONDITIONAL([ENABLE_SHERLOCK265], [test "x$enable_sherlock265" != "xno"])


# --- output configuration results ---

AC_CONFIG_FILES([Makefile])
AC_CONFIG_FILES([libde265/Makefile])
AC_CONFIG_FILES([libde265/x86/Makefile])
AC_CONFIG_FILES([libde265/de265-version.h])
AC_CONFIG_FILES([dec265/Makefile])
AC_CONFIG_FILES([sherlock265/Makefile])
AC_OUTPUT


echo "---------------------------------------"
echo "Building dec265 example:" $enable_dec265
echo "Building sherlock265 example:" $enable_sherlock265
echo "---------------------------------------"
