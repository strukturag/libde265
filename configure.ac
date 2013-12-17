#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([libde265], [0.3], [farin@struktur.de])
AC_CONFIG_SRCDIR([libde265/de265.c])
AC_CONFIG_HEADERS([config.h])

AC_CANONICAL_SYSTEM

LIBDE265_CURRENT=0
LIBDE265_REVISION=2
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
AC_CHECK_HEADERS([stdint.h stdlib.h string.h])

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
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memmove memset])

AC_SEARCH_LIBS([sqrt], [m])


# --- machine dependent optimizations ---

#AX_EXT

case $target_cpu in
  powerpc*)
    ;;

  i[[3456]]86*|x86_64*|amd64*)

    AX_CHECK_COMPILE_FLAG(-msse4.1, ax_cv_support_sse41_ext=yes, [])
    if test x"$ax_cv_support_sse41_ext" = x"yes"; then
      SIMD_FLAGS="$SIMD_FLAGS -msse4.1"
      AC_DEFINE(HAVE_SSE4_1,1,[Support SSSE4.1 (Streaming SIMD Extensions 4.1) instructions])
    else
      AC_MSG_WARN([Your compiler does not support SSE4.1 instructions, can you try another compiler?])
    fi
    ;;

esac

AM_CONDITIONAL([ENABLE_SSE_OPT], [test x"$ax_cv_support_sse_ext" = x"yes"])

CFLAGS+=$SIMD_FLAGS
CFLAGS+=" -march=x86-64"

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
  AC_CHECK_PROG([HAVE_VIDEOGFX], [libvideogfx-config], [yes])
  if eval "test x$HAVE_VIDEOGFX = xyes" ; then
    VIDEOGFX_CFLAGS=`libvideogfx-config --cflags`
    VIDEOGFX_LIBS=`libvideogfx-config --libs`
    AC_SUBST(VIDEOGFX_CFLAGS)
    AC_SUBST(VIDEOGFX_LIBS)
    AC_DEFINE([HAVE_VIDEOGFX], [1], [Whether libvideogfx was found.])
  else
    echo "Did not find libvideogfx (is libvideogfx-config in your PATH ?)."
    echo "... Disabling building of sherlock265 ..."
    enable_sherlock265="no"
  fi
fi

if eval "test $enable_sherlock265 = yes" ; then
  PKG_CHECK_MODULES([QT], [QtCore QtGui])
fi

AM_CONDITIONAL([ENABLE_DEC265], [test "x$enable_dec265" != "xno"])
AM_CONDITIONAL([ENABLE_SHERLOCK265], [test "x$enable_sherlock265" != "xno"])


# --- output configuration results ---

AC_CONFIG_FILES([Makefile])
AC_CONFIG_FILES([libde265/Makefile])
AC_CONFIG_FILES([dec265/Makefile])
AC_CONFIG_FILES([sherlock265/Makefile])
AC_OUTPUT


echo "---------------------------------------"
echo "Building dec265 example:" $enable_dec265
echo "Building sherlock265 example:" $enable_sherlock265
echo "---------------------------------------"
