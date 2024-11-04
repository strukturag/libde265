## Makefile.am for libde265

SUBDIRS = libde265

ACLOCAL_AMFLAGS = -I m4

if ENABLE_DEC265
SUBDIRS+=dec265
endif

if ENABLE_ENCODER
SUBDIRS+=enc265
endif

SUBDIRS+=tools

# Remove acceleration-speed test program, because it needs many internal functions (that have to be dllexported).
# On the other hand, that tool was only used for development.
# SUBDIRS+=acceleration-speed

if ENABLE_SHERLOCK265
SUBDIRS+=sherlock265
endif

EXTRA_DIST = \
  autogen.sh \
  build.bat \
  m4/ax_compare_version.m4 \
  m4/ax_cxx_compile_stdcxx_11.m4 \
  m4/m4_ax_check_compile_flag.m4 \
  m4/visibility.m4 \
  Makefile.vc7 \
  CMakeLists.txt \
  README.md \
  libde265.png \
  */COPYING

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libde265.pc
