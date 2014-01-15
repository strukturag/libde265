## Makefile.am for libde265

SUBDIRS = libde265

ACLOCAL_AMFLAGS = -I m4

if ENABLE_DEC265
SUBDIRS+=dec265
endif

if ENABLE_SHERLOCK265
SUBDIRS+=sherlock265
endif

EXTRA_DIST = .travis.yml \
  autogen.sh \
  build.bat \
  m4/m4_ax_ext.m4 \
  m4/m4_ax_gcc_x86_avx_xgetbv.m4 \
  m4/m4_ax_gcc_x86_cpuid.m4 \
  build
