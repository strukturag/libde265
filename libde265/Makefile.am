AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libde265.la

libde265_ladir = \
        $(includedir)/libde265

libde265_la_CPPFLAGS =

libde265_la_LDFLAGS = -version-info $(LIBDE265_CURRENT):$(LIBDE265_REVISION):$(LIBDE265_AGE)

libde265_la_SOURCES = \
  bitstream.c \
  cabac.c \
  de265.c \
  deblock.c \
  decctx.c \
  dpb.c \
  dpb.h \
  image.c \
  intrapred.c \
  md5.c \
  nal.c \
  pps.c \
  transform.c \
  refpic.c \
  sao.c \
  scan.c \
  sei.c \
  slice.c \
  sps.c \
  util.c \
  vps.c \
  bitstream.h \
  cabac.h \
  deblock.h \
  decctx.h \
  image.h \
  intrapred.h \
  md5.h \
  nal.h \
  pps.h \
  pps_func.h \
  transform.h \
  refpic.h \
  sao.h \
  scan.h \
  sei.h \
  slice.h \
  slice_func.h \
  sps.h \
  sps_func.h \
  util.h \
  vps.h \
  motion.c motion.h motion_func.h \
  threads.c threads.h \
  acceleration.h \
  fallback.c fallback.h fallback-motion.c fallback-motion.h \
  fallback-dct.h fallback-dct.c

if ENABLE_SSE_OPT
  SUBDIRS = x86
  libde265_la_LIBADD = x86/libde265_x86.la
endif

if MINGW
  libde265_la_SOURCES += ../extra/win32cond.c ../extra/win32cond.h
  libde265_la_LDFLAGS += -no-undefined -static-libgcc -static-libstdc++
endif

EXTRA_DIST = Makefile.vc7 \
  ../extra/stdbool.h \
  ../extra/stdint.h

libde265_la_HEADERS = \
  de265.h \
  de265-version.h
