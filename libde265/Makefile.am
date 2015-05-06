AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libde265.la

libde265_ladir = \
        $(includedir)/libde265

libde265_la_CPPFLAGS =
libde265_la_CFLAGS = \
  $(CFLAG_VISIBILITY) \
  -DLIBDE265_EXPORTS
libde265_la_CXXFLAGS = \
  $(CFLAG_VISIBILITY) \
  -DLIBDE265_EXPORTS

if HAVE_VISIBILITY
 libde265_la_CFLAGS += -DHAVE_VISIBILITY
 libde265_la_CXXFLAGS += -DHAVE_VISIBILITY
endif

libde265_la_LDFLAGS = -version-info $(LIBDE265_CURRENT):$(LIBDE265_REVISION):$(LIBDE265_AGE)
libde265_la_LIBADD = -lstdc++

libde265_la_SOURCES = \
  bitstream.cc \
  cabac.cc \
  de265.cc \
  deblock.cc \
  decctx.cc \
  nal-parser.cc \
  nal-parser.h \
  dpb.cc \
  dpb.h \
  image.cc \
  intrapred.cc \
  md5.cc \
  nal.cc \
  pps.cc \
  transform.cc \
  refpic.cc \
  sao.cc \
  scan.cc \
  sei.cc \
  slice.cc \
  sps.cc \
  util.cc \
  vps.cc \
  bitstream.h \
  cabac.h \
  deblock.h \
  decctx.h \
  image.h \
  intrapred.h \
  md5.h \
  nal.h \
  pps.h \
  transform.h \
  refpic.h \
  sao.h \
  scan.h \
  sei.h \
  slice.h \
  sps.h \
  util.h \
  vps.h \
  motion.cc motion.h \
  threads.cc threads.h \
  visualize.cc visualize.h \
  acceleration.h \
  fallback.cc fallback.h fallback-motion.cc fallback-motion.h \
  fallback-dct.h fallback-dct.cc \
  quality.cc quality.h \
  configparam.cc configparam.h \
  image-io.h image-io.cc \
  alloc_pool.h alloc_pool.cc \
  en265.h en265.cc \
  contextmodel.cc

SUBDIRS = encoder
libde265_la_LIBADD += encoder/libde265_encoder.la

if ENABLE_SSE_OPT
  SUBDIRS += x86
  libde265_la_LIBADD += x86/libde265_x86.la
endif

if ENABLE_ARM_OPT
  SUBDIRS += arm
  libde265_la_LIBADD += arm/libde265_arm.la
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
