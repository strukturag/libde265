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

libde265_la_SOURCES = \
  acceleration.h \
  acceleration.cc \
  alloc_pool.h \
  alloc_pool.cc \
  bitstream.cc \
  bitstream.h \
  cabac.cc \
  cabac.h \
  codingdata.h \
  codingdata-impl.h \
  configparam.cc \
  configparam.h \
  contextmodel.cc \
  contextmodel.h \
  de265.cc \
  deblock.cc \
  deblock.h \
  decctx.cc \
  decctx.h \
  en265.h \
  en265.cc \
  fallback.cc \
  fallback.h \
  fallback-dct.h \
  fallback-dct.cc \
  fallback-intra-dc.h \
  fallback-intra-dc.cc \
  fallback-motion.cc \
  fallback-motion.h \
  fallback-pixelformat.cc \
  fallback-pixelformat.h \
  fallback-sao.h \
  fallback-sao.cc \
  frame-dropper.h \
  frame-dropper.cc \
  frontend-syntax-decoder.h \
  frontend-syntax-decoder.cc \
  funcs.cc \
  funcs.h \
  dpb.cc \
  dpb.h \
  image.cc \
  image.h \
  image-io.h \
  image-io.cc \
  image-unit.h \
  image-unit.cc \
  intrapred.cc \
  intrapred.h \
  md5.cc \
  md5.h \
  motion.cc \
  motion.h \
  nal.cc \
  nal.h \
  nal-parser.cc \
  nal-parser.h \
  pps.cc \
  pps.h \
  quality.cc \
  quality.h \
  refpic.cc \
  refpic.h \
  sao.cc \
  sao.h \
  scan.cc \
  scan.h \
  sei.cc \
  sei.h \
  slice.cc \
  slice.h \
  sps.cc \
  sps.h \
  threads.cc \
  threads.h \
  transform.cc \
  transform.h \
  util.cc \
  util.h \
  visualize.cc \
  visualize.h \
  vps.cc \
  vps.h \
  vui.cc \
  vui.h

SUBDIRS = encoder
libde265_la_LIBADD = encoder/libde265_encoder.la

if ENABLE_SSE_OPT
  SUBDIRS += x86
  libde265_la_LIBADD += x86/libde265_x86.la
endif

if ENABLE_ARM_OPT
  SUBDIRS += arm
  libde265_la_LIBADD += arm/libde265_arm.la
endif

if MINGW
  libde265_la_CXXFLAGS += -D_WIN32_WINNT=0x0600
  libde265_la_LDFLAGS += -no-undefined -static-libgcc -static-libstdc++
endif

if HAVE_VIDEOGFX
  libde265_la_CXXFLAGS += $(VIDEOGFX_CFLAGS)
  libde265_la_LDFLAGS += $(VIDEOGFX_LIBS)
endif

EXTRA_DIST = Makefile.vc7 \
  CMakeLists.txt \
  ../extra/stdbool.h \
  ../extra/stdint.h

libde265_la_HEADERS = \
  de265.h \
  de265-version.h
