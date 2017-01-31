noinst_LTLIBRARIES = libde265_encoder.la

libde265_encoder_la_CXXFLAGS = -I..
libde265_encoder_la_SOURCES = \
  encoder-core.cc encoder-core.h \
  encoder-types.h encoder-types.cc \
  encoder-params.h encoder-params.cc \
  encoder-context.h encoder-context.cc \
  encoder-syntax.h encoder-syntax.cc \
  encpicbuf.h encpicbuf.cc \
  sop.h sop.cc

libde265_encoder_la_CFLAGS = \
  $(CFLAG_VISIBILITY) \
  -DLIBDE265_EXPORTS
libde265_encoder_la_CXXFLAGS += \
  $(CFLAG_VISIBILITY) \
  -DLIBDE265_EXPORTS

if HAVE_VISIBILITY
 libde265_encoder_la_CFLAGS += -DHAVE_VISIBILITY
 libde265_encoder_la_CXXFLAGS += -DHAVE_VISIBILITY
endif

SUBDIRS=algo
libde265_encoder_la_LIBADD = algo/libde265_encoder_algo.la

EXTRA_DIST = \
  CMakeLists.txt
