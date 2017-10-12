noinst_LTLIBRARIES = libde265_encoder_screensharing.la

libde265_encoder_screensharing_la_CXXFLAGS = -I..
libde265_encoder_screensharing_la_SOURCES = \
  screensharing-core.h screensharing-core.cc

libde265_encoder_screensharing_la_CFLAGS = \
  $(CFLAG_VISIBILITY) \
  -DLIBDE265_EXPORTS
libde265_encoder_screensharing_la_CXXFLAGS += \
  $(CFLAG_VISIBILITY) \
  -DLIBDE265_EXPORTS

if HAVE_VISIBILITY
 libde265_encoder_screensharing_la_CFLAGS += -DHAVE_VISIBILITY
 libde265_encoder_screensharing_la_CXXFLAGS += -DHAVE_VISIBILITY
endif

# SUBDIRS=algo screensharing
# libde265_encoder_screensharing_la_LIBADD = algo/libde265_encoder_algo.la

if HAVE_VIDEOGFX
  libde265_encoder_screensharing_la_CXXFLAGS += $(VIDEOGFX_CFLAGS)
#  enc265_LDFLAGS += $(VIDEOGFX_LIBS)
endif

EXTRA_DIST = \
  CMakeLists.txt
