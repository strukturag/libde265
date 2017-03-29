noinst_LTLIBRARIES = libde265_x86.la  libde265_x86_sse.la

libde265_x86_la_CXXFLAGS = -I.. $(CFLAG_VISIBILITY)
libde265_x86_la_SOURCES = sse.cc sse.h
libde265_x86_la_LIBADD = libde265_x86_sse.la

if HAVE_VISIBILITY
 libde265_x86_la_CXXFLAGS += -DHAVE_VISIBILITY
endif


# SSE4 specific functions

libde265_x86_sse_la_CXXFLAGS = $(SSE_COMPILER_OPTION) -I.. $(CFLAG_VISIBILITY)
libde265_x86_sse_la_SOURCES = \
  sse-motion.cc sse-motion.h \
  sse-motion-new.cc sse-motion-new.h \
  sse-dct.h sse-dct.cc \
  sse-sao.cc sse-sao.h \
  sse-intra-dc.h sse-intra-dc.cc \
  sse-pixelformat.h sse-pixelformat.cc

if HAVE_VISIBILITY
 libde265_x86_sse_la_CXXFLAGS += -DHAVE_VISIBILITY
endif

EXTRA_DIST = \
  CMakeLists.txt
