noinst_LTLIBRARIES = libde265_encoder.la

libde265_encoder_la_CXXFLAGS = -I$(top_srcdir) -I$(top_builddir) -I$(top_srcdir)/libde265
libde265_encoder_la_SOURCES = \
  encoder-core.cc encoder-core.h \
  encoder-types.h encoder-types.cc \
  encoder-params.h encoder-params.cc \
  encoder-context.h encoder-context.cc \
  encoder-syntax.h encoder-syntax.cc \
  encoder-intrapred.h encoder-intrapred.cc \
  encoder-motion.h encoder-motion.cc \
  encpicbuf.h encpicbuf.cc \
  sop.h sop.cc

SUBDIRS=algo
libde265_encoder_la_LIBADD = algo/libde265_encoder_algo.la

EXTRA_DIST = \
  CMakeLists.txt
