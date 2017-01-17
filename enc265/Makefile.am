
bin_PROGRAMS =

AM_CPPFLAGS = -I../libde265

if ENABLE_ENC265
  bin_PROGRAMS += enc265

  enc265_DEPENDENCIES = ../libde265/libde265.la
  enc265_CXXFLAGS =
  enc265_LDFLAGS =
  enc265_LDADD = ../libde265/libde265.la -lstdc++
  enc265_SOURCES = enc265.cc image-io-png.cc image-io-png.h

if HAVE_VIDEOGFX
  enc265_CXXFLAGS += $(VIDEOGFX_CFLAGS)
  enc265_LDFLAGS += $(VIDEOGFX_LIBS)
endif
endif

EXTRA_DIST = \
  CMakeLists.txt \
  Makefile.vc7
