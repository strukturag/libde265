
bin_PROGRAMS = enc265

AM_CPPFLAGS = -I../libde265

enc265_DEPENDENCIES = ../libde265/libde265.la
enc265_CXXFLAGS =
enc265_LDFLAGS =
enc265_LDADD = ../libde265/libde265.la -lstdc++ -lvideogfx -lX11 -lXv -lXext
enc265_SOURCES = enc265.cc

if HAVE_VIDEOGFX
  enc265_CXXFLAGS += $(VIDEOGFX_CFLAGS)
  enc265_LDFLAGS += $(VIDEOGFX_LIBS)
endif
