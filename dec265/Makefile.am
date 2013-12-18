
bin_PROGRAMS = dec265

AM_CPPFLAGS = -I../libde265

dec265_DEPENDENCIES = ../libde265/libde265.la
dec265_CXXFLAGS = $(VIDEOGFX_CFLAGS)
dec265_LDFLAGS = $(VIDEOGFX_LIBS)
dec265_LDADD = ../libde265/libde265.la -lstdc++ -lpthread
dec265_SOURCES = dec265.cc
