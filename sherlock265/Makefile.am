
bin_PROGRAMS = sherlock265

AM_CPPFLAGS = -I../libde265

sherlock265_DEPENDENCIES = ../libde265/libde265.la
sherlock265_CXXFLAGS = $(QT_CFLAGS) -std=c++0x -fPIC
sherlock265_LDFLAGS = $(QT_LIBS)
sherlock265_LDADD = ../libde265/libde265.la -lstdc++ -lpthread
sherlock265_SOURCES = \
  sherlock265.cc \
  VideoPlayer.cc \
  VideoDecoder.cc \
  VideoWidget.cc \
  VideoPlayer.h \
  VideoDecoder.h \
  VideoWidget.h

nodist_sherlock265_SOURCES = \
  moc_VideoPlayer.cpp \
  moc_VideoDecoder.cpp \
  moc_VideoWidget.cpp

CLEANFILES = \
  moc_VideoPlayer.cpp \
  moc_VideoDecoder.cpp \
  moc_VideoWidget.cpp

if HAVE_VIDEOGFX
  sherlock265_CXXFLAGS += $(VIDEOGFX_CFLAGS)
  sherlock265_LDFLAGS += $(VIDEOGFX_LIBS)
endif

if HAVE_SWSCALE
  sherlock265_CXXFLAGS += $(SWSCALE_CFLAGS)
  sherlock265_LDFLAGS += $(SWSCALE_LIBS)
endif

moc_VideoWidget.cpp: VideoWidget.h
	$(QTMOC) $(DEFINES) $(INCPATH) VideoWidget.h -o moc_VideoWidget.cpp

moc_VideoPlayer.cpp: VideoPlayer.h
	$(QTMOC) $(DEFINES) $(INCPATH) VideoPlayer.h -o moc_VideoPlayer.cpp

moc_VideoDecoder.cpp: VideoDecoder.h
	$(QTMOC) $(DEFINES) $(INCPATH) VideoDecoder.h -o moc_VideoDecoder.cpp

EXTRA_DIST = \
  README
