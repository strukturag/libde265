
bin_PROGRAMS = sherlock265

INCLUDES = -I../libde265

sherlock265_DEPENDENCIES = ../libde265/libde265.la
sherlock265_CXXFLAGS = $(QT_CFLAGS) $(VIDEOGFX_CFLAGS)
sherlock265_LDFLAGS = $(QT_LIBS) $(VIDEOGFX_LIBS)
sherlock265_LDADD = ../libde265/libde265.la -lstdc++ -lpthread
sherlock265_SOURCES = \
  sherlock265.cc \
  VideoPlayer.cc \
  VideoDecoder.cc \
  VideoWidget.cc \
  moc_VideoPlayer.cpp \
  moc_VideoDecoder.cpp \
  moc_VideoWidget.cpp \
  VideoPlayer.hh \
  VideoDecoder.hh \
  VideoWidget.hh

moc_VideoWidget.cpp: VideoWidget.hh
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) VideoWidget.hh -o moc_VideoWidget.cpp

moc_VideoPlayer.cpp: VideoPlayer.hh
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) VideoPlayer.hh -o moc_VideoPlayer.cpp

moc_VideoDecoder.cpp: VideoDecoder.hh
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) VideoDecoder.hh -o moc_VideoDecoder.cpp
