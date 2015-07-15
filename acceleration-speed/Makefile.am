
bin_PROGRAMS = acceleration_speed

AM_CPPFLAGS = -I../libde265

acceleration_speed_DEPENDENCIES = ../libde265/libde265.la
acceleration_speed_CXXFLAGS =
acceleration_speed_LDFLAGS =
acceleration_speed_LDADD = ../libde265/libde265.la -lstdc++
acceleration_speed_SOURCES = acceleration-speed.cc dct.cc dct-scalar.cc

if ENABLE_SSE_OPT
  acceleration_speed_SOURCES += dct-sse.cc
endif
