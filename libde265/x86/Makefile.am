noinst_LTLIBRARIES = libde265_x86.la  libde265_x86_sse.la

libde265_x86_la_CXXFLAGS = -I..
libde265_x86_la_SOURCES = sse.cc sse.h
libde265_x86_la_LIBADD = libde265_x86_sse.la


# SSE4 specific functions

libde265_x86_sse_la_CXXFLAGS = -msse4.1 -I..
libde265_x86_sse_la_SOURCES = sse-motion.cc sse-motion.h sse-dct.h sse-dct.cc


