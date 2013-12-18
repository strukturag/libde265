noinst_LTLIBRARIES = libde265_x86.la  libde265_x86_sse.la

libde265_x86_la_CFLAGS = -I..
libde265_x86_la_SOURCES = sse.c sse.h
libde265_x86_la_LIBADD = libde265_x86_sse.la


# SSE4 specific functions

libde265_x86_sse_la_CFLAGS = -msse4.1 -I..
libde265_x86_sse_la_SOURCES = sse-motion.c sse-motion.h sse-dct.h sse-dct.c


