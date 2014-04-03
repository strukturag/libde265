/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DE265_UTIL_H
#define DE265_UTIL_H

#ifndef _MSC_VER
#include <inttypes.h>
#endif

#include <stdio.h>


#ifdef _MSC_VER
#define LIBDE265_DECLARE_ALIGNED( var, n ) __declspec(align(n)) var
#define likely(x)      (x)
#define unlikely(x)    (x)
#else
#define LIBDE265_DECLARE_ALIGNED( var, n ) var __attribute__((aligned(n)))
#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)
#endif

#define ALIGNED_32( var ) LIBDE265_DECLARE_ALIGNED( var, 32 )
#define ALIGNED_16( var ) LIBDE265_DECLARE_ALIGNED( var, 16 )
#define ALIGNED_8( var )  LIBDE265_DECLARE_ALIGNED( var, 8 )
#define ALIGNED_4( var )  LIBDE265_DECLARE_ALIGNED( var, 4 )



//inline uint8_t Clip1_8bit(int16_t value) { if (value<=0) return 0; else if (value>=255) return 255; else return value; }
#define Clip1_8bit(value) ((value)<0 ? 0 : (value)>255 ? 255 : (value))
#define Clip3(low,high,value) ((value)<(low) ? (low) : (value)>(high) ? (high) : (value))
#define Sign(value) (((value)>0) ? 1 : ((value)<0) ? -1 : 0)
#define abs_value(a) (((a)<0) ? -(a) : (a))
#define libde265_min(a,b) (((a)<(b)) ? (a) : (b))
#define libde265_max(a,b) (((a)>(b)) ? (a) : (b))

int ceil_div(int num,int denom);
int ceil_log2(int val);
int Log2(int v);



// === logging ===

enum LogModule {
  LogHighlevel,
  LogHeaders,
  LogSlice,
  LogDPB,
  LogMotion,
  LogTransform,
  LogDeblock,
  LogSAO,
  LogSEI,
  LogIntraPred,
  LogPixels,
  LogCABAC
};


#if defined(DE265_LOG_ERROR) || defined(DE265_LOG_INFO) || defined(DE265_LOG_DEBUG) || defined(DE265_LOG_TRACE)
# define DE265_LOGGING 1
void enablelog();
#else
#define enablelog() { }
#endif

#ifdef DE265_LOGGING
void log_set_current_POC(int poc);
#else
#define log_set_current_POC(poc) { }
#endif

#ifdef DE265_LOG_ERROR
void logerror(enum LogModule module, const char* string, ...);
#else
#define logerror(a,b, ...) { }
#endif

#ifdef DE265_LOG_INFO
void loginfo (enum LogModule module, const char* string, ...);
#else
#define loginfo(a,b, ...) { }
#endif

#ifdef DE265_LOG_DEBUG
void logdebug(enum LogModule module, const char* string, ...);
#else
#define logdebug(a,b, ...) { }
#endif

#ifdef DE265_LOG_TRACE
void logtrace(enum LogModule module, const char* string, ...);
#else
#define logtrace(a,b, ...) { }
#endif

void log2fh(FILE* fh, const char* string, ...);

#endif
