/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "util.h"
#include "de265.h"

#include <stdarg.h>
#include <stdio.h>


int ceil_div(int num,int denom)
{
  num += denom-1;
  return num/denom;
}


int ceil_log2(int val)
{
  int n=0;
  while (val > (1<<n)) {
    n++;
  }

  return n;
}


int Log2(int v)
{
  int n=0;
  while (v>1) {
    n++;
    v>>=1;
  }

  return n;
}


#ifdef DE265_LOGGING
static int current_poc=0;
static int log_poc_start=-9999; // frame-numbers can be negative
static int enable_log = 1;
void log_set_current_POC(int poc) { current_poc=poc; }
#endif


static int disable_logging=0;

LIBDE265_API void de265_disable_logging()
{
  disable_logging=1;
}

#if defined(DE265_LOG_ERROR) || defined(DE265_LOG_INFO) || defined(DE265_LOG_DEBUG) || defined(DE265_LOG_INFO)
void enablelog() { enable_log=1; }
#endif

#ifdef DE265_LOG_ERROR
void logerror(enum LogModule module, const char* string, ...)
{
  if (disable_logging) return;
  if (current_poc < log_poc_start) { return; }
  if (!enable_log) return;

  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) fprintf(stdout, "ERR: ");
  va_start(va, string);
  vfprintf(stdout, string + (noPrefix ? 1 : 0), va);
  va_end(va);
  fflush(stdout);
}
#endif

#ifdef DE265_LOG_INFO
void loginfo (enum LogModule module, const char* string, ...)
{
  if (disable_logging) return;
  if (current_poc < log_poc_start) { return; }
  if (!enable_log) return;

  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) fprintf(stdout, "INFO: ");
  va_start(va, string);
  vfprintf(stdout, string + (noPrefix ? 1 : 0), va);
  va_end(va);
  fflush(stdout);
}
#endif

#ifdef DE265_LOG_DEBUG
void logdebug(enum LogModule module, const char* string, ...)
{
  if (disable_logging) return;
  if (current_poc < log_poc_start) { return; }
  if (!enable_log) return;

  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) fprintf(stdout, "DEBUG: ");
  va_start(va, string);
  vfprintf(stdout, string + (noPrefix ? 1 : 0), va);
  va_end(va);
  fflush(stdout);
}
#endif

#ifdef DE265_LOG_TRACE
void logtrace(enum LogModule module, const char* string, ...)
{
  if (disable_logging) return;
  if (current_poc < log_poc_start) { return; }
  if (!enable_log) return;

  //if (module != LogCABAC) return;

  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) { } // fprintf(stdout, "ERR: ");
  va_start(va, string);
  vfprintf(stdout, string + (noPrefix ? 1 : 0), va);
  va_end(va);
  fflush(stdout);
}
#endif

void log2fh(FILE* fh, const char* string, ...)
{
  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) fprintf(stdout, "INFO: ");
  va_start(va, string);
  vfprintf(fh, string + (noPrefix ? 1 : 0), va);
  va_end(va);
  fflush(stdout);
}
