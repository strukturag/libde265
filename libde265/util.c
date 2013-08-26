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


/*
int Clip1_8bit_func(int value)
{
  if (value<0) return 0;
  if (value>255) return 255;
  return value;
}
*/

/*
int Clip3_func(int low,int high, int value)
{
  if (value<=low) return low;
  if (value>=high) return high;
  return value;
}
*/

/*
int min(int a,int b)
{
  return a<b ? a : b;
}
*/

/*
int abs_value_func(int a)
{
  return a<0 ? -a : a;
}
*/




#ifdef DE265_LOG_ERROR
void logerror(enum LogModule module, const char* string, ...)
{
  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) fprintf(stderr, "ERR: ");
  va_start(va, string);
  vfprintf(stderr, string + (noPrefix ? 1 : 0), va);
  va_end(va);
}
#endif

#ifdef DE265_LOG_INFO
void loginfo (enum LogModule module, const char* string, ...)
{
  va_list va;

  int noPrefix = (string[0]=='*');
  if (!noPrefix) fprintf(stderr, "INFO: ");
  va_start(va, string);
  vfprintf(stderr, string + (noPrefix ? 1 : 0), va);
  va_end(va);
}
#endif

#ifdef DE265_LOG_DEBUG
void logdebug(enum LogModule module, const char* string, ...)
{
  va_list va;

  va_start(va, string);
  vfprintf(stderr, string, va);
  va_end(va);
}
#endif

#ifdef DE265_LOG_TRACE
void logtrace(enum LogModule module, const char* string, ...)
{
  va_list va;

  va_start(va, string);
  vfprintf(stderr, string, va);
  va_end(va);
}
#endif
