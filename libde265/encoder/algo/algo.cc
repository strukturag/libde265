/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: Dirk Farin <farin@struktur.de>
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

#include "libde265/encoder/algo/algo.h"
#include "libde265/encoder/encoder-context.h"

#include <stdarg.h>



#ifdef DE265_LOG_DEBUG
static int descendLevel = 0;

void Algo::descend(enc_node* node, const char* option, ...)
{
  if (logdebug_enabled(LogEncoder)) {
    descendLevel++;
    printf("%d ",descendLevel);
    for (int i=0;i<descendLevel;i++) { printf(" "); }

    va_list va;
    va_start(va, option);
    va_end(va);

    fprintf(stdout, "%s(", name());
    vfprintf(stdout, option, va);
    fprintf(stdout, ") %d;%d %dx%d\n",node->x,node->y,1<<node->log2Size,1<<node->log2Size);
  }
}

void Algo::ascend(const char* fmt, ...)
{
  if (logdebug_enabled(LogEncoder)) {
    if (fmt != NULL) {
      printf("%d ",descendLevel);
      for (int i=0;i<descendLevel;i++) { printf(" "); }

      va_list va;
      va_start(va, fmt);
      va_end(va);

      fprintf(stdout, "<%s(", name());
      vfprintf(stdout, fmt, va);
      fprintf(stdout, ")\n");
    }

    descendLevel--;
  }
}

void Algo::leaf(enc_node* node, const char* option, ...)
{
  if (logdebug_enabled(LogEncoder)) {
    printf("%d ",descendLevel+1);
    for (int i=0;i<descendLevel+1;i++) { printf(" "); }

    va_list va;
    va_start(va, option);
    va_end(va);

    fprintf(stdout, "%s(", name());
    vfprintf(stdout, option, va);
    fprintf(stdout, ") %d;%d %dx%d\n",node->x,node->y,1<<node->log2Size,1<<node->log2Size);
  }
}

#endif
