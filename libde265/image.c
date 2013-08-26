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

#include "image.h"
#include <stdlib.h>
#include <string.h>

static const int alignment = 16;

void de265_alloc_image(de265_image* img, int w,int h, enum de265_chroma c, int border)
{
  int chroma_width = w;
  int chroma_height= h;

  if (c==de265_chroma_420) {
    chroma_width  = (chroma_width +1)/2;
    chroma_height = (chroma_height+1)/2;
  }

  if (c==de265_chroma_422) {
    chroma_height = (chroma_height+1)/2;
  }

  img->stride        = (w           +2*border+alignment-1) / alignment * alignment;
  img->chroma_stride = (chroma_width+2*border+alignment-1) / alignment * alignment;

  img->width = w;
  img->height= h;
  img->chroma_width = chroma_width;
  img->chroma_height= chroma_height;

  img->chroma_format= c;

  img->y_mem = malloc(img->stride * (h+2*border));
  img->y     = img->y_mem + border + 2*border*img->stride;

  if (c != de265_chroma_mono) {
    img->cb_mem = malloc(img->chroma_stride * (chroma_height+2*border));
    img->cr_mem = malloc(img->chroma_stride * (chroma_height+2*border));

    img->cb     = img->cb_mem + border + 2*border*img->chroma_stride;
    img->cr     = img->cr_mem + border + 2*border*img->chroma_stride;
  } else {
    img->cb_mem = NULL;
    img->cr_mem = NULL;
    img->cb     = NULL;
    img->cr     = NULL;
  }
}


void de265_free_image(de265_image* img)
{
  if (img->y)  free(img->y_mem);
  if (img->cb) free(img->cb_mem);
  if (img->cr) free(img->cr_mem);

  img->y  = NULL;
  img->cb = NULL;
  img->cr = NULL;
  img->y_mem  = NULL;
  img->cb_mem = NULL;
  img->cr_mem = NULL;
}


void de265_init_image(de265_image* img) // (optional) init variables, do not alloc image
{
  img->y  = NULL;
  img->cb = NULL;
  img->cr = NULL;
  img->y_mem  = NULL;
  img->cb_mem = NULL;
  img->cr_mem = NULL;
  img->width = 0;
  img->height= 0;
  img->stride= 0;
  img->chroma_width = 0;
  img->chroma_height= 0;
  img->chroma_stride= 0;
}


void de265_fill_image(de265_image* img, int y,int cb,int cr)
{
  if (y>=0) {
    memset(img->y_mem, y, img->stride * (img->height+2*img->border));
  }

  if (cb>=0) {
    memset(img->cb_mem, cb, img->chroma_stride * (img->chroma_height+2*img->border));
  }

  if (cr>=0) {
    memset(img->cr_mem, cr, img->chroma_stride * (img->chroma_height+2*img->border));
  }
}


void de265_copy_image(de265_image* dest, const de265_image* src)
{
  for (int y=0;y<src->height;y++) {
    memcpy(dest->y+y*dest->stride, src->y+y*src->stride, src->width);
  }

  if (src->chroma_format != de265_chroma_mono) {
    for (int y=0;y<src->chroma_height;y++) {
      memcpy(dest->cb+y*dest->chroma_stride, src->cb+y*src->chroma_stride, src->chroma_width);
      memcpy(dest->cr+y*dest->chroma_stride, src->cr+y*src->chroma_stride, src->chroma_width);
    }
  }
}

