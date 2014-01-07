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
#include <malloc.h>

#ifdef _WIN32
#define ALLOC_ALIGNED(alignment, size)      _aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                   _aligned_free((mem))
#else
#define ALLOC_ALIGNED(alignment, size)      memalign((alignment), (size))
#define FREE_ALIGNED(mem)                   free((mem))
#endif

#define ALLOC_ALIGNED_16(size)              ALLOC_ALIGNED(16, size)

static const int alignment = 16;

void de265_alloc_image(de265_image* img, int w,int h, enum de265_chroma c,
                       const seq_parameter_set* sps)
{
  const int border=0;  // TODO: remove the border altogether


  // check if we can reuse old image buffer

  if (img->width==w && img->height==h && img->chroma_format==c && img->border==border) {
    return;
  }


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
  img->border=border;
  img->chroma_width = chroma_width;
  img->chroma_height= chroma_height;

  img->chroma_format= c;

  img->y_mem = (uint8_t *)ALLOC_ALIGNED_16(img->stride * (h+2*border));
  img->y     = img->y_mem + border + 2*border*img->stride;

  if (c != de265_chroma_mono) {
    img->cb_mem = (uint8_t *)ALLOC_ALIGNED_16(img->chroma_stride * (chroma_height+2*border));
    img->cr_mem = (uint8_t *)ALLOC_ALIGNED_16(img->chroma_stride * (chroma_height+2*border));

    img->cb     = img->cb_mem + border + 2*border*img->chroma_stride;
    img->cr     = img->cr_mem + border + 2*border*img->chroma_stride;
  } else {
    img->cb_mem = NULL;
    img->cr_mem = NULL;
    img->cb     = NULL;
    img->cr     = NULL;
  }


  // --- allocate decoding info arrays ---

  if (sps) {
    int intraPredModeSize = sps->PicWidthInMinPUs * sps->PicHeightInMinPUs;
    img->intraPredModeSize = intraPredModeSize;
    img->intraPredMode = malloc(intraPredModeSize * sizeof(*img->intraPredMode));
  }
}


void de265_free_image(de265_image* img)
{
  if (img->y)  FREE_ALIGNED(img->y_mem);
  if (img->cb) FREE_ALIGNED(img->cb_mem);
  if (img->cr) FREE_ALIGNED(img->cr_mem);

  img->y  = NULL;
  img->cb = NULL;
  img->cr = NULL;
  img->y_mem  = NULL;
  img->cb_mem = NULL;
  img->cr_mem = NULL;


  if (img->cb_info) free(img->cb_info);
  img->cb_info = NULL;

  if (img->pb_info) free(img->pb_info);
  img->pb_info = NULL;

  if (img->pb_rootIdx) free(img->pb_rootIdx);
  img->pb_rootIdx = NULL;
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
  img->border= 0;
  img->chroma_width = 0;
  img->chroma_height= 0;
  img->chroma_stride= 0;

  img->picture_order_cnt_lsb = -1; // undefined
  img->PicOrderCntVal = -1; // undefined
  img->PicOutputFlag = 0;
  img->PicState = UnusedForReference;

  img->cb_info = NULL;
  img->cb_info_size = 0;

  img->pb_info = NULL;
  img->pb_info_size = 0;

  img->pb_rootIdx = NULL;
  img->pb_info_nextRootIdx = 0;
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


