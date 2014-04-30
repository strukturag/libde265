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

#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif

#ifdef HAVE_SSE4_1
#define MEMORY_PADDING  8
#else
#define MEMORY_PADDING  0
#endif

#ifdef HAVE___MINGW_ALIGNED_MALLOC
#define ALLOC_ALIGNED(alignment, size)         __mingw_aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                      __mingw_aligned_free((mem))
#elif _WIN32
#define ALLOC_ALIGNED(alignment, size)         _aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                      _aligned_free((mem))
#elif __APPLE__
static inline void *ALLOC_ALIGNED(size_t alignment, size_t size) {
    void *mem = NULL;
    if (posix_memalign(&mem, alignment, size) != 0) {
        return NULL;
    }
    return mem;
};
#define FREE_ALIGNED(mem)                      free((mem))
#else
#define ALLOC_ALIGNED(alignment, size)      memalign((alignment), (size))
#define FREE_ALIGNED(mem)                   free((mem))
#endif

#define ALLOC_ALIGNED_16(size)              ALLOC_ALIGNED(16, size)

static const int alignment = 16;


void de265_init_image(de265_image* img) // (optional) init variables, do not alloc image
{
  memset(img, 0, sizeof(de265_image));

  img->picture_order_cnt_lsb = -1; // undefined
  img->PicOrderCntVal = -1; // undefined
  img->PicState = UnusedForReference;

  de265_mutex_init(&img->mutex);
  de265_cond_init(&img->finished_cond);
}



de265_error de265_alloc_image(de265_image* img, int w,int h, enum de265_chroma c,
                              const seq_parameter_set* sps)
{
  const int border=0;  // TODO: remove the border altogether

  // --- allocate image buffer (or reuse old one) ---

  if (img->width != w || img->height != h || img->chroma_format != c || img->border != border) {

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

    FREE_ALIGNED(img->y_mem);
    img->y_mem = (uint8_t *)ALLOC_ALIGNED_16(img->stride * (h+2*border) + MEMORY_PADDING);
    img->y     = img->y_mem + border + 2*border*img->stride;

    if (c != de265_chroma_mono) {
      FREE_ALIGNED(img->cb_mem);
      FREE_ALIGNED(img->cr_mem);
      img->cb_mem = (uint8_t *)ALLOC_ALIGNED_16(img->chroma_stride * (chroma_height+2*border) + MEMORY_PADDING);
      img->cr_mem = (uint8_t *)ALLOC_ALIGNED_16(img->chroma_stride * (chroma_height+2*border) + MEMORY_PADDING);

      img->cb     = img->cb_mem + border + 2*border*img->chroma_stride;
      img->cr     = img->cr_mem + border + 2*border*img->chroma_stride;
    } else {
      img->cb_mem = NULL;
      img->cr_mem = NULL;
      img->cb     = NULL;
      img->cr     = NULL;
    }
  }


  // check for memory shortage

  if (img->y_mem  == NULL ||
      img->cb_mem == NULL ||
      img->cr_mem == NULL)
    {
      de265_free_image(img);
      return DE265_ERROR_OUT_OF_MEMORY;
    }


  // --- allocate decoding info arrays ---

  if (sps) {
    // intra pred mode

    img->intraPredMode.alloc(sps->PicWidthInMinPUs, sps->PicHeightInMinPUs,
                             sps->Log2MinPUSize);

    // cb info

    img->cb_info.alloc(sps->PicWidthInMinCbsY, sps->PicHeightInMinCbsY,
                       sps->Log2MinCbSizeY);

    // pb info

    int puWidth  = sps->PicWidthInMinCbsY  << (sps->Log2MinCbSizeY -2);
    int puHeight = sps->PicHeightInMinCbsY << (sps->Log2MinCbSizeY -2);

    img->pb_info.alloc(puWidth,puHeight, 2);


    // tu info

    img->tu_info.alloc(sps->PicWidthInTbsY, sps->PicHeightInTbsY,
                       sps->Log2MinTrafoSize);

    // deblk info

    int deblk_w = (sps->pic_width_in_luma_samples +3)/4;
    int deblk_h = (sps->pic_height_in_luma_samples+3)/4;

    img->deblk_info.alloc(deblk_w, deblk_h, 2);

    // CTB info

    if (img->ctb_info.data_size != sps->PicSizeInCtbsY)
      {
        for (int i=0;i<img->ctb_info.data_size;i++)
          { de265_progress_lock_destroy(&img->ctb_progress[i]); }

        //free(img->ctb_info);
        free(img->ctb_progress);
        //img->ctb_info.data_size  = sps->PicSizeInCtbsY;
        //img->ctb_info     = (CTB_info *)malloc( sizeof(CTB_info)   * img->ctb_info_size);

        img->ctb_info.alloc(sps->PicWidthInCtbsY, sps->PicHeightInCtbsY, sps->Log2CtbSizeY);

        img->ctb_progress = (de265_progress_lock*)malloc( sizeof(de265_progress_lock)
                                                          * img->ctb_info.data_size);
        //img->Log2CtbSizeY = sps->Log2CtbSizeY;
        //img->PicWidthInCtbsY = sps->PicWidthInCtbsY;

        for (int i=0;i<img->ctb_info.data_size;i++)
          { de265_progress_lock_init(&img->ctb_progress[i]); }
      }


    // check for memory shortage

    if (//img->ctb_info == NULL ||
        //img->intraPredMode == NULL ||
        //img->cb_info == NULL ||
        //img->pb_info == NULL ||
        //img->tu_info == NULL ||
        //img->deblk_info == NULL ||
        0)
      {
        de265_free_image(img);
        return DE265_ERROR_OUT_OF_MEMORY;
      }
  }

  return DE265_OK;
}


void de265_free_image(de265_image* img)
{
  if (img->y)  FREE_ALIGNED(img->y_mem);
  if (img->cb) FREE_ALIGNED(img->cb_mem);
  if (img->cr) FREE_ALIGNED(img->cr_mem);

  for (int i=0;i<img->ctb_info.data_size;i++)
    { de265_progress_lock_destroy(&img->ctb_progress[i]); }

  free(img->ctb_progress);
  //free(img->cb_info);
  //free(img->pb_info);
  //free(img->tu_info);
  //free(img->deblk_info);
  //free(img->ctb_info);
  // free(img->intraPredMode);

  de265_cond_destroy(&img->finished_cond);
  de265_mutex_destroy(&img->mutex);

  memset(img, 0, sizeof(de265_image));
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
  if (src->stride == dest->stride) {
    memcpy(dest->y, src->y, src->height*src->stride);
  }
  else {
    for (int y=0;y<src->height;y++) {
      memcpy(dest->y+y*dest->stride, src->y+y*src->stride, src->width);
    }
  }

  if (src->chroma_format != de265_chroma_mono) {
    if (src->chroma_stride == dest->chroma_stride) {
      memcpy(dest->cb, src->cb, src->chroma_height*src->chroma_stride);
      memcpy(dest->cr, src->cr, src->chroma_height*src->chroma_stride);
    }
    else {
      for (int y=0;y<src->chroma_height;y++) {
        memcpy(dest->cb+y*dest->chroma_stride, src->cb+y*src->chroma_stride, src->chroma_width);
        memcpy(dest->cr+y*dest->chroma_stride, src->cr+y*src->chroma_stride, src->chroma_width);
      }
    }
  }
}


void set_conformance_window(de265_image* img, int left,int right,int top,int bottom)
{
  int WinUnitX, WinUnitY;

  switch (img->chroma_format) {
  case de265_chroma_mono: WinUnitX=1; WinUnitY=1; break;
  case de265_chroma_420:  WinUnitX=2; WinUnitY=2; break;
  case de265_chroma_422:  WinUnitX=2; WinUnitY=1; break;
  case de265_chroma_444:  WinUnitX=1; WinUnitY=1; break;
  default:
    assert(0);
  }

  img->y_confwin = img->y + left*WinUnitX + top*WinUnitY*img->stride;
  img->cb_confwin= img->cb+ left + top*img->chroma_stride;
  img->cr_confwin= img->cr+ left + top*img->chroma_stride;

  img->width_confwin = img->width - (left+right)*WinUnitX;
  img->height_confwin= img->height- (top+bottom)*WinUnitY;
  img->chroma_width_confwin = img->chroma_width -left-right;
  img->chroma_height_confwin= img->chroma_height-top-bottom;
}

void increase_pending_tasks(de265_image* img, int n)
{
  de265_sync_add_and_fetch(&img->tasks_pending, n);
}

void decrease_pending_tasks(de265_image* img, int n)
{
  de265_mutex_lock(&img->mutex);

  int pending = de265_sync_sub_and_fetch(&img->tasks_pending, n);

  //printf("pending: %d\n",pending);

  assert(pending >= 0);

  if (pending==0) {
    de265_cond_broadcast(&img->finished_cond, &img->mutex);
  }

  de265_mutex_unlock(&img->mutex);
}

void wait_for_completion(de265_image* img)
{
  de265_mutex_lock(&img->mutex);
  while (img->tasks_pending>0) {
    de265_cond_wait(&img->finished_cond, &img->mutex);
  }
  de265_mutex_unlock(&img->mutex);
}



void img_clear_decoding_data(de265_image* img)
{
  // TODO: maybe we could avoid the memset by ensuring that all data is written to
  // during decoding (especially log2CbSize), but it is unlikely to be faster than the memset.

  img->cb_info.clear();
  img->tu_info.clear();
  img->ctb_info.clear();
  img->deblk_info.clear();

  // --- reset CTB progresses ---

  for (int i=0;i<img->ctb_info.data_size;i++) {
    img->ctb_progress[i].progress = CTB_PROGRESS_NONE;
  }
}


void set_mv_info(de265_image* img,int x,int y, int nPbW,int nPbH, const PredVectorInfo* mv)
{
  int log2PuSize = 2;

  int xPu = x >> log2PuSize;
  int yPu = y >> log2PuSize;
  int wPu = nPbW >> log2PuSize;
  int hPu = nPbH >> log2PuSize;

  int stride = img->pb_info.width_in_units;

  for (int pby=0;pby<hPu;pby++)
    for (int pbx=0;pbx<wPu;pbx++)
      {               
        img->pb_info[ xPu+pbx + (yPu+pby)*stride ].mvi = *mv;
      }
}
