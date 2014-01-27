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
#include <assert.h>

#ifdef HAVE___MINGW_ALIGNED_MALLOC
#define ALLOC_ALIGNED(alignment, size)         __mingw_aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                      __mingw_aligned_free((mem))
#elif _WIN32
#define ALLOC_ALIGNED(alignment, size)         _aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                      _aligned_free((mem))
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



void de265_alloc_image(de265_image* img, int w,int h, enum de265_chroma c,
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
    img->y_mem = (uint8_t *)ALLOC_ALIGNED_16(img->stride * (h+2*border));
    img->y     = img->y_mem + border + 2*border*img->stride;

    if (c != de265_chroma_mono) {
      FREE_ALIGNED(img->cb_mem);
      FREE_ALIGNED(img->cr_mem);
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
  }

  // --- allocate decoding info arrays ---

  if (sps) {
    // intra pred mode

    int intraPredModeSize = sps->PicWidthInMinPUs * sps->PicHeightInMinPUs;
    if (intraPredModeSize != img->intraPredModeSize) {
      img->intraPredModeSize = intraPredModeSize;
      free(img->intraPredMode);
      img->intraPredMode = (uint8_t *) malloc(intraPredModeSize * sizeof(*img->intraPredMode));
    }


    // cb info

    if (img->cb_info_size != sps->PicSizeInMinCbsY ||
        img->cb_info == NULL) {
      img->cb_info_size = sps->PicSizeInMinCbsY;
      free(img->cb_info);
      img->cb_info = (CB_ref_info*)malloc(sizeof(CB_ref_info) * img->cb_info_size);
    }


    // pb info

    int puWidth  = sps->PicWidthInMinCbsY  << (sps->Log2MinCbSizeY -2);
    int puHeight = sps->PicHeightInMinCbsY << (sps->Log2MinCbSizeY -2);

    if (img->pb_info_size != puWidth*puHeight ||
        img->pb_info == NULL) {
      img->pb_info_size   = puWidth*puHeight;
      img->pb_info_stride = puWidth;
      free(img->pb_info);
      img->pb_info = (PB_ref_info*)malloc(sizeof(PB_ref_info) * img->pb_info_size);
      // ctx->img->pb_rootIdx = (int*)malloc(sizeof(int) * ctx->img->pb_info_size);
    }

    // tu info

    if (img->tu_info_size != sps->PicSizeInTbsY ||
        img->tu_info == NULL) {
      img->tu_info_size = sps->PicSizeInTbsY;
      free(img->tu_info);
      img->tu_info = (uint8_t*)malloc(sizeof(uint8_t) * img->tu_info_size);
    }
  }
}


void de265_free_image(de265_image* img)
{
  if (img->y)  FREE_ALIGNED(img->y_mem);
  if (img->cb) FREE_ALIGNED(img->cb_mem);
  if (img->cr) FREE_ALIGNED(img->cr_mem);

  free(img->cb_info);
  free(img->pb_info);
  //free(img->pb_rootIdx);

  free(img->intraPredMode);

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


void get_image_plane(const de265_image* img, int cIdx, uint8_t** image, int* stride)
{
  switch (cIdx) {
  case 0: *image = img->y;  if (stride) *stride = img->stride; break;
  case 1: *image = img->cb; if (stride) *stride = img->chroma_stride; break;
  case 2: *image = img->cr; if (stride) *stride = img->chroma_stride; break;
  default: *image = NULL; if (stride) *stride = 0; break;
  }
}


void increase_pending_tasks(de265_image* img, int n)
{
  de265_sync_add_and_fetch(&img->tasks_pending, n);
}

void decrease_pending_tasks(de265_image* img, int n)
{
  int pending = de265_sync_sub_and_fetch(&img->tasks_pending, n);

  assert(pending >= 0);

  if (pending==0) {
    de265_cond_broadcast(&img->finished_cond, &img->mutex);
  }
}

void wait_for_completion(de265_image* img)
{
  de265_mutex_lock(&img->mutex);
  while (img->tasks_pending>0) {
    de265_cond_wait(&img->finished_cond, &img->mutex);
  }
  de265_mutex_unlock(&img->mutex);
}



void prepare_image_for_decoding(de265_image* img)
{
  // TODO: maybe we could avoid the memset by ensuring that all data is written to
  // during decoding (especially log2CbSize), but it is unlikely to be faster than the memset.

  memset(img->cb_info,  0,img->cb_info_size * sizeof(CB_ref_info));

  memset(img->tu_info,  0,img->tu_info_size * sizeof(uint8_t));
}


#define PIXEL2CB(x) (x >> sps->Log2MinCbSizeY)
#define CB_IDX(x0,y0) (PIXEL2CB(x0) + PIXEL2CB(y0)*sps->PicWidthInMinCbsY)
#define SET_CB_BLK(x,y,log2BlkWidth,  Field,value)                      \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - sps->Log2MinCbSizeY);                \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      {                                                                 \
        img->cb_info[ cbx + cby*sps->PicWidthInMinCbsY ].Field = value; \
      }

#define SET_CB_BLK_SAVE(x,y,log2BlkWidth,  Field,value)                 \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - sps->Log2MinCbSizeY);                \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      if (cbx < sps->PicWidthInMinCbsY &&                               \
          cby < sps->PicHeightInMinCbsY)                                \
      {                                                                 \
        img->cb_info[ cbx + cby*sps->PicWidthInMinCbsY ].Field = value; \
      }


void    set_cu_skip_flag(const seq_parameter_set* sps, de265_image* img,
                         int x,int y, int log2BlkWidth, uint8_t flag)
{
  SET_CB_BLK(x,y,log2BlkWidth, cu_skip_flag, flag);
}

uint8_t get_cu_skip_flag(const seq_parameter_set* sps, const de265_image* img, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return img->cb_info[ cbX + cbY*sps->PicWidthInMinCbsY ].cu_skip_flag;
}


void set_pred_mode(de265_image* img, const seq_parameter_set* sps,
                   int x,int y, int log2BlkWidth, enum PredMode mode)
{
  SET_CB_BLK(x,y,log2BlkWidth, PredMode, mode);
}

enum PredMode get_pred_mode(const de265_image* img, const seq_parameter_set* sps, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return (enum PredMode)img->cb_info[ cbX + cbY*sps->PicWidthInMinCbsY ].PredMode;
}


int  get_log2CbSize(const de265_image* img, const seq_parameter_set* sps, int x0, int y0)
{
  int cbX = PIXEL2CB(x0);
  int cbY = PIXEL2CB(y0);

  return (enum PredMode)img->cb_info[ cbX + cbY*sps->PicWidthInMinCbsY ].log2CbSize;
}

void set_log2CbSize(de265_image* img, const seq_parameter_set* sps, int x0, int y0, int log2CbSize)
{
  int cbX = PIXEL2CB(x0);
  int cbY = PIXEL2CB(y0);

  img->cb_info[ cbX + cbY*sps->PicWidthInMinCbsY ].log2CbSize = log2CbSize;

  // assume that remaining cb_info blocks are initialized to zero
}

// coordinates in CB units
int  get_log2CbSize_cbUnits(de265_image* img, const seq_parameter_set* sps, int x0, int y0)
{
  return (enum PredMode)img->cb_info[ x0 + y0*sps->PicWidthInMinCbsY ].log2CbSize;
}


void          set_PartMode(de265_image* img, const seq_parameter_set* sps,
                           int x,int y, enum PartMode mode)
{
  img->cb_info[ CB_IDX(x,y) ].PartMode = mode;
}

enum PartMode get_PartMode(const de265_image* img, const seq_parameter_set* sps, int x,int y)
{
  return (enum PartMode)img->cb_info[ CB_IDX(x,y) ].PartMode;
}


void set_ctDepth(de265_image* img, const seq_parameter_set* sps,
                 int x,int y, int log2BlkWidth, int depth)
{
  SET_CB_BLK(x,y,log2BlkWidth, ctDepth, depth);
}

int get_ctDepth(const de265_image* img, const seq_parameter_set* sps, int x,int y)
{
  return img->cb_info[ CB_IDX(x,y) ].ctDepth;
}


void set_QPY(de265_image* img, const seq_parameter_set* sps,
             const pic_parameter_set* pps, int x,int y, int QP_Y)
{
  assert(x>=0 && x<sps->pic_width_in_luma_samples);
  assert(y>=0 && y<sps->pic_height_in_luma_samples);

  SET_CB_BLK_SAVE(x,y,pps->Log2MinCuQpDeltaSize, QP_Y, QP_Y);
}

int  get_QPY(const de265_image* img, const seq_parameter_set* sps,int x,int y)
{
  return img->cb_info[CB_IDX(x,y)].QP_Y;
}


#define PIXEL2TU(x) (x >> sps->Log2MinTrafoSize)
#define TU_IDX(x0,y0) (PIXEL2TU(x0) + PIXEL2TU(y0)*sps->PicWidthInTbsY)

#define OR_TU_BLK(x,y,log2BlkWidth,  Field,value)                       \
  int tuX = PIXEL2TU(x);                                                \
  int tuY = PIXEL2TU(y);                                                \
  int width = 1 << (log2BlkWidth - sps->Log2MinTrafoSize);              \
  for (int tuy=tuY;tuy<tuY+width;tuy++)                                 \
    for (int tux=tuX;tux<tuX+width;tux++)                               \
      {                                                                 \
        ctx->tu_info[ tux + tuy*sps->PicWidthInTbsY ].Field |= value;   \
      }

void set_split_transform_flag(de265_image* img,const seq_parameter_set* sps,
                              int x0,int y0,int trafoDepth)
{
  img->tu_info[TU_IDX(x0,y0)] |= (1<<trafoDepth);
}

int  get_split_transform_flag(const de265_image* img, const seq_parameter_set* sps,
                              int x0,int y0,int trafoDepth)
{
  int idx = TU_IDX(x0,y0);
  return (img->tu_info[idx] & (1<<trafoDepth));
}
