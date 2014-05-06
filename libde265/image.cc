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
#include "decctx.h"

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


de265_image::de265_image() // (optional) init variables, do not alloc image
{
  // TODO: dangerous
  //memset(this, 0, sizeof(de265_image));

  for (int c=0;c<3;c++) {
    pixels[c] = NULL;
    pixels_mem[c] = NULL;
    pixels_confwin[c] = NULL;
  }

  width=height=0;

  ctb_progress = NULL;

  pts = 0;
  user_data = NULL;

  picture_order_cnt_lsb = -1; // undefined
  PicOrderCntVal = -1; // undefined
  PicState = UnusedForReference;

  de265_mutex_init(&mutex);
  de265_cond_init(&finished_cond);
}



de265_error de265_image::alloc_image(int w,int h, enum de265_chroma c,
                                     const seq_parameter_set* sps)
{
  const int _border=0;  // TODO: remove the border altogether

  // --- allocate image buffer (or reuse old one) ---

  if (width != w || height != h || chroma_format != c || border != _border) {

    chroma_width = w;
    chroma_height= h;

    if (c==de265_chroma_420) {
      chroma_width  = (chroma_width +1)/2;
      chroma_height = (chroma_height+1)/2;
    }

    if (c==de265_chroma_422) {
      chroma_height = (chroma_height+1)/2;
    }

    stride        = (w           +2*_border+alignment-1) / alignment * alignment;
    chroma_stride = (chroma_width+2*_border+alignment-1) / alignment * alignment;

    width = w;
    height= h;
    border=_border;

    chroma_format= c;

    FREE_ALIGNED(pixels_mem[0]);
    pixels_mem[0] = (uint8_t *)ALLOC_ALIGNED_16(stride * (h+2*border) + MEMORY_PADDING);
    pixels[0]     = pixels_mem[0] + border + 2*border*stride;

    if (c != de265_chroma_mono) {
      for (int col=1;col<=2;col++) {
        FREE_ALIGNED(pixels_mem[col]);
        pixels_mem[col] = (uint8_t *)ALLOC_ALIGNED_16(chroma_stride *
                                                      (chroma_height+2*border) + MEMORY_PADDING);

        pixels[col] = pixels_mem[col] + border + 2*border*chroma_stride;
      }
    } else {
      pixels[1] = pixels[2] = NULL;
      pixels_mem[1] = pixels_mem[2] = NULL;
    }
  }


  // check for memory shortage

  if (pixels_mem[0]  == NULL ||
      pixels_mem[1] == NULL ||
      pixels_mem[2] == NULL)
    {
      return DE265_ERROR_OUT_OF_MEMORY;
    }


  // --- allocate decoding info arrays ---

  bool mem_alloc_success = true;

  if (sps) {
    // intra pred mode

    mem_alloc_success &= intraPredMode.alloc(sps->PicWidthInMinPUs, sps->PicHeightInMinPUs,
                                             sps->Log2MinPUSize);

    // cb info

    mem_alloc_success &= cb_info.alloc(sps->PicWidthInMinCbsY, sps->PicHeightInMinCbsY,
                                       sps->Log2MinCbSizeY);

    // pb info

    int puWidth  = sps->PicWidthInMinCbsY  << (sps->Log2MinCbSizeY -2);
    int puHeight = sps->PicHeightInMinCbsY << (sps->Log2MinCbSizeY -2);

    mem_alloc_success &= pb_info.alloc(puWidth,puHeight, 2);


    // tu info

    mem_alloc_success &= tu_info.alloc(sps->PicWidthInTbsY, sps->PicHeightInTbsY,
                                       sps->Log2MinTrafoSize);

    // deblk info

    int deblk_w = (sps->pic_width_in_luma_samples +3)/4;
    int deblk_h = (sps->pic_height_in_luma_samples+3)/4;

    mem_alloc_success &= deblk_info.alloc(deblk_w, deblk_h, 2);

    // CTB info

    if (ctb_info.data_size != sps->PicSizeInCtbsY)
      {
        for (int i=0;i<ctb_info.data_size;i++)
          { de265_progress_lock_destroy(&ctb_progress[i]); }

        free(ctb_progress);

        mem_alloc_success &= ctb_info.alloc(sps->PicWidthInCtbsY, sps->PicHeightInCtbsY,
                                            sps->Log2CtbSizeY);

        ctb_progress = (de265_progress_lock*)malloc( sizeof(de265_progress_lock)
                                                     * ctb_info.data_size);

        for (int i=0;i<ctb_info.data_size;i++)
          { de265_progress_lock_init(&ctb_progress[i]); }
      }


    // check for memory shortage

    if (!mem_alloc_success)
      {
        return DE265_ERROR_OUT_OF_MEMORY;
      }
  }

  return DE265_OK;
}


de265_image::~de265_image()
{
  for (int c=0;c<3;c++) {
    if (pixels_mem[c])  FREE_ALIGNED(pixels_mem[c]);
  }

  for (int i=0;i<ctb_info.data_size;i++)
    { de265_progress_lock_destroy(&ctb_progress[i]); }


  for (int i=0;i<slices.size();i++) {
    delete slices[i];
  }
  slices.clear();

  free(ctb_progress);

  de265_cond_destroy(&finished_cond);
  de265_mutex_destroy(&mutex);
}


void de265_image::fill_image(int y,int cb,int cr)
{
  if (y>=0) {
    memset(pixels_mem[0], y, stride * (height+2*border));
  }

  if (cb>=0) {
    memset(pixels_mem[1], cb, chroma_stride * (chroma_height+2*border));
  }

  if (cr>=0) {
    memset(pixels_mem[2], cr, chroma_stride * (chroma_height+2*border));
  }
}


void de265_image::copy_image(const de265_image* src)
{
  alloc_image(src->width, src->height, src->chroma_format, NULL);

  assert(src->stride == stride &&
         src->chroma_stride == chroma_stride);


  if (src->stride == stride) {
    memcpy(pixels[0], src->pixels[0], src->height*src->stride);
  }
  else {
    for (int yp=0;yp<src->height;yp++) {
      memcpy(pixels[0]+yp*stride, src->pixels[0]+yp*src->stride, src->width);
    }
  }

  if (src->chroma_format != de265_chroma_mono) {
    if (src->chroma_stride == chroma_stride) {
      memcpy(pixels[1], src->pixels[1], src->chroma_height*src->chroma_stride);
      memcpy(pixels[2], src->pixels[2], src->chroma_height*src->chroma_stride);
    }
    else {
      for (int y=0;y<src->chroma_height;y++) {
        memcpy(pixels[1]+y*chroma_stride, src->pixels[1]+y*src->chroma_stride, src->chroma_width);
        memcpy(pixels[2]+y*chroma_stride, src->pixels[2]+y*src->chroma_stride, src->chroma_width);
      }
    }
  }
}


void de265_image::set_conformance_window()
{
  int left   = sps.conf_win_left_offset;
  int right  = sps.conf_win_right_offset;
  int top    = sps.conf_win_top_offset;
  int bottom = sps.conf_win_bottom_offset;

  int WinUnitX, WinUnitY;

  switch (chroma_format) {
  case de265_chroma_mono: WinUnitX=1; WinUnitY=1; break;
  case de265_chroma_420:  WinUnitX=2; WinUnitY=2; break;
  case de265_chroma_422:  WinUnitX=2; WinUnitY=1; break;
  case de265_chroma_444:  WinUnitX=1; WinUnitY=1; break;
  default:
    assert(0);
  }

  pixels_confwin[0] = pixels[0] + left*WinUnitX + top*WinUnitY*stride;
  pixels_confwin[1] = pixels[1] + left + top*chroma_stride;
  pixels_confwin[2] = pixels[2] + left + top*chroma_stride;

  width_confwin = width - (left+right)*WinUnitX;
  height_confwin= height- (top+bottom)*WinUnitY;
  chroma_width_confwin = chroma_width -left-right;
  chroma_height_confwin= chroma_height-top-bottom;
}

void de265_image::increase_pending_tasks(int n)
{
  de265_sync_add_and_fetch(&tasks_pending, n);
}

void de265_image::decrease_pending_tasks(int n)
{
  de265_mutex_lock(&mutex);

  int pending = de265_sync_sub_and_fetch(&tasks_pending, n);

  assert(pending >= 0);

  if (pending==0) {
    de265_cond_broadcast(&finished_cond, &mutex);
  }

  de265_mutex_unlock(&mutex);
}

void de265_image::wait_for_completion()
{
  de265_mutex_lock(&mutex);
  while (tasks_pending>0) {
    de265_cond_wait(&finished_cond, &mutex);
  }
  de265_mutex_unlock(&mutex);
}



void de265_image::clear_metadata()
{
  for (int i=0;i<slices.size();i++)
    delete slices[i];
  slices.clear();

  // TODO: maybe we could avoid the memset by ensuring that all data is written to
  // during decoding (especially log2CbSize), but it is unlikely to be faster than the memset.

  cb_info.clear();
  tu_info.clear();
  ctb_info.clear();
  deblk_info.clear();

  // --- reset CTB progresses ---

  for (int i=0;i<ctb_info.data_size;i++) {
    ctb_progress[i].progress = CTB_PROGRESS_NONE;
  }
}


void de265_image::set_mv_info(int x,int y, int nPbW,int nPbH, const PredVectorInfo* mv)
{
  int log2PuSize = 2;

  int xPu = x >> log2PuSize;
  int yPu = y >> log2PuSize;
  int wPu = nPbW >> log2PuSize;
  int hPu = nPbH >> log2PuSize;

  int stride = pb_info.width_in_units;

  for (int pby=0;pby<hPu;pby++)
    for (int pbx=0;pbx<wPu;pbx++)
      {
        pb_info[ xPu+pbx + (yPu+pby)*stride ].mvi = *mv;
      }
}


void de265_image::mark_slice_headers_as_unused(decoder_context* ctx)
{
  if (integrity == INTEGRITY_UNAVAILABLE_REFERENCE) {
    return;
  }

  for (int i=0;i<ctb_info.data_size;i++)
    {
      int sliceHeaderIdx = ctb_info[i].SliceHeaderIndex;

      slice_segment_header* shdr;
      //shdr = ctx->img->slices[ sliceHeaderIdx ];
      //shdr = &ctx->slice[ sliceHeaderIdx ];
      
      //printf("cleanup SHDR %d\n",sliceHeaderIdx);
      
      //shdr->inUse = false;
    }
}



bool de265_image::available_zscan(int xCurr,int yCurr, int xN,int yN) const
{
  if (xN<0 || yN<0) return false;
  if (xN>=sps.pic_width_in_luma_samples ||
      yN>=sps.pic_height_in_luma_samples) return false;

  int minBlockAddrN = pps.MinTbAddrZS[ (xN>>sps.Log2MinTrafoSize) +
                                       (yN>>sps.Log2MinTrafoSize) * sps.PicWidthInTbsY ];
  int minBlockAddrCurr = pps.MinTbAddrZS[ (xCurr>>sps.Log2MinTrafoSize) +
                                          (yCurr>>sps.Log2MinTrafoSize) * sps.PicWidthInTbsY ];

  if (minBlockAddrN > minBlockAddrCurr) return false;

  int xCurrCtb = xCurr >> sps.Log2CtbSizeY;
  int yCurrCtb = yCurr >> sps.Log2CtbSizeY;
  int xNCtb = xN >> sps.Log2CtbSizeY;
  int yNCtb = yN >> sps.Log2CtbSizeY;

  if (get_SliceAddrRS(xCurrCtb,yCurrCtb) !=
      get_SliceAddrRS(xNCtb,   yNCtb)) {
    return false;
  }

  if (pps.TileIdRS[xCurrCtb + yCurrCtb*sps.PicWidthInCtbsY] !=
      pps.TileIdRS[xNCtb    + yNCtb   *sps.PicWidthInCtbsY]) {
    return false;
  }

  return true;
}


bool de265_image::available_pred_blk(int xC,int yC, int nCbS, int xP, int yP,
                                     int nPbW, int nPbH, int partIdx, int xN,int yN) const
{
  logtrace(LogMotion,"C:%d;%d P:%d;%d N:%d;%d size=%d;%d\n",xC,yC,xP,yP,xN,yN,nPbW,nPbH);

  int sameCb = (xC <= xN && xN < xC+nCbS &&
                yC <= yN && yN < yC+nCbS);

  bool availableN;

  if (!sameCb) {
    availableN = available_zscan(xP,yP,xN,yN);
  }
  else {
    availableN = !(nPbW<<1 == nCbS && nPbH<<1 == nCbS &&
                   partIdx==1 &&
                   yN >= yC+nPbH && xN < xC+nPbW);
  }

  if (availableN && get_pred_mode(xN,yN) == MODE_INTRA) {
    availableN = false;
  }

  return availableN;
}

