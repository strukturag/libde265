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
#include "encoder/encoder-context.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <limits>


#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif

#ifdef HAVE_SSE4_1
#define MEMORY_PADDING  8
#else
#define MEMORY_PADDING  0
#endif

#define STANDARD_ALIGNMENT 16

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

LIBDE265_API void* de265_alloc_image_plane(struct de265_image* img, int cIdx,
                                           void* inputdata, int inputstride, void *userdata)
{
  int alignment = STANDARD_ALIGNMENT;
  int stride = (img->get_width(cIdx) + alignment-1) / alignment * alignment;
  int height = img->get_height(cIdx);

  uint8_t* p = (uint8_t *)ALLOC_ALIGNED_16(stride * height + MEMORY_PADDING);

  if (p==NULL) { return NULL; }

  img->set_image_plane(cIdx, p, stride, userdata);

  // copy input data if provided

  if (inputdata != NULL) {
    if (inputstride == stride) {
      memcpy(p, inputdata, stride*height);
    }
    else {
      for (int y=0;y<height;y++) {
        memcpy(p+y*stride, ((char*)inputdata) + inputstride*y, inputstride);
      }
    }
  }

  return p;
}


LIBDE265_API void de265_free_image_plane(struct de265_image* img, int cIdx)
{
  uint8_t* p = (uint8_t*)img->get_image_plane(cIdx);
  assert(p);
  FREE_ALIGNED(p);
}


static int  de265_image_get_buffer(de265_decoder_context* ctx,
                                   de265_image_spec* spec, de265_image* img, void* userdata)
{
  int luma_stride   = (spec->width   + spec->alignment-1) / spec->alignment * spec->alignment;
  int chroma_stride = (spec->width/2 + spec->alignment-1) / spec->alignment * spec->alignment;

  assert(img->sps.BitDepth_Y >= 8 && img->sps.BitDepth_Y <= 16);
  assert(img->sps.BitDepth_C >= 8 && img->sps.BitDepth_C <= 16);

  int luma_bpl   = luma_stride   * (img->sps.BitDepth_Y+7)/8;
  int chroma_bpl = chroma_stride * (img->sps.BitDepth_C+7)/8;

  int luma_height   = spec->height;
  int chroma_height = (spec->height+1)/2;

  uint8_t* p[3] = { 0,0,0 };
  p[0] = (uint8_t *)ALLOC_ALIGNED_16(luma_height   * luma_bpl   + MEMORY_PADDING);
  p[1] = (uint8_t *)ALLOC_ALIGNED_16(chroma_height * chroma_bpl + MEMORY_PADDING);
  p[2] = (uint8_t *)ALLOC_ALIGNED_16(chroma_height * chroma_bpl + MEMORY_PADDING);

  if (p[0]==NULL || p[1]==NULL || p[2]==NULL) {
    for (int i=0;i<3;i++)
      if (p[i]) {
        FREE_ALIGNED(p[i]);
      }

    return 0;
  }

  img->set_image_plane(0, p[0], luma_stride, NULL);
  img->set_image_plane(1, p[1], chroma_stride, NULL);
  img->set_image_plane(2, p[2], chroma_stride, NULL);

  return 1;
}

static void de265_image_release_buffer(de265_decoder_context* ctx,
                                       de265_image* img, void* userdata)
{
  for (int i=0;i<3;i++) {
    uint8_t* p = (uint8_t*)img->get_image_plane(i);
    assert(p);
    FREE_ALIGNED(p);
  }
}


de265_image_allocation de265_image::default_image_allocation = {
  de265_image_get_buffer,
  de265_image_release_buffer
};


void de265_image::set_image_plane(int cIdx, uint8_t* mem, int stride, void *userdata)
{
  pixels[cIdx] = mem;
  plane_user_data[cIdx] = userdata;

  if (cIdx==0) { this->stride        = stride; }
  else         { this->chroma_stride = stride; }
}


uint32_t de265_image::s_next_image_ID = 0;

de265_image::de265_image()
{
  ID = -1;
  removed_at_picture_id = 0; // picture not used, so we can assume it has been removed

  decctx = NULL;
  encctx = NULL;

  encoder_image_release_func = NULL;

  //alloc_functions.get_buffer = NULL;
  //alloc_functions.release_buffer = NULL;

  for (int c=0;c<3;c++) {
    pixels[c] = NULL;
    pixels_confwin[c] = NULL;
    plane_user_data[c] = NULL;
  }

  width=height=0;

  pts = 0;
  user_data = NULL;

  ctb_progress = NULL;

  integrity = INTEGRITY_NOT_DECODED;

  picture_order_cnt_lsb = -1; // undefined
  PicOrderCntVal = -1; // undefined
  PicState = UnusedForReference;
  PicOutputFlag = false;

  nThreadsQueued   = 0;
  nThreadsRunning  = 0;
  nThreadsBlocked  = 0;
  nThreadsFinished = 0;
  nThreadsTotal    = 0;

  de265_mutex_init(&mutex);
  de265_cond_init(&finished_cond);
}


de265_error de265_image::alloc_image(int w,int h, enum de265_chroma c,
                                     const seq_parameter_set* sps, bool allocMetadata,
                                     decoder_context* dctx,
                                     encoder_context* ectx,
                                     de265_PTS pts, void* user_data,
                                     bool useCustomAllocFunc)
{
  //if (allocMetadata) { assert(sps); }
  assert(sps);

  this->sps = *sps;

  release(); /* TODO: review code for efficient allocation when arrays are already
                allocated to the requested size. Without the release, the old image-data
                will not be freed. */

  ID = s_next_image_ID++;
  removed_at_picture_id = std::numeric_limits<int32_t>::max();

  decctx = dctx;
  encctx = ectx;

  // --- allocate image buffer ---

  chroma_format= c;

  width = w;
  height = h;
  chroma_width = w;
  chroma_height= h;

  this->user_data = user_data;
  this->pts = pts;

  de265_image_spec spec;

  int WinUnitX, WinUnitY;

  switch (chroma_format) {
  case de265_chroma_mono: WinUnitX=1; WinUnitY=1; break;
  case de265_chroma_420:  WinUnitX=2; WinUnitY=2; break;
  case de265_chroma_422:  WinUnitX=2; WinUnitY=1; break;
  case de265_chroma_444:  WinUnitX=1; WinUnitY=1; break;
  default:
    assert(0);
  }

  switch (chroma_format) {
  case de265_chroma_420:
    spec.format = de265_image_format_YUV420P8;
    chroma_width  = (chroma_width +1)/2;
    chroma_height = (chroma_height+1)/2;
    break;

  case de265_chroma_422:
    spec.format = de265_image_format_YUV422P8;
    chroma_width = (chroma_width+1)/2;
    break;

  default:
    return DE265_ERROR_NOT_IMPLEMENTED_YET;
    break;
  }

  spec.width  = w;
  spec.height = h;
  spec.alignment = STANDARD_ALIGNMENT;


  // conformance window cropping

  int left   = sps ? sps->conf_win_left_offset : 0;
  int right  = sps ? sps->conf_win_right_offset : 0;
  int top    = sps ? sps->conf_win_top_offset : 0;
  int bottom = sps ? sps->conf_win_bottom_offset : 0;

  width_confwin = width - (left+right)*WinUnitX;
  height_confwin= height- (top+bottom)*WinUnitY;
  chroma_width_confwin = chroma_width -left-right;
  chroma_height_confwin= chroma_height-top-bottom;

  spec.crop_left  = left *WinUnitX;
  spec.crop_right = right*WinUnitX;
  spec.crop_top   = top   *WinUnitY;
  spec.crop_bottom= bottom*WinUnitY;

  spec.visible_width = width_confwin;
  spec.visible_height= height_confwin;


  bpp_shift[0] = (sps->BitDepth_Y > 8) ? 1 : 0;
  bpp_shift[1] = (sps->BitDepth_C > 8) ? 1 : 0;
  bpp_shift[2] = bpp_shift[1];


  // allocate memory and set conformance window pointers

  void* alloc_userdata = NULL;
  if (decctx) alloc_userdata = decctx->param_image_allocation_userdata;
  if (encctx) alloc_userdata = encctx->param_image_allocation_userdata; // actually not needed

  if (encctx && useCustomAllocFunc) {
    encoder_image_release_func = encctx->release_func;

    // if we do not provide a release function, use our own

    if (encoder_image_release_func == NULL) {
      image_allocation_functions = de265_image::default_image_allocation;
    }
    else {
      image_allocation_functions.get_buffer     = NULL;
      image_allocation_functions.release_buffer = NULL;
    }
  }
  else if (decctx && useCustomAllocFunc) {
    image_allocation_functions = decctx->param_image_allocation_functions;
  }
  else {
    image_allocation_functions = de265_image::default_image_allocation;
  }

  bool mem_alloc_success = true;

  if (image_allocation_functions.get_buffer != NULL) {
    mem_alloc_success = image_allocation_functions.get_buffer(decctx, &spec, this,
                                                              alloc_userdata);

    pixels_confwin[0] = pixels[0] + left*WinUnitX + top*WinUnitY*stride;
    pixels_confwin[1] = pixels[1] + left + top*chroma_stride;
    pixels_confwin[2] = pixels[2] + left + top*chroma_stride;


    // check for memory shortage

    if (!mem_alloc_success)
      {
        return DE265_ERROR_OUT_OF_MEMORY;
      }
  }

  //alloc_functions = *allocfunc;
  //alloc_userdata  = userdata;

  // --- allocate decoding info arrays ---

  if (allocMetadata) {
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
        delete[] ctb_progress;

        mem_alloc_success &= ctb_info.alloc(sps->PicWidthInCtbsY, sps->PicHeightInCtbsY,
                                            sps->Log2CtbSizeY);

        ctb_progress = new de265_progress_lock[ ctb_info.data_size ];
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
  release();

  // free progress locks

  if (ctb_progress) {
    delete[] ctb_progress;
  }

  de265_cond_destroy(&finished_cond);
  de265_mutex_destroy(&mutex);
}


void de265_image::release()
{
  // free image memory

  if (pixels[0])
    {
      if (encoder_image_release_func != NULL) {
        encoder_image_release_func(encctx, this,
                                   encctx->param_image_allocation_userdata);
      }
      else {
        image_allocation_functions.release_buffer(decctx, this,
                                                decctx ?
                                                  decctx->param_image_allocation_userdata :
                                                  NULL);
      }

      for (int i=0;i<3;i++)
        {
          pixels[i] = NULL;
          pixels_confwin[i] = NULL;
        }
    }

  // free slices

  for (int i=0;i<slices.size();i++) {
    delete slices[i];
  }
  slices.clear();
}


void de265_image::fill_image(int y,int cb,int cr)
{
  if (y>=0) {
    memset(pixels[0], y, stride * height);
  }

  if (cb>=0) {
    memset(pixels[1], cb, chroma_stride * chroma_height);
  }

  if (cr>=0) {
    memset(pixels[2], cr, chroma_stride * chroma_height);
  }
}


de265_error de265_image::copy_image(const de265_image* src)
{
  /* TODO: actually, since we allocate the image only for internal purpose, we
     do not have to call the external allocation routines for this. However, then
     we have to track for each image how to release it again.
     Another option would be to safe the copied data not in an de265_image at all.
  */

  de265_error err = alloc_image(src->width, src->height, src->chroma_format, &src->sps, false,
                                src->decctx, src->encctx, src->pts, src->user_data, false);
  if (err != DE265_OK) {
    return err;
  }

  copy_lines_from(src, 0, src->height);

  return err;
}

void de265_image::copy_metadata(const de265_image* src)
{
  if (width != src->get_width() || height != src->get_height()) {
    assert( false );
  }
  
  ctb_info.copy(&src->ctb_info);
  cb_info.copy(&src->cb_info);
  pb_info.copy(&src->pb_info);
  intraPredMode.copy(&src->intraPredMode);
  tu_info.copy(&src->tu_info);
  deblk_info.copy(&src->deblk_info);

  // Copy the pointers to the slice segment headers.
  // TODO: Is this a good idea?
  slices.clear();
  for (int i = 0; i < src->slices.size(); i++) {
    slices.push_back(src->slices.at(i));
  }
}

/* H.8.1.4.2 Resampling process of picture motion and mode parameters
 *
 * Upsample the metadata from the given picture using the given scaling parameters.
*/
void de265_image::upsample_metadata(const de265_image* src, int scaling_parameters[10])
{
  int PicWidthInSamplesCurrY      = get_width();
  int PiPicHeightInSamplesCurrY   = get_height();
  int PicWidthInSamplesRefLayerY  = src->get_width();
  int PicHeightInSamplesRefLayerY = src->get_height();

  int ScaledRefRegionWidthInSamplesY  = scaling_parameters[6];
  int RefLayerRegionWidthInSamplesY   = scaling_parameters[7];
  int ScaledRefRegionHeightInSamplesY = scaling_parameters[8];
  int RefLayerRegionHeightInSamplesY  = scaling_parameters[9];

  int xMax = ((PicWidthInSamplesCurrY    + 15) >> 4) - 1;
  int yMax = ((PiPicHeightInSamplesCurrY + 15) >> 4) - 1;

  int xPb, yPb, xPCtr, yPCtr, xRef, yRef, xRL, yRL, width, height;
  for (int xB = 0; xB <= xMax; xB++) {
    for (int yB = 0; yB <= yMax; yB++) {
      xPb = xB << 4;
      yPb = yB << 4;

      // 1. The center location ( xPCtr, yPCtr ) of the luma prediction block is derived as follows:
      xPCtr = xPb + 8;  // (H 65)
      yPCtr = yPb + 8;  // (H 66)

      // 2. The variables xRef and yRef are derived as follows:
      xRef = (((xPCtr - scaling_parameters[0]) * scaling_parameters[2] + (1 << 15)) >> 16 ) + scaling_parameters[4];  // (H 67)
      yRef = (((yPCtr - scaling_parameters[1]) * scaling_parameters[3] + (1 << 15)) >> 16 ) + scaling_parameters[5];  // (H 68)

      // 3. The rounded reference layer luma sample location ( xRL, yRL ) is derived as follows:
      xRL = ((xRef + 4) >> 4) << 4;  // (H 69)
      yRL = ((yRef + 4) >> 4) << 4;  // (H 70)

      // Calculate the width and height of the block. This is necessary since "set_pred_mode" and
      // "set_mv_info" are not capable of handling blocks that reach outside the picture bundary.
      width = 16;
      height = 16;
      if (xPb+16 > PicWidthInSamplesCurrY || yPb+16 > PiPicHeightInSamplesCurrY) {
        // The width or height of the block is not 16x16
        width = (PicWidthInSamplesCurrY - xPb);
        height = (PiPicHeightInSamplesCurrY - yPb);
      }

      // 4. Upsample the prediction mode. (H 71)
      PredMode rsPredMode;
      if( xRL < 0 || xRL >= PicWidthInSamplesRefLayerY || yRL < 0 || yRL >= PicHeightInSamplesRefLayerY ) {
        rsPredMode = MODE_INTRA;
      }
      else {
        rsPredMode = src->get_pred_mode(xRL, yRL);
      }
      set_pred_mode(xPb, yPb, width, height, rsPredMode);

      // 5. Upsample the motion vectors and prediction flags
      MotionVectorSpec mv_dst;
      if (rsPredMode == MODE_INTER) {
        const MotionVectorSpec *mv_src = src->get_mv_info(xRL, yRL);
        // For X being each of 0 and 1...
        for (int l=0; l<2; l++) {
          // RefIdx, predFlag
          mv_dst.refIdx[l] = mv_src->refIdx[l];     // (H 72)
          mv_dst.predFlag[l] = mv_src->predFlag[l]; // (H 73)

          // Motion vector. X-component.
          if (ScaledRefRegionWidthInSamplesY != RefLayerRegionWidthInSamplesY) {
            int rlMvLX = mv_src->mv[l].x;
            int scaleMVX = Clip3( -4096, 4095, ((ScaledRefRegionWidthInSamplesY << 8) + (RefLayerRegionWidthInSamplesY >> 1)) / RefLayerRegionWidthInSamplesY); // (H 74)
            mv_dst.mv[l].x = Clip3( -32768, 32767, Sign( scaleMVX *	rlMvLX) * ((abs_value(scaleMVX * rlMvLX) + 127) >> 8)); // (H 75)
          }
          else {
            mv_dst.mv[l].x = mv_src->mv[l].x; // (H 76)
          }

          // Motion vector. Y-component.
          if (ScaledRefRegionHeightInSamplesY != RefLayerRegionHeightInSamplesY) {
            int rlMvLX = mv_src->mv[l].y;
            int scaleMVX = Clip3( -4096, 4095, ((ScaledRefRegionHeightInSamplesY << 8) + (RefLayerRegionHeightInSamplesY >> 1)) / RefLayerRegionHeightInSamplesY); // (H 77)
            mv_dst.mv[l].y = Clip3( -32768, 32767, Sign( scaleMVX *	rlMvLX) * ((abs_value(scaleMVX * rlMvLX) + 127) >> 8)); // (H 78)
          }
          else {
            mv_dst.mv[l].y = mv_src->mv[l].y;  // (H 79)
          }
        }
      }
      else {
        // Otherwise (rsPredMode is equal to MODE_INTRA), the following applies:
        mv_dst.mv[0].x = 0;
        mv_dst.mv[0].y = 0;
        mv_dst.mv[1].x = 0;
        mv_dst.mv[1].y = 0;
        mv_dst.refIdx[0] = -1;
        mv_dst.refIdx[1] = -1;
        mv_dst.predFlag[0] = 0;
        mv_dst.predFlag[1] = 0;
      }
      // Set the derived motion info in the output picture
      set_mv_info(xPb, yPb, width, height, mv_dst);
    }
  }
}

// end = last line + 1
void de265_image::copy_lines_from(const de265_image* src, int first, int end)
{
  if (end > src->height) end=src->height;

  assert(first % 2 == 0);
  assert(end   % 2 == 0);

  int luma_bpp   = (sps.BitDepth_Y+7)/8;
  int chroma_bpp = (sps.BitDepth_C+7)/8;

  if (src->stride == stride) {
    memcpy(pixels[0]      + first*stride * luma_bpp,
           src->pixels[0] + first*src->stride * luma_bpp,
           (end-first)*stride * luma_bpp);
  }
  else {
    for (int yp=first;yp<end;yp++) {
      memcpy(pixels[0]+yp*stride * luma_bpp,
             src->pixels[0]+yp*src->stride * luma_bpp,
             src->width * luma_bpp);
    }
  }

  int first_chroma = first>>1;
  int end_chroma   = end>>1;

  if (src->chroma_format != de265_chroma_mono) {
    if (src->chroma_stride == chroma_stride) {
      memcpy(pixels[1]      + first_chroma*chroma_stride * chroma_bpp,
             src->pixels[1] + first_chroma*chroma_stride * chroma_bpp,
             (end_chroma-first_chroma) * chroma_stride * chroma_bpp);
      memcpy(pixels[2]      + first_chroma*chroma_stride * chroma_bpp,
             src->pixels[2] + first_chroma*chroma_stride * chroma_bpp,
             (end_chroma-first_chroma) * chroma_stride * chroma_bpp);
    }
    else {
      for (int y=first_chroma;y<end_chroma;y++) {
        memcpy(pixels[1]+y*chroma_stride * chroma_bpp,
               src->pixels[1]+y*src->chroma_stride * chroma_bpp,
               src->chroma_width * chroma_bpp);
        memcpy(pixels[2]+y*chroma_stride * chroma_bpp,
               src->pixels[2]+y*src->chroma_stride * chroma_bpp,
               src->chroma_width * chroma_bpp);
      }
    }
  }
}


void de265_image::exchange_pixel_data_with(de265_image& b)
{
  for (int i=0;i<3;i++) {
    std::swap(pixels[i], b.pixels[i]);
    std::swap(pixels_confwin[i], b.pixels_confwin[i]);
    std::swap(plane_user_data[i], b.plane_user_data[i]);
  }

  std::swap(stride, b.stride);
  std::swap(chroma_stride, b.chroma_stride);
  std::swap(image_allocation_functions, b.image_allocation_functions);
}


void de265_image::thread_start(int nThreads)
{
  de265_mutex_lock(&mutex);

  //printf("nThreads before: %d %d\n",nThreadsQueued, nThreadsTotal);

  nThreadsQueued += nThreads;
  nThreadsTotal += nThreads;

  //printf("nThreads after: %d %d\n",nThreadsQueued, nThreadsTotal);

  de265_mutex_unlock(&mutex);
}

void de265_image::thread_run(const thread_task* task)
{
  //printf("run thread %s\n", task->name().c_str());

  de265_mutex_lock(&mutex);
  nThreadsQueued--;
  nThreadsRunning++;
  de265_mutex_unlock(&mutex);
}

void de265_image::thread_blocks()
{
  de265_mutex_lock(&mutex);
  nThreadsRunning--;
  nThreadsBlocked++;
  de265_mutex_unlock(&mutex);
}

void de265_image::thread_unblocks()
{
  de265_mutex_lock(&mutex);
  nThreadsBlocked--;
  nThreadsRunning++;
  de265_mutex_unlock(&mutex);
}

void de265_image::thread_finishes(const thread_task* task)
{
  //printf("finish thread %s\n", task->name().c_str());

  de265_mutex_lock(&mutex);

  nThreadsRunning--;
  nThreadsFinished++;
  assert(nThreadsRunning >= 0);

  if (nThreadsFinished==nThreadsTotal) {
    de265_cond_broadcast(&finished_cond, &mutex);
  }

  de265_mutex_unlock(&mutex);
}

void de265_image::wait_for_progress(thread_task* task, int ctbx,int ctby, int progress)
{
  const int ctbW = sps.PicWidthInCtbsY;

  wait_for_progress(task, ctbx + ctbW*ctby, progress);
}

void de265_image::wait_for_progress(thread_task* task, int ctbAddrRS, int progress)
{
  if (task==NULL) { return; }

  de265_progress_lock* progresslock = &ctb_progress[ctbAddrRS];
  if (progresslock->get_progress() < progress) {
    thread_blocks();

    assert(task!=NULL);
    task->state = thread_task::Blocked;

    /* TODO: check whether we are the first blocked task in the list.
       If we are, we have to conceal input errors.
       Simplest concealment: do not block.
    */

    progresslock->wait_for_progress(progress);
    task->state = thread_task::Running;
    thread_unblocks();
  }
}


void de265_image::wait_for_completion()
{
  de265_mutex_lock(&mutex);
  while (nThreadsFinished!=nThreadsTotal) {
    de265_cond_wait(&finished_cond, &mutex);
  }
  de265_mutex_unlock(&mutex);
}

bool de265_image::debug_is_completed() const
{
  return nThreadsFinished==nThreadsTotal;
}



void de265_image::clear_metadata()
{
  // TODO: maybe we could avoid the memset by ensuring that all data is written to
  // during decoding (especially log2CbSize), but it is unlikely to be faster than the memset.

  cb_info.clear();
  //tu_info.clear();  // done on the fly
  ctb_info.clear();
  deblk_info.clear();

  // --- reset CTB progresses ---

  for (int i=0;i<ctb_info.data_size;i++) {
    ctb_progress[i].reset(CTB_PROGRESS_NONE);
  }
}


void de265_image::set_mv_info(int x,int y, int nPbW,int nPbH, const MotionVectorSpec& mv)
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
        pb_info[ xPu+pbx + (yPu+pby)*stride ].mv = mv;
      }
}

void de265_image::set_pred_mode(int x, int y, int nPbW, int nPbH, enum PredMode mode)
{
  int log2PuSize = 2;

  int cbX = x >> cb_info.log2unitSize;
  int cbY = y >> cb_info.log2unitSize;

  int wPu = nPbW >> cb_info.log2unitSize;
  int hPu = nPbH >> cb_info.log2unitSize;

  for (int cby=cbY;cby<cbY+wPu;cby++)
    for (int cbx=cbX;cbx<cbX+hPu;cbx++) {
      cb_info[ cbx + cby*cb_info.width_in_units ].PredMode = mode;
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
    availableN = !(nPbW<<1 == nCbS && nPbH<<1 == nCbS &&  // NxN
                   partIdx==1 &&
                   yN >= yC+nPbH && xN < xC+nPbW);  // xN/yN inside partIdx 2
  }

  if (availableN && get_pred_mode(xN,yN) == MODE_INTRA) {
    availableN = false;
  }

  return availableN;
}
