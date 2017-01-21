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

#ifdef HAVE_CONFIG_H
#include "config.h"  // NOLINT(build/include)
#endif

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <limits>
#include <utility>

#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif

#include "libde265/image.h"
#include "libde265/decctx.h"
#include "libde265/encoder/encoder-context.h"

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
#elif defined(HAVE_POSIX_MEMALIGN)
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


image::supplementary_data::supplementary_data()
{
  crop_left  =0;
  crop_right =0;
  crop_top   =0;
  crop_bottom=0;
}


void image::supplementary_data::set_from_SPS(std::shared_ptr<const seq_parameter_set> sps)
{
  crop_left   = sps->conf_win_left_offset   * sps->get_chroma_horizontal_subsampling();
  crop_right  = sps->conf_win_right_offset  * sps->get_chroma_horizontal_subsampling();
  crop_top    = sps->conf_win_top_offset    * sps->get_chroma_vertical_subsampling();
  crop_bottom = sps->conf_win_bottom_offset * sps->get_chroma_vertical_subsampling();
}


LIBDE265_API void* de265_alloc_image_plane(struct image* img, int cIdx,
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


LIBDE265_API void de265_free_image_plane(struct image* img, int cIdx)
{
  uint8_t* p = (uint8_t*)img->get_image_plane(cIdx);
  assert(p);
  FREE_ALIGNED(p);
}


static int  image_get_buffer(de265_image_intern* de265_img,
                             const de265_image_spec* spec, void* userdata)
{
  image* img = (image*)de265_img;

  const int rawChromaWidth  = spec->width  / img->SubWidthC;
  const int rawChromaHeight = spec->height / img->SubHeightC;

  int luma_stride   = (spec->width    + spec->alignment-1) / spec->alignment * spec->alignment;
  int chroma_stride = (rawChromaWidth + spec->alignment-1) / spec->alignment * spec->alignment;

  assert(img->BitDepth_Y >= 8 && img->BitDepth_Y <= 16);
  assert(img->BitDepth_C >= 8 && img->BitDepth_C <= 16);

  int luma_bpl   = luma_stride   * ((img->BitDepth_Y+7)/8);
  int chroma_bpl = chroma_stride * ((img->BitDepth_C+7)/8);

  int luma_height   = spec->height;
  int chroma_height = rawChromaHeight;

  bool alloc_failed = false;

  uint8_t* p[3] = { 0,0,0 };
  p[0] = (uint8_t *)ALLOC_ALIGNED_16(luma_height   * luma_bpl   + MEMORY_PADDING);
  if (p[0]==NULL) { alloc_failed=true; }

  if (img->get_chroma_format() != de265_chroma_mono) {
    p[1] = (uint8_t *)ALLOC_ALIGNED_16(chroma_height * chroma_bpl + MEMORY_PADDING);
    p[2] = (uint8_t *)ALLOC_ALIGNED_16(chroma_height * chroma_bpl + MEMORY_PADDING);

    if (p[1]==NULL || p[2]==NULL) { alloc_failed=true; }
  }
  else {
    p[1] = NULL;
    p[2] = NULL;
    chroma_stride = 0;
  }

  if (alloc_failed) {
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

static void image_release_buffer(de265_image_intern* de265_img, void* userdata)
{
  image* img = (image*)de265_img;

  for (int i=0;i<3;i++) {
    uint8_t* p = (uint8_t*)img->get_image_plane(i);
    if (p) {
      FREE_ALIGNED(p);
    }
  }
}


de265_image_allocation image::default_image_allocation = {
  image_get_buffer,
  image_release_buffer
};


int image::image_allocation_get_buffer_NOP(struct de265_image_intern*,
                                           const struct de265_image_spec*,
                                           void* userdata)
{
  // NOP

  return 1;
}


void image::set_image_plane(int cIdx, uint8_t* mem, int stride, void *userdata)
{
  pixels[cIdx] = mem;
  plane_user_data[cIdx] = userdata;

  if (cIdx==0) { this->stride        = stride; }
  else         { this->chroma_stride = stride; }
}


uint32_t image::s_next_image_ID = 1; // start with ID 1, as 0 means 'no ID'

image::image()
{
  ID = 0;
  removed_at_picture_id = 0; // picture not used, so we can assume it has been removed

  decctx = NULL;
  encctx = NULL;

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
  PicLatencyCount = 0;
  PicState = UnusedForReference;
  PicOutputFlag = false;

  /*
  nThreadsQueued   = 0;
  nThreadsRunning  = 0;
  nThreadsBlocked  = 0;
  nThreadsFinished = 0;
  nThreadsTotal    = 0;
  */
}


de265_error image::alloc_image(int w,int h, enum de265_chroma c,
                               int bitDepth_luma, int bitDepth_chroma,
                               de265_PTS pts,
                               const supplementary_data& supp_data,
                               void* user_data,
                               const de265_image_allocation* alloc_functions)
{
  m_supplementary_data = supp_data;

  release(); /* TODO: review code for efficient allocation when arrays are already
                allocated to the requested size. Without the release, the old image-data
                will not be freed. */

  ID = s_next_image_ID++;
  removed_at_picture_id = std::numeric_limits<int32_t>::max();

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
    chroma_width  = (chroma_width +1)/2;
    chroma_height = (chroma_height+1)/2;
    SubWidthC  = 2;
    SubHeightC = 2;
    break;

  case de265_chroma_422:
    chroma_width = (chroma_width+1)/2;
    SubWidthC  = 2;
    SubHeightC = 1;
    break;

  case de265_chroma_444:
    SubWidthC  = 1;
    SubHeightC = 1;
    break;

  case de265_chroma_mono:
    chroma_width = 0;
    chroma_height= 0;
    SubWidthC  = 1;
    SubHeightC = 1;
    break;

  default:
    assert(false);
    break;
  }

  spec.width  = w;
  spec.height = h;
  spec.chroma = chroma_format;
  spec.alignment = STANDARD_ALIGNMENT;

  spec.luma_bits_per_pixel = bitDepth_luma;
  spec.chroma_bits_per_pixel = bitDepth_chroma;


  // conformance window cropping

  int left   = m_supplementary_data.crop_left;
  int right  = m_supplementary_data.crop_right;
  int top    = m_supplementary_data.crop_top;
  int bottom = m_supplementary_data.crop_bottom;

  width_confwin = width - (left+right);
  height_confwin= height- (top+bottom);
  chroma_width_confwin = chroma_width -(left+right)/WinUnitX;
  chroma_height_confwin= chroma_height-(top+bottom)/WinUnitY;

  spec.crop_left  = left;
  spec.crop_right = right;
  spec.crop_top   = top;
  spec.crop_bottom= bottom;

  spec.visible_width = width_confwin;
  spec.visible_height= height_confwin;


  BitDepth_Y = bitDepth_luma;
  BitDepth_C = bitDepth_chroma;


  // allocate memory and set conformance window pointers

  if (alloc_functions != nullptr) {
    image_allocation_functions = *alloc_functions;
  }
  else {
    image_allocation_functions = image::default_image_allocation;
  }

  bool mem_alloc_success = true;

  if (image_allocation_functions.get_buffer != NULL) {
    mem_alloc_success = image_allocation_functions.get_buffer((de265_image_intern*)this, &spec,
                                                              image_allocation_functions.allocation_userdata);

    pixels_confwin[0] = pixels[0] + left*WinUnitX + top*WinUnitY*stride;
    pixels_confwin[1] = pixels[1] + left + top*chroma_stride;
    pixels_confwin[2] = pixels[2] + left + top*chroma_stride;


    // check for memory shortage

    if (!mem_alloc_success)
      {
        return DE265_ERROR_OUT_OF_MEMORY;
      }
  }

  return DE265_OK;
}


de265_error image::alloc_metadata(std::shared_ptr<const seq_parameter_set> sps)
{
  assert(sps);
  this->sps = sps;


  // --- allocate decoding info arrays ---

  // intra pred mode

  bool mem_alloc_success = true;

  mem_alloc_success &= intraPredMode.alloc(sps->PicWidthInMinPUs, sps->PicHeightInMinPUs,
                                           sps->Log2MinPUSize);

  mem_alloc_success &= intraPredModeC.alloc(sps->PicWidthInMinPUs, sps->PicHeightInMinPUs,
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

#if D_MT
      for (int i=0;i<sps->PicSizeInCtbsY;i++) {
        int x = i % sps->PicWidthInCtbsY;
        int y = i / sps->PicWidthInCtbsY;

        char buf[100];
        sprintf(buf,"CTB[%d;%d]",x,y);
        ctb_progress[i].set_name(buf);
      }
#endif
    }


  // check for memory shortage

  if (!mem_alloc_success)
    {
      return DE265_ERROR_OUT_OF_MEMORY;
    }


  return DE265_OK;
}


image::~image()
{
  release();

  // free progress locks

  if (ctb_progress) {
    delete[] ctb_progress;
  }
}


void image::release()
{
  // free image memory

  if (pixels[0])
    {
      image_allocation_functions.release_buffer((de265_image_intern*)this,
                                                image_allocation_functions.allocation_userdata);

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


void image::fill_image(int y,int cb,int cr)
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


de265_error image::copy_image(const image* src)
{
  /* TODO: actually, since we allocate the image only for internal purpose, we
     do not have to call the external allocation routines for this. However, then
     we have to track for each image how to release it again.
     Another option would be to safe the copied data not in an image at all.
  */

  de265_error err = alloc_image(src->width, src->height, src->chroma_format,
                                src->BitDepth_Y,
                                src->BitDepth_C,
                                src->pts,
                                src->get_supplementary_data(),
                                src->user_data, nullptr);
  if (err != DE265_OK) {
    return err;
  }

  set_decoder_context(src->decctx);
  set_encoder_context(src->encctx);

  copy_lines_from(src, 0, src->height);

  return err;
}


// end = last line + 1
void image::copy_lines_from(const image* src, int first, int end)
{
  if (end > src->height) end=src->height;

  assert(first % 2 == 0);
  assert(end   % 2 == 0);

  int luma_bpp   = (BitDepth_Y+7)/8;
  int chroma_bpp = (BitDepth_C+7)/8;

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

  int first_chroma = first / src->SubHeightC;
  int end_chroma   = end   / src->SubHeightC;

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


void image::exchange_pixel_data_with(image& b)
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


/*
void image::thread_start(int nThreads)
{
  mutex.lock();

  //printf("nThreads before: %d %d\n",nThreadsQueued, nThreadsTotal);

  nThreadsQueued += nThreads;
  nThreadsTotal += nThreads;

  //printf("nThreads after: %d %d\n",nThreadsQueued, nThreadsTotal);

  mutex.unlock();
}

void image::thread_run(const thread_task* task)
{
  //printf("run thread %s\n", task->name().c_str());

  mutex.lock();
  nThreadsQueued--;
  nThreadsRunning++;
  mutex.unlock();
}


void image::thread_blocks()
{
  mutex.lock();
  nThreadsRunning--;
  nThreadsBlocked++;
  mutex.unlock();
}

void image::thread_unblocks()
{
  mutex.lock();
  nThreadsBlocked--;
  nThreadsRunning++;
  mutex.unlock();
}


void image::thread_finishes(const thread_task* task)
{
  //printf("finish thread %s\n", task->name().c_str());

  mutex.lock();

  nThreadsRunning--;
  nThreadsFinished++;
  assert(nThreadsRunning >= 0);

  if (nThreadsFinished==nThreadsTotal) {
    finished_cond.broadcast(mutex);
  }

  mutex.unlock();
}
*/


void image::debug_show_ctb_progress() const
{
#if 0
  for (int i=0;i<ctb_info.data_size;i++) {
    int ctbx = i%sps->PicWidthInCtbsY;
    int ctby = i/sps->PicWidthInCtbsY;

    printf("%d %d;%d: %d\n",i,ctbx,ctby,
           ctb_progress[i].get_progress());
  }
#endif
}


void image::wait_for_progress(int ctbx,int ctby, int progress) const
{
  const int ctbW = sps->PicWidthInCtbsY;

  wait_for_progress(ctbx + ctbW*ctby, progress);
}

void image::wait_for_progress_ctb_row(int ctby, int progress) const
{
  // Wait from right CTB to left CTB. This should reduce the number of cond.variable locks.

  for (int x=sps->PicWidthInCtbsY-1; x>=0; x--) {
    wait_for_progress(x,ctby,progress);
  }
}

void image::wait_for_progress_at_pixel(int x,int y, int progress) const
{
  int ctbx = x/sps->CtbSizeY;
  int ctby = y/sps->CtbSizeY;

  if (ctbx >= sps->PicWidthInCtbsY)  ctbx = sps->PicWidthInCtbsY-1;
  if (ctby >= sps->PicHeightInCtbsY) ctby = sps->PicHeightInCtbsY-1;
  if (ctbx < 0) ctbx=0;
  if (ctby < 0) ctby=0;

  wait_for_progress(ctbx,ctby,progress);
}

void image::wait_for_progress(int ctbAddrRS, int progress) const
{
  //if (task==NULL) { return; }

  de265_progress_lock* progresslock = &ctb_progress[ctbAddrRS];
  //printf("wait for progress %d %d\n",progresslock->get_progress() , progress);
  if (progresslock->get_progress() < progress) {
    //thread_blocks();

    //assert(task!=NULL);

    /* TODO: check whether we are the first blocked task in the list.
       If we are, we have to conceal input errors.
       Simplest concealment: do not block.
    */

    progresslock->wait_for_progress(progress);
    //thread_unblocks();
  }
}


void image::wait_until_all_CTBs_have_progress(int progress) const
{
  for (int i=0;i<ctb_info.data_size;i++) {
    //printf("wait for CTB %i   %p\n",i,task);
    wait_for_progress(i,progress);
  }
}


bool image::do_all_CTBs_have_progress(int progress) const
{
  for (int i=0;i<ctb_info.data_size;i++) {
    if (ctb_progress[i].get_progress() < progress) {
      return false;
    }
  }

  return true;
}


/*
void image::wait_for_completion()
{
  mutex.lock();
  while (nThreadsFinished!=nThreadsTotal) {
    finished_cond.wait(mutex);
  }
  mutex.unlock();
}

bool image::debug_is_completed() const
{
  bool completed;

  mutex.lock();
  completed = (nThreadsFinished==nThreadsTotal);
  mutex.unlock();

  return completed;
}
*/



void image::clear_metadata()
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


void image::set_mv_info(int x,int y, int nPbW,int nPbH, const PBMotion& mv)
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
        pb_info[ xPu+pbx + (yPu+pby)*stride ] = mv;
      }
}


void image::write_image(const char* name) const
{
  FILE* fh = fopen(name,"wb");

  for (int c=0;c<3;c++) {
    int w = get_width(c);
    int h = get_height(c);

    for (int y=0;y<h;y++) {
      const uint8_t* p = get_image_plane_at_pos<uint8_t>(c, 0,y);
      fwrite(p,1,w,fh);
    }
  }

  fclose(fh);
}
