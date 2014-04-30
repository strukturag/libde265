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

#include "dpb.h"
#include "decctx.h"
#include <string.h>
#include <assert.h>


void init_dpb(decoded_picture_buffer* dpb)
{
  memset(dpb, 0, sizeof(decoded_picture_buffer));

  for (int i=0;i<DE265_DPB_SIZE;i++) {
    de265_init_image(&dpb->dpb[i]);
  }
}


void free_dpb(decoded_picture_buffer* dpb)
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    de265_free_image(&dpb->dpb[i]);
  }
}


void decoded_picture_buffer::log_dpb_content() const
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    loginfo(LogHighlevel, " DPB %d: POC=%d %s %s\n", i, dpb[i].PicOrderCntVal,
            dpb[i].PicState == UnusedForReference ? "unused" :
            dpb[i].PicState == UsedForShortTermReference ? "short-term" : "long-term",
            dpb[i].PicOutputFlag ? "output" : "---");
  }
}


bool decoded_picture_buffer::has_free_dpb_picture(bool high_priority) const
{
  int nImages = high_priority ? DE265_DPB_SIZE : DE265_DPB_OUTPUT_IMAGES;

  for (int i=0;i<nImages;i++) {
    if (dpb[i].PicOutputFlag==false && dpb[i].PicState == UnusedForReference) {
      return true;
    }
  }

  return false;
}


int decoded_picture_buffer::DPB_index_of_picture_with_POC(int poc) const
{
  logdebug(LogHeaders,"DPB_index_of_picture_with_POC POC=\n",poc);

  //log_dpb_content(ctx);
  //loginfo(LogDPB,"searching for short-term reference POC=%d\n",poc);

  for (int k=0;k<DE265_DPB_SIZE;k++) {
    if (dpb[k].PicOrderCntVal == poc &&
        dpb[k].PicState != UnusedForReference) {
      return k;
    }
  }

  return -1;
}


int decoded_picture_buffer::DPB_index_of_picture_with_LSB(int lsb) const
{
  logdebug(LogHeaders,"get access to picture with PSB %d from DPB\n",lsb);

  for (int k=0;k<DE265_DPB_SIZE;k++) {
    if (dpb[k].picture_order_cnt_lsb == lsb &&
        dpb[k].PicState != UnusedForReference) {
      return k;
    }
  }

  return -1;
}


void decoded_picture_buffer::flush_next_picture_from_reorder_buffer()
{
  assert(reorder_output_queue_length>0);

  // search for picture in reorder buffer with minimum POC

  int minPOC = reorder_output_queue[0]->PicOrderCntVal;
  int minIdx = 0;
  for (int i=1;i<reorder_output_queue_length;i++)
    {
      if (reorder_output_queue[i]->PicOrderCntVal < minPOC) {
        minPOC = reorder_output_queue[i]->PicOrderCntVal;
        minIdx = i;
      }
    }


  // put image into output queue

  assert(image_output_queue_length < DE265_DPB_SIZE);
  image_output_queue[ image_output_queue_length ] = reorder_output_queue[minIdx];
  image_output_queue_length++;


  // remove image from reorder buffer

  for (int i=minIdx+1; i<reorder_output_queue_length; i++) {
    reorder_output_queue[i-1] = reorder_output_queue[i];
  }
  reorder_output_queue_length--;
}


bool decoded_picture_buffer::flush_reorder_buffer()
{
  // return 'false' when there are no pictures in reorder buffer
  if (reorder_output_queue_length==0) return false;

  while (reorder_output_queue_length>0) {
    flush_next_picture_from_reorder_buffer();
  }

  return true;
}


void decoded_picture_buffer::clear_images(decoder_context* ctx)
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    if (dpb[i].PicOutputFlag ||
        dpb[i].PicState != UnusedForReference)
      {
        dpb[i].PicOutputFlag = false;
        dpb[i].PicState = UnusedForReference;
        cleanup_image(ctx, &dpb[i]);
      }
  }

  reorder_output_queue_length=0;
  image_output_queue_length=0;
}


/* Alloc a new image in the DPB and return its index.
   If there is no space for a new image, return -1.
 */
int decoded_picture_buffer::initialize_new_DPB_image(const seq_parameter_set* sps)
{
  loginfo(LogHeaders,"initialize_new_DPB_image\n");

  //printf("initialize_new_DPB_image()\n");
  log_dpb_content();

  int free_image_buffer_idx = -1;
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    if (dpb[i].PicOutputFlag==false && dpb[i].PicState == UnusedForReference) {
      free_image_buffer_idx = i;
      break;
    }
  }

  //printf("free buffer index = %d\n", free_image_buffer_idx);

  if (free_image_buffer_idx == -1) {
    return -1;
  }

  de265_image* img = &dpb[free_image_buffer_idx];

  int w = sps->pic_width_in_luma_samples;
  int h = sps->pic_height_in_luma_samples;

  enum de265_chroma chroma;
  switch (sps->chroma_format_idc) {
  case 0: chroma = de265_chroma_mono; break;
  case 1: chroma = de265_chroma_420;  break;
  case 2: chroma = de265_chroma_422;  break;
  case 3: chroma = de265_chroma_444;  break;
  default: chroma = de265_chroma_420; assert(0); break; // should never happen
  }

  de265_alloc_image(img, w,h, chroma, sps);

  img->integrity = INTEGRITY_CORRECT;

  return free_image_buffer_idx;
}


void decoded_picture_buffer::pop_next_picture_in_output_queue()
{
  for (int i=1;i<image_output_queue_length;i++)
    {
      image_output_queue[i-1] = image_output_queue[i];
    }

  image_output_queue_length--;

  image_output_queue[ image_output_queue_length ] = NULL;


  loginfo(LogDPB, "DPB output queue: ");
  for (int i=0;i<image_output_queue_length;i++) {
    loginfo(LogDPB, "*%d ", image_output_queue[i]->PicOrderCntVal);
  }
  loginfo(LogDPB,"*\n");
}


void decoded_picture_buffer::log_dpb_queues() const
{
    loginfo(LogDPB, "DPB reorder queue (after push): ");
    for (int i=0;i<num_pictures_in_reorder_buffer();i++) {
      loginfo(LogDPB, "*%d ", reorder_output_queue[i]->PicOrderCntVal);
    }
    loginfo(LogDPB,"*\n");

    loginfo(LogDPB, "DPB output queue (after push): ");
    for (int i=0;i<num_pictures_in_output_queue();i++) {
      loginfo(LogDPB, "*%d ", image_output_queue[i]->PicOrderCntVal);
    }
    loginfo(LogDPB,"*\n");
}
