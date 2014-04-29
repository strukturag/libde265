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


void log_dpb_content(const decoded_picture_buffer *dpb)
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    loginfo(LogHighlevel, " DPB %d: POC=%d %s %s\n", i, dpb->dpb[i].PicOrderCntVal,
            dpb->dpb[i].PicState == UnusedForReference ? "unused" :
            dpb->dpb[i].PicState == UsedForShortTermReference ? "short-term" : "long-term",
            dpb->dpb[i].PicOutputFlag ? "output" : "---");
  }
}


bool has_free_dpb_picture(const decoded_picture_buffer* dpb, bool high_priority)
{
  int nImages = high_priority ? DE265_DPB_SIZE : DE265_DPB_OUTPUT_IMAGES;

  for (int i=0;i<nImages;i++) {
    if (dpb->dpb[i].PicOutputFlag==false && dpb->dpb[i].PicState == UnusedForReference) {
      return true;
    }
  }

  return false;
}


int DPB_index_of_picture_with_POC(decoded_picture_buffer* dpb, int poc)
{
  logdebug(LogHeaders,"DPB_index_of_picture_with_POC POC=\n",poc);

  //log_dpb_content(ctx);
  //loginfo(LogDPB,"searching for short-term reference POC=%d\n",poc);

  for (int k=0;k<DE265_DPB_SIZE;k++) {
    if (dpb->dpb[k].PicOrderCntVal == poc &&
        dpb->dpb[k].PicState != UnusedForReference) {
      return k;
    }
  }

  return -1;
}


int DPB_index_of_picture_with_LSB(decoded_picture_buffer* dpb, int lsb)
{
  logdebug(LogHeaders,"get access to picture with PSB %d from DPB\n",lsb);

  for (int k=0;k<DE265_DPB_SIZE;k++) {
    if (dpb->dpb[k].picture_order_cnt_lsb == lsb &&
        dpb->dpb[k].PicState != UnusedForReference) {
      return k;
    }
  }

  return -1;
}


void flush_next_picture_from_reorder_buffer(decoded_picture_buffer *dpb)
{
  assert(dpb->reorder_output_queue_length>0);

  // search for picture in reorder buffer with minimum POC

  int minPOC = dpb->reorder_output_queue[0]->PicOrderCntVal;
  int minIdx = 0;
  for (int i=1;i<dpb->reorder_output_queue_length;i++)
    {
      if (dpb->reorder_output_queue[i]->PicOrderCntVal < minPOC) {
        minPOC = dpb->reorder_output_queue[i]->PicOrderCntVal;
        minIdx = i;
      }
    }


  // put image into output queue

  assert(dpb->image_output_queue_length < DE265_DPB_SIZE);
  dpb->image_output_queue[ dpb->image_output_queue_length ] = dpb->reorder_output_queue[minIdx];
  dpb->image_output_queue_length++;


  // remove image from reorder buffer

  for (int i=minIdx+1; i<dpb->reorder_output_queue_length; i++) {
    dpb->reorder_output_queue[i-1] = dpb->reorder_output_queue[i];
  }
  dpb->reorder_output_queue_length--;
}


bool flush_reorder_buffer(decoded_picture_buffer* dpb)
{
  // return 'false' when there are no pictures in reorder buffer
  if (dpb->reorder_output_queue_length==0) return false;

  while (dpb->reorder_output_queue_length>0) {
    flush_next_picture_from_reorder_buffer(dpb);
  }

  return true;
}


void dpb_clear_images(decoded_picture_buffer* dpb, decoder_context* ctx)
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    if (dpb->dpb[i].PicOutputFlag ||
        dpb->dpb[i].PicState != UnusedForReference)
      {
        dpb->dpb[i].PicOutputFlag = false;
        dpb->dpb[i].PicState = UnusedForReference;
        cleanup_image(ctx, &dpb->dpb[i]);
      }
  }

  dpb->reorder_output_queue_length=0;
  dpb->image_output_queue_length=0;
}


/* Alloc a new image in the DPB and return its index.
   If there is no space for a new image, return -1.
 */
int initialize_new_DPB_image(decoded_picture_buffer* dpb,const seq_parameter_set* sps)
{
  loginfo(LogHeaders,"initialize_new_DPB_image\n");

  //printf("initialize_new_DPB_image()\n");
  log_dpb_content(dpb);

  int free_image_buffer_idx = -1;
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    if (dpb->dpb[i].PicOutputFlag==false && dpb->dpb[i].PicState == UnusedForReference) {
      free_image_buffer_idx = i;
      break;
    }
  }

  //printf("free buffer index = %d\n", free_image_buffer_idx);

  if (free_image_buffer_idx == -1) {
    return -1;
  }

  de265_image* img = &dpb->dpb[free_image_buffer_idx];

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


void dpb_pop_next_picture_in_output_queue(decoded_picture_buffer* dpb)
{
  for (int i=1;i<dpb->image_output_queue_length;i++)
    {
      dpb->image_output_queue[i-1] = dpb->image_output_queue[i];
    }

  dpb->image_output_queue_length--;

  dpb->image_output_queue[ dpb->image_output_queue_length ] = NULL;


  loginfo(LogDPB, "DPB output queue: ");
  for (int i=0;i<dpb->image_output_queue_length;i++) {
    loginfo(LogDPB, "*%d ", dpb->image_output_queue[i]->PicOrderCntVal);
  }
  loginfo(LogDPB,"*\n");
}


void log_dpb_queues(const decoded_picture_buffer* dpb)
{
    loginfo(LogDPB, "DPB reorder queue (after push): ");
    for (int i=0;i<dpb_num_pictures_in_reorder_buffer(dpb);i++) {
      loginfo(LogDPB, "*%d ", dpb->reorder_output_queue[i]->PicOrderCntVal);
    }
    loginfo(LogDPB,"*\n");

    loginfo(LogDPB, "DPB output queue (after push): ");
    for (int i=0;i<dpb_num_pictures_in_output_queue(dpb);i++) {
      loginfo(LogDPB, "*%d ", dpb->image_output_queue[i]->PicOrderCntVal);
    }
    loginfo(LogDPB,"*\n");
}
