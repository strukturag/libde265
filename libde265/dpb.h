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

#ifndef DE265_DPB_H
#define DE265_DPB_H

#include "libde265/image.h"
#include "libde265/de265.h"
#include "libde265/sps.h"

// TODO: check required value
#define DE265_DPB_OUTPUT_IMAGES  20
#define DE265_DPB_RESILIENCE_IMAGES 5
#define DE265_DPB_SIZE  (DE265_DPB_OUTPUT_IMAGES + DE265_DPB_RESILIENCE_IMAGES)


typedef struct decoded_picture_buffer {

  // --- decoded picture buffer ---

  de265_image dpb[DE265_DPB_SIZE]; // decoded picture buffer

  de265_image* reorder_output_queue[DE265_DPB_SIZE];
  int          reorder_output_queue_length;

  de265_image* image_output_queue[DE265_DPB_SIZE];
  int          image_output_queue_length;

  //de265_image* last_decoded_image;

} decoded_picture_buffer;


void init_dpb(decoded_picture_buffer*);
void free_dpb(decoded_picture_buffer*);

void dpb_clear_images(decoded_picture_buffer* dpb, struct decoder_context* ctx);

// output next picture from reorder buffer, TODO: rename function
void flush_next_picture_from_reorder_buffer(decoded_picture_buffer* dpb);

// Move all pictures in reorder buffer to output buffer. Return true if there were any pictures.
bool flush_reorder_buffer(decoded_picture_buffer* dpb);

int initialize_new_DPB_image(decoded_picture_buffer* dpb,const seq_parameter_set* sps);

bool has_free_dpb_picture(const decoded_picture_buffer* dpb, bool high_priority);
//void push_current_picture_to_output_queue(decoded_picture_buffer* dpb);

int DPB_index_of_picture_with_POC(decoded_picture_buffer* dpb, int poc);
int DPB_index_of_picture_with_LSB(decoded_picture_buffer* dpb, int lsb);

void log_dpb_content(const decoded_picture_buffer* dpb);
void log_dpb_queues(const decoded_picture_buffer* dpb);

LIBDE265_INLINE static de265_image* dpb_get_next_picture_in_output_queue(decoded_picture_buffer* dpb)
{
  assert(dpb->image_output_queue_length>0);
  return dpb->image_output_queue[0];
}
LIBDE265_INLINE static int dpb_num_pictures_in_output_queue(const decoded_picture_buffer* dpb)
{
  return dpb->image_output_queue_length;
}
LIBDE265_INLINE static int dpb_num_pictures_in_reorder_buffer(const decoded_picture_buffer* dpb)
{
  return dpb->reorder_output_queue_length;
}
void dpb_pop_next_picture_in_output_queue(decoded_picture_buffer* dpb);

LIBDE265_INLINE static de265_image* dpb_get_image(decoded_picture_buffer* dpb, int index)
{
  return &dpb->dpb[index];
}
LIBDE265_INLINE static const de265_image* dpb_get_image_const(const decoded_picture_buffer* dpb, int index)
{
  return &dpb->dpb[index];
}

LIBDE265_INLINE static void dpb_insert_image_into_reorder_buffer(decoded_picture_buffer* dpb, de265_image* img)
{
  dpb->reorder_output_queue[ dpb->reorder_output_queue_length++ ] = img;
}

#endif
