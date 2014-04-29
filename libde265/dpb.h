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

  void clear_images(struct decoder_context* ctx);

  void pop_next_picture_in_output_queue();

  // output next picture from reorder buffer, TODO: rename function
  void flush_next_picture_from_reorder_buffer();
  
  // Move all pictures in reorder buffer to output buffer. Return true if there were any pictures.
  bool flush_reorder_buffer();
  
  int initialize_new_DPB_image(const seq_parameter_set* sps);
  
  bool has_free_dpb_picture(bool high_priority) const;
  //void push_current_picture_to_output_queue(decoded_picture_buffer* dpb);
  
  int DPB_index_of_picture_with_POC(int poc) const;
  int DPB_index_of_picture_with_LSB(int lsb) const;
  
  void log_dpb_content() const;
  void log_dpb_queues() const;
  
  de265_image* get_next_picture_in_output_queue() const { return image_output_queue[0]; }
  int num_pictures_in_output_queue() const { return image_output_queue_length; }
  int num_pictures_in_reorder_buffer() const { return reorder_output_queue_length; }

  /* */ de265_image* get_image(int index)       { return &dpb[index]; }
  const de265_image* get_image(int index) const { return &dpb[index]; }

  void insert_image_into_reorder_buffer(de265_image* img) { reorder_output_queue[ reorder_output_queue_length++ ] = img; }


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

#endif
