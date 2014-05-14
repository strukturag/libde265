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
#include "libde265/sps.h"

#include <deque>
#include <vector>

// TODO: check required value
#define DE265_DPB_OUTPUT_IMAGES  20
#define DE265_DPB_RESILIENCE_IMAGES 5
#define DE265_DPB_SIZE  (DE265_DPB_OUTPUT_IMAGES + DE265_DPB_RESILIENCE_IMAGES)


struct decoded_picture_buffer {
  decoded_picture_buffer();
  ~decoded_picture_buffer();

  /* Alloc a new image in the DPB and return its index.
     If there is no space for a new image, return -1. */
  int new_image(const seq_parameter_set* sps, de265_image_allocation* alloc);
  
  /* Check for a free slot in the DPB. There are some slots reserved for
     unavailable reference frames. If high_priority==true, these reserved slots
     are included in the check. */
  bool has_free_dpb_picture(bool high_priority) const;

  /* Remove all pictures from DPB and queues. Decoding should be stopped when calling this. */
  void clear();


  /* Raw access to the images. */
  /* */ de265_image* get_image(int index)       { return &dpb[index]; }
  const de265_image* get_image(int index) const { return &dpb[index]; }

  /* Search DPB for the slot index of a specific picture. */
  int DPB_index_of_picture_with_POC(int poc, int currentID, bool preferLongTerm=false) const;
  int DPB_index_of_picture_with_LSB(int lsb, int currentID, bool preferLongTerm=false) const;
  int DPB_index_of_picture_with_ID (int id) const;
  

  // --- reorder buffer ---

  void insert_image_into_reorder_buffer(de265_image* img) {
    reorder_output_queue.push_back(img);
  }

  int num_pictures_in_reorder_buffer() const { return reorder_output_queue.size(); }

  // move next picture in reorder buffer to output queue
  void output_next_picture_in_reorder_buffer();
  
  // Move all pictures in reorder buffer to output buffer. Return true if there were any pictures.
  bool flush_reorder_buffer();
  

  // --- output buffer ---

  int num_pictures_in_output_queue() const { return image_output_queue.size(); }

  /* Get the next picture in the output queue, but do not remove it from the queue. */
  de265_image* get_next_picture_in_output_queue() const { return image_output_queue.front(); }

  /* Remove the next picture in the output queue. */
  void pop_next_picture_in_output_queue();


  // --- debug ---

  void log_dpb_content() const;
  void log_dpb_queues() const;
  
private:
  de265_image dpb[DE265_DPB_SIZE]; // decoded picture buffer

  std::vector<de265_image*> reorder_output_queue;
  std::deque<de265_image*>  image_output_queue;

private:
  decoded_picture_buffer(const decoded_picture_buffer&); // no copy
  decoded_picture_buffer& operator=(const decoded_picture_buffer&); // no copy
};

#endif
