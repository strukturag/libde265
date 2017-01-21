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
#include <memory>
#include <vector>

class decoder_context;


/* Buffer of all decoded pictures that can still be used as a reference picture.
 */
class decoded_picture_buffer {
public:
  decoded_picture_buffer();
  ~decoded_picture_buffer();

  void set_max_size_of_DPB(int n)  { max_images_in_DPB=n; }
  void set_norm_size_of_DPB(int n) { norm_images_in_DPB=n; }

  /* Alloc a new image in the DPB and return its index.
     If there is no space for a new image, return -1. */
  int new_image(std::shared_ptr<const seq_parameter_set> sps, decoder_context* decctx,
                de265_PTS pts, void* user_data,
                const de265_image_allocation* alloc_functions = nullptr);

  /* Check for a free slot in the DPB. There are some slots reserved for
     unavailable reference frames. If high_priority==true, these reserved slots
     are included in the check. */
  bool has_free_dpb_picture(bool high_priority) const;

  /* Remove all pictures from DPB and queues. Decoding should be stopped while calling this. */
  void clear();

  int size() const { return dpb.size(); }

  /* Raw access to the images. */

  std::shared_ptr</* */ image> get_image(int index)       {
    lock_guard lock(m_mutex);
    if (index>=dpb.size()) return std::shared_ptr<image>();
    return dpb[index];
  }

  std::shared_ptr<const image> get_image(int index) const {
    lock_guard lock(m_mutex);
    if (index>=dpb.size()) return std::shared_ptr<image>();
    return dpb[index];
  }

  /* Search DPB for the slot index of a specific picture. */
  int DPB_index_of_picture_with_POC(int poc, int currentID, bool preferLongTerm=false) const;
  int DPB_index_of_picture_with_LSB(int lsb, int currentID, bool preferLongTerm=false) const;
  int DPB_index_of_picture_with_ID (int id) const;

  void lock() const { m_mutex.lock(); }
  void unlock() const { m_mutex.unlock(); }


  // --- debug ---

  void log_dpb_content() const;

private:
  mutable de265_mutex m_mutex;

  int max_images_in_DPB;
  int norm_images_in_DPB;

  std::vector<image_ptr> dpb; // decoded picture buffer

private:
  decoded_picture_buffer(const decoded_picture_buffer&); // no copy
  decoded_picture_buffer& operator=(const decoded_picture_buffer&); // no copy
};


/* The picture_output_queue holds all decoded pictures (and also the skipped pictures).
   The queue consists of two parts:
   - First, the pictures enter a reordering queue and after the reordering delay,
     they are forwarded to
   - the output FIFO.
 */
class picture_output_queue
{
public:
  picture_output_queue();

  void set_num_reorder_pics(int n) { m_num_reorder_pics = n; }
  void set_max_latency(int n) { m_max_latency = n; } // set to zero to disable max_latency (default)

  void clear();


  // --- reorder buffer ---

  void insert_image_into_reorder_buffer(image_ptr img);

  int num_pictures_in_reorder_buffer() const;

  // Move all pictures in reorder buffer to output buffer. Return true if there were any pictures.
  bool flush_reorder_buffer();


  // --- output buffer ---

  int num_pictures_in_output_queue() const;

  /* Get the next picture in the output queue, but do not remove it from the queue. */
  image_ptr get_next_picture_in_output_queue() const;

  /* Remove the next picture in the output queue. */
  void pop_next_picture_in_output_queue();


  // --- debug ---

  void log_dpb_queues() const;
  void dump_queues() const;

 private:
  int m_num_reorder_pics;
  int m_max_latency;

  std::vector<image_ptr> reorder_output_queue;
  std::deque<image_ptr>  image_output_queue;

  mutable de265_mutex m_mutex;

  // move next picture in reorder buffer to output queue
  void move_next_picture_in_reorder_buffer_to_output_queue();
};

#endif
