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

#ifndef DE265_IMAGE_UNIT_H
#define DE265_IMAGE_UNIT_H

#include "libde265/vps.h"
#include "libde265/sps.h"
#include "libde265/pps.h"
#include "libde265/nal.h"
#include "libde265/slice.h"
#include "libde265/image.h"
#include "libde265/motion.h"
#include "libde265/de265.h"
#include "libde265/dpb.h"
#include "libde265/sei.h"
#include "libde265/threads.h"
#include "libde265/acceleration.h"
#include "libde265/nal-parser.h"

#include <memory>

#define DE265_MAX_VPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_SPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_PPS_SETS 64   // this is the maximum as defined in the standard

#define MAX_WARNINGS 20


class slice_segment_header;
class decoder_context;
class image_unit;
class thread_context;


class slice_unit
{
public:
  slice_unit(decoder_context* decctx);
  ~slice_unit();

  NAL_unit_ptr nal;   // we are the owner
  slice_segment_header* shdr;  // not the owner (de265_image is owner)
  bitreader reader;

  image_unit* imgunit;

  bool flush_reorder_buffer;


  // decoding status

  enum SliceDecodingProgress { Unprocessed,
                               InProgress,
                               Decoded
  } state;

  de265_progress_lock finished_threads;
  int nThreads;

  // CTBs that are covered by this slice-unit.
  // Note: this does not necessarily equal exactly which CTBs are decoded. If some slices
  // are missing in the stream, the missing CTBs are added to some slice-units. Hence, the
  // whole picture is always covered. Even if some slices are missing.
  int first_CTB_TS;
  int last_CTB_TS;

  void allocate_thread_contexts(int n);
  thread_context* get_thread_context(int n);
  int num_thread_contexts() const { return nThreadContexts; }


  void mark_whole_slice_as_processed(int progress);

private:
  thread_context* thread_contexts; /* NOTE: cannot use std::vector, because thread_context has
                                      no copy constructor. */
  int nThreadContexts;

public:
  decoder_context* ctx;

private:
  slice_unit(const slice_unit&); // not allowed
  const slice_unit& operator=(const slice_unit&); // not allowed
};



class image_unit
{
public:
  image_unit();
  ~image_unit();

  uint8_t nal_unit_type; // of NAL containing first slice header

  image_ptr img;
  image  sao_output; // if SAO is used, this is allocated and used as SAO output buffer

  std::vector<slice_unit*> slice_units;
  std::vector<sei_message> suffix_SEIs;

  slice_unit* get_next_unprocessed_slice_segment() const {
    for (int i=0;i<slice_units.size();i++) {
      if (slice_units[i]->state == slice_unit::Unprocessed) {
        return slice_units[i];
      }
    }

    return NULL;
  }

  slice_unit* get_prev_slice_segment(slice_unit* s) const {
    for (int i=1; i<slice_units.size(); i++) {
      if (slice_units[i]==s) {
        return slice_units[i-1];
      }
    }

    return NULL;
  }

  slice_unit* get_next_slice_segment(slice_unit* s) const {
    for (int i=0; i<slice_units.size()-1; i++) {
      if (slice_units[i]==s) {
        return slice_units[i+1];
      }
    }

    return NULL;
  }

  void dump_slices() const {
    for (int i=0; i<slice_units.size(); i++) {
      printf("[%d] = %p\n",i,slice_units[i]);
    }
  }

  bool all_slice_segments_processed() const {
    if (slice_units.size()==0) return true;
    if (slice_units.back()->state != slice_unit::Unprocessed) return true;
    return false;
  }

  bool is_first_slice_segment(const slice_unit* s) const {
    if (slice_units.size()==0) return false;
    return (slice_units[0] == s);
  }

  enum { Invalid, // headers not read yet
         Unknown, // SPS/PPS available
         Reference, // will be used as reference
         Leaf       // not a reference picture
  } role;

  enum { Unprocessed,
         InProgress,
         Decoded,
         Dropped         // will not be decoded
  } state;


  // The main purpose of having pointers to all tasks is to keep them alive until the
  // image is decoded, and to check whether they are finished.
  std::vector<thread_task_ptr> tasks;
  de265_thread_ptr master_task;


  /* Saved context models for WPP.
     There is one saved model for the initialization of each CTB row.
     The array is unused for non-WPP streams. */
  std::vector<context_model_table> ctx_models;  // TODO: move this into image ?


  bool did_finish_decoding() const;
  void wait_to_finish_decoding();
};


typedef std::shared_ptr<image_unit> image_unit_ptr;


class image_unit_sink
{
 public:
  virtual ~image_unit_sink() { }

  virtual void send_image_unit(image_unit_ptr) = 0; // transfers ownership of image_unit
  virtual void send_end_of_stream() = 0;
};



#endif
