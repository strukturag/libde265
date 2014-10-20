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

#include "libde265/encpicbuf.h"


encoder_picture_buffer::encoder_picture_buffer()
{
}

encoder_picture_buffer::~encoder_picture_buffer()
{
  flush_images();
}


encoder_picture_buffer::image_data::image_data()
{
  frame_number = 0;

  input = NULL;
  reconstruction = NULL;

  // SOP metadata

  sps_index = -1;
  temporal_layer = 0;
  skip_priority = 0;
  is_intra = true;

  state = state_unprocessed;
}

encoder_picture_buffer::image_data::~image_data()
{
  if (input) { delete input; }
  if (reconstruction) { delete reconstruction; }
}


// --- input pushed by the input process ---

void encoder_picture_buffer::reset()
{
  flush_images();

  mEndOfStream = false;
}


void encoder_picture_buffer::flush_images()
{
  while (!mImages.empty()) {
    delete mImages.front();
    mImages.pop_front();
  }
}


void encoder_picture_buffer::insert_next_image_in_encoding_order(const de265_image* img,
                                                                 int frame_number)
{
  image_data* data = new image_data();
  data->frame_number = frame_number;
  data->input = img;

  mImages.push_back(data);
}

void encoder_picture_buffer::insert_end_of_stream()
{
  mEndOfStream = true;
}


// --- SOP structure ---

void encoder_picture_buffer::set_image_intra()
{
  image_data* data = mImages.back();
  data->is_intra = true;
}

void encoder_picture_buffer::set_image_references(int sps_index, // -1 -> custom
                                                  const std::vector<int>& l0,
                                                  const std::vector<int>& l1,
                                                  const std::vector<int>& lt,
                                                  const std::vector<int>& keepMoreReferences)
{
  image_data* data = mImages.back();

  data->sps_index = sps_index;
  data->ref0 = l0;
  data->ref1 = l1;
  data->longterm = lt;
  data->keep = keepMoreReferences;
}

void encoder_picture_buffer::set_temporal_layer(int temporal_layer)
{
  image_data* data = mImages.back();

  data->temporal_layer = temporal_layer;
}

void encoder_picture_buffer::set_skip_priority(int skip_priority)
{
  image_data* data = mImages.back();

  data->skip_priority = skip_priority;
}

void encoder_picture_buffer::sop_metadata_commit(int frame_number)
{
  image_data* data = mImages.back();
  assert(data->frame_number == frame_number);

  data->state = image_data::state_sop_metadata_available;
}



// --- infos pushed by encoder ---

void encoder_picture_buffer::mark_encoding_started(int frame_number)
{
  image_data* data = get_picture(frame_number);

  data->state = image_data::state_encoding;
}

int  encoder_picture_buffer::set_reconstruction_image(int frame_number, const de265_image* reco)
{
  image_data* data = get_picture(frame_number);

  data->reconstruction = reco;
}

void encoder_picture_buffer::mark_encoding_finished(int frame_number)
{
  image_data* data = get_picture(frame_number);

  data->state = image_data::state_keep_for_reference;


  // --- delete images that are not required anymore ---

  // first, mark all images unused

  for (auto imgdata : mImages) {
    imgdata->mark_used = false;
  }

  // mark all images that will be used later

  for (int f : data->ref0)     { get_picture(f)->mark_used=true; }
  for (int f : data->ref1)     { get_picture(f)->mark_used=true; }
  for (int f : data->longterm) { get_picture(f)->mark_used=true; }
  for (int f : data->keep)     { get_picture(f)->mark_used=true; }
  data->mark_used=true;

  // copy over all images that we still keep

  std::deque<image_data*> newImageSet;
  for (auto imgdata : mImages) {
    if (imgdata->mark_used) {
      newImageSet.push_back(imgdata);
    }
    else {
      // image is not needed anymore for reference, remove it from EncPicBuf

      delete imgdata;
    }
  }

  mImages = newImageSet;
}



// --- data access ---

bool encoder_picture_buffer::have_more_frames_to_encode() const
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->state < image_data::state_encoding) {
      return true;
    }
  }

  return false;
}


const encoder_picture_buffer::image_data*
encoder_picture_buffer::get_next_picture_to_encode()
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->state < image_data::state_encoding) {
      return mImages[i];
    }
  }

  return NULL;
}


const encoder_picture_buffer::image_data*
encoder_picture_buffer::get_picture(int frame_number) const
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->frame_number == frame_number)
      return mImages[i];
  }

  assert(false);
  return NULL;
}


encoder_picture_buffer::image_data* encoder_picture_buffer::get_picture(int frame_number)
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->frame_number == frame_number)
      return mImages[i];
  }

  assert(false);
  return NULL;
}


void encoder_picture_buffer::release_input_image(int frame_number)
{
  image_data* idata = get_picture(frame_number);
  assert(idata);

  delete idata->input;
  idata->input = NULL;
}

