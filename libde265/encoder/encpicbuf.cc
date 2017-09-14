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

#include "libde265/encoder/encpicbuf.h"
#include "libde265/util.h"


picture_encoding_data::picture_encoding_data(encoder_picture_buffer* encpicbuf)
{
  //printf("new %p\n",this);

  frame_number = 0;

  input.reset();
  //prediction.reset();
  reconstruction.reset();

  // SOP metadata

  sps_index = -1;
  skip_priority = 0;
  //is_intra = true;

  state = state_unprocessed;

  mEncPicBuf = encpicbuf;
}


picture_encoding_data::picture_encoding_data(image_ptr in_img,
                                             int in_frame_number,
                                             std::shared_ptr<video_parameter_set>& in_vps,
                                             std::shared_ptr<seq_parameter_set>& in_sps,
                                             std::shared_ptr<pic_parameter_set>& in_pps,
                                             int ctb_size_log2)
{
  frame_number = in_frame_number;

  input = in_img;


  // SOP metadata

  sps_index = -1;
  skip_priority = 0;

  state = state_unprocessed;

  mEncPicBuf = nullptr;


  vps = in_vps;
  sps = in_sps;
  pps = in_pps;

  ctbs.alloc(in_img->get_width(),
             in_img->get_height(),
             ctb_size_log2);


  // --- reconstruction image and coding metadata

  reconstruction = std::make_shared<image>();
  reconstruction->set_headers(in_vps, in_sps, in_pps);
  reconstruction->PicOrderCntVal = in_img->PicOrderCntVal;

  reconstruction->alloc_image(in_img->get_width(),
                              in_img->get_height(),
                              in_img->get_chroma_format(), 8,8,
                              0, // PTS
                              image::supplementary_data(),
                              NULL, // user data
                              nullptr); // alloc_funcs
  /*
  ectx->img->set_encoder_context(ectx);
  */

  reconstruction->alloc_metadata(in_sps);
  reconstruction->clear_metadata();
}


picture_encoding_data::~picture_encoding_data()
{
  //printf("delete %p\n",this);
}



void picture_encoding_data::mark_sop_metadata_set()
{
  state = picture_encoding_data::state_sop_metadata_available;
}



// --- infos pushed by encoder ---

void picture_encoding_data::mark_encoding_started()
{
  state = picture_encoding_data::state_encoding;
}

void picture_encoding_data::mark_encoding_finished()
{
  state = picture_encoding_data::state_keep_for_reference;

  mEncPicBuf->purge_unused_images_from_queue();
}

/*
void encoder_picture_buffer::set_prediction_image(int frame_number, image_ptr pred)
{
  picture_encoding_data* data = get_picture(frame_number);

  data->prediction = pred;
}
*/

void picture_encoding_data::set_reconstruction_image(image_ptr reco)
{
  reconstruction = reco;
}



// --- SOP structure ---

/*
void picture_encoding_data::set_intra()
{
  //shdr.slice_type = SLICE_TYPE_I;
  //is_intra = true;
}
*/


void picture_encoding_data::set_NAL_type(uint8_t nalType)
{
  nal.nal_unit_type = nalType;
}


void picture_encoding_data::set_references(int sps_index, // -1 -> custom
                                const std::vector<int>& l0,
                                const std::vector<int>& l1,
                                const std::vector<int>& lt,
                                const std::vector<int>& keepMoreReferences)
{
  this->sps_index = sps_index;
  ref0 = l0;
  ref1 = l1;
  longterm = lt;
  keep = keepMoreReferences;


  // TODO: pps.num_ref_idx_l0_default_active
}


void picture_encoding_data::set_NAL_temporal_id(int temporal_id)
{
  this->nal.nuh_temporal_id = temporal_id;
}


void picture_encoding_data::set_skip_priority(int skip_priority)
{
  this->skip_priority = skip_priority;
}




encoder_picture_buffer::encoder_picture_buffer()
{
  mEndOfInput = false;
}


encoder_picture_buffer::~encoder_picture_buffer()
{
}


void encoder_picture_buffer::clear()
{
  mImages.clear();
  mEndOfInput = false;
}


// --- input pushed by the input process ---


void encoder_picture_buffer::insert_next_image_in_encoding_order(std::shared_ptr<picture_encoding_data> picdata)
{
  picdata->setEncPicBuf(this);

  mImages.push_back(picdata);
}


void encoder_picture_buffer::insert_end_of_input()
{
  mEndOfInput = true;
}


/* TODO: This probably does not work anymore after my code refactorization */
void encoder_picture_buffer::purge_unused_images_from_queue()
{
  // --- delete images that are not required anymore ---

  // first, mark all images unused

  FOR_LOOP(std::shared_ptr<picture_encoding_data>, picdata, mImages) {
    picdata->in_use = false;
  }

  // mark all images that will be used later

  FOR_LOOP(std::shared_ptr<picture_encoding_data>, picdata, mImages) {
    FOR_LOOP(int, f, picdata->ref0)     { get_picture(f)->in_use=true; }
    FOR_LOOP(int, f, picdata->ref1)     { get_picture(f)->in_use=true; }
    FOR_LOOP(int, f, picdata->longterm) { get_picture(f)->in_use=true; }
    FOR_LOOP(int, f, picdata->keep)     { get_picture(f)->in_use=true; }
  }

  // copy over all images that we still keep

  std::deque< std::shared_ptr<picture_encoding_data> > newImageSet;
  FOR_LOOP(std::shared_ptr<picture_encoding_data>, picdata, mImages) {
    if (picdata->in_use) {
      picdata->reconstruction->PicState = UsedForShortTermReference; // TODO: this is only a hack

      newImageSet.push_back(picdata);
    }
  }

  mImages = newImageSet;
}



// --- data access ---

bool encoder_picture_buffer::have_more_frames_to_encode() const
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->state < picture_encoding_data::state_encoding) {
      return true;
    }
  }

  return false;
}


std::shared_ptr<picture_encoding_data> encoder_picture_buffer::get_next_picture_to_encode()
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->state < picture_encoding_data::state_encoding) {
      return mImages[i];
    }
  }

  return NULL;
}


std::shared_ptr<const picture_encoding_data> encoder_picture_buffer::get_picture(int frame_number) const
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->frame_number == frame_number)
      return mImages[i];
  }

  assert(false);
  return NULL;
}


std::shared_ptr<picture_encoding_data> encoder_picture_buffer::get_picture(int frame_number)
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->frame_number == frame_number)
      return mImages[i];
  }

  assert(false);
  return NULL;
}


bool encoder_picture_buffer::has_picture(int frame_number) const
{
  for (int i=0;i<mImages.size();i++) {
    if (mImages[i]->frame_number == frame_number)
      return true;
  }

  return false;
}
