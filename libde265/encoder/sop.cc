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

#include "libde265/encoder/sop.h"
#include "libde265/encoder/encoder-context.h"


sop_creator_intra_only::sop_creator_intra_only()
{
}


void sop_creator_intra_only::fill_sps(std::shared_ptr<seq_parameter_set> sps)
{
  sps->log2_max_pic_order_cnt_lsb = get_num_poc_lsb_bits();
}


void sop_creator_intra_only::insert_new_input_image(image_ptr img)
{
  img->PicOrderCntVal = get_pic_order_count();

  reset_poc();
  int poc = get_pic_order_count();

  assert(mEncPicBuf);
  auto imgdata = mEncPicBuf->insert_next_image_in_encoding_order(img, get_frame_number());

  auto shdr = std::make_shared<slice_segment_header>();
  shdr->set_defaults();
  shdr->slice_type = SLICE_TYPE_I;
  shdr->slice_pic_order_cnt_lsb = get_pic_order_count_lsb();

  imgdata->reconstruction->add_slice_segment_header(shdr);

  imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);

  imgdata->mark_sop_metadata_set();

  advance_frame();
}


// ---------------------------------------------------------------------------


sop_creator_trivial_low_delay::sop_creator_trivial_low_delay()
{
}


void sop_creator_trivial_low_delay::fill_sps(std::shared_ptr<seq_parameter_set> sps)
{
  ref_pic_set rps;
  rps.DeltaPocS0[0] = -1;
  rps.UsedByCurrPicS0[0] = true;
  rps.NumNegativePics = 1;
  rps.NumPositivePics = 0;
  rps.compute_derived_values();
  sps->ref_pic_sets.push_back(rps);
  sps->log2_max_pic_order_cnt_lsb = get_num_poc_lsb_bits();
}


void sop_creator_trivial_low_delay::insert_new_input_image(image_ptr img)
{
  img->PicOrderCntVal = get_pic_order_count();

  // fake input image state, so that we can use it for MC in some fast algorithms
  img->PicState = UsedForShortTermReference;

  int frame = get_frame_number();

  std::vector<int> l0, l1, empty;
  if (!isIntra(frame)) {
    l0.push_back(frame-1);
  }

  assert(mEncPicBuf);
  auto imgdata = mEncPicBuf->insert_next_image_in_encoding_order(img, get_frame_number());

  auto shdr = std::make_shared<slice_segment_header>();
  shdr->set_defaults();
  shdr->slice_pic_order_cnt_lsb = get_pic_order_count_lsb();

  imgdata->reconstruction->add_slice_segment_header(shdr);

  if (isIntra(frame)) {
    reset_poc();
    imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);
    shdr->slice_type = SLICE_TYPE_I;
  } else {
    shdr->num_ref_idx_l0_active = l0.size();
    //shdr.num_ref_idx_l1_active = l1.size();

    assert(l0.size() < MAX_NUM_REF_PICS);
    for (int i=0;i<l0.size();i++) {
      shdr->RefPicList[0][i] = l0[i];
    }

    /*
      assert(l1.size() < MAX_NUM_REF_PICS);
      for (int i=0;i<l1.size();i++) {
      shdr.RefPicList[1][i] = l1[i];
      }
    */


    imgdata->set_references(0, l0,l1, empty,empty);
    imgdata->set_NAL_type(NAL_UNIT_TRAIL_R);
    shdr->slice_type = SLICE_TYPE_P;
  }

  imgdata->mark_sop_metadata_set();

  advance_frame();
}
