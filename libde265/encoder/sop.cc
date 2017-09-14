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


void sop_creator_intra_only::insert_new_input_image(image_ptr img)
{
  // --- allocate headers, if not done already

  if (!pps) {
    vps = std::make_shared<video_parameter_set>();
    sps = std::make_shared<seq_parameter_set>();
    pps = std::make_shared<pic_parameter_set>();

    vps->set_defaults(Profile_Main, 6,2);
    vps->video_parameter_set_id = 0;

    sps->set_defaults();
    sps->seq_parameter_set_id = 0;
    sps->video_parameter_set_id = 0;

    pps->set_defaults();
    pps->pic_parameter_set_id = 0;
    pps->seq_parameter_set_id = 0;
    pps->sps = sps;


    // --- set image size

    sps->pic_width_in_luma_samples  = img->get_width();
    sps->pic_height_in_luma_samples = img->get_height();
    sps->compute_derived_values();


    // --- set some more default values ---

    sps->log2_max_pic_order_cnt_lsb = get_num_poc_lsb_bits();


    // turn off deblocking filter
    pps->deblocking_filter_control_present_flag = true;
    pps->deblocking_filter_override_enabled_flag = false;
    pps->pic_disable_deblocking_filter_flag = true;
    pps->pps_loop_filter_across_slices_enabled_flag = false;

    pps->set_derived_values(sps.get());
  }


  // --- initialize picture_encoding_data

  img->PicOrderCntVal = get_pic_order_count();

  reset_poc();
  int poc = get_pic_order_count();


  // slice header

  auto shdr = std::make_shared<slice_segment_header>();
  shdr->set_defaults();
  shdr->slice_type = SLICE_TYPE_I;
  shdr->slice_pic_order_cnt_lsb = get_pic_order_count_lsb();


  // picture data

  auto imgdata = std::make_shared<picture_encoding_data>(img,
                                                         get_frame_number(),
                                                         vps, sps, pps,
                                                         mEncCtx->get_CTB_size_log2());

  assert(mEncPicBuf);
  mEncPicBuf->insert_next_image_in_encoding_order(imgdata);

  imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);
  imgdata->mark_sop_metadata_set();

  imgdata->reconstruction->add_slice_segment_header(shdr);


  advance_frame();
}


// ---------------------------------------------------------------------------


sop_creator_trivial_low_delay::sop_creator_trivial_low_delay()
{
}


void sop_creator_trivial_low_delay::insert_new_input_image(image_ptr img)
{
  /* TODO

  ref_pic_set rps;
  rps.DeltaPocS0[0] = -1;
  rps.UsedByCurrPicS0[0] = true;
  rps.NumNegativePics = 1;
  rps.NumPositivePics = 0;
  rps.compute_derived_values();
  sps->ref_pic_sets.push_back(rps);
  sps->log2_max_pic_order_cnt_lsb = get_num_poc_lsb_bits();
   */

  img->PicOrderCntVal = get_pic_order_count();

  // fake input image state, so that we can use it for MC in some fast algorithms
  img->PicState = UsedForShortTermReference;

  int frame = get_frame_number();

  std::vector<int> l0, l1, empty;
  if (!isIntra(frame)) {
    l0.push_back(frame-1);
  }

  assert(mEncPicBuf);
  auto imgdata = std::make_shared<picture_encoding_data>(img,
                                                         get_frame_number(),
                                                         vps, sps, pps,
                                                         mEncCtx->get_CTB_size_log2());
  mEncPicBuf->insert_next_image_in_encoding_order(imgdata);

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
