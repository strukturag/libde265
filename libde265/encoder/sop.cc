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
    //sps->compute_derived_values();


    // --- set some more default values ---

    sps->log2_max_pic_order_cnt_lsb = get_num_poc_lsb_bits();

    sps->pcm_enabled_flag = true;


    // turn off deblocking filter
    pps->deblocking_filter_control_present_flag = true;
    pps->deblocking_filter_override_enabled_flag = false;
    pps->pic_disable_deblocking_filter_flag = true;
    pps->pps_loop_filter_across_slices_enabled_flag = false;

    pps->set_derived_values(sps.get());


    mEncCtx->fill_headers(vps,sps,pps, img);
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

  imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);
  imgdata->mark_sop_metadata_set();

  imgdata->ctbs.add_slice_header(shdr);

  mEncPicBuf->insert_next_image_in_encoding_order(imgdata);

  advance_frame();
}


// ---------------------------------------------------------------------------


sop_creator_trivial_low_delay::sop_creator_trivial_low_delay()
{
}


void sop_creator_trivial_low_delay::insert_new_input_image(image_ptr img)
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
    //sps->compute_derived_values();


    // --- set SPS RefPicSets

    ref_pic_set rps;
    rps.addRefL0(-1, ref_pic_set::image_mode::used);

    rps.DeltaPocS0[0] = -1;
    rps.UsedByCurrPicS0[0] = true;
    rps.NumNegativePics = 1;
    rps.NumPositivePics = 0;
    rps.compute_derived_values();

    sps->ref_pic_sets.push_back(rps);


    // --- set some more default values ---

    sps->log2_max_pic_order_cnt_lsb = get_num_poc_lsb_bits();

    sps->pcm_enabled_flag = false;


    // turn off deblocking filter
    pps->deblocking_filter_control_present_flag = true;
    pps->deblocking_filter_override_enabled_flag = false;
    pps->pic_disable_deblocking_filter_flag = true;
    pps->pps_loop_filter_across_slices_enabled_flag = false;

    pps->set_derived_values(sps.get());


    mEncCtx->fill_headers(vps,sps,pps, img);
  }


  // --- initialize picture_encoding_data

  img->PicOrderCntVal = get_pic_order_count();


  // fake input image state, so that we can use it for MC in some fast algorithms
  img->PicState = UsedForShortTermReference;

  int frame_num = get_frame_number();



  // picture data

  assert(mEncPicBuf);
  auto imgdata = std::make_shared<picture_encoding_data>(img,
                                                         get_frame_number(),
                                                         vps, sps, pps,
                                                         mEncCtx->get_CTB_size_log2());

  auto shdr = std::make_shared<slice_segment_header>();
  shdr->set_defaults();

  imgdata->reconstruction->add_slice_segment_header(shdr);

  if (isIntra(frame_num)) {
    reset_poc();
    imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);
    shdr->slice_type = SLICE_TYPE_I;
  } else {

    // fill list l0 with preceding image

    std::vector<int> l0, l1, empty;
    l0.push_back(frame_num - 1);


    // set RefPicList in slice header

    shdr->num_ref_idx_l0_active = l0.size();

    assert(l0.size() < MAX_NUM_REF_PICS);
    for (int i=0;i<l0.size();i++) {
      shdr->RefPicList[0][i] = l0[i];
    }


    imgdata->set_references(0, l0,l1, empty,empty);
    imgdata->set_NAL_type(NAL_UNIT_TRAIL_R);
    shdr->slice_type = SLICE_TYPE_P;
    shdr->five_minus_max_num_merge_cand = 5-1;
  }

  shdr->slice_pic_order_cnt_lsb = get_pic_order_count_lsb();

  imgdata->mark_sop_metadata_set();
  imgdata->ctbs.add_slice_header(shdr);

  mEncPicBuf->insert_next_image_in_encoding_order(imgdata);

  advance_frame();
}
