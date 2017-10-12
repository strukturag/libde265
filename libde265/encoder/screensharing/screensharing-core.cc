/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
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


#include "libde265/encoder/screensharing/screensharing-core.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>
#include <iostream>
#include <fstream>



EncoderCore_Screensharing::encoder_params::encoder_params()
{
  //rateControlMethod = RateControlMethod_ConstantQP;

  min_cb_size.set_ID("min-cb-size"); min_cb_size.set_valid_values(power2_range_list(8,64)); min_cb_size.set_default(8);
  max_cb_size.set_ID("max-cb-size"); max_cb_size.set_valid_values(power2_range_list(8,64)); max_cb_size.set_default(32);
  min_tb_size.set_ID("min-tb-size"); min_tb_size.set_valid_values(power2_range_list(4,32)); min_tb_size.set_default(4);
  max_tb_size.set_ID("max-tb-size"); max_tb_size.set_valid_values(power2_range_list(8,32)); max_tb_size.set_default(32);

  max_transform_hierarchy_depth_intra.set_ID("max-transform-hierarchy-depth-intra");
  max_transform_hierarchy_depth_intra.set_range(0,4);
  max_transform_hierarchy_depth_intra.set_default(3);

  max_transform_hierarchy_depth_inter.set_ID("max-transform-hierarchy-depth-inter");
  max_transform_hierarchy_depth_inter.set_range(0,4);
  max_transform_hierarchy_depth_inter.set_default(3);

  sop_structure.set_ID("sop-structure");

  mAlgo_TB_IntraPredMode.set_ID("TB-IntraPredMode");
  mAlgo_CB_Skip.set_ID("CB-Skip");

  mAlgo_TB_IntraPredMode_Subset.set_ID("TB-IntraPredMode-subset");
  mAlgo_CB_IntraPartMode.set_ID("CB-IntraPartMode");

  mAlgo_TB_RateEstimation.set_ID("TB-RateEstimation");

  mAlgo_MEMode.set_ID("MEMode");
}


void EncoderCore_Screensharing::encoder_params::registerParams(config_parameters& config)
{
  config.add_option(&min_cb_size);
  config.add_option(&max_cb_size);
  config.add_option(&min_tb_size);
  config.add_option(&max_tb_size);
  config.add_option(&max_transform_hierarchy_depth_intra);
  config.add_option(&max_transform_hierarchy_depth_inter);

  config.add_option(&sop_structure);

  config.add_option(&mAlgo_TB_IntraPredMode);
  config.add_option(&mAlgo_CB_Skip);
  config.add_option(&mAlgo_TB_IntraPredMode_Subset);
  config.add_option(&mAlgo_CB_IntraPartMode);

  config.add_option(&mAlgo_MEMode);
  config.add_option(&mAlgo_TB_RateEstimation);

  mSOP_LowDelay.registerParams(config);
}



int EncoderCore_Screensharing::get_CTB_size_log2() const
{
  return Log2(params.max_cb_size);
}


void EncoderCore_Screensharing::fill_headers(std::shared_ptr<video_parameter_set> vps,
                                             std::shared_ptr<seq_parameter_set> sps,
                                             std::shared_ptr<pic_parameter_set> pps,
                                             image_ptr img) const
{
  sps->set_CB_size_range(params.min_cb_size, params.max_cb_size);
  sps->set_TB_size_range(params.min_tb_size, params.max_tb_size);
  sps->max_transform_hierarchy_depth_intra = params.max_transform_hierarchy_depth_intra;
  sps->max_transform_hierarchy_depth_inter = params.max_transform_hierarchy_depth_inter;
  sps->set_PCM_size_range(8,8); // TODO

  if (img->get_chroma_format() == de265_chroma_444) {
    sps->chroma_format_idc = CHROMA_444;
  }

  pps->pic_init_qp = getPPS_QP();

  de265_error err = sps->compute_derived_values(true);
  if (err != DE265_OK) {
    fprintf(stderr,"invalid SPS parameters\n");
    exit(10);
  }

  pps->set_derived_values(sps.get());
}


void EncoderCore_Screensharing::initialize(encoder_picture_buffer* encpicbuf,
                                           encoder_context* ectx)
{
  EncoderCore::initialize(encpicbuf,ectx);


  // alloc SOP-creator

  if (params.sop_structure() == SOP_Intra) {
    mSOPCreator = std::make_shared<sop_creator_intra_only>();
  }
  else {
    auto sop = std::make_shared<sop_creator_trivial_low_delay>();
    sop->setParams(params.mSOP_LowDelay);
    mSOPCreator = sop;
  }

  mSOPCreator->set_encoder_context(ectx);
  mSOPCreator->set_encoder_picture_buffer(encpicbuf);


  // build algorithm tree

  mAlgo_CTB_QScale_Constant.setChildAlgo(&mAlgo_CB_Split_BruteForce);

  Algo_CB* algo_CB_skip = NULL;
  switch (params.mAlgo_CB_Skip()) {
  case ALGO_CB_Skip_BruteForce:
    mAlgo_CB_Skip_BruteForce.setSkipAlgo(&mAlgo_CB_MergeIndex_Fixed);
    mAlgo_CB_Skip_BruteForce.setNonSkipAlgo(&mAlgo_CB_IntraInter_BruteForce);
    algo_CB_skip = &mAlgo_CB_Skip_BruteForce;
    break;

  case ALGO_CB_Skip_ScreenFast:
    mAlgo_CB_Skip_ScreenFast.setNonSkipAlgo(&mAlgo_CB_MV_Screen);
    algo_CB_skip = &mAlgo_CB_Skip_ScreenFast;
    break;

  case ALGO_CB_Skip_ScreenRegion:
    algo_CB_skip = &mAlgo_CB_MV_ScreenRegion;
    break;
  }

  mAlgo_CB_Split_BruteForce.setChildAlgo(algo_CB_skip);


  Algo_CB_IntraPartMode* algo_CB_IntraPartMode = NULL;
  switch (params.mAlgo_CB_IntraPartMode()) {
  case ALGO_CB_IntraPartMode_BruteForce:
    algo_CB_IntraPartMode = &mAlgo_CB_IntraPartMode_BruteForce;
    break;
  case ALGO_CB_IntraPartMode_Fixed:
    algo_CB_IntraPartMode = &mAlgo_CB_IntraPartMode_Fixed;
    break;
  }

  mAlgo_CB_MV_Screen.setIntraChildAlgo(algo_CB_IntraPartMode);
  mAlgo_CB_MV_ScreenRegion.setIntraAlgo(algo_CB_IntraPartMode);

  mAlgo_CB_IntraInter_BruteForce.setIntraChildAlgo(&mAlgo_CB_PCM);
  mAlgo_CB_IntraInter_BruteForce.setInterChildAlgo(&mAlgo_CB_InterPartMode_Fixed);

  mAlgo_CB_PCM.setChildAlgo(algo_CB_IntraPartMode);

  mAlgo_CB_MergeIndex_Fixed.setChildAlgo(&mAlgo_TB_Split_BruteForce);

  Algo_PB_MV* pbAlgo = NULL;
  switch (params.mAlgo_MEMode()) {
  case MEMode_Test:
    pbAlgo = &mAlgo_PB_MV_Test;
    break;
  case MEMode_Search:
    pbAlgo = &mAlgo_PB_MV_Search;
    break;
  }

  mAlgo_CB_InterPartMode_Fixed.setChildAlgo(pbAlgo);
  pbAlgo->setChildAlgo(&mAlgo_TB_Split_BruteForce);


  Algo_TB_IntraPredMode_ModeSubset* algo_TB_IntraPredMode = NULL;
  switch (params.mAlgo_TB_IntraPredMode()) {
  case ALGO_TB_IntraPredMode_BruteForce:
    algo_TB_IntraPredMode = &mAlgo_TB_IntraPredMode_BruteForce;
    break;
  case ALGO_TB_IntraPredMode_FastBrute:
    algo_TB_IntraPredMode = &mAlgo_TB_IntraPredMode_FastBrute;
    break;
  case ALGO_TB_IntraPredMode_MinResidual:
    algo_TB_IntraPredMode = &mAlgo_TB_IntraPredMode_MinResidual;
    break;
  }

  algo_CB_IntraPartMode->setChildAlgo(algo_TB_IntraPredMode);

  mAlgo_TB_Split_BruteForce.setAlgo_TB_IntraPredMode(algo_TB_IntraPredMode);
  mAlgo_TB_Split_BruteForce.setAlgo_TB_Residual(&mAlgo_TB_Transform);

  Algo_TB_RateEstimation* algo_TB_RateEstimation = NULL;
  switch (params.mAlgo_TB_RateEstimation()) {
  case ALGO_TB_RateEstimation_None:  algo_TB_RateEstimation = &mAlgo_TB_RateEstimation_None;  break;
  case ALGO_TB_RateEstimation_Exact: algo_TB_RateEstimation = &mAlgo_TB_RateEstimation_Exact; break;
  }
  mAlgo_TB_Transform.setAlgo_TB_RateEstimation(algo_TB_RateEstimation);
  //mAlgo_TB_Split_BruteForce.setParams(params.TB_Split_BruteForce);

  algo_TB_IntraPredMode->setChildAlgo(&mAlgo_TB_Split_BruteForce);


  // ===== set algorithm parameters ======

  //mAlgo_CB_IntraPartMode_Fixed.setParams(params.CB_IntraPartMode_Fixed);

  //mAlgo_TB_IntraPredMode_FastBrute.setParams(params.TB_IntraPredMode_FastBrute);
  //mAlgo_TB_IntraPredMode_MinResidual.setParams(params.TB_IntraPredMode_MinResidual);


  //mAlgo_CTB_QScale_Constant.setParams(params.CTB_QScale_Constant);


  algo_TB_IntraPredMode->enableIntraPredModeSubset( params.mAlgo_TB_IntraPredMode_Subset() );
}


void EncoderCore_Screensharing::push_picture(image_ptr img)
{
  // --- put image into encoding queue ---

  mSOPCreator->insert_new_input_image(img);
}


void EncoderCore_Screensharing::preprocess_image(encoder_context* ectx,
                                                 std::shared_ptr<picture_encoding_data> imgdata)
{
  int w = imgdata->input->get_width();
  int h = imgdata->input->get_height();

  auto metadata = std::make_shared<Screensharing_ImageMetadata>();
  imgdata->algoCoreImageMetadata = metadata;

  bool success = metadata->blockInfo.alloc( ceil_div(w,8), ceil_div(h,8), Log2(8) );
  assert(success); // TODO
  metadata->blockInfo.clear();

  printf("preprocess image %d. ref0 list: ", imgdata->frame_number);
  for (int f : imgdata->ref0) {
    printf("%d ",f);
  }
  printf("\n");
}
