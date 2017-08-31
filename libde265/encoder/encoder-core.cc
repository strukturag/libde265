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


#include "libde265/encoder/encoder-core.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>
#include <iostream>
#include <fstream>



FixedHeadersHelper::FixedHeadersHelper()
{
  vps = std::make_shared<video_parameter_set>();
  sps = std::make_shared<seq_parameter_set>();
  pps = std::make_shared<pic_parameter_set>();

  vps->set_defaults(Profile_Main, 6,2);

  sps->set_defaults();

  pps->set_defaults();
  pps->sps = sps;


  // --- set some more default values ---

  // turn off deblocking filter
  pps->deblocking_filter_control_present_flag = true;
  pps->deblocking_filter_override_enabled_flag = false;
  pps->pic_disable_deblocking_filter_flag = true;
  pps->pps_loop_filter_across_slices_enabled_flag = false;

  pps->set_derived_values(sps.get());
}


void FixedHeadersHelper::set_image_size(image_ptr img)
{
  sps->set_resolution(img->get_width(), img->get_height());
}


void FixedHeadersHelper::encode_headers(encoder_context* ectx)
{
  // write headers

  en265_packet* pck;

  nal_header nal;

  CABAC_encoder_bitstream cabac_encoder;

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(cabac_encoder);
  vps->write(ectx, cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = ectx->create_packet(EN265_PACKET_VPS, cabac_encoder);
  pck->nal_unit_type = EN265_NUT_VPS;
  ectx->push_output_packet(pck);

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(cabac_encoder);
  sps->write(ectx, cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = ectx->create_packet(EN265_PACKET_SPS, cabac_encoder);
  pck->nal_unit_type = EN265_NUT_SPS;
  ectx->push_output_packet(pck);

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(cabac_encoder);
  pps->write(ectx, cabac_encoder, sps.get());
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = ectx->create_packet(EN265_PACKET_PPS, cabac_encoder);
  pck->nal_unit_type = EN265_NUT_PPS;
  ectx->push_output_packet(pck);


  mHeadersHaveBeenSent = true;
}



static int IntraPredModeCnt[7][35];
static int MPM_used[7][35];

static int IntraPredModeCnt_total[35];
static int MPM_used_total[35];

/*
void statistics_IntraPredMode(const encoder_context* ectx, int x,int y, const enc_cb* cb)
{
  if (cb->split_cu_flag) {
    for (int i=0;i<4;i++)
      if (cb->children[i]) {
        statistics_IntraPredMode(ectx, childX(x,i,cb->log2Size), childY(y,i,cb->log2Size), cb->children[i]);
      }
  }
  else {
    int cnt;
    int size = cb->log2Size;

    if (cb->PartMode == PART_NxN) { cnt=4; size--; } else cnt=1;

    for (int i=0;i<cnt;i++) {
      IntraPredModeCnt[size][ cb->intra.pred_mode[i] ]++;
      IntraPredModeCnt_total[ cb->intra.pred_mode[i] ]++;

      int xi = childX(x,i,cb->log2Size);
      int yi = childY(y,i,cb->log2Size);

      enum IntraPredMode candModeList[3];
      fillIntraPredModeCandidates(candModeList,xi,yi, xi>0, yi>0, ectx->img);

      int predmode = cb->intra.pred_mode[i];
      if (candModeList[0]==predmode ||
          candModeList[1]==predmode ||
          candModeList[2]==predmode) {
        MPM_used[size][predmode]++;
        MPM_used_total[predmode]++;
      }
    }
  }
}
*/

void statistics_print()
{
  for (int i=0;i<35;i++) {
    printf("%d",i);
    printf("  %d %d",IntraPredModeCnt_total[i], MPM_used_total[i]);

    for (int k=2;k<=6;k++) {
      printf("  %d %d",IntraPredModeCnt[k][i], MPM_used[k][i]);
    }

    printf("\n");
  }
}


void print_tb_tree_rates(const enc_tb* tb, int level)
{
  for (int i=0;i<level;i++)
    std::cout << "  ";

  std::cout << "TB rate=" << tb->rate << " (" << tb->rate_withoutCbfChroma << ")\n";
  if (tb->split_transform_flag) {
    for (int i=0;i<4;i++)
      print_tb_tree_rates(tb->children[i], level+1);
  }
}


void print_cb_tree_rates(const enc_cb* cb, int level)
{
  for (int i=0;i<level;i++)
    std::cout << "  ";

  std::cout << "CB rate=" << cb->rate << "\n";
  if (cb->split_cu_flag) {
    for (int i=0;i<4;i++)
      print_cb_tree_rates(cb->children[i], level+1);
  }
  else {
    print_tb_tree_rates(cb->transform_tree, level+1);
  }
}



/*
void EncoderCore::send_encoded_picture_packet(std::shared_ptr<encoded_picture_data> encpic)
{
}
*/

void EncoderCore::initialize(encoder_picture_buffer* encpicbuf,
                             encoder_context* ectx)
{
  mECtx = ectx;
  mEncPicBuf = encpicbuf;
}



EncoderCore_Custom::encoder_params::encoder_params()
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


void EncoderCore_Custom::encoder_params::registerParams(config_parameters& config)
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



int EncoderCore_Custom::get_CTB_size_log2() const
{
  return Log2(params.max_cb_size);
}


void EncoderCore_Custom::fill_sps(std::shared_ptr<seq_parameter_set> sps) const
{
  sps->set_CB_log2size_range( Log2(params.min_cb_size), Log2(params.max_cb_size));
  sps->set_TB_log2size_range( Log2(params.min_tb_size), Log2(params.max_tb_size));
  sps->max_transform_hierarchy_depth_intra = params.max_transform_hierarchy_depth_intra;
  sps->max_transform_hierarchy_depth_inter = params.max_transform_hierarchy_depth_inter;
}


void EncoderCore_Custom::initialize(encoder_picture_buffer* encpicbuf,
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

  mAlgo_CB_IntraInter_BruteForce.setIntraChildAlgo(algo_CB_IntraPartMode);
  mAlgo_CB_IntraInter_BruteForce.setInterChildAlgo(&mAlgo_CB_InterPartMode_Fixed);

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


void EncoderCore_Custom::encode_picture(image_ptr img)
{
  if (!mFixedHeadersHelper.have_headers_been_sent()) {
    mFixedHeadersHelper.set_image_size(img);

    mSOPCreator->fill_sps(mFixedHeadersHelper.get_sps());
    mSOPCreator->fill_pps(mFixedHeadersHelper.get_pps());

    // compute derived values (TODO: is this the right place?)
    de265_error err = mFixedHeadersHelper.get_sps()->compute_derived_values(true);
    if (err != DE265_OK) {
      fprintf(stderr,"invalid SPS parameters\n");
      exit(10);
    }

    mFixedHeadersHelper.encode_headers(mECtx);
  }

  mSOPCreator->insert_new_input_image(img);
}



void Logging::print_logging(const encoder_context* ectx, const char* id, const char* filename)
{
#if 000
  if (strcmp(id,logging_tb_split.name())==0) {
    logging_tb_split.print(ectx,filename);
  }
#endif
}


void en265_print_logging(const encoder_context* ectx, const char* id, const char* filename)
{
  Logging::print_logging(ectx,id,filename);
}
