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

  //mAlgo_TB_Split_BruteForce.setAlgo_TB_IntraPredMode(algo_TB_IntraPredMode);
  //mAlgo_TB_Split_BruteForce.setAlgo_TB_Residual(&mAlgo_TB_Transform);

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



void EncoderCore_Screensharing::findStaticBlocks(std::shared_ptr<const image> reference,
                                                 std::shared_ptr<const image> current,
                                                 MetaDataArray<BlockInfo_Screensharing>& blockInfo) const
{
  int w = current->get_width();
  int h = current->get_height();

  int width_blocks  = ceil_div(w,BLK_SIZE);
  int height_blocks = ceil_div(h,BLK_SIZE);

  int strideA = reference->get_image_stride(0);
  int strideB = current  ->get_image_stride(0);


  int threshold = 10*10 * BLK_SIZE*BLK_SIZE;
  int LOG2_BLK_SIZE = Log2(BLK_SIZE);

  for (int yb=0;yb<height_blocks;yb++) {
    for (int xb=0;xb<width_blocks;xb++) {
      const uint8_t* pA = reference->get_image_plane_at_pos<const uint8_t>(0, xb*BLK_SIZE, yb*BLK_SIZE);
      const uint8_t* pB = current  ->get_image_plane_at_pos<const uint8_t>(0, xb*BLK_SIZE, yb*BLK_SIZE);

      uint32_t ssd = SSD(pA,strideA, pB,strideB, BLK_SIZE, BLK_SIZE);

      blockInfo.get(xb,yb).blkSize_log2 = LOG2_BLK_SIZE; // TODO: should not be set here

      if (ssd < threshold) {
        blockInfo.get(xb,yb).isStatic = true;
        printf(".");
      }
      else {
        printf("X");
      }
    }

    printf("\n");
  }
}


#if ENABLE_TEST_MODE
#include <libvideogfx.hh>
using namespace videogfx;

Image<Pixel> videogfxImage_from_Libde265Image(std::shared_ptr<const image> image)
{
  int w = image->get_width();
  int h = image->get_height();

  Image<Pixel> img;

  if (image->get_supplementary_data().colorspace == de265_colorspace_GBR) {
    img.Create(w,h, Colorspace_RGB);

    for (int y=0;y<h;y++) {
      memcpy( img.AskFrameG()[y], image->get_image_plane_at_pos(0,0,y), w);
      memcpy( img.AskFrameB()[y], image->get_image_plane_at_pos(1,0,y), w);
      memcpy( img.AskFrameR()[y], image->get_image_plane_at_pos(2,0,y), w);
    }
  }
  else {
    assert(false);
  }

  return img;
}
#endif


//inline bool same_sign(int a,int b) { return (a^b)>=0; }

static uint16_t block_hash_DJB2a(const uint8_t* p, int stride, int blkSize)
{
  uint16_t hash = 0;

  for (int yy=0;yy<blkSize;yy++)
    for (int xx=0;xx<blkSize;xx++) {
      hash = ((hash << 5) + hash) ^ *(p+xx+yy*stride); /* (hash * 33) ^ c */
    }

  return hash;
}


#define block_hash block_hash_DJB2a


void EncoderCore_Screensharing::computeAndMatchFeaturePoints(std::shared_ptr<const image> currImage,
                                                             Screensharing_ImageMetadata& currMetadata,
                                                             std::shared_ptr<const image> prevImage,
                                                             std::shared_ptr<const Screensharing_ImageMetadata> prevmMtadata,
                                                             int frame_number) const
{
  int w = currImage->get_width();
  int h = currImage->get_height();

  const uint8_t* pimg = currImage->get_image_plane(0);
  const int stride    = currImage->get_image_stride(0);

#if ENABLE_TEST_MODE
  Image<Pixel> visu;
  visu = videogfxImage_from_Libde265Image(currImage);
#endif

  memset(currMetadata.hash, 0, 65536*sizeof(Screensharing_ImageMetadata::HashInfo));


  int nFeatures = 0;
  int nCollisions = 0;

  for (int y=HASH_BLK_RADIUS; y<h-HASH_BLK_RADIUS; y++)
    for (int x=HASH_BLK_RADIUS; x<w-HASH_BLK_RADIUS; x++) {
      const uint8_t* p = pimg + x + y*stride;

      const bool isFeaturePoint = (*p > *(p-1) &&
                                   *p > *(p-stride) &&
                                   *p < *(p-stride-1) && // opposite sign to reduce number of features
                                   *p > *(p-stride+1) &&
                                   *p > *(p+stride-1));

      if (isFeaturePoint) {
#if ENABLE_TEST_MODE
        int r = 1;

        DrawLine(visu,
                 x-r,y-r,
                 x+r,y+r,
                 Color<Pixel>(255,50,50));
        DrawLine(visu,
                 x-r,y+r,
                 x+r,y-r,
                 Color<Pixel>(255,50,50));
#endif

        uint16_t hash_value = block_hash_DJB2a(p,stride, 2*HASH_BLK_RADIUS+1);
        currMetadata.hash[hash_value].cnt++;

        if (currMetadata.hash[hash_value].cnt > 1) {
          nCollisions++;
        }

        nFeatures++;


        // --- find a corresponding feature point in the previous image
      }
    }


#if ENABLE_TEST_MODE
  char buf[100];
  sprintf(buf,"features%05d.png", frame_number);
  WriteImage_PNG(buf, visu);
#endif

  printf("nFeatures=%d nCollisions=%d\n",nFeatures,nCollisions);
}


void EncoderCore_Screensharing::preprocess_image(encoder_context* ectx,
                                                 std::shared_ptr<picture_encoding_data> imgdata)
{
  int w = imgdata->input->get_width();
  int h = imgdata->input->get_height();

  auto metadata = std::make_shared<Screensharing_ImageMetadata>();
  imgdata->algoCoreImageMetadata = metadata;

  bool success = metadata->blockInfo.alloc( ceil_div(w,BLK_SIZE), ceil_div(h,BLK_SIZE), Log2(BLK_SIZE) );
  assert(success); // TODO
  metadata->blockInfo.clear();

  printf("preprocess image %d. ref0 list: ", imgdata->frame_number);
  for (int f : imgdata->ref0) {
    printf("%d ",f);
  }
  printf("\n");


  // --- find static blocks

  if (imgdata->ctbs.get_slice_header(0,0)->slice_type != SLICE_TYPE_I) {
    assert(imgdata->ref0.size()>0);
    int refImageNumber = imgdata->ref0[0];
    auto prevImage = ectx->get_input_image_history().get_image(refImageNumber);

    findStaticBlocks(prevImage, imgdata->input, metadata->blockInfo);
  }


  // --- compute feature points

  if (imgdata->ctbs.get_slice_header(0,0)->slice_type == SLICE_TYPE_I) {
    computeAndMatchFeaturePoints(imgdata->input, *metadata,
                                 nullptr, nullptr,
                                 imgdata->frame_number);
  }
  else {
    int refImageNumber = imgdata->ref0[0];
    auto prevImage = ectx->get_input_image_history().get_image(refImageNumber);

    auto prev_metadata_base = ectx->picbuf.get_picture(refImageNumber)->algoCoreImageMetadata;
    auto prev_metadata = std::dynamic_pointer_cast<Screensharing_ImageMetadata>(prev_metadata_base);

    computeAndMatchFeaturePoints(imgdata->input, *metadata,
                                 prevImage, prev_metadata,
                                 imgdata->frame_number);
  }
}
