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


#include "libde265/encoder/analyze.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>


#define ENCODER_DEVELOPMENT 1



enc_cb* Algo_CB_IntraPartMode_BruteForce::analyze(encoder_context* ectx,
                                                  context_model_table ctxModel,
                                                  const de265_image* input,
                                                  int x0,int y0, int log2CbSize, int ctDepth,
                                                  int qp)
{
  enc_cb* cb[2] =
    { NULL, // 2Nx2N  (always checked)
      NULL  //  NxN   (only checked at MinCbSize)
    };


  const bool can_use_NxN = ((log2CbSize == ectx->sps.Log2MinCbSizeY) &&
                            (log2CbSize >  ectx->sps.Log2MinTrafoSize));

  // Test NxN intra prediction mode only when at minimum Cb size.
  const int lastMode = (can_use_NxN ? 2 : 1);


  // test all modes

  for (int p=0;p<lastMode;p++) {

    cb[p] = ectx->enc_cb_pool.get_new();

    cb[p]->split_cu_flag = false;
    cb[p]->log2CbSize = log2CbSize;
    cb[p]->ctDepth = ctDepth;

    cb[p]->cu_transquant_bypass_flag = false;


    // --- set intra prediction mode ---

    cb[p]->PredMode = MODE_INTRA;
    cb[p]->PartMode = (p==0 ? PART_2Nx2N : PART_NxN);

    ectx->img->set_pred_mode(x0,y0, log2CbSize, cb[p]->PredMode);
    ectx->img->set_PartMode (x0,y0, cb[p]->PartMode);  // TODO: probably unnecessary


    // encode transform tree

    int IntraSplitFlag= (cb[p]->PredMode == MODE_INTRA && cb[p]->PartMode == PART_NxN);
    int MaxTrafoDepth = ectx->sps.max_transform_hierarchy_depth_intra + IntraSplitFlag;

    cb[p]->transform_tree = mTBIntraPredModeAlgo->analyze(ectx, ctxModel, input, NULL, cb[p],
                                                          x0,y0, x0,y0, log2CbSize,
                                                          0,
                                                          0, MaxTrafoDepth, IntraSplitFlag,
                                                          qp);

    cb[p]->distortion = cb[p]->transform_tree->distortion;
    cb[p]->rate       = cb[p]->transform_tree->rate;


    // rate for cu syntax

    CABAC_encoder_estim estim;
    ectx->switch_CABAC(ctxModel, &estim);
    encode_coding_unit(ectx,cb[p],x0,y0,log2CbSize, false);
    cb[p]->rate += estim.getRDBits();


    // estimate distortion

    cb[p]->write_to_image(ectx->img, x0,y0, true);
    cb[p]->distortion = compute_distortion_ssd(ectx->img, input, x0,y0, log2CbSize, 0);
  }


  // choose from 2Nx2N and NxN

  if (cb[0] && cb[1]) {
    double rd_cost_2Nx2N = cb[0]->distortion + ectx->lambda * cb[0]->rate;
    double rd_cost_NxN   = cb[1]->distortion + ectx->lambda * cb[1]->rate;

    if (rd_cost_2Nx2N < rd_cost_NxN) {
      cb[0]->write_to_image(ectx->img, x0,y0, true);
      cb[0]->reconstruct(&ectx->accel, ectx->img, x0,y0, qp);
      return cb[0];
    } else {
      return cb[1];
    }
  }

  return cb[0]; // 2Nx2N
}


enc_cb* Algo_CB_IntraPartMode_Fixed::analyze(encoder_context* ectx,
                                             context_model_table ctxModel,
                                             const de265_image* input,
                                             int x0,int y0, int log2CbSize, int ctDepth,
                                             int qp)
{
  enum PartMode PartMode = mParams.partMode();


  // NxN can only be applied at minimum CB size.
  // If we are not at the minimum size, we have to use 2Nx2N.

  if (PartMode==PART_NxN && log2CbSize != ectx->sps.Log2MinCbSizeY) {
    PartMode = PART_2Nx2N;
  }


  // --- create new CB ---

  enc_cb* cb = ectx->enc_cb_pool.get_new();

  cb->split_cu_flag = false;
  cb->log2CbSize = log2CbSize;
  cb->ctDepth = ctDepth;

  cb->cu_transquant_bypass_flag = false;


  // --- set intra prediction mode ---

  cb->PredMode = MODE_INTRA;
  cb->PartMode = PartMode;

  ectx->img->set_pred_mode(x0,y0, log2CbSize, cb->PredMode);
  ectx->img->set_PartMode (x0,y0, cb->PartMode);  // TODO: probably unnecessary


  // encode transform tree

  int IntraSplitFlag= (cb->PredMode == MODE_INTRA && cb->PartMode == PART_NxN);
  int MaxTrafoDepth = ectx->sps.max_transform_hierarchy_depth_intra + IntraSplitFlag;

  cb->transform_tree = mTBIntraPredModeAlgo->analyze(ectx, ctxModel, input, NULL, cb,
                                                     x0,y0, x0,y0, log2CbSize,
                                                     0,
                                                     0, MaxTrafoDepth, IntraSplitFlag,
                                                     qp);
  

  // rate and distortion for this CB

  cb->distortion = cb->transform_tree->distortion;
  cb->rate       = cb->transform_tree->rate;


  // rate for cu syntax

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);
  encode_coding_unit(ectx,cb,x0,y0,log2CbSize, false);
  cb->rate += estim.getRDBits();

  return cb;
}




bool Algo_CB_Split::forcedSplit(const de265_image* input, int x0,int y0, int Log2CbSize) const
{
  int w = input->get_width();
  int h = input->get_height();
  int cbSize = (1<<Log2CbSize);

  if (x0+cbSize > w) return true;
  if (y0+cbSize > h) return true;
  return false;
}


// Utility function to encode all four children in a splitted CB.
// Children are coded with the specified algo_cb_split.
enc_cb* Algo_CB_Split::encode_cb_split(encoder_context* ectx,
                                       context_model_table ctxModel,
                                       const de265_image* input,
                                       int x0,int y0, int Log2CbSize, int ctDepth, int qp)
{
  int w = input->get_width();
  int h = input->get_height();


  // create a splitted CB node
  enc_cb* cb = ectx->enc_cb_pool.get_new();

  cb->split_cu_flag = true;

  cb->cu_transquant_bypass_flag = false;
  cb->log2CbSize = Log2CbSize;
  cb->ctDepth = ctDepth;


  // rate for split_cu_flag (=true)

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);
  encode_quadtree(ectx,cb,x0,y0,Log2CbSize,ctDepth, false);

  cb->distortion = 0;
  cb->rate       = estim.getRDBits();


  // encode all 4 children and sum their distortions and rates

  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (Log2CbSize-1);
    int dy = (i>>1) << (Log2CbSize-1);

    if (x0+dx>=w || y0+dy>=h) {
      cb->children[i] = NULL;
    }
    else {
      cb->children[i] = analyze(ectx, ctxModel,
                                input, x0+dx, y0+dy,
                                Log2CbSize-1, ctDepth+1, qp);

      cb->distortion += cb->children[i]->distortion;
      cb->rate       += cb->children[i]->rate;
    }
  }

  return cb;
}




enc_cb* Algo_CB_Split_BruteForce::analyze(encoder_context* ectx,
                                          context_model_table ctxModel,
                                          const de265_image* input,
                                          int x0,int y0, int Log2CbSize,
                                          int ctDepth,
                                          int qp)
{
  // if we try both variants, make a copy of the ctxModel and use the copy for splitting

  const bool can_split_CB   = (Log2CbSize > ectx->sps.Log2MinCbSizeY);
  const bool can_nosplit_CB = !forcedSplit(input,x0,y0,Log2CbSize);

  context_model_table ctxCopy;
  context_model* ctxSplit = ctxModel;

  if (can_split_CB && can_nosplit_CB) {
    copy_context_model_table(ctxCopy, ctxModel);
    ctxSplit=ctxCopy;
  }


  // try encoding without splitting

  enc_cb* cb_no_split = NULL;
  enc_cb* cb_split    = NULL;

  if (can_nosplit_CB) {
    cb_no_split = mIntraPartModeAlgo->analyze(ectx, ctxModel, input,
                                              x0,y0, Log2CbSize, ctDepth, qp);
  }

  // if possible, try to split CB

  if (can_split_CB) {
    cb_split = encode_cb_split(ectx, ctxSplit,
                               input,x0,y0, Log2CbSize, ctDepth, qp);
  }


  // if only one variant has been tested, choose this

  if (!can_nosplit_CB) { return cb_split;    }
  if (!can_split_CB)   { return cb_no_split; }


  // compute RD costs for both variants

  const float rd_cost_split    = cb_split->distortion    + ectx->lambda * cb_split->rate;
  const float rd_cost_no_split = cb_no_split->distortion + ectx->lambda * cb_no_split->rate;

  const bool split_is_better =  (rd_cost_split < rd_cost_no_split);

  if (split_is_better) {
    copy_context_model_table(ctxModel, ctxCopy);
    return cb_split;
  }
  else {
    // have to reconstruct state of the first option

    cb_no_split->write_to_image(ectx->img, x0,y0, true);
    cb_no_split->reconstruct(&ectx->accel, ectx->img, x0,y0, qp);
    return cb_no_split;
  }
}




enc_cb* Algo_CTB_QScale_Constant::analyze(encoder_context* ectx,
                                          context_model_table ctxModel,
                                          const de265_image* input,
                                          int ctb_x,int ctb_y,
                                          int log2CtbSize, int ctDepth)
{
  return mChildAlgo->analyze(ectx,ctxModel,input,ctb_x,ctb_y,log2CtbSize,ctDepth,mParams.mQP);
}



static int IntraPredModeCnt[7][35];
static int MPM_used[7][35];

static int IntraPredModeCnt_total[35];
static int MPM_used_total[35];

void statistics_IntraPredMode(const encoder_context* ectx, int x,int y, const enc_cb* cb)
{
  if (cb->split_cu_flag) {
    for (int i=0;i<4;i++)
      if (cb->children[i]) {
        statistics_IntraPredMode(ectx, childX(x,i,cb->log2CbSize), childY(y,i,cb->log2CbSize), cb->children[i]);
      }
  }
  else {
    int cnt;
    int size = cb->log2CbSize;

    if (cb->PartMode == PART_NxN) { cnt=4; size--; } else cnt=1;

    for (int i=0;i<cnt;i++) {
      IntraPredModeCnt[size][ cb->intra.pred_mode[i] ]++;
      IntraPredModeCnt_total[ cb->intra.pred_mode[i] ]++;

      int xi = childX(x,i,cb->log2CbSize);
      int yi = childY(y,i,cb->log2CbSize);

      int candModeList[3];
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


double encode_image(encoder_context* ectx,
                    const de265_image* input,
                    EncodingAlgorithm& algo)
{
  int stride=input->get_image_stride(0);

  int w = ectx->sps.pic_width_in_luma_samples;
  int h = ectx->sps.pic_height_in_luma_samples;

  ectx->img = new de265_image;
  ectx->img->vps  = ectx->vps;
  ectx->img->sps  = ectx->sps;
  ectx->img->pps  = ectx->pps;

  ectx->img->alloc_image(w,h, de265_chroma_420, &ectx->sps, true,
                         NULL /* no decctx */, ectx, 0,NULL,false);
  ectx->img->alloc_encoder_data(&ectx->sps);
  ectx->img->clear_metadata();

  initialize_CABAC_models(ectx->ctx_model, ectx->shdr.initType, ectx->shdr.SliceQPY);

  int Log2CtbSize = ectx->sps.Log2CtbSizeY;

  uint8_t* luma_plane = ectx->img->get_image_plane(0);
  uint8_t* cb_plane   = ectx->img->get_image_plane(1);
  uint8_t* cr_plane   = ectx->img->get_image_plane(2);


  // encode CTB by CTB

  for (int y=0;y<ectx->sps.PicHeightInCtbsY;y++)
    for (int x=0;x<ectx->sps.PicWidthInCtbsY;x++)
      {
        ectx->img->set_SliceAddrRS(x, y, ectx->shdr.SliceAddrRS);

        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;

        logtrace(LogSlice,"encode CTB at %d %d\n",x0,y0);

        // make a copy of the context model that we can modify for testing alternatives

        context_model_table ctxModel;
        copy_context_model_table(ctxModel, ectx->ctx_model_bitstream);

#if 1
        /*
        enc_cb* cb = encode_cb_may_split(ectx, ctxModel,
                                         input, x0,y0, Log2CtbSize, 0, qp);
        */

        enc_cb* cb = algo.getAlgoCTBQScale()->analyze(ectx,ctxModel,
                                                      input, x0,y0, Log2CtbSize,0);
#else
        float minCost = std::numeric_limits<float>::max();
        int bestQ = 0;
        int qp = ectx->params.constant_QP;

        enc_cb* cb;
        for (int q=1;q<51;q++) {
          copy_context_model_table(ctxModel, ectx->ctx_model_bitstream);

          enc_cb* cbq = encode_cb_may_split(ectx, ctxModel,
                                           input, x0,y0, Log2CtbSize, 0, q);

          float cost = cbq->distortion + ectx->lambda * cbq->rate;
          if (cost<minCost) { minCost=cost; bestQ=q; }

          if (q==qp) { cb=cbq; }
        }

        printf("Q %d\n",bestQ);
        fflush(stdout);
#endif

        //statistics_IntraPredMode(ectx, x0,y0, cb);


        cb->write_to_image(ectx->img, x<<Log2CtbSize, y<<Log2CtbSize, true);


        // --- write bitstream ---

        ectx->switch_CABAC_to_bitstream();
        //int preSize = ectx->cabac->size();
        encode_ctb(ectx, cb, x,y);
        //int postSize = ectx->cabac->size();
        //printf("real: %d  estim: %f\n",postSize-preSize, cb->rate/8);


        int last = (y==ectx->sps.PicHeightInCtbsY-1 &&
                    x==ectx->sps.PicWidthInCtbsY-1);
        ectx->cabac_bitstream.write_CABAC_term_bit(last);


        ectx->free_all_pools();
      }


  //statistics_print();


  // frame PSNR

  double psnr = PSNR(MSE(input->get_image_plane(0), input->get_image_stride(0),
                         luma_plane, ectx->img->get_image_stride(0),
                         input->get_width(), input->get_height()));
  return psnr;
}



void EncodingAlgorithm_Custom::setParams(encoder_params& params)
{
  // build algorithm tree

  mAlgo_CTB_QScale_Constant.setChildAlgo(&mAlgo_CB_Split_BruteForce);

  Algo_CB_IntraPartMode* algo_CB_IntraPartMode = NULL;
  switch (params.mAlgo_CB_IntraPartMode()) {
  case ALGO_CB_IntraPartMode_BruteForce:
    algo_CB_IntraPartMode = &mAlgo_CB_IntraPartMode_BruteForce;
    break;
  case ALGO_CB_IntraPartMode_Fixed:
    algo_CB_IntraPartMode = &mAlgo_CB_IntraPartMode_Fixed;
    break;
  }
  mAlgo_CB_Split_BruteForce.setChildAlgo(algo_CB_IntraPartMode);


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
  //mAlgo_TB_Split_BruteForce.setParams(params.TB_Split_BruteForce);

  algo_TB_IntraPredMode->setChildAlgo(&mAlgo_TB_Split_BruteForce);


  // ===== set algorithm parameters ======

  //mAlgo_CB_IntraPartMode_Fixed.setParams(params.CB_IntraPartMode_Fixed);

  //mAlgo_TB_IntraPredMode_FastBrute.setParams(params.TB_IntraPredMode_FastBrute);
  //mAlgo_TB_IntraPredMode_MinResidual.setParams(params.TB_IntraPredMode_MinResidual);


  //mAlgo_CTB_QScale_Constant.setParams(params.CTB_QScale_Constant);


  switch (params.mAlgo_TB_IntraPredMode_Subset())
    {
    case ALGO_TB_IntraPredMode_Subset_All: // activate all is the default
      break;
    case ALGO_TB_IntraPredMode_Subset_DC:
      algo_TB_IntraPredMode->disableAllIntraPredModes();
      algo_TB_IntraPredMode->enableIntraPredMode(INTRA_DC);
      break;
    case ALGO_TB_IntraPredMode_Subset_Planar:
      algo_TB_IntraPredMode->disableAllIntraPredModes();
      algo_TB_IntraPredMode->enableIntraPredMode(INTRA_PLANAR);
      break;
    case ALGO_TB_IntraPredMode_Subset_HVPlus:
      algo_TB_IntraPredMode->disableAllIntraPredModes();
      algo_TB_IntraPredMode->enableIntraPredMode(INTRA_DC);
      algo_TB_IntraPredMode->enableIntraPredMode(INTRA_PLANAR);
      algo_TB_IntraPredMode->enableIntraPredMode(INTRA_ANGULAR_10);
      algo_TB_IntraPredMode->enableIntraPredMode(INTRA_ANGULAR_26);
      break;
    }
}
