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


#include "libde265/encoder/algo/cb-intrapartmode.h"
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


