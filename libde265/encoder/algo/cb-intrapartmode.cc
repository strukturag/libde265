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
                                                  enc_cb* cb_in)
{
  const int log2CbSize = cb_in->log2CbSize;
  const int x = cb_in->x;
  const int y = cb_in->y;

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

    cb[p] = new enc_cb();
    *cb[p] = *cb_in;


    // --- set intra prediction mode ---

    cb[p]->PredMode = MODE_INTRA; // TODO: obsolete
    cb[p]->PartMode = (p==0 ? PART_2Nx2N : PART_NxN);

    ectx->img->set_pred_mode(x,y, log2CbSize, cb[p]->PredMode);
    ectx->img->set_PartMode (x,y, cb[p]->PartMode);  // TODO: probably unnecessary


    // encode transform tree

    int IntraSplitFlag= (cb[p]->PredMode == MODE_INTRA && cb[p]->PartMode == PART_NxN);
    int MaxTrafoDepth = ectx->sps.max_transform_hierarchy_depth_intra + IntraSplitFlag;

    cb[p]->transform_tree = mTBIntraPredModeAlgo->analyze(ectx, ctxModel,
                                                          ectx->imgdata->input, NULL, cb[p],
                                                          x,y, x,y, log2CbSize,
                                                          0,
                                                          0, MaxTrafoDepth, IntraSplitFlag);

    cb[p]->distortion = cb[p]->transform_tree->distortion;
    cb[p]->rate       = cb[p]->transform_tree->rate;


    // rate for cu syntax

    CABAC_encoder_estim estim;
    ectx->switch_CABAC(ctxModel, &estim);
    encode_coding_unit(ectx,cb[p],x,y,log2CbSize, false);
    cb[p]->rate += estim.getRDBits();


    // estimate distortion

    cb[p]->write_to_image(ectx->img, x,y, true);
    cb[p]->distortion = compute_distortion_ssd(ectx->img, ectx->imgdata->input, x,y, log2CbSize, 0);
  }

  delete cb_in;


  // choose from 2Nx2N and NxN

  if (cb[0] && cb[1]) {
    double rd_cost_2Nx2N = cb[0]->distortion + ectx->lambda * cb[0]->rate;
    double rd_cost_NxN   = cb[1]->distortion + ectx->lambda * cb[1]->rate;

    if (rd_cost_2Nx2N < rd_cost_NxN) {
      cb[0]->write_to_image(ectx->img, x,y, true);
      cb[0]->reconstruct(&ectx->accel, ectx->img, x,y);
      delete cb[1];
      return cb[0];
    } else {
      delete cb[0];
      return cb[1];
    }
  }

  return cb[0]; // 2Nx2N
}


enc_cb* Algo_CB_IntraPartMode_Fixed::analyze(encoder_context* ectx,
                                             context_model_table ctxModel,
                                             enc_cb* cb)
{
  enum PartMode PartMode = mParams.partMode();


  const int log2CbSize = cb->log2CbSize;
  const int x = cb->x;
  const int y = cb->y;


  // NxN can only be applied at minimum CB size.
  // If we are not at the minimum size, we have to use 2Nx2N.

  if (PartMode==PART_NxN && log2CbSize != ectx->sps.Log2MinCbSizeY) {
    PartMode = PART_2Nx2N;
  }


  // --- set intra prediction mode ---

  cb->PredMode = MODE_INTRA; // TODO: obsolete
  cb->PartMode = PartMode;

  ectx->img->set_pred_mode(x,y, log2CbSize, cb->PredMode);
  ectx->img->set_PartMode (x,y, cb->PartMode);  // TODO: probably unnecessary


  // encode transform tree

  int IntraSplitFlag= (cb->PredMode == MODE_INTRA && cb->PartMode == PART_NxN);
  int MaxTrafoDepth = ectx->sps.max_transform_hierarchy_depth_intra + IntraSplitFlag;

  cb->transform_tree = mTBIntraPredModeAlgo->analyze(ectx, ctxModel,
                                                     ectx->imgdata->input, NULL, cb,
                                                     cb->x,cb->y, cb->x,cb->y, log2CbSize,
                                                     0,
                                                     0, MaxTrafoDepth, IntraSplitFlag);
  

  // rate and distortion for this CB

  cb->distortion = cb->transform_tree->distortion;
  cb->rate       = cb->transform_tree->rate;


  // rate for cu syntax

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);
  encode_coding_unit(ectx,cb,x,y,log2CbSize, false);
  cb->rate += estim.getRDBits();

  return cb;
}


