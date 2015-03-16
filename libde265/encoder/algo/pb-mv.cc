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


#include "libde265/encoder/algo/pb-mv.h"
#include "libde265/encoder/algo/coding-options.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>



enc_cb* Algo_PB_MV_Test::analyze(encoder_context* ectx,
                                 context_model_table& ctxModel,
                                 enc_cb* cb,
                                 int PBidx, int x,int y,int w,int h)
{
  enum MVTestMode testMode = mParams.testMode();

  motion_spec&   spec = cb->inter.pb[PBidx].spec;
  PredVectorInfo& vec = cb->inter.pb[PBidx].motion;

  spec.merge_flag = 0;
  spec.merge_idx  = 0;

  spec.inter_pred_idc = PRED_L0;
  spec.refIdx[0] = vec.refIdx[0] = 0;
  spec.mvp_l0_flag = 0;

  int value = 8;

  switch (testMode) {
  case MVTestMode_Zero:
    spec.mvd[0][0]=0;
    spec.mvd[0][1]=0;
    break;

  case MVTestMode_Random:
    spec.mvd[0][0] = (rand() % (2*value+1)) - value;
    spec.mvd[0][1] = (rand() % (2*value+1)) - value;
    break;

  case MVTestMode_Horizontal:
    spec.mvd[0][0]=value;
    spec.mvd[0][1]=0;
    break;

  case MVTestMode_Vertical:
    spec.mvd[0][0]=0;
    spec.mvd[0][1]=value;
    break;
  }


  generate_inter_prediction_samples(ectx, ectx->img, ectx->shdr,
                                    cb->x,cb->y, // int xC,int yC,
                                    0,0,         // int xB,int yB,
                                    1<<cb->log2Size, // int nCS,
                                    1<<cb->log2Size,
                                    1<<cb->log2Size, // int nPbW,int nPbH,
                                    &vec);

  // TODO estimate rate for sending MV

  int IntraSplitFlag = 0;
  int MaxTrafoDepth = ectx->sps.max_transform_hierarchy_depth_inter;

  if (mCodeResidual) {
    assert(false);
#if 0
    cb->transform_tree = mTBSplit->analyze(ectx,ctxModel, ectx->imgdata->input, NULL, cb,
                                           cb->x,cb->y,cb->x,cb->y, cb->log2Size,0,
                                           0, MaxTrafoDepth, IntraSplitFlag);

    cb->inter.rqt_root_cbf = ! cb->transform_tree->isZeroBlock();

    cb->distortion = cb->transform_tree->distortion;
    cb->rate       = cb->transform_tree->rate;
#endif
  }
  else {
    const de265_image* input = ectx->imgdata->input;
    de265_image* img   = ectx->img;
    int x0 = cb->x;
    int y0 = cb->y;
    int tbSize = 1<<cb->log2Size;

    cb->distortion = compute_distortion_ssd(input, img, x0,y0, cb->log2Size, 0);
    cb->rate = 5; // fake (MV)

    cb->inter.rqt_root_cbf = 0;
  }

  return cb;
}
