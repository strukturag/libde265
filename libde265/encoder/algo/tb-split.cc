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
#include "libde265/encoder/encoder-syntax.h"
#include <assert.h>
#include <limits>
#include <math.h>
#include <iostream>


#define ENCODER_DEVELOPMENT 1


void recursively_add_cbfChroma_rate(CABAC_encoder_estim* cabac,
                                    enc_tb* tb, int log2TrafoSize, int trafoDepth)
{
  float bits_pre = cabac->getRDBits();

  // --- CBF CB/CR ---

  // For 4x4 luma, there is no signaling of chroma CBF, because only the
  // chroma CBF for 8x8 is relevant.
  if (log2TrafoSize>2) {
    if (trafoDepth==0 || tb->parent->cbf[1]) {
      encode_cbf_chroma(cabac, trafoDepth, tb->cbf[1]);
    }
    if (trafoDepth==0 || tb->parent->cbf[2]) {
      encode_cbf_chroma(cabac, trafoDepth, tb->cbf[2]);
    }
  }

  if (tb->split_transform_flag) {
    for (int i=0;i<4;i++) {
      recursively_add_cbfChroma_rate(cabac, tb->children[i], log2TrafoSize-1, trafoDepth+1);
    }
  }

  float bits_post = cabac->getRDBits();

  tb->rate = tb->rate_withoutCbfChroma + (bits_post - bits_pre);
}


enc_tb* Algo_TB_Split::encode_transform_tree_split(encoder_context* ectx,
                                                   context_model_table& ctxModel,
                                                   const de265_image* input,
                                                   const enc_tb* parent,
                                                   enc_cb* cb,
                                                   int x0,int y0, int log2TbSize,
                                                   int TrafoDepth, int MaxTrafoDepth,
                                                   int IntraSplitFlag)
{
  const de265_image* img = ectx->img;

  enc_tb* tb = new enc_tb();

  tb->parent = parent;
  tb->split_transform_flag = true;
  tb->log2Size = log2TbSize;
  tb->x = x0;
  tb->y = y0;

  tb->distortion = 0;
  tb->rate = 0;
  tb->rate_withoutCbfChroma = 0;


  // Since we try to code all sub-blocks, we enable all CBF flags.
  // Should we see later that the child TBs are zero, we clear those flags later.

  tb->cbf[0]=1;
  tb->cbf[1]=1;
  tb->cbf[2]=1;


  context_model ctxModelCbfChroma[4];
  for (int i=0;i<4;i++) {
    ctxModelCbfChroma[i] = ctxModel[CONTEXT_MODEL_CBF_CHROMA+i];
  }


  // --- encode all child nodes ---

  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (log2TbSize-1);
    int dy = (i>>1) << (log2TbSize-1);

    if (cb->PredMode == MODE_INTRA) {
      descend(tb,"intra");
      tb->children[i] = mAlgo_TB_IntraPredMode->analyze(ectx, ctxModel, input, tb, cb,
                                                        x0+dx, y0+dy, x0,y0,
                                                        log2TbSize-1, i,
                                                        TrafoDepth+1, MaxTrafoDepth, IntraSplitFlag);
      ascend("bits:%f",tb->rate);
    }
    else {
      descend(tb,"inter");
      tb->children[i] = this->analyze(ectx, ctxModel, input, tb, cb,
                                      x0+dx, y0+dy, x0,y0,
                                      log2TbSize-1, i,
                                      TrafoDepth+1, MaxTrafoDepth, IntraSplitFlag);
      ascend();
    }

    tb->distortion            += tb->children[i]->distortion;
    tb->rate_withoutCbfChroma += tb->children[i]->rate_withoutCbfChroma;
  }

  tb->set_cbf_flags_from_children();


  // --- add rate for this TB level ---

  CABAC_encoder_estim estim;
  estim.set_context_models(&ctxModel);




  const seq_parameter_set* sps = &ectx->img->sps;

  if (log2TbSize <= sps->Log2MaxTrafoSize &&
      log2TbSize >  sps->Log2MinTrafoSize &&
      TrafoDepth < MaxTrafoDepth &&
      !(IntraSplitFlag && TrafoDepth==0))
    {
      encode_split_transform_flag(ectx, &estim, log2TbSize, 1);
      tb->rate_withoutCbfChroma += estim.getRDBits();
      estim.reset();
    }

  // restore chroma CBF context models

  for (int i=0;i<4;i++) {
    ctxModel[CONTEXT_MODEL_CBF_CHROMA+i] = ctxModelCbfChroma[i];
  }

  recursively_add_cbfChroma_rate(&estim,tb, log2TbSize, TrafoDepth);
  //tb->rate = tb->rate_withoutCbfChroma + estim.getRDBits();

  return tb;
}



struct Logging_TB_Split : public Logging
{
  int skipTBSplit, noskipTBSplit;
  int zeroBlockCorrelation[6][2][5];

  const char* name() const { return "tb-split"; }

  void print(const encoder_context* ectx, const char* filename)
  {
    for (int tb=3;tb<=5;tb++) {
      for (int z=0;z<=1;z++) {
        float total = 0;

        for (int c=0;c<5;c++)
          total += zeroBlockCorrelation[tb][z][c];

        for (int c=0;c<5;c++) {
          printf("%d %d %d : %d %5.2f\n", tb,z,c,
                 zeroBlockCorrelation[tb][z][c],
                 total==0 ? 0 : zeroBlockCorrelation[tb][z][c]/total*100);
        }
      }
    }


    for (int z=0;z<2;z++) {
      printf("\n");
      for (int tb=3;tb<=5;tb++) {
        float total = 0;

        for (int c=0;c<5;c++)
          total += zeroBlockCorrelation[tb][z][c];

        printf("%dx%d ",1<<tb,1<<tb);

        for (int c=0;c<5;c++) {
          printf("%5.2f ", total==0 ? 0 : zeroBlockCorrelation[tb][z][c]/total*100);
        }
        printf("\n");
      }
    }
  }
} logging_tb_split;



enc_tb*
Algo_TB_Split_BruteForce::analyze(encoder_context* ectx,
                                  context_model_table& ctxModel,
                                  const de265_image* input,
                                  const enc_tb* parent,
                                  enc_cb* cb,
                                  int x0,int y0, int xBase,int yBase, int log2TbSize,
                                  int blkIdx,
                                  int TrafoDepth, int MaxTrafoDepth,
                                  int IntraSplitFlag)
{
  bool test_split = (log2TbSize > 2 &&
                     TrafoDepth < MaxTrafoDepth &&
                     log2TbSize > ectx->sps.Log2MinTrafoSize);

  bool test_no_split = true;
  if (IntraSplitFlag && TrafoDepth==0) test_no_split=false; // we have to split
  if (log2TbSize > ectx->sps.Log2MaxTrafoSize) test_no_split=false;

  context_model_table ctxSplit;
  if (test_split) {
    ctxSplit = ctxModel.copy();
  }


  enc_tb* tb_no_split = NULL;
  enc_tb* tb_split    = NULL;
  float rd_cost_no_split = std::numeric_limits<float>::max();
  float rd_cost_split    = std::numeric_limits<float>::max();

  if (test_no_split) {
    descend(cb,"no split");
    tb_no_split = mAlgo_TB_Residual->analyze(ectx, ctxModel, input, parent,
                                             cb, x0,y0, xBase,yBase, log2TbSize,
                                             blkIdx,
                                             TrafoDepth,MaxTrafoDepth,IntraSplitFlag);
    ascend("bits:%f/%f",tb_no_split->rate,tb_no_split->rate_withoutCbfChroma);

    rd_cost_no_split = tb_no_split->distortion + ectx->lambda * tb_no_split->rate;

    if (log2TbSize <= mParams.zeroBlockPrune()) {
      bool zeroBlock = tb_no_split->isZeroBlock();

      if (zeroBlock) {
        test_split = false;
        logging_tb_split.skipTBSplit++;
      }
      else
        logging_tb_split.noskipTBSplit++;
    }
  }


  if (test_split) {
    descend(cb,"split");
    tb_split = encode_transform_tree_split(ectx, ctxSplit, input, parent, cb,
                                           x0,y0, log2TbSize,
                                           TrafoDepth, MaxTrafoDepth, IntraSplitFlag);
    ascend();

    rd_cost_split    = tb_split->distortion    + ectx->lambda * tb_split->rate;
  }


  if (test_split && test_no_split) {
    bool zero_block = tb_no_split->isZeroBlock();

    int nChildZero = 0;
    for (int i=0;i<4;i++) {
      if (tb_split->children[i]->isZeroBlock()) nChildZero++;
    }

    logging_tb_split.zeroBlockCorrelation[log2TbSize][zero_block ? 0 : 1][nChildZero]++;
  }


  bool split = (rd_cost_split < rd_cost_no_split);

  if (split) {
    ctxModel = ctxSplit;

    delete tb_no_split;
    assert(tb_split);
    return tb_split;
  }
  else {
    delete tb_split;
    assert(tb_no_split);
    tb_no_split->reconstruct(ectx, ectx->img,
                             cb, blkIdx);

    return tb_no_split;
  }
}
