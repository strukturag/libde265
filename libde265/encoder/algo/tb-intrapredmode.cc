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


#include "libde265/encoder/encoder-context.h"
#include "libde265/encoder/algo/tb-split.h"
#include "libde265/encoder/algo/coding-options.h"
#include <assert.h>
#include <limits>
#include <math.h>
#include <algorithm>
#include <iostream>


enum IntraPredMode find_best_intra_mode(de265_image& img,int x0,int y0, int log2BlkSize, int cIdx,
                                        const uint8_t* ref, int stride)
{
  //return INTRA_ANGULAR_20;

  enum IntraPredMode best_mode;
  int min_sad=-1;

  enum IntraPredMode candidates[3];

  const seq_parameter_set* sps = &img.sps;


  fillIntraPredModeCandidates(candidates, x0,y0,
                              sps->getPUIndexRS(x0,y0),
                              x0>0, y0>0, &img);

  // --- test candidates first ---

  for (int idx=0;idx<3;idx++) {
    enum IntraPredMode mode = (enum IntraPredMode)candidates[idx];
    decode_intra_prediction(&img, x0,y0, (enum IntraPredMode)mode, 1<<log2BlkSize, cIdx);

    uint32_t distortion = SSD(ref,stride,
      img.get_image_plane_at_pos(cIdx, x0,y0), img.get_image_stride(cIdx),
      1<<log2BlkSize, 1<<log2BlkSize);

    int sad=distortion;

    sad *= 0.5;
    //sad *= 0.9;

    if (mode==0 || sad<min_sad) {
      min_sad = sad;
      best_mode = (enum IntraPredMode)mode;
    }
  }


  // --- test all modes ---

  for (int idx=0;idx<35;idx++) {
    enum IntraPredMode mode = (enum IntraPredMode)idx; //candidates[idx];
    decode_intra_prediction(&img, x0,y0, (enum IntraPredMode)mode, 1<<log2BlkSize, cIdx);


    uint32_t distortion = SSD(ref,stride,
      img.get_image_plane_at_pos(cIdx, x0,y0), img.get_image_stride(cIdx),
      1<<log2BlkSize, 1<<log2BlkSize);

    int sad=distortion;

    if (min_sad<0 || sad<min_sad) {
      min_sad = sad;
      best_mode = (enum IntraPredMode)mode;
    }
  }

  return best_mode;
}


float get_intra_pred_mode_bits(const enum IntraPredMode candidates[3],
                               enum IntraPredMode intraMode,
                               enum IntraPredMode intraModeC,
                               context_model_table& context_models,
                               bool includeChroma)
{
  float rate;
  int enc_bin;

  /**/ if (candidates[0]==intraMode) { rate = 1; enc_bin=1; }
  else if (candidates[1]==intraMode) { rate = 2; enc_bin=1; }
  else if (candidates[2]==intraMode) { rate = 2; enc_bin=1; }
  else { rate = 5; enc_bin=0; }

  CABAC_encoder_estim estim;
  estim.set_context_models(&context_models);
  logtrace(LogSymbols,"$1 prev_intra_luma_pred_flag=%d\n",enc_bin);
  estim.write_CABAC_bit(CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG, enc_bin);

  // TODO: currently we make the chroma-pred-mode decision for each part even
  // in NxN part mode. Since we always set this to the same value, it does not
  // matter. However, we should only add the rate for it once (for blkIdx=0).

  if (includeChroma) {
    assert(intraMode == intraModeC);

    logtrace(LogSymbols,"$1 intra_chroma_pred_mode=%d\n",0);
    estim.write_CABAC_bit(CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE,0);
  }
  rate += estim.getRDBits();

  return rate;
}




float estim_TB_bitrate(const encoder_context* ectx,
                       const de265_image* input,
                       int x0,int y0, int log2BlkSize,
                       enum TBBitrateEstimMethod method)
{
  int blkSize = 1<<log2BlkSize;

  float distortion;

  switch (method)
    {
    case TBBitrateEstim_SSD:
      return SSD(input->get_image_plane_at_pos(0, x0,y0),
                 input->get_image_stride(0),
                 ectx->img->get_image_plane_at_pos(0, x0,y0),
                 ectx->img->get_image_stride(0),
                 1<<log2BlkSize, 1<<log2BlkSize);
      break;

    case TBBitrateEstim_SAD:
      return SAD(input->get_image_plane_at_pos(0, x0,y0),
                 input->get_image_stride(0),
                 ectx->img->get_image_plane_at_pos(0, x0,y0),
                 ectx->img->get_image_stride(0),
                 1<<log2BlkSize, 1<<log2BlkSize);
      break;

    case TBBitrateEstim_SATD_DCT:
    case TBBitrateEstim_SATD_Hadamard:
      {
        int16_t coeffs[32*32];
        int16_t diff[32*32];

        diff_blk(diff,blkSize,
                 input->get_image_plane_at_pos(0, x0,y0), input->get_image_stride(0),
                 ectx->img->get_image_plane_at_pos(0, x0,y0), ectx->img->get_image_stride(0),
                 blkSize);

        if (method == TBBitrateEstim_SATD_Hadamard) {
          ectx->acceleration.hadamard_transform_8[log2BlkSize-2](coeffs, diff, &diff[blkSize] - &diff[0]);
        }
        else {
          ectx->acceleration.fwd_transform_8[log2BlkSize-2](coeffs, diff, &diff[blkSize] - &diff[0]);
        }

        float distortion=0;
        for (int i=0;i<blkSize*blkSize;i++) {
          distortion += abs_value((int)coeffs[i]);
        }

        return distortion;
      }
      break;

      /*
    case TBBitrateEstim_AccurateBits:
      assert(false);
      return 0;
      */
    }

  assert(false);
  return 0;
}



enc_tb*
Algo_TB_IntraPredMode_BruteForce::analyze(encoder_context* ectx,
                                          context_model_table& ctxModel,
                                          const de265_image* input,
                                          enc_tb* tb,
                                          int TrafoDepth, int MaxTrafoDepth,
                                          int IntraSplitFlag)
{
  enter();

  enc_cb* cb = tb->cb;

  bool selectIntraPredMode = false;
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_2Nx2N && TrafoDepth==0);
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_NxN   && TrafoDepth==1);

  if (selectIntraPredMode) {

    CodingOptions<enc_tb> options(ectx, tb, ctxModel);
    CodingOption<enc_tb>  option[35];

    for (int i=0;i<35;i++) {
      bool computeIntraMode = isPredModeEnabled((enum IntraPredMode)i);
      option[i] = options.new_option(computeIntraMode);
    }

    options.start();


    tb->writeSurroundingMetadata(ectx, ectx->img,
                                 enc_node::METADATA_INTRA_MODES,
                                 tb->get_rectangle());

    const de265_image* img = ectx->img;
    const seq_parameter_set* sps = &img->sps;
    enum IntraPredMode candidates[3];
    fillIntraPredModeCandidates(candidates, tb->x,tb->y,
                                sps->getPUIndexRS(tb->x,tb->y),
                                tb->x > 0, tb->y > 0, img);


    for (int i = 0; i<35; i++) {
      if (!option[i]) {
        continue;
      }


      enum IntraPredMode intraMode = (IntraPredMode)i;


      option[i].begin();

      enc_tb* tb_option = option[i].get_node();

      enum IntraPredMode intraModeC = intraMode;

      tb_option->intra_mode        = intraMode;

      // set chroma mode to same mode is its luma mode
      if (cb->PartMode==PART_NxN) {
        tb_option->intra_mode_chroma = tb->parent->children[0]->intra_mode;
      }
      else {
        tb_option->intra_mode_chroma = intraMode;
      }

      descend(tb_option,"%d",intraMode);
      tb_option = mTBSplitAlgo->analyze(ectx,option[i].get_context(),input,tb_option,
                                        TrafoDepth, MaxTrafoDepth, IntraSplitFlag);
      option[i].set_node(tb_option);
      ascend();

      float intraPredModeBits = get_intra_pred_mode_bits(candidates,
                                                         intraMode,
                                                         intraModeC,
                                                         option[i].get_context(),
                                                         tb_option->blkIdx == 0);

      tb_option->rate_withoutCbfChroma += intraPredModeBits;
      tb_option->rate += intraPredModeBits;

      option[i].end();
    }


    options.compute_rdo_costs();

    return options.return_best_rdo_node();
  }
  else {
    descend(tb,"NOP"); // TODO: not parent
    enc_tb* new_tb = mTBSplitAlgo->analyze(ectx, ctxModel, input, tb,
                                           TrafoDepth, MaxTrafoDepth, IntraSplitFlag);
    ascend();
    return new_tb;
  }

  assert(false);
  return nullptr;
}



enc_tb*
Algo_TB_IntraPredMode_MinResidual::analyze(encoder_context* ectx,
                                           context_model_table& ctxModel,
                                           const de265_image* input,
                                           enc_tb* tb,
                                           int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag)
{
  enter();

  enc_cb* cb = tb->cb;

  int x0 = tb->x;
  int y0 = tb->y;
  int xBase = cb->x;
  int yBase = cb->y;
  int log2TbSize = tb->log2Size;

  bool selectIntraPredMode = false;
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_2Nx2N && TrafoDepth==0);
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_NxN   && TrafoDepth==1);

  //printf("tb-intrapredmode: %d %d %d\n", cb->PredMode, cb->PartMode, TrafoDepth);

  if (selectIntraPredMode) {

    *tb->downPtr = tb;

    enum IntraPredMode intraMode;
    float minDistortion = std::numeric_limits<float>::max();

    assert(nPredModesEnabled()>=1);

    if (nPredModesEnabled()==1) {
      intraMode = getPredMode(0);
    }
    else {
      for (int idx=0;idx<nPredModesEnabled();idx++) {
        enum IntraPredMode mode = getPredMode(idx);
        decode_intra_prediction(ectx->img, x0,y0, (enum IntraPredMode)mode, 1<<log2TbSize, 0);

        float distortion;
        distortion = estim_TB_bitrate(ectx, input, x0,y0, log2TbSize,
                                      mParams.bitrateEstimMethod());

        if (distortion<minDistortion) {
          minDistortion = distortion;
          intraMode = mode;
        }
      }
    }

    //cb->intra.pred_mode[blkIdx] = intraMode;
    //if (blkIdx==0) { cb->intra.chroma_mode = intraMode; }

    //intraMode=(IntraPredMode)17; // HACK

    //printf("INTRA MODE (%d;%d) = %d\n",x0,y0,intraMode);
    // ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode); // DEPRECATED

    enum IntraPredMode intraModeC = intraMode;

    tb->intra_mode        = intraMode;

    // set chroma mode to same mode is its luma mode
    if (cb->PartMode==PART_NxN) {
      tb->intra_mode_chroma = tb->parent->children[0]->intra_mode;
    }
    else {
      tb->intra_mode_chroma = intraMode;
    }


    /*
      decode_intra_prediction(ectx->img, x0,y0,       intraMode, 1<< log2TbSize,    0);
      decode_intra_prediction(ectx->img, x0>>1,y0>>1, intraMode, 1<<(log2TbSize-1), 1);
      decode_intra_prediction(ectx->img, x0>>1,y0>>1, intraMode, 1<<(log2TbSize-1), 2);
    */

    // Note: cannot prepare intra prediction pixels here, because this has to
    // be done at the lowest TB split level.


    descend(tb,"%d",intraMode);
    tb = mTBSplitAlgo->analyze(ectx,ctxModel,input,tb,
                               TrafoDepth, MaxTrafoDepth, IntraSplitFlag);
    ascend();

    debug_show_image(ectx->img, 0);


    tb->writeSurroundingMetadata(ectx, ectx->img,
                                 enc_node::METADATA_INTRA_MODES,
                                 tb->get_rectangle());

    enum IntraPredMode candidates[3];
    fillIntraPredModeCandidates(candidates, x0,y0,
                                ectx->img->sps.getPUIndexRS(x0,y0),
                                x0>0, y0>0, ectx->img);

    float intraPredModeBits = get_intra_pred_mode_bits(candidates,
                                                       intraMode,
                                                       intraModeC,
                                                       ctxModel,
                                                       tb->blkIdx == 0);

    tb->rate_withoutCbfChroma += intraPredModeBits;
    tb->rate += intraPredModeBits;

    return tb;
  }
  else {
    descend(tb,"NOP");
    enc_tb* nop_tb = mTBSplitAlgo->analyze(ectx, ctxModel, input, tb,
                                           TrafoDepth, MaxTrafoDepth,
                                           IntraSplitFlag);
    ascend();
    return nop_tb;
  }

  assert(false);
  return nullptr;
}

static bool sortDistortions(std::pair<enum IntraPredMode,float> i,
                            std::pair<enum IntraPredMode,float> j)
{
  return i.second < j.second;
}

enc_tb*
Algo_TB_IntraPredMode_FastBrute::analyze(encoder_context* ectx,
                                         context_model_table& ctxModel,
                                         const de265_image* input,
                                         const enc_tb* parent,
                                         enc_cb* cb,
                                         int x0,int y0, int xBase,int yBase,
                                         int log2TbSize, int blkIdx,
                                         int TrafoDepth, int MaxTrafoDepth,
                                         int IntraSplitFlag)
{
  //printf("encode_transform_tree_may_split %d %d (%d %d) size %d\n",x0,y0,xBase,yBase,1<<log2TbSize);

  /*
    enum IntraPredMode pre_intraMode = find_best_intra_mode(ectx->img,x0,y0, log2TbSize, 0,
    input->get_image_plane_at_pos(0,x0,y0),
    input->get_image_stride(0));
  */

  bool selectIntraPredMode = false;
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_2Nx2N && TrafoDepth==0);
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_NxN   && TrafoDepth==1);

  if (selectIntraPredMode) {
    float minCost = std::numeric_limits<float>::max();
    int   minCostIdx=0;
    float minCandCost;

    const de265_image* img = ectx->img;
    const seq_parameter_set* sps = &img->sps;
    enum IntraPredMode candidates[3];
    fillIntraPredModeCandidates(candidates, x0,y0,
                                sps->getPUIndexRS(x0,y0),
                                x0>0, y0>0, img);



    std::vector< std::pair<enum IntraPredMode,float> > distortions;

    for (int idx=0;idx<35;idx++)
      if (idx!=candidates[0] && idx!=candidates[1] && idx!=candidates[2] &&
          isPredModeEnabled((enum IntraPredMode)idx))
        {
          enum IntraPredMode mode = (enum IntraPredMode)idx;
          decode_intra_prediction(ectx->img, x0,y0, (enum IntraPredMode)mode, 1<<log2TbSize, 0);

          float distortion;
          distortion = estim_TB_bitrate(ectx, input, x0,y0, log2TbSize,
                                        mParams.bitrateEstimMethod());

          distortions.push_back( std::make_pair((enum IntraPredMode)idx, distortion) );
        }

    std::sort( distortions.begin(), distortions.end(), sortDistortions );


    for (int i=0;i<distortions.size();i++)
      {
        //printf("%d -> %f\n",i,distortions[i].second);
      }

    int keepNBest=std::min((int)mParams.keepNBest, (int)distortions.size());
    distortions.resize(keepNBest);
    distortions.push_back(std::make_pair((enum IntraPredMode)candidates[0],0));
    distortions.push_back(std::make_pair((enum IntraPredMode)candidates[1],0));
    distortions.push_back(std::make_pair((enum IntraPredMode)candidates[2],0));


    enc_tb* tb[35];
    context_model_table contexts[35];

    for (int i=0;i<35;i++) tb[i]=NULL;

    for (int i=0;i<distortions.size();i++) {

      //copy_context_model_table(ctxIntra, ctxModel);

      enum IntraPredMode intraMode  = (IntraPredMode)distortions[i].first;
      enum IntraPredMode intraModeC = intraMode;

      if (!isPredModeEnabled(intraMode)) { continue; }

      assert(tb[intraMode]->blkIdx == blkIdx);

      //cb->intra.pred_mode[blkIdx] = intraMode;
      //if (blkIdx==0) { cb->intra.chroma_mode = intraMode; }

      tb[intraMode]->intra_mode        = intraMode;

      // set chroma mode to same mode is its luma mode
      if (cb->PartMode==PART_NxN) {
        tb[intraMode]->intra_mode_chroma = cb->transform_tree->children[0]->intra_mode;
      }
      else {
        tb[intraMode]->intra_mode_chroma = intraMode;
      }


      ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

      contexts[intraMode] = ctxModel.copy();
      descend(cb,"%d",intraMode); // TODO: not cb
      assert(false); /*
                       tb[intraMode] = mTBSplitAlgo->analyze(ectx,contexts[intraMode],input,parent,
                       cb, x0,y0, xBase,yBase, log2TbSize, blkIdx,
                       TrafoDepth, MaxTrafoDepth, IntraSplitFlag);
                     */
      ascend();


      float intraPredModeBits = get_intra_pred_mode_bits(candidates,
                                                         intraMode,
                                                         intraModeC,
                                                         contexts[intraMode],
                                                         blkIdx == 0);

      tb[intraMode]->rate_withoutCbfChroma += intraPredModeBits;
      tb[intraMode]->rate += intraPredModeBits;

      //printf("QQQ %f %f\n", b, estim.getRDBits());

      float cost = tb[intraMode]->distortion + ectx->lambda * tb[intraMode]->rate;

      //printf("idx:%d mode:%d cost:%f\n",i,intraMode,cost);

      if (cost<minCost) {
        minCost=cost;
        minCostIdx=intraMode;
        //minCandCost=c;
      }
    }


    enum IntraPredMode intraMode = (IntraPredMode)minCostIdx;

    assert(tb[minCostIdx]->blkIdx == blkIdx);

    //cb->intra.pred_mode[blkIdx] = intraMode;
    //if (blkIdx==0) { cb->intra.chroma_mode  = intraMode; } //INTRA_CHROMA_LIKE_LUMA;

    tb[minCostIdx]->intra_mode        = intraMode;

    // set chroma mode to same mode is its luma mode
    if (cb->PartMode==PART_NxN) {
      tb[minCostIdx]->intra_mode_chroma = cb->transform_tree->children[0]->intra_mode;
    }
    else {
      tb[minCostIdx]->intra_mode_chroma = intraMode;
    }


    ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    tb[minCostIdx]->reconstruct(ectx, ectx->img);
    ctxModel = contexts[minCostIdx];

    for (int i = 0; i<35; i++) {
      if (i != minCostIdx) {
        delete tb[i];
      }
    }

    return tb[minCostIdx];
  }
  else {
    descend(cb,"NOP"); // TODO: not cb
    assert(false);
    enc_tb* tb;
    /*
      enc_tb* tb = mTBSplitAlgo->analyze(ectx, ctxModel, input, parent, cb,
      x0,y0,xBase,yBase, log2TbSize,
      blkIdx, TrafoDepth, MaxTrafoDepth,
      IntraSplitFlag);
    */
    ascend();
    return tb;
  }

  assert(false);
  return nullptr;
}
