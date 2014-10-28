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
#include "libde265/encoder/algo/tb-split.h"
#include <assert.h>
#include <limits>
#include <math.h>


#define ENCODER_DEVELOPMENT 1



enum IntraPredMode find_best_intra_mode(de265_image& img,int x0,int y0, int log2BlkSize, int cIdx,
                                        const uint8_t* ref, int stride)
{
  //return INTRA_ANGULAR_20;

  enum IntraPredMode best_mode;
  int min_sad=-1;

  int candidates[3];

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


static void diff_blk(int16_t* out,int out_stride,
                     const uint8_t* a_ptr, int a_stride,
                     const uint8_t* b_ptr, int b_stride,
                     int blkSize)
{
  for (int by=0;by<blkSize;by++)
    for (int bx=0;bx<blkSize;bx++)
      {
        out[by*out_stride+bx] = a_ptr[by*a_stride+bx] - b_ptr[by*b_stride+bx];
      }
}



#if 0
void encode_transform_unit(encoder_context* ectx,
                           enc_tb* tb,
                           const de265_image* input,
                           int x0,int y0, // luma position
                           int log2TbSize, // chroma adapted
                           const enc_cb* cb,
                           int qp, int cIdx)
{
  int xC = x0;
  int yC = y0;
  int tbSize = 1<<log2TbSize;


  // --- do intra prediction ---

  enum IntraPredMode intraPredMode  = ectx->img->get_IntraPredMode(x0,y0);

  if (cIdx>0) {
    intraPredMode = cb->intra.chroma_mode; //lumaPredMode_to_chromaPredMode(intraPredMode,
    //cb->intra.chroma_mode);

    xC >>= 1;
    yC >>= 1;
  }
  
  decode_intra_prediction(ectx->img, xC,  yC,   intraPredMode,  tbSize  , cIdx);


  // --- subtract prediction from input ---

  int16_t blk[32*32];
  uint8_t* pred = ectx->img->get_image_plane(cIdx);
  int stride = ectx->img->get_image_stride(cIdx);

  diff_blk(blk,tbSize,
           input->get_image_plane_at_pos(cIdx,xC,yC), input->get_image_stride(cIdx),
           &pred[yC*stride+xC],stride, tbSize);


  // --- forward transform ---

  tb->coeff[cIdx] = ectx->enc_coeff_pool.get_new(tbSize*tbSize);

  int trType = 0;
  if (cIdx==0 && log2TbSize==2) trType=1; // TODO: inter mode

  fwd_transform(&ectx->accel, tb->coeff[cIdx], tbSize, log2TbSize, trType,  blk, tbSize);


  // --- quantization ---

  quant_coefficients(tb->coeff[cIdx], tb->coeff[cIdx], log2TbSize,   qp, true);

  tb->cbf[cIdx] = has_nonzero_value(tb->coeff[cIdx], 1<<(log2TbSize<<1));
}


const enc_tb* encode_transform_tree_no_split(encoder_context* ectx,
                                             context_model_table ctxModel,
                                             const de265_image* input,
                                             const enc_tb* parent,
                                             enc_cb* cb,
                                             int x0,int y0, int xBase,int yBase, int log2TbSize,
                                             int blkIdx,
                                             int trafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                             int qp)
{
  //printf("--- TT at %d %d, size %d, trafoDepth %d\n",x0,y0,1<<log2TbSize,trafoDepth);

  de265_image* img = ectx->img;

  int stride = ectx->img->get_image_stride(0);

  uint8_t* luma_plane = ectx->img->get_image_plane(0);
  uint8_t* cb_plane = ectx->img->get_image_plane(1);
  uint8_t* cr_plane = ectx->img->get_image_plane(2);

  // --- compute transform coefficients ---

  enc_tb* tb = ectx->enc_tb_pool.get_new();

  tb->parent = parent;
  tb->split_transform_flag = false;
  tb->log2TbSize = log2TbSize;
  tb->cbf[0] = tb->cbf[1] = tb->cbf[2] = 0;


  // luma block

  encode_transform_unit(ectx, tb, input, x0,y0, log2TbSize, cb, qp, 0 /* Y */);


  // chroma blocks

  if (log2TbSize > 2) {
    encode_transform_unit(ectx, tb, input, x0,y0, log2TbSize-1, cb, qp, 1 /* Cb */);
    encode_transform_unit(ectx, tb, input, x0,y0, log2TbSize-1, cb, qp, 2 /* Cr */);
  }
  else if (blkIdx==3) {
    encode_transform_unit(ectx, tb, input, xBase,yBase, log2TbSize, cb, qp, 1 /* Cb */);
    encode_transform_unit(ectx, tb, input, xBase,yBase, log2TbSize, cb, qp, 2 /* Cr */);
  }

#if 0
  uint32_t distortion;
  if (log2TbSize==3) {
    distortion = SAD(input->get_image_plane_at_pos(0, x0,y0),
                     input->get_image_stride(0),
                     ectx->img->get_image_plane_at_pos(0, x0,y0),
                     ectx->img->get_image_stride(0),
                     1<<log2TbSize, 1<<log2TbSize);

    int16_t coeffs[64];
    int16_t diff[64];

    diff_blk(diff,8,
             input->get_image_plane_at_pos(0, x0,y0), input->get_image_stride(0),
             ectx->img->get_image_plane_at_pos(0, x0,y0), ectx->img->get_image_stride(0),
             8);

    fdct_8x8_8_fallback(coeffs, diff, &diff[8] - &diff[0]);

    distortion=0;
    for (int i=0;i<64;i++) {
      //printf("%d %d\n",i,(int)coeffs[i]);
      distortion += abs_value((int)coeffs[i]);
    }
  }
#endif

  // reconstruction

  tb->reconstruct(&ectx->accel, ectx->img, x0,y0, xBase,yBase, cb, qp, blkIdx);



  // measure rate

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);

  encode_transform_tree(ectx, tb, cb, x0,y0, xBase,yBase,
                        log2TbSize, trafoDepth, blkIdx, MaxTrafoDepth, IntraSplitFlag, true);


  tb->rate = estim.getRDBits();

#if 0
  if (log2TbSize==3) {
    printf("RATE %d %f\n",distortion,tb->rate);
  }
#endif

  // measure distortion

  int tbSize = 1<<log2TbSize;
  tb->distortion = SSD(input->get_image_plane_at_pos(0, x0,y0), input->get_image_stride(0),
                       img  ->get_image_plane_at_pos(0, x0,y0), img  ->get_image_stride(0),
                       tbSize, tbSize);

  return tb;
}
#endif

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
          ectx->accel.hadamard_transform_8[log2BlkSize-2](coeffs, diff, &diff[blkSize] - &diff[0]);
        }
        else {
          ectx->accel.fwd_transform_8[log2BlkSize-2](coeffs, diff, &diff[blkSize] - &diff[0]);
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



const enc_tb*
Algo_TB_IntraPredMode_BruteForce::analyze(encoder_context* ectx,
                                          context_model_table ctxModel,
                                          const de265_image* input,
                                          const enc_tb* parent,
                                          enc_cb* cb,
                                          int x0,int y0, int xBase,int yBase,
                                          int log2TbSize, int blkIdx,
                                          int TrafoDepth, int MaxTrafoDepth,
                                          int IntraSplitFlag, int qp)
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
    const enc_tb* tb[35];

    float minCost = std::numeric_limits<float>::max();
    int   minCostIdx=0;
    float minCandCost;

    const de265_image* img = ectx->img;
    const seq_parameter_set* sps = &img->sps;
    int candidates[3];
    fillIntraPredModeCandidates(candidates, x0,y0,
                                sps->getPUIndexRS(x0,y0),
                                x0>0, y0>0, img);


    for (int i = 0; i<35; i++) {
      if (!mPredMode_enabled[i]) {
        tb[i]=NULL;
        continue;
      }


      context_model_table ctxIntra;
      copy_context_model_table(ctxIntra, ctxModel);

      enum IntraPredMode intraMode = (IntraPredMode)i;

      cb->intra.pred_mode[blkIdx] = intraMode;
      if (blkIdx==0) { cb->intra.chroma_mode = intraMode; }

      ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

      tb[intraMode] = mTBSplitAlgo->analyze(ectx,ctxIntra,input,parent,
                                            cb, x0,y0, xBase,yBase, log2TbSize, blkIdx,
                                            TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                            qp);


      float sad;
      if ((1<<log2TbSize)==8) {
        decode_intra_prediction(ectx->img, x0,y0, intraMode, 1<<log2TbSize, 0);
        sad = estim_TB_bitrate(ectx,input, x0,y0, log2TbSize, TBBitrateEstim_SAD);
      }


      float rate = tb[intraMode]->rate;
      int enc_bin;

      if (log2TbSize==3) {
        // printf("RATE2 %d %f %f\n",log2TbSize,tb[intraMode]->rate,sad);
      }

      /**/ if (candidates[0]==intraMode) { rate += 1; enc_bin=1; }
      else if (candidates[1]==intraMode) { rate += 2; enc_bin=1; }
      else if (candidates[2]==intraMode) { rate += 2; enc_bin=1; }
      else { rate += 5; enc_bin=0; }

      rate += CABAC_encoder::RDBits_for_CABAC_bin(&ctxIntra[CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG], enc_bin);

      float cost = tb[intraMode]->distortion + ectx->lambda * rate;
      if (cost<minCost) {
        minCost=cost;
        minCostIdx=intraMode;
        //minCandCost=c;
      }
    }


    enum IntraPredMode intraMode = (IntraPredMode)minCostIdx;

    cb->intra.pred_mode[blkIdx] = intraMode;
    if (blkIdx==0) { cb->intra.chroma_mode  = intraMode; } //INTRA_CHROMA_LIKE_LUMA;
    ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    tb[minCostIdx]->reconstruct(&ectx->accel,
                                ectx->img, x0,y0, xBase,yBase,
                                cb, qp, blkIdx);


    //printf("INTRA %d %d  %d\n",pre_intraMode,intraMode,minCandCost);

    return tb[minCostIdx];
  }
  else {
    return mTBSplitAlgo->analyze(ectx, ctxModel, input, parent, cb,
                                 x0,y0,xBase,yBase, log2TbSize,
                                 blkIdx, TrafoDepth, MaxTrafoDepth,
                                 IntraSplitFlag, qp);
  }
}



const enc_tb*
Algo_TB_IntraPredMode_MinResidual::analyze(encoder_context* ectx,
                                           context_model_table ctxModel,
                                           const de265_image* input,
                                           const enc_tb* parent,
                                           enc_cb* cb,
                                           int x0,int y0, int xBase,int yBase,
                                           int log2TbSize, int blkIdx,
                                           int TrafoDepth, int MaxTrafoDepth,
                                           int IntraSplitFlag, int qp)
{

  bool selectIntraPredMode = false;
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_2Nx2N && TrafoDepth==0);
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_NxN   && TrafoDepth==1);

  if (selectIntraPredMode) {

    enum IntraPredMode intraMode;
    float minDistortion;

    for (int idx=0;idx<35;idx++) {
      enum IntraPredMode mode = (enum IntraPredMode)idx;
      decode_intra_prediction(ectx->img, x0,y0, (enum IntraPredMode)mode, 1<<log2TbSize, 0);

      float distortion;
      distortion = estim_TB_bitrate(ectx, input, x0,y0, log2TbSize,
                                    mParams.bitrateEstimMethod());

      if (idx==0 || distortion<minDistortion) {
        minDistortion = distortion;
        intraMode = mode;
      }
    }

    //intraMode=(enum IntraPredMode)(rand()%35);


    cb->intra.pred_mode[blkIdx] = intraMode;
    if (blkIdx==0) { cb->intra.chroma_mode = intraMode; }

    ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    const enc_tb* tb = mTBSplitAlgo->analyze(ectx,ctxModel,input,parent,
                                             cb, x0,y0, xBase,yBase, log2TbSize, blkIdx,
                                             TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                             qp);

    tb->reconstruct(&ectx->accel,
                    ectx->img, x0,y0, xBase,yBase,
                    cb, qp, blkIdx);

    return tb;
  }
  else {
    return mTBSplitAlgo->analyze(ectx, ctxModel, input, parent, cb,
                                 x0,y0,xBase,yBase, log2TbSize,
                                 blkIdx, TrafoDepth, MaxTrafoDepth,
                                 IntraSplitFlag, qp);
  }
}

#include <algorithm>
static bool sortDistortions(std::pair<enum IntraPredMode,float> i,
                            std::pair<enum IntraPredMode,float> j)
{
  return i.second < j.second;
}

const enc_tb*
Algo_TB_IntraPredMode_FastBrute::analyze(encoder_context* ectx,
                                         context_model_table ctxModel,
                                         const de265_image* input,
                                         const enc_tb* parent,
                                         enc_cb* cb,
                                         int x0,int y0, int xBase,int yBase,
                                         int log2TbSize, int blkIdx,
                                         int TrafoDepth, int MaxTrafoDepth,
                                         int IntraSplitFlag, int qp)
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
    const enc_tb* tb[35];

    float minCost = std::numeric_limits<float>::max();
    int   minCostIdx=0;
    float minCandCost;

    const de265_image* img = ectx->img;
    const seq_parameter_set* sps = &img->sps;
    int candidates[3];
    fillIntraPredModeCandidates(candidates, x0,y0,
                                sps->getPUIndexRS(x0,y0),
                                x0>0, y0>0, img);



    std::vector< std::pair<enum IntraPredMode,float> > distortions;

    for (int idx=0;idx<35;idx++)
      if (idx!=candidates[0] && idx!=candidates[1] && idx!=candidates[2] && mPredMode_enabled[idx])
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

    int keepNBest=mParams.keepNBest;
    distortions.resize(keepNBest);
    distortions.push_back(std::make_pair((enum IntraPredMode)candidates[0],0));
    distortions.push_back(std::make_pair((enum IntraPredMode)candidates[1],0));
    distortions.push_back(std::make_pair((enum IntraPredMode)candidates[2],0));

    for (int i=0;i<35;i++) tb[i]=NULL;


    for (int i=0;i<distortions.size();i++) {

      context_model_table ctxIntra;
      copy_context_model_table(ctxIntra, ctxModel);

      enum IntraPredMode intraMode = (IntraPredMode)distortions[i].first;

      cb->intra.pred_mode[blkIdx] = intraMode;
      if (blkIdx==0) { cb->intra.chroma_mode = intraMode; }

      ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

      tb[intraMode] = mTBSplitAlgo->analyze(ectx,ctxIntra,input,parent,
                                            cb, x0,y0, xBase,yBase, log2TbSize, blkIdx,
                                            TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                            qp);


      float rate = tb[intraMode]->rate;
      int enc_bin;

      /**/ if (candidates[0]==intraMode) { rate += 1; enc_bin=1; }
      else if (candidates[1]==intraMode) { rate += 2; enc_bin=1; }
      else if (candidates[2]==intraMode) { rate += 2; enc_bin=1; }
      else { rate += 5; enc_bin=0; }

      rate += CABAC_encoder::RDBits_for_CABAC_bin(&ctxIntra[CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG], enc_bin);

      float cost = tb[intraMode]->distortion + ectx->lambda * rate;
      if (cost<minCost) {
        minCost=cost;
        minCostIdx=intraMode;
        //minCandCost=c;
      }
    }


    enum IntraPredMode intraMode = (IntraPredMode)minCostIdx;

    cb->intra.pred_mode[blkIdx] = intraMode;
    if (blkIdx==0) { cb->intra.chroma_mode  = intraMode; } //INTRA_CHROMA_LIKE_LUMA;
    ectx->img->set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    tb[minCostIdx]->reconstruct(&ectx->accel,
                                ectx->img, x0,y0, xBase,yBase,
                                cb, qp, blkIdx);


    //printf("INTRA %d %d  %d\n",pre_intraMode,intraMode,minCandCost);

    return tb[minCostIdx];
  }
  else {
    return mTBSplitAlgo->analyze(ectx, ctxModel, input, parent, cb,
                                 x0,y0,xBase,yBase, log2TbSize,
                                 blkIdx, TrafoDepth, MaxTrafoDepth,
                                 IntraSplitFlag, qp);
  }
}

