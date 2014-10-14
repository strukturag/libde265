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


#include "libde265/analyze.h"
#include "libde265/encoder-context.h"
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


static bool has_nonzero_value(const int16_t* data, int n)
{
  for (int i=0;i<n;i++)
    if (data[i]) return true;

  return false;
}


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

  enum IntraPredMode intraPredMode  = ectx->img.get_IntraPredMode(x0,y0);

  if (cIdx>0) {
    intraPredMode = cb->intra.chroma_mode; //lumaPredMode_to_chromaPredMode(intraPredMode,
    //cb->intra.chroma_mode);

    xC >>= 1;
    yC >>= 1;
  }
  
  decode_intra_prediction(&ectx->img, xC,  yC,   intraPredMode,  tbSize  , cIdx);


  // --- subtract prediction from input ---

  int16_t blk[32*32];
  uint8_t* pred = ectx->img.get_image_plane(cIdx);
  int stride = ectx->img.get_image_stride(cIdx);

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

  de265_image* img = &ectx->img;

  int stride = ectx->img.get_image_stride(0);

  uint8_t* luma_plane = ectx->img.get_image_plane(0);
  uint8_t* cb_plane = ectx->img.get_image_plane(1);
  uint8_t* cr_plane = ectx->img.get_image_plane(2);

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
                     ectx->img.get_image_plane_at_pos(0, x0,y0),
                     ectx->img.get_image_stride(0),
                     1<<log2TbSize, 1<<log2TbSize);

    int16_t coeffs[64];
    int16_t diff[64];

    diff_blk(diff,8,
             input->get_image_plane_at_pos(0, x0,y0), input->get_image_stride(0),
             ectx->img.get_image_plane_at_pos(0, x0,y0), ectx->img.get_image_stride(0),
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

  tb->reconstruct(&ectx->accel, &ectx->img, x0,y0, xBase,yBase, cb, qp, blkIdx);



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
                 ectx->img.get_image_plane_at_pos(0, x0,y0),
                 ectx->img.get_image_stride(0),
                 1<<log2BlkSize, 1<<log2BlkSize);
      break;

    case TBBitrateEstim_SAD:
      return SAD(input->get_image_plane_at_pos(0, x0,y0),
                 input->get_image_stride(0),
                 ectx->img.get_image_plane_at_pos(0, x0,y0),
                 ectx->img.get_image_stride(0),
                 1<<log2BlkSize, 1<<log2BlkSize);
      break;

    case TBBitrateEstim_SATD_DCT:
    case TBBitrateEstim_SATD_Hadamard:
      {
        int16_t coeffs[32*32];
        int16_t diff[32*32];

        diff_blk(diff,blkSize,
                 input->get_image_plane_at_pos(0, x0,y0), input->get_image_stride(0),
                 ectx->img.get_image_plane_at_pos(0, x0,y0), ectx->img.get_image_stride(0),
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


const enc_tb* Algo_TB_Split::encode_transform_tree_split(encoder_context* ectx,
                                                         context_model_table ctxModel,
                                                         const de265_image* input,
                                                         const enc_tb* parent,
                                                         enc_cb* cb,
                                                         int x0,int y0, int log2TbSize,
                                                         int TrafoDepth, int MaxTrafoDepth,
                                                         int IntraSplitFlag,
                                                         int qp)
{
  const de265_image* img = &ectx->img;

  enc_tb* tb = ectx->enc_tb_pool.get_new();

  tb->parent = parent;
  tb->split_transform_flag = true;
  tb->log2TbSize = log2TbSize;

  tb->rate = 0;
  tb->distortion = 0;


  // --- encode all child nodes ---

  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (log2TbSize-1);
    int dy = (i>>1) << (log2TbSize-1);

    tb->children[i] = mAlgo_TB_IntraPredMode->analyze(ectx, ctxModel, input, tb, cb,
                                                      x0+dx, y0+dy, x0,y0,
                                                      log2TbSize-1, i,
                                                      TrafoDepth+1, MaxTrafoDepth, IntraSplitFlag,
                                                      qp);

    tb->distortion += tb->children[i]->distortion;
    tb->rate       += tb->children[i]->rate;
  }  

  tb->set_cbf_flags_from_children();


  // --- add rate for this TB level ---

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);

  encode_transform_tree(ectx, tb, cb,
                        x0,y0, x0,y0,
                        log2TbSize, TrafoDepth, 0 /* blkIdx */,
                        MaxTrafoDepth, IntraSplitFlag, false);

  tb->rate += estim.getRDBits();

  return tb;
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

    const de265_image& img = ectx->img;
    const seq_parameter_set* sps = &img.sps;
    int candidates[3];
    fillIntraPredModeCandidates(candidates, x0,y0,
                                sps->getPUIndexRS(x0,y0),
                                x0>0, y0>0, &img);


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

      ectx->img.set_IntraPredMode(x0,y0,log2TbSize, intraMode);

      tb[intraMode] = mTBSplitAlgo->analyze(ectx,ctxIntra,input,parent,
                                            cb, x0,y0, xBase,yBase, log2TbSize, blkIdx,
                                            TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                            qp);


      float sad;
      if ((1<<log2TbSize)==8) {
        decode_intra_prediction(&ectx->img, x0,y0, intraMode, 1<<log2TbSize, 0);
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
    ectx->img.set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    tb[minCostIdx]->reconstruct(&ectx->accel,
                                &ectx->img, x0,y0, xBase,yBase,
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
      decode_intra_prediction(&ectx->img, x0,y0, (enum IntraPredMode)mode, 1<<log2TbSize, 0);

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

    ectx->img.set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    const enc_tb* tb = mTBSplitAlgo->analyze(ectx,ctxModel,input,parent,
                                             cb, x0,y0, xBase,yBase, log2TbSize, blkIdx,
                                             TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                             qp);

    tb->reconstruct(&ectx->accel,
                    &ectx->img, x0,y0, xBase,yBase,
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

    const de265_image& img = ectx->img;
    const seq_parameter_set* sps = &img.sps;
    int candidates[3];
    fillIntraPredModeCandidates(candidates, x0,y0,
                                sps->getPUIndexRS(x0,y0),
                                x0>0, y0>0, &img);



    std::vector< std::pair<enum IntraPredMode,float> > distortions;

    for (int idx=0;idx<35;idx++)
      if (idx!=candidates[0] && idx!=candidates[1] && idx!=candidates[2] && mPredMode_enabled[idx])
        {
          enum IntraPredMode mode = (enum IntraPredMode)idx;
          decode_intra_prediction(&ectx->img, x0,y0, (enum IntraPredMode)mode, 1<<log2TbSize, 0);
          
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

      ectx->img.set_IntraPredMode(x0,y0,log2TbSize, intraMode);

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
    ectx->img.set_IntraPredMode(x0,y0,log2TbSize, intraMode);

    tb[minCostIdx]->reconstruct(&ectx->accel,
                                &ectx->img, x0,y0, xBase,yBase,
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



class Logging
{
public:
  virtual ~Logging() { }

  static void print_logging(const encoder_context* ectx, const char* id, const char* filename);

  virtual const char* name() const = 0;
  virtual void print(const encoder_context* ectx, const char* filename) = 0;
};


void en265_print_logging(const encoder_context* ectx, const char* id, const char* filename)
{
  Logging::print_logging(ectx,id,filename);
}


struct Logging_TB_Split : public Logging
{
  int skipTBSplit, noskipTBSplit;
  int zeroBlockCorrelation[6][2][5];

  const char* name() const { return "tb-split"; }

  void print(const encoder_context* ectx, const char* filename)
  {
    printf("%d %d\n\n",skipTBSplit, noskipTBSplit);

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


void Logging::print_logging(const encoder_context* ectx, const char* id, const char* filename)
{
  if (strcmp(id,logging_tb_split.name())==0) {
    logging_tb_split.print(ectx,filename);
  }
}


const enc_tb*
Algo_TB_Split_BruteForce::analyze(encoder_context* ectx,
                                  context_model_table ctxModel,
                                  const de265_image* input,
                                  const enc_tb* parent,
                                  enc_cb* cb,
                                  int x0,int y0, int xBase,int yBase, int log2TbSize,
                                  int blkIdx,
                                  int TrafoDepth, int MaxTrafoDepth,
                                  int IntraSplitFlag,
                                  int qp)
{
  bool test_split = (log2TbSize > 2 &&
                     TrafoDepth < MaxTrafoDepth &&
                     log2TbSize > ectx->sps.Log2MinTrafoSize);

  bool test_no_split = (IntraSplitFlag==0 || TrafoDepth>0);

  context_model_table ctxSplit;
  if (test_split) {
    copy_context_model_table(ctxSplit, ctxModel);
  }


  /*
    printf("log2TbSize:%d TrafoDepth:%d MaxTrafoDepth:%d log2TbSize:%d MinTrafoSize:%d\n",
    log2TbSize,
    TrafoDepth,
    MaxTrafoDepth,
    log2TbSize,
    ectx->sps.Log2MinTrafoSize);
    printf("  intra split flag: %d\n",IntraSplitFlag);
  */
  const enc_tb* tb_no_split = NULL;
  const enc_tb* tb_split    = NULL;
  float rd_cost_no_split = std::numeric_limits<float>::max();
  float rd_cost_split    = std::numeric_limits<float>::max();

  if (test_no_split) {
    //printf("test no split\n");
    tb_no_split = encode_transform_tree_no_split(ectx, ctxModel, input, parent,
                                                 cb, x0,y0, xBase,yBase, log2TbSize,
                                                 blkIdx,
                                                 TrafoDepth,MaxTrafoDepth,IntraSplitFlag,
                                                 qp);

    rd_cost_no_split = tb_no_split->distortion + ectx->lambda * tb_no_split->rate;
    //printf("-\n");

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
    //printf("test split\n");
    tb_split = encode_transform_tree_split(ectx, ctxSplit, input, parent, cb,
                                           x0,y0, log2TbSize,
                                           TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                           qp);
    
    rd_cost_split    = tb_split->distortion    + ectx->lambda * tb_split->rate;
    //printf("-\n");
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
    assert(tb_split);
    return tb_split;
  }
  else {
    assert(tb_no_split);
    tb_no_split->reconstruct(&ectx->accel,
                             &ectx->img, x0,y0, xBase,yBase,
                             cb, qp, blkIdx);

    return tb_no_split;
  }
}


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


  const bool can_use_NxN = (log2CbSize == ectx->sps.Log2MinCbSizeY);

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

    ectx->img.set_pred_mode(x0,y0, log2CbSize, cb[p]->PredMode);
    ectx->img.set_PartMode (x0,y0, cb[p]->PartMode);  // TODO: probably unnecessary


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

    cb[p]->write_to_image(&ectx->img, x0,y0, true);
    cb[p]->distortion = compute_distortion_ssd(&ectx->img, input, x0,y0, log2CbSize, 0);
  }


  // choose from 2Nx2N and NxN

  if (cb[0] && cb[1]) {
    double rd_cost_2Nx2N = cb[0]->distortion + ectx->lambda * cb[0]->rate;
    double rd_cost_NxN   = cb[1]->distortion + ectx->lambda * cb[1]->rate;

    if (rd_cost_2Nx2N < rd_cost_NxN) {
      cb[0]->write_to_image(&ectx->img, x0,y0, true);
      cb[0]->reconstruct(&ectx->accel, &ectx->img, x0,y0, qp);
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

  ectx->img.set_pred_mode(x0,y0, log2CbSize, cb->PredMode);
  ectx->img.set_PartMode (x0,y0, cb->PartMode);  // TODO: probably unnecessary


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

    cb_no_split->write_to_image(&ectx->img, x0,y0, true);
    cb_no_split->reconstruct(&ectx->accel, &ectx->img, x0,y0, qp);
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
      fillIntraPredModeCandidates(candModeList,xi,yi, xi>0, yi>0, &ectx->img);

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

  ectx->img.alloc_image(w,h, de265_chroma_420, &ectx->sps, true,
                        NULL /* no decctx */, 0,NULL,false);
  ectx->img.alloc_encoder_data(&ectx->sps);
  ectx->img.clear_metadata();

  initialize_CABAC_models(ectx->ctx_model, ectx->shdr.initType, ectx->shdr.SliceQPY);

  int Log2CtbSize = ectx->sps.Log2CtbSizeY;

  uint8_t* luma_plane = ectx->img.get_image_plane(0);
  uint8_t* cb_plane   = ectx->img.get_image_plane(1);
  uint8_t* cr_plane   = ectx->img.get_image_plane(2);


  // encode CTB by CTB

  for (int y=0;y<ectx->sps.PicHeightInCtbsY;y++)
    for (int x=0;x<ectx->sps.PicWidthInCtbsY;x++)
      {
        ectx->img.set_SliceAddrRS(x, y, ectx->shdr.SliceAddrRS);

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


        cb->write_to_image(&ectx->img, x<<Log2CtbSize, y<<Log2CtbSize, true);


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
                         luma_plane, ectx->img.get_image_stride(0),
                         input->get_width(), input->get_height()));
  return psnr;
}


void encode_sequence(encoder_context* ectx)
{
  EncodingAlgorithm_Custom algo;
  algo.setParams(ectx->params);

  // TODO: must be <30, because Y->C mapping (tab8_22) is not implemented yet
  int qp = algo.getPPS_QP();

  //lambda = ectx->params.lambda;
  ectx->lambda = 0.0242 * pow(1.27245, qp);


  nal_header nal;

  // VPS

  ectx->vps.set_defaults(Profile_Main, 6,2);


  // SPS

  ectx->sps.set_defaults();
  ectx->sps.set_CB_log2size_range( Log2(ectx->params.min_cb_size), Log2(ectx->params.max_cb_size));
  ectx->sps.set_TB_log2size_range( Log2(ectx->params.min_tb_size), Log2(ectx->params.max_tb_size));
  ectx->sps.max_transform_hierarchy_depth_intra = ectx->params.max_transform_hierarchy_depth_intra;

  ectx->sps.set_resolution(ectx->img_source->get_width(),
                           ectx->img_source->get_height());
  ectx->sps.compute_derived_values();

  // PPS

  ectx->pps.set_defaults();
  ectx->pps.pic_init_qp = qp;

  // turn off deblocking filter
  ectx->pps.deblocking_filter_control_present_flag = true;
  ectx->pps.deblocking_filter_override_enabled_flag = false;
  ectx->pps.pic_disable_deblocking_filter_flag = true;
  ectx->pps.pps_loop_filter_across_slices_enabled_flag = false;

  ectx->pps.set_derived_values(&ectx->sps);


  // slice

  ectx->shdr.set_defaults(&ectx->pps);
  ectx->shdr.slice_deblocking_filter_disabled_flag = true;
  ectx->shdr.slice_loop_filter_across_slices_enabled_flag = false;

  ectx->img.vps  = ectx->vps;
  ectx->img.sps  = ectx->sps;
  ectx->img.pps  = ectx->pps;



  // write headers

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(ectx->cabac);
  ectx->vps.write(&ectx->errqueue, ectx->cabac);
  ectx->cabac->flush_VLC();
  ectx->write_packet();

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(ectx->cabac);
  ectx->sps.write(&ectx->errqueue, ectx->cabac);
  ectx->cabac->flush_VLC();
  ectx->write_packet();

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(ectx->cabac);
  ectx->pps.write(&ectx->errqueue, ectx->cabac, &ectx->sps);
  ectx->cabac->flush_VLC();
  ectx->write_packet();

  ectx->img_source->skip_frames( ectx->params.first_frame );

  int maxPoc = ectx->params.max_number_of_frames;
  for (int poc=0; poc<maxPoc ;poc++)
    {
      fprintf(stderr,"encoding frame %d\n",poc);

      de265_image* input_image = ectx->img_source->get_image();
      if (input_image==NULL) { break; } // EOF


      // write slice header

      //shdr.slice_pic_order_cnt_lsb = poc & 0xFF;

      nal.set(NAL_UNIT_IDR_W_RADL);
      nal.write(ectx->cabac);
      ectx->shdr.write(&ectx->errqueue, ectx->cabac, &ectx->sps, &ectx->pps, nal.nal_unit_type);
      ectx->cabac->skip_bits(1);
      ectx->cabac->flush_VLC();

      ectx->cabac->init_CABAC();
      double psnr = encode_image(ectx,input_image, algo);
      fprintf(stderr,"  PSNR-Y: %f\n", psnr);
      ectx->cabac->flush_CABAC();
      ectx->write_packet();


      // --- write reconstruction ---

      if (ectx->reconstruction_sink) {
        ectx->reconstruction_sink->send_image(&ectx->img);
      }


      // --- release input image ---

      delete input_image;
    }
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
  mAlgo_TB_Split_BruteForce.setParams(params.TB_Split_BruteForce);

  algo_TB_IntraPredMode->setChildAlgo(&mAlgo_TB_Split_BruteForce);


  // ===== set algorithm parameters ======

  mAlgo_CB_IntraPartMode_Fixed.setParams(params.CB_IntraPartMode_Fixed);

  mAlgo_TB_IntraPredMode_FastBrute.setParams(params.TB_IntraPredMode_FastBrute);
  mAlgo_TB_IntraPredMode_MinResidual.setParams(params.TB_IntraPredMode_MinResidual);


  mAlgo_CTB_QScale_Constant.setParams(params.CTB_QScale_Constant);


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
