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

#include "encode.h"
#include "slice.h"
#include "intrapred.h"


void enc_cb::write_to_image(de265_image* img, int x,int y,int log2blkSize, bool intra)
{
  if (!split_cu_flag) {
    if (intra) {
      img->set_IntraPredMode(x,y,log2blkSize, intra_pb[0]->pred_mode);
      //img->set_IntraChromaPredMode(x,y,log2blkSize, intra_pb[0]->pred_mode_chroma);

      if (PartMode == PART_NxN) {
        int h = 1<<(log2blkSize-1);
        img->set_IntraPredMode(x+h,y  ,log2blkSize-1, intra_pb[1]->pred_mode);
        img->set_IntraPredMode(x  ,y+h,log2blkSize-1, intra_pb[2]->pred_mode);
        img->set_IntraPredMode(x+h,y+h,log2blkSize-1, intra_pb[3]->pred_mode);
      }
    }
    else {
      assert(0); // TODO: inter mode
    }
  }
}


static void encode_split_cu_flag(encoder_context* ectx,
                                 int x0, int y0, int ctDepth, int split_flag)
{
  // check if neighbors are available

  int availableL = check_CTB_available(ectx->img,ectx->shdr, x0,y0, x0-1,y0);
  int availableA = check_CTB_available(ectx->img,ectx->shdr, x0,y0, x0,y0-1);

  int condL = 0;
  int condA = 0;

  if (availableL && ectx->img->get_ctDepth(x0-1,y0) > ctDepth) condL=1;
  if (availableA && ectx->img->get_ctDepth(x0,y0-1) > ctDepth) condA=1;

  int contextOffset = condL + condA;
  int context = contextOffset;

  // decode bit

  logtrace(LogSlice,"> split_cu_flag = %d\n",split_flag);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_SPLIT_CU_FLAG + context], split_flag);
}


static void encode_part_mode(encoder_context* ectx,
                             enum PredMode PredMode, enum PartMode PartMode)
{
  logtrace(LogSlice,"> part_mode = %d\n",PartMode);

  if (PredMode == MODE_INTRA) {
    int bin = (PartMode==PART_2Nx2N);
    ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_PART_MODE], bin);
  }
  else {
    assert(0); // TODO
  }
}


static void encode_prev_intra_luma_pred_flag(encoder_context* ectx, int intraPred)
{
  int bin = (intraPred>=0);

  logtrace(LogSlice,"> prev_intra_luma_pred_flag = %d\n",bin);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG], bin);
}

static void encode_intra_mpm_or_rem(encoder_context* ectx, int intraPred)
{
  if (intraPred>=0) {
    logtrace(LogSlice,"> mpm_idx = %d\n",intraPred);
    ectx->cabac_encoder->write_CABAC_TU_bypass(intraPred, 2);
  }
  else {
    logtrace(LogSlice,"> rem_intra_luma_pred_mode = %d\n",-intraPred-1);
    ectx->cabac_encoder->write_CABAC_FL_bypass(-intraPred-1, 5);
  }
}


static void encode_intra_chroma_pred_mode(encoder_context* ectx, int mode)
{
  logtrace(LogSlice,"> intra_chroma_pred_mode = %d\n",mode);

  if (mode==4) {
    ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE],0);
  }
  else {
    assert(mode<4);

    ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE],1);
    ectx->cabac_encoder->write_CABAC_FL_bypass(mode, 2);
  }
}


/* Optimized variant that tests most likely branch first.
 */
enum IntraChromaPredMode find_chroma_pred_mode(enum IntraPredMode chroma_mode,
                                               enum IntraPredMode luma_mode)
{
  // most likely mode: chroma mode = luma mode

  if (luma_mode==chroma_mode) {
    return INTRA_CHROMA_LIKE_LUMA;
  }


  // check remaining candidates

  IntraPredMode mode = chroma_mode;

  // angular-34 is coded by setting the coded mode equal to the luma_mode
  if (chroma_mode == INTRA_ANGULAR_34) {
    mode = luma_mode;
  }

  switch (mode) {
  case INTRA_PLANAR:     return INTRA_CHROMA_PLANAR_OR_34;
  case INTRA_ANGULAR_26: return INTRA_CHROMA_ANGULAR_26_OR_34;
  case INTRA_ANGULAR_10: return INTRA_CHROMA_ANGULAR_10_OR_34;
  case INTRA_DC:         return INTRA_CHROMA_DC_OR_34;
  }

  assert(false);
}



static void encode_split_transform_flag(encoder_context* ectx, int log2TrafoSize, int split_flag)
{
  logtrace(LogSlice,"> split_transform_flag = %d\n",split_flag);

  int context = 5-log2TrafoSize;
  assert(context >= 0 && context <= 2);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_SPLIT_TRANSFORM_FLAG + context], split_flag);
}


static void encode_cbf_luma(encoder_context* ectx, bool zeroTrafoDepth, int cbf_luma)
{
  logtrace(LogSlice,"> cbf_luma = %d\n",cbf_luma);

  int context = (zeroTrafoDepth ? 1 : 0);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_CBF_LUMA + context],
                                       cbf_luma);
}


static void encode_cbf_chroma(encoder_context* ectx, int trafoDepth, int cbf_chroma)
{
  logtrace(LogSlice,"> cbf_chroma = %d\n",cbf_chroma);

  int context = trafoDepth;
  assert(context >= 0 && context <= 3);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_CBF_CHROMA + context],
                                       cbf_chroma);
}

// ---------------------------------------------------------------------------

void findLastSignificantCoeff(int16_t* coeff, int log2TrafoSize,
                              int* lastSignificantX, int* lastSignificantY)
{
  int n = (1<<(log2TrafoSize<<1));

  for (int i=n ;n-->0 ;) {
    if (coeff[i] != 0) {
      *lastSignificantX = i & ((1<<log2TrafoSize)-1);
      *lastSignificantY = i >> log2TrafoSize;
      return;
    }
  }

  // all coefficients == 0 ? cannot be since cbf should be false in this case
  assert(false);
}


void encode_residual(encoder_context* ectx, const enc_tb* tb,
                     int xBase,int yBase,int log2TrafoSize,int cIdx)
{
  const seq_parameter_set& sps = ectx->img->sps;
  const pic_parameter_set& pps = ectx->img->pps;

  if (pps.transform_skip_enabled_flag && 1 /* TODO */) {
  }

  int lastSignificantX, lastSignificantY;
  findLastSignificantCoeff(tb->coeff[blkIdx], log2TrafoSize,
                           &lastSignificantX, &lastSignificantY);

  encode_last_signficiant_coeff_prefix(ectx, log2TrafoSize, cIdx, lastSignificantX);
  encode_last_signficiant_coeff_prefix(ectx, log2TrafoSize, cIdx, lastSignificantY);
}


void encode_transform_unit(encoder_context* ectx, const enc_tb* tb,
                           int x0,int y0, int xBase,int yBase,
                           int log2TrafoSize, int trafoDepth, int blkIdx)
{
  if (tb->cbf_luma || tb->cbf_cb || tb->cbf_cr) {
    if (ectx->img->pps.cu_qp_delta_enabled_flag &&
        1 /*!ectx->IsCuQpDeltaCoded*/) {
      assert(0);
    }

    if (tb->cbf_luma) {
      encode_residual(ectx,tb,x0,y0,log2TrafoSize,0);
    }

    // larger than 4x4
    if (log2TrafoSize>2) {
      if (tb->cbf_cb) {
        encode_residual(ectx,tb,x0,y0,log2TrafoSize-1,1);
      }
      if (tb->cbf_cr) {
        encode_residual(ectx,tb,x0,y0,log2TrafoSize-1,2);
      }
    }
    else if (blkIdx==3) {
      if (tb->parent->cbf_cb) {
        encode_residual(ectx,tb,xBase,yBase,log2TrafoSize,1);
      }
      if (tb->parent->cbf_cr) {
        encode_residual(ectx,tb,xBase,yBase,log2TrafoSize,2);
      }
    }
  }
}


void encode_transform_tree(encoder_context* ectx, const enc_tb* tb, const enc_cb* cb,
                           int x0,int y0, int xBase,int yBase,
                           int log2TrafoSize, int trafoDepth, int blkIdx,
                           int MaxTrafoDepth, int IntraSplitFlag)
{
  //de265_image* img = ectx->img;
  const seq_parameter_set* sps = &ectx->img->sps;

  if (log2TrafoSize <= sps->Log2MaxTrafoSize &&
      log2TrafoSize >  sps->Log2MinTrafoSize &&
      trafoDepth < MaxTrafoDepth &&
      !(IntraSplitFlag && trafoDepth==0))
    {
      int split_transform_flag = tb->split_transform_flag;
      encode_split_transform_flag(ectx, log2TrafoSize, split_transform_flag);
    }
  else
    {
      int interSplitFlag=0; // TODO

      bool split_transform_flag = (log2TrafoSize > sps->Log2MaxTrafoSize ||
                                   (IntraSplitFlag==1 && trafoDepth==0) ||
                                   interSplitFlag==1) ? 1:0;

      assert(tb->split_transform_flag == split_transform_flag);
    }

  // --- CBF CB/CR ---

  // For 4x4 luma, there is no signaling of chroma CBF, because only the
  // chroma CBF for 8x8 is relevant.
  if (log2TrafoSize>2) {
    if (trafoDepth==0 || tb->parent->cbf_cb) {
      encode_cbf_chroma(ectx, trafoDepth, tb->cbf_cb);
    }
    if (trafoDepth==0 || tb->parent->cbf_cr) {
      encode_cbf_chroma(ectx, trafoDepth, tb->cbf_cr);
    }
  }

  if (tb->split_transform_flag) {
    assert(0); // TODO
  }
  else {
    if (cb->PredMode == MODE_INTRA || trafoDepth != 0 ||
        tb->cbf_cb || tb->cbf_cr) {
      encode_cbf_luma(ectx, trafoDepth==0, tb->cbf_luma);
    }
    else {
      assert(tb->cbf_luma==true);
    }

    encode_transform_unit(ectx, tb, x0,y0, xBase,yBase, log2TrafoSize, trafoDepth, blkIdx);
  }
}


void encode_coding_unit(encoder_context* ectx,
                        const enc_cb* cb, int x0,int y0, int log2CbSize)
{
  de265_image* img = ectx->img;
  const slice_segment_header* shdr = ectx->shdr;
  const seq_parameter_set* sps = &ectx->img->sps;


  int nCbS = 1<<log2CbSize;

  enum PredMode PredMode = cb->PredMode;
  enum PartMode PartMode = PART_2Nx2N;
  int IntraSplitFlag=0;

  if (PredMode != MODE_INTRA ||
      log2CbSize == sps->Log2MinCbSizeY) {
    PartMode = cb->PartMode;
    encode_part_mode(ectx, PredMode, PartMode);
  }

  if (PredMode == MODE_INTRA) {

    int availableA0 = check_CTB_available(img, shdr, x0,y0, x0-1,y0);
    int availableB0 = check_CTB_available(img, shdr, x0,y0, x0,y0-1);

    if (PartMode==PART_2Nx2N) {
      int PUidx = (x0>>sps->Log2MinPUSize) + (y0>>sps->Log2MinPUSize)*sps->PicWidthInMinPUs;

      int candModeList[3];
      fillIntraPredModeCandidates(candModeList,x0,y0,PUidx,
                                  availableA0,availableB0, img);

      enum IntraPredMode mode = cb->intra_pb[0]->pred_mode;
      int intraPred = find_intra_pred_mode(mode, candModeList);
      encode_prev_intra_luma_pred_flag(ectx, intraPred);
      encode_intra_mpm_or_rem(ectx, intraPred);
    }
    else {
      IntraSplitFlag=1;

      int pbOffset = nCbS/2;
      int PUidx;

      int intraPred[4];
      int childIdx=0;

      for (int j=0;j<nCbS;j+=pbOffset)
        for (int i=0;i<nCbS;i+=pbOffset, childIdx++)
          {
            int x=x0+i, y=y0+j;

            int availableA = availableA0 || (i>0); // left candidate always available for right blk
            int availableB = availableB0 || (j>0); // top candidate always available for bottom blk

            PUidx = (x>>sps->Log2MinPUSize) + (y>>sps->Log2MinPUSize)*sps->PicWidthInMinPUs;

            int candModeList[3];
            fillIntraPredModeCandidates(candModeList,x,y,PUidx,
                                        availableA,availableB, img);

            enum IntraPredMode mode = cb->intra_pb[childIdx]->pred_mode;
            intraPred[2*j+i] = find_intra_pred_mode(mode, candModeList);
          }

      for (int i=0;i<4;i++)
        encode_prev_intra_luma_pred_flag(ectx, intraPred[i]);

      for (int i=0;i<4;i++)
        encode_intra_mpm_or_rem(ectx, intraPred[i]);
    }
    
    encode_intra_chroma_pred_mode(ectx,
                                  find_chroma_pred_mode(cb->intra_pb[0]->pred_mode_chroma,
                                                        cb->intra_pb[0]->pred_mode));
  }


  int MaxTrafoDepth;
  if (PredMode == MODE_INTRA)
    { MaxTrafoDepth = sps->max_transform_hierarchy_depth_intra + IntraSplitFlag; }
  else 
    { MaxTrafoDepth = sps->max_transform_hierarchy_depth_inter; }


  encode_transform_tree(ectx, cb->transform_tree, cb,
                        x0,y0, x0,y0, log2CbSize, 0, 0, MaxTrafoDepth, IntraSplitFlag);
}


void encode_quadtree(encoder_context* ectx,
                     const enc_cb* cb, int x0,int y0, int log2CbSize, int ctDepth)
{
  //de265_image* img = ectx->img;
  const seq_parameter_set* sps = &ectx->img->sps;

  int split_flag;

  /*
     CU split flag:

          | overlaps | minimum ||
     case | border   | size    ||  split
     -----+----------+---------++----------
       A  |    0     |     0   || optional
       B  |    0     |     1   ||    0
       C  |    1     |     0   ||    1
       D  |    1     |     1   ||    0
   */
  if (x0+(1<<log2CbSize) <= sps->pic_width_in_luma_samples &&
      y0+(1<<log2CbSize) <= sps->pic_height_in_luma_samples &&
      log2CbSize > sps->Log2MinCbSizeY) {

    // case A

    split_flag = cb->split_cu_flag;

    encode_split_cu_flag(ectx, x0,y0, ctDepth, split_flag);
  } else {
    // case B/C/D

    if (log2CbSize > sps->Log2MinCbSizeY) { split_flag=1; }
    else                                  { split_flag=0; }
  }



  if (split_flag) {
    int x1 = x0 + (1<<(log2CbSize-1));
    int y1 = y0 + (1<<(log2CbSize-1));

    encode_quadtree(ectx, cb->children[0], x0,y0, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples)
      encode_quadtree(ectx, cb->children[1], x1,y0, log2CbSize-1, ctDepth+1);

    if (y1<sps->pic_height_in_luma_samples)
      encode_quadtree(ectx, cb->children[2], x0,y1, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples &&
        y1<sps->pic_height_in_luma_samples)
      encode_quadtree(ectx, cb->children[3], x1,y1, log2CbSize-1, ctDepth+1);
  }
  else {
    encode_coding_unit(ectx, cb,x0,y0, log2CbSize);
  }
}


void encode_ctb(encoder_context* ectx, enc_cb* cb, int ctbX,int ctbY)
{
  de265_image* img = ectx->img;
  int log2ctbSize = img->sps.Log2CtbSizeY;

  encode_quadtree(ectx, cb, ctbX,ctbY, log2ctbSize, 0);
}


// ---------------------------------------------------------------------------

/*
void encode_transform_tree(encoder_context* ectx,
                           int x0,int y0, int xBase,int yBase,
                           int log2TrafoSize, int trafoDepth, int blkIdx,
                           int MaxTrafoDepth, int IntraSplitFlag)
{
  de265_image* img = ectx->img;
  const seq_parameter_set* sps = &img->sps;

  if (log2TrafoSize <= sps->Log2MaxTrafoSize &&
      log2TrafoSize >  sps->Log2MinTrafoSize &&
      trafoDepth < MaxTrafoDepth &&
      !(IntraSplitFlag && trafoDepth==0))
    {
      int split_transform_flag = !!img->get_split_transform_flag(x0,y0, trafoDepth);
      encode_split_transform_flag(ectx, log2TrafoSize, split_transform_flag);
    }
  else
    {
      int interSplitFlag=0; // TODO

      bool split_transform_flag = (log2TrafoSize > sps->Log2MaxTrafoSize ||
                                   (IntraSplitFlag==1 && trafoDepth==0) ||
                                   interSplitFlag==1) ? 1:0;

      assert(img->get_split_transform_flag(x0,y0, trafoDepth) == split_transform_flag);
    }
}


void encode_coding_unit(encoder_context* ectx,
                        int x0,int y0, int log2CbSize)
{
  de265_image* img = ectx->img;
  const slice_segment_header* shdr = ectx->shdr;
  const seq_parameter_set* sps = &img->sps;


  int nCbS = 1<<log2CbSize;

  enum PredMode PredMode = img->get_pred_mode(x0,y0);
  enum PartMode PartMode = PART_2Nx2N;
  int IntraSplitFlag=0;

  if (PredMode != MODE_INTRA ||
      log2CbSize == sps->Log2MinCbSizeY) {
    PartMode = img->get_PartMode(x0,y0);
    encode_part_mode(ectx, PredMode, PartMode);
  }

  if (PredMode == MODE_INTRA) {

    int availableA0 = check_CTB_available(img, shdr, x0,y0, x0-1,y0);
    int availableB0 = check_CTB_available(img, shdr, x0,y0, x0,y0-1);

    if (PartMode==PART_2Nx2N) {
      int PUidx = (x0>>sps->Log2MinPUSize) + (y0>>sps->Log2MinPUSize)*sps->PicWidthInMinPUs;

      int candModeList[3];
      fillIntraPredModeCandidates(candModeList,x0,y0,PUidx,
                                  availableA0,availableB0, img);

      enum IntraPredMode mode = img->get_IntraPredMode(x0,y0);
      int intraPred = find_intra_pred_mode(mode, candModeList);
      encode_prev_intra_luma_pred_flag(ectx, intraPred);
      encode_intra_mpm_or_rem(ectx, intraPred);
    }
    else {
      IntraSplitFlag=1;

      int pbOffset = nCbS/2;
      int PUidx;

      int intraPred[4];

      for (int j=0;j<nCbS;j+=pbOffset)
        for (int i=0;i<nCbS;i+=pbOffset)
          {
            int x=x0+i, y=y0+j;

            int availableA = availableA0 || (i>0); // left candidate always available for right blk
            int availableB = availableB0 || (j>0); // top candidate always available for bottom blk

            PUidx = (x>>sps->Log2MinPUSize) + (y>>sps->Log2MinPUSize)*sps->PicWidthInMinPUs;

            int candModeList[3];
            fillIntraPredModeCandidates(candModeList,x,y,PUidx,
                                        availableA,availableB, img);

            enum IntraPredMode mode = img->get_IntraPredMode(x,y);
            intraPred[2*j+i] = find_intra_pred_mode(mode, candModeList);
          }

      for (int i=0;i<4;i++)
        encode_prev_intra_luma_pred_flag(ectx, intraPred[i]);

      for (int i=0;i<4;i++)
        encode_intra_mpm_or_rem(ectx, intraPred[i]);
    }
    
    encode_intra_chroma_pred_mode(ectx, img->get_IntraChromaPredMode(x0,y0));
  }


  int MaxTrafoDepth;
  if (PredMode == MODE_INTRA)
    { MaxTrafoDepth = sps->max_transform_hierarchy_depth_intra + IntraSplitFlag; }
  else 
    { MaxTrafoDepth = sps->max_transform_hierarchy_depth_inter; }

  encode_transform_tree(ectx, x0,y0, x0,y0, log2CbSize, 0, 0, MaxTrafoDepth, IntraSplitFlag);
}
*/

#if 0
void encode_quadtree(encoder_context* ectx,
                     int x0,int y0, int log2CbSize, int ctDepth)
{
  de265_image* img = ectx->img;
  const seq_parameter_set* sps = &img->sps;

  int split_flag;

  /*
     CU split flag:

          | overlaps | minimum ||
     case | border   | size    ||  split
     -----+----------+---------++----------
       A  |    0     |     0   || optional
       B  |    0     |     1   ||    0
       C  |    1     |     0   ||    1
       D  |    1     |     1   ||    0
   */
  if (x0+(1<<log2CbSize) <= sps->pic_width_in_luma_samples &&
      y0+(1<<log2CbSize) <= sps->pic_height_in_luma_samples &&
      log2CbSize > sps->Log2MinCbSizeY) {

    // case A

    split_flag = (img->get_ctDepth(x0,y0) != ctDepth);

    encode_split_cu_flag(ectx, x0,y0, ctDepth, split_flag);
  } else {
    // case B/C/D

    if (log2CbSize > sps->Log2MinCbSizeY) { split_flag=1; }
    else                                  { split_flag=0; }
  }



  if (split_flag) {
    int x1 = x0 + (1<<(log2CbSize-1));
    int y1 = y0 + (1<<(log2CbSize-1));

    encode_quadtree(ectx,x0,y0, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples)
      encode_quadtree(ectx,x1,y0, log2CbSize-1, ctDepth+1);

    if (y1<sps->pic_height_in_luma_samples)
      encode_quadtree(ectx,x0,y1, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples &&
        y1<sps->pic_height_in_luma_samples)
      encode_quadtree(ectx,x1,y1, log2CbSize-1, ctDepth+1);
  }
  else {
    encode_coding_unit(ectx,x0,y0, log2CbSize);
  }
}


void encode_image(encoder_context* ectx)
{
  de265_image* img = ectx->img;
  int log2ctbSize = img->sps.Log2CtbSizeY;

  for (int ctbY=0;ctbY<img->sps.PicHeightInCtbsY;ctbY++)
    for (int ctbX=0;ctbX<img->sps.PicWidthInCtbsY;ctbX++)
      {
        encode_quadtree(ectx, ctbX,ctbY, log2ctbSize, 0);
      }
}
#endif
