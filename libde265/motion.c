/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "motion.h"
#include "motion_func.h"
#include "decctx.h"
#include "util.h"
#include <assert.h>


#include <sys/types.h>
#include <signal.h>
#include <string.h>


enum {
  // important! order like shown in 8.5.3.1.1
  PRED_A1  = 0,
  PRED_B1  = 1,
  PRED_B0  = 2,
  PRED_A0  = 3,
  PRED_B2  = 4,
  PRED_COL = 5,
  PRED_ZERO= 6
};


typedef struct
{
  uint8_t available[7];
  PredVectorInfo pred_vector[7];
} MergingCandidates;


void reset_pred_vector(PredVectorInfo* pvec)
{
  for (int X=0;X<2;X++) {
    pvec->mv[X].x = 0;
    pvec->mv[X].y = 0;
    pvec->refIdx[X] = -1;
    pvec->predFlag[X] = 0;
  }
}


#define MAX_CU_SIZE 64

// 8.5.3.2
// NOTE: for full-pel shifts, we can introduce a fast path, simply copying without shifts
void generate_inter_prediction_samples(decoder_context* ctx,
                                       slice_segment_header* shdr,
                                       int xC,int yC,
                                       int xB,int yB,
                                       int nCS, int nPbW,int nPbH,
                                       const VectorInfo* vi)
{
  int nCbSL = nCS;
  int nCbSC = nCS>>1;

  const seq_parameter_set* sps = ctx->current_sps;

  uint16_t predSamplesL                 [2 /* LX */][MAX_CU_SIZE* MAX_CU_SIZE];
  uint16_t predSamplesC[2 /* chroma */ ][2 /* LX */][MAX_CU_SIZE* MAX_CU_SIZE];

  int xP = xC+xB;
  int yP = yC+yB;

  for (int l=0;l<2;l++) {
    if (vi->lum.predFlag[l]) {
      // 8.5.3.2.1

      de265_image* refPic;
      refPic = &ctx->dpb[ shdr->RefPicList[l][vi->lum.refIdx[l]] ];

      assert(refPic->PicState != UnusedForReference);


      // 8.5.3.2.2

      int xFracL = vi->lum.mv[l].x & 3;
      int yFracL = vi->lum.mv[l].y & 3;

      int xIntOffsL = xP + (vi->lum.mv[l].x>>2);
      int yIntOffsL = yP + (vi->lum.mv[l].y>>2);

      // luma sample interpolation process (8.5.3.2.2.1)

      const int shift1 = sps->BitDepth_Y-8;
      const int shift2 = 6;
      const int shift3 = 14 - sps->BitDepth_Y;

      assert(xFracL==0 && yFracL==0); // TODO...

      int w = sps->pic_width_in_luma_samples;
      int h = sps->pic_height_in_luma_samples;

      for (int y=0;y<nPbH;y++)
        for (int x=0;x<nPbW;x++) {

          int xA = Clip3(0,w-1,x + xIntOffsL);
          int yA = Clip3(0,h-1,y + yIntOffsL);

          predSamplesL[l][y*nCS+x] = refPic->y[ x+xA + yA*refPic->stride ] << shift3;
        }


      // chroma sample interpolation process (8.5.3.2.2.2)

      const int shiftC1 = sps->BitDepth_C-8;
      const int shiftC2 = 6;
      const int shiftC3 = 14 - sps->BitDepth_C;

      int wC = sps->pic_width_in_luma_samples /sps->SubWidthC;
      int hC = sps->pic_height_in_luma_samples/sps->SubHeightC;

      int xFracC = vi->lum.mv[l].x & 7;
      int yFracC = vi->lum.mv[l].y & 7;

      int xIntOffsC = xP/2 + (vi->lum.mv[l].x>>3);
      int yIntOffsC = yP/2 + (vi->lum.mv[l].y>>3);

      assert(xFracC == 0 && yFracC == 0);

      for (int y=0;y<nPbH/2;y++)
        for (int x=0;x<nPbW/2;x++) {

          int xB = Clip3(0,wC-1,x + xIntOffsC);
          int yB = Clip3(0,hC-1,y + yIntOffsC);

          predSamplesC[0][l][y*nCS+x] = refPic->cb[ x+xB + yB*refPic->stride ] << shiftC3;
          predSamplesC[1][l][y*nCS+x] = refPic->cr[ x+xB + yB*refPic->stride ] << shiftC3;
        }
    }
  }


  // weighted sample prediction  (8.5.3.2.3)

  const int shift1 = 6; // TODO
  const int offset1= 1<<(shift1-1);

  if (shdr->slice_type == SLICE_TYPE_P) {
    if (ctx->current_pps->weighted_pred_flag==0) {
      if (vi->lum.predFlag[0]==1 && vi->lum.predFlag[1]==0) {
        for (int y=0;y<nPbH;y++)
          for (int x=0;x<nPbW;x++) {
            // TODO: clip to real bit depth
            ctx->img->y[xP+x + (yP+y)*ctx->img->stride] =
              Clip1_8bit((predSamplesL[0][x+y*nCS] + offset1)>>shift1);
          }

        for (int y=0;y<nPbH/2;y++)
          for (int x=0;x<nPbW/2;x++) {
            // TODO: clip to real bit depth
            ctx->img->cb[xP/2+x + (yP/2+y)*ctx->img->chroma_stride] =
              Clip1_8bit((predSamplesC[0][0][x+y*nCS] + offset1)>>shift1);
            ctx->img->cr[xP/2+x + (yP/2+y)*ctx->img->chroma_stride] =
              Clip1_8bit((predSamplesC[1][0][x+y*nCS] + offset1)>>shift1);
          }
      }
      else {
        assert(false); // TODO
      }
    }
    else {
        assert(false); // TODO
    }
  }
  else {
        assert(false); // TODO
  }
}


void logmvcand(PredVectorInfo p)
{
  for (int v=0;v<2;v++) {
    logtrace(LogMotion,"  %d: %s  %d;%d ref=%d\n", v, p.predFlag[v] ? "yes":"no ",
             p.mv[v].x,p.mv[v].y, p.refIdx[v]);
  }
}


// 8.5.3.1.2
void derive_spatial_merging_candidates(const decoder_context* ctx,
                                       int xC, int yC, int nCS, int xP, int yP,
                                       uint8_t singleMCLFlag,
                                       int nPbW, int nPbH,
                                       int partIdx,
                                       MergingCandidates* out_cand)
{
  const pic_parameter_set* pps = ctx->current_pps;
  int log2_parallel_merge_level = pps->log2_parallel_merge_level;

  enum PartMode PartMode = get_PartMode(ctx,xC,yC);

  // --- A1 ---

  int xA1 = xP-1;
  int yA1 = yP+nPbH-1;

  bool availableA1;

  if (xP>>log2_parallel_merge_level == xA1>>log2_parallel_merge_level &&
      yP>>log2_parallel_merge_level == yA1>>log2_parallel_merge_level) {
    availableA1 = false;
  }
  else if (!singleMCLFlag &&
           partIdx==1 &&
           (PartMode==PART_Nx2N ||
            PartMode==PART_nLx2N ||
            PartMode==PART_nRx2N)) {
    availableA1 = false;
  }
  else {
    availableA1 = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xA1,yA1);
  }

  if (!availableA1) {
    out_cand->available[PRED_A1] = 0;
    reset_pred_vector(&out_cand->pred_vector[PRED_A1]);

    logtrace(LogMotion,"spatial merging candidate A1: unavailable\n");
  }
  else {
    out_cand->available[PRED_A1] = 1;
    out_cand->pred_vector[PRED_A1] = *get_mv_info(ctx,xA1,yA1);

    logtrace(LogMotion,"spatial merging candidate A1:\n");
    logmvcand(out_cand->pred_vector[PRED_A1]);
  }

  // TODO...
  out_cand->available[PRED_A0] = 0;
  out_cand->available[PRED_B0] = 0;
  out_cand->available[PRED_B1] = 0;
  out_cand->available[PRED_B2] = 0;
  //assert(false);
}


// 8.5.3.1.4
void derive_zero_motion_vector_candidates(decoder_context* ctx,
                                          slice_segment_header* shdr,
                                          PredVectorInfo* inout_mergeCandList,
                                          int* inout_numMergeCand)
{
  int numRefIdx;

  if (shdr->slice_type==SLICE_TYPE_P) {
    numRefIdx = ctx->current_pps->num_ref_idx_l0_default_active;
  }
  else {
    numRefIdx = min(ctx->current_pps->num_ref_idx_l0_default_active,
                    ctx->current_pps->num_ref_idx_l1_default_active);
  }


  int numInputMergeCand = *inout_numMergeCand;
  int zeroIdx = 0;

  while (*inout_numMergeCand < shdr->MaxNumMergeCand) {
    // 1.

    PredVectorInfo* newCand = &inout_mergeCandList[*inout_numMergeCand];

    if (shdr->slice_type==SLICE_TYPE_P) {
      newCand->refIdx[0] = (zeroIdx < numRefIdx) ? zeroIdx : 0;
      newCand->refIdx[1] = -1;
      newCand->predFlag[0] = 1;
      newCand->predFlag[1] = 0;
    }
    else {
      newCand->refIdx[0] = (zeroIdx < numRefIdx) ? zeroIdx : 0;
      newCand->refIdx[1] = (zeroIdx < numRefIdx) ? zeroIdx : 0;
      newCand->predFlag[0] = 1;
      newCand->predFlag[1] = 1;
    }

    newCand->mv[0].x = 0;
    newCand->mv[0].y = 0;
    newCand->mv[1].x = 0;
    newCand->mv[1].y = 0;

    (*inout_numMergeCand)++;

    // 2.

    zeroIdx++;
  }
}


// (L1003) 8.5.3.2.8

void derive_collocated_motion_vectors(decoder_context* ctx,
                                      slice_segment_header* shdr,
                                      int xP,int yP,
                                      int colPic,
                                      int xColPb,int yColPb,
                                      int* refIdxL,
                                      MotionVector* out_mvLXCol,
                                      uint8_t* out_availableFlagLXCol)
{
  // TODO: has to get pred_mode from reference picture
  enum PredMode predMode = get_img_pred_mode(ctx, &ctx->dpb[colPic], xP,yP);

  if (predMode == MODE_INTRA) {
    out_mvLXCol[0].x = 0;
    out_mvLXCol[0].y = 0;
    out_mvLXCol[1].x = 0;
    out_mvLXCol[1].y = 0;
    *out_availableFlagLXCol = 0;
    return;
  }
  else {
    assert(0); // TODO

    // P.136
  }
}


// 8.5.3.1.7
void derive_temporal_luma_vector_prediction(decoder_context* ctx,
                                            slice_segment_header* shdr,
                                            int xP,int yP,
                                            int nPbW,int nPbH,
                                            int* refIdxL,
                                            MotionVector* out_mvLXCol,
                                            uint8_t*      out_availableFlagLXCol)
{

  if (shdr->slice_temporal_mvp_enabled_flag == 0) {
    out_mvLXCol[0].x = 0;
    out_mvLXCol[0].y = 0;
    out_mvLXCol[1].x = 0;
    out_mvLXCol[1].y = 0;
    *out_availableFlagLXCol = 0;
    return;
  }

  int Log2CtbSizeY = ctx->current_sps->Log2CtbSizeY;

  int colPic;

  if (shdr->slice_type == SLICE_TYPE_B &&
      shdr->collocated_from_l0_flag == 0)
    {
      colPic = shdr->RefPicList[1][ shdr->collocated_ref_idx ];
    }
  else
    {
      colPic = shdr->RefPicList[0][ shdr->collocated_ref_idx ];
    }


  int xColPb,yColPb;
  int yColBr = yP + nPbH; // bottom right collocated motion vector position
  int xColBr = xP + nPbW;

  if ((yP>>Log2CtbSizeY) == (yColBr>>Log2CtbSizeY) &&
      xColBr < ctx->current_sps->pic_width_in_luma_samples &&
      yColBr < ctx->current_sps->pic_height_in_luma_samples)
    {
      xColPb = xColBr & ~0x0F;
      yColPb = yColBr & ~0x0F;

      derive_collocated_motion_vectors(ctx,shdr, xP,yP, colPic, xColPb,yColPb, refIdxL,
                                       out_mvLXCol, out_availableFlagLXCol);
    }
  else
    {
      out_mvLXCol[0].x = 0;
      out_mvLXCol[0].y = 0;
      out_mvLXCol[1].x = 0;
      out_mvLXCol[1].y = 0;
      *out_availableFlagLXCol = 0;
    }


  if (*out_availableFlagLXCol==0) {

    int xColCtr = xP+(nPbW>>1);
    int yColCtr = yP+(nPbH>>1);

    xColPb = xColCtr & ~0x0F;
    yColPb = yColCtr & ~0x0F;

    derive_collocated_motion_vectors(ctx,shdr, xP,yP, colPic, xColPb,yColPb, refIdxL,
                                     out_mvLXCol, out_availableFlagLXCol);
  }
}


// 8.5.3.1.1
void derive_luma_motion_merge_mode(decoder_context* ctx,
                                   slice_segment_header* shdr,
                                   int xC,int yC, int xP,int yP,
                                   int nCS, int nPbW,int nPbH, int partIdx,
                                   VectorInfo* out_vi)
{
  int singleMCLFlag;
  singleMCLFlag = (ctx->current_pps->log2_parallel_merge_level > 2 && nCS==8);

  if (singleMCLFlag) {
    xP=xC;
    yP=yC;
    nPbW=nCS;
    nPbH=nCS;
  }

  MergingCandidates mergeCand;
  derive_spatial_merging_candidates(ctx, xC,yC, nCS, xP,yP, singleMCLFlag,
                                    nPbW,nPbH,partIdx, &mergeCand);

  int refIdxCol[2] = { 0,0 };

  MotionVector mvCol[2];
  uint8_t availableFlagLCol;
  derive_temporal_luma_vector_prediction(ctx,shdr, xP,yP,nPbW,nPbH, refIdxCol, mvCol,
                                         &availableFlagLCol);

  int availableFlagCol = availableFlagLCol;
  uint8_t predFlagLCol[2];
  predFlagLCol[0] = availableFlagLCol;
  predFlagLCol[1] = 0;

  // 4.

  PredVectorInfo mergeCandList[5];
  int numMergeCand=0;

  for (int i=0;i<5;i++) {
    if (mergeCand.available[i]) {
      mergeCandList[numMergeCand++] = mergeCand.pred_vector[i];
    }
  }

  if (availableFlagCol) {
    // TODO: save in mergeCand directly...
    mergeCand.available[PRED_COL] = availableFlagCol;
    mergeCand.pred_vector[PRED_COL].mv[0] = mvCol[0];
    mergeCand.pred_vector[PRED_COL].mv[1] = mvCol[1];
    mergeCand.pred_vector[PRED_COL].predFlag[0] = predFlagLCol[0];
    mergeCand.pred_vector[PRED_COL].predFlag[1] = predFlagLCol[1];
    mergeCand.pred_vector[PRED_COL].refIdx[0] = refIdxCol[0];
    mergeCand.pred_vector[PRED_COL].refIdx[1] = refIdxCol[1];

    mergeCandList[numMergeCand++] = mergeCand.pred_vector[PRED_COL];
  }

  // 5.

  int numOrigMergeCand = numMergeCand;

  // 6.

  int numCombMergeCand = 0; // TODO

  //slice_segment_header* shdr = get_SliceHeader(ctx,xC,yC);
  if (shdr->slice_type == SLICE_TYPE_B) {
    assert(false); // TODO
  }


  // 7.

  derive_zero_motion_vector_candidates(ctx, shdr,
                                       mergeCandList, &numMergeCand);

  // 8.

  int merge_idx = get_merge_idx(ctx,xP,yP);
  out_vi->lum = mergeCandList[merge_idx];

  // 9.

  if (out_vi->lum.predFlag[0] && out_vi->lum.predFlag[1] && nPbW+nPbH==12) {
    out_vi->lum.refIdx[1] = -1;
    out_vi->lum.predFlag[1] = 0;
  }
}


// 8.5.3.1
void motion_vectors_indices(decoder_context* ctx,
                            slice_segment_header* shdr,
                            int xC,int yC, int xB,int yB, int nCS, int nPbW,int nPbH, int partIdx,
                            VectorInfo* out_vi)
{
  int xP = xC+xB;
  int yP = yC+yB;

  if (get_pred_mode(ctx, xC,yC) == MODE_SKIP) {
    derive_luma_motion_merge_mode(ctx,shdr, xC,yC, xP,yP, nCS,nPbW,nPbH, partIdx, out_vi);
  }
}


// 8.5.3
void decode_prediction_unit(decoder_context* ctx,slice_segment_header* shdr,
                            int xC,int yC, int xB,int yB, int nCS, int nPbW,int nPbH, int partIdx)
{
  int nCS_L = nCS;
  int nCS_C = nCS>>1;

  // 1.

  VectorInfo vi;
  motion_vectors_indices(ctx,shdr, xC,yC, xB,yB, nCS, nPbW,nPbH, partIdx, &vi);

  // 2.

  generate_inter_prediction_samples(ctx,shdr, xC,yC, xB,yB, nCS, nPbW,nPbH, &vi);


  set_mv_info(ctx,xB,yB,nPbW,nPbH, &vi.lum);
}


// 8.5.2
void inter_prediction(decoder_context* ctx,slice_segment_header* shdr,
                      int xC,int yC, int log2CbSize)
{
  int nCS_L = 1<<log2CbSize;
  int nCS_C = nCS_L>>1;
  int nCS1L = nCS_L>>1;

  enum PartMode partMode = get_PartMode(ctx,xC,yC);
  switch (partMode) {
  case PART_2Nx2N:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,0, nCS_L, nCS_L,nCS_L, 0);
    break;

  // ...

  default:
    assert(false); // TODO
  }
}
