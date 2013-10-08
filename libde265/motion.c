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

      logtrace(LogMotion, "refIdx: %d -> dpb[%d]\n", vi->lum.refIdx[l], shdr->RefPicList[l][vi->lum.refIdx[l]]);

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

      xFracL = yFracL=0; // TODO: TMP HACK

      assert(xFracL==0 && yFracL==0); // TODO...

      int w = sps->pic_width_in_luma_samples;
      int h = sps->pic_height_in_luma_samples;

      for (int y=0;y<nPbH;y++)
        for (int x=0;x<nPbW;x++) {

          int xA = Clip3(0,w-1,x + xIntOffsL);
          int yA = Clip3(0,h-1,y + yIntOffsL);

          predSamplesL[l][y*nCS+x] = refPic->y[ xA + yA*refPic->stride ] << shift3;
        }


      // chroma sample interpolation process (8.5.3.2.2.2)

      const int shiftC1 = sps->BitDepth_C-8;
      const int shiftC2 = 6;
      const int shiftC3 = 14 - sps->BitDepth_C;

      int wC = sps->pic_width_in_luma_samples /sps->SubWidthC;
      int hC = sps->pic_height_in_luma_samples/sps->SubHeightC;

      int xFracC = vi->lum.mv[l].x & 7;
      int yFracC = vi->lum.mv[l].y & 7;

      xFracC = yFracC=0; // TODO: TMP HACK

      int xIntOffsC = xP/2 + (vi->lum.mv[l].x>>3);
      int yIntOffsC = yP/2 + (vi->lum.mv[l].y>>3);

      assert(xFracC == 0 && yFracC == 0);

      for (int y=0;y<nPbH/2;y++)
        for (int x=0;x<nPbW/2;x++) {

          int xB = Clip3(0,wC-1,x + xIntOffsC);
          int yB = Clip3(0,hC-1,y + yIntOffsC);

          predSamplesC[0][l][y*nCS+x] = refPic->cb[ xB + yB*refPic->chroma_stride ] << shiftC3;
          predSamplesC[1][l][y*nCS+x] = refPic->cr[ xB + yB*refPic->chroma_stride ] << shiftC3;
        }
    }
  }


  // weighted sample prediction  (8.5.3.2.3)

  const int shift1 = 6; // TODO
  const int offset1= 1<<(shift1-1);

  logtrace(LogMotion,"predFlags: %d %d\n", vi->lum.predFlag[0], vi->lum.predFlag[1]);

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


bool equal_cand_MV(const PredVectorInfo* a, const PredVectorInfo* b)
{
  // TODO: is this really correct? no check for predFlag? Standard says so... (p.127)

  for (int i=0;i<2;i++) {
    if (a->predFlag[i]) {
      if (a->mv[i].x != b->mv[i].x) return false;
      if (a->mv[i].y != b->mv[i].y) return false;
      if (a->refIdx[i] != b->refIdx[i]) return false;
    }
  }

  return true;
}


/*
     +--+                +--+--+
     |B2|                |B1|B0|
     +--+----------------+--+--+
        |                   |
        |                   |
        |                   |
        |                   |
        |                   |
        |                   |
        |                   |
     +--+                   |
     |A1|                   |
     +--+-------------------+
     |A0|
     +--+
*/


// 8.5.3.1.2
// TODO: check: can we fill the candidate list directly in this function and omit to copy later
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

  // a pixel within A1
  int xA1 = xP-1;
  int yA1 = yP+nPbH-1;

  bool availableA1;

  if (xP>>log2_parallel_merge_level == xA1>>log2_parallel_merge_level &&
      yP>>log2_parallel_merge_level == yA1>>log2_parallel_merge_level) {
    availableA1 = false;
    logtrace(LogMotion,"spatial merging candidate A1: below parallel merge level\n");
  }
  else if (!singleMCLFlag &&
           partIdx==1 &&
           (PartMode==PART_Nx2N ||
            PartMode==PART_nLx2N ||
            PartMode==PART_nRx2N)) {
    availableA1 = false;
    logtrace(LogMotion,"spatial merging candidate A1: second part ignore\n");
  }
  else {
    availableA1 = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xA1,yA1);
    if (!availableA1) logtrace(LogMotion,"spatial merging candidate A1: unavailable\n");
  }

  if (!availableA1) {
    out_cand->available[PRED_A1] = 0;
    reset_pred_vector(&out_cand->pred_vector[PRED_A1]);
  }
  else {
    out_cand->available[PRED_A1] = 1;
    out_cand->pred_vector[PRED_A1] = *get_mv_info(ctx,xA1,yA1);

    logtrace(LogMotion,"spatial merging candidate A1:\n");
    logmvcand(out_cand->pred_vector[PRED_A1]);
  }


  // --- B1 ---

  int xB1 = xP+nPbW-1;
  int yB1 = yP-1;

  bool availableB1;

  if (xP>>log2_parallel_merge_level == xB1>>log2_parallel_merge_level &&
      yP>>log2_parallel_merge_level == yB1>>log2_parallel_merge_level) {
    availableB1 = false;
    logtrace(LogMotion,"spatial merging candidate B1: below parallel merge level\n");
  }
  else if (!singleMCLFlag &&
           partIdx==1 &&
           (PartMode==PART_2NxN ||
            PartMode==PART_2NxnU ||
            PartMode==PART_2NxnD)) {
    availableB1 = false;
    logtrace(LogMotion,"spatial merging candidate B1: second part ignore\n");
  }
  else {
    availableB1 = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xB1,yB1);
    if (!availableB1) logtrace(LogMotion,"spatial merging candidate B1: unavailable\n");
  }

  if (!availableB1) {
    out_cand->available[PRED_B1] = 0;
    reset_pred_vector(&out_cand->pred_vector[PRED_B1]);
  }
  else {
    out_cand->available[PRED_B1] = 1;
    out_cand->pred_vector[PRED_B1] = *get_mv_info(ctx,xB1,yB1);

    if (availableA1 &&
        equal_cand_MV(&out_cand->pred_vector[PRED_A1],
                      &out_cand->pred_vector[PRED_B1])) {
      out_cand->available[PRED_B1] = 0;
      logtrace(LogMotion,"spatial merging candidate B1: redundant to A1\n");
    }
    else {
      logtrace(LogMotion,"spatial merging candidate B1:\n");
      logmvcand(out_cand->pred_vector[PRED_B1]);
    }
  }


  // --- B0 ---

  int xB0 = xP+nPbW;
  int yB0 = yP-1;

  bool availableB0;

  if (xP>>log2_parallel_merge_level == xB1>>log2_parallel_merge_level &&
      yP>>log2_parallel_merge_level == yB1>>log2_parallel_merge_level) {
    availableB0 = false;
    logtrace(LogMotion,"spatial merging candidate B0: below parallel merge level\n");
  }
  else {
    availableB0 = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xB0,yB0);
    if (!availableB0) logtrace(LogMotion,"spatial merging candidate B0: unavailable\n");
  }

  if (!availableB0) {
    out_cand->available[PRED_B0] = 0;
    reset_pred_vector(&out_cand->pred_vector[PRED_B0]);
  }
  else {
    out_cand->available[PRED_B0] = 1;
    out_cand->pred_vector[PRED_B0] = *get_mv_info(ctx,xB0,yB0);

    if (availableB1 &&
        equal_cand_MV(&out_cand->pred_vector[PRED_B1],
                      &out_cand->pred_vector[PRED_B0])) {
      out_cand->available[PRED_B0] = 0;
      logtrace(LogMotion,"spatial merging candidate B0: redundant to B1\n");
    }
    else {
      logtrace(LogMotion,"spatial merging candidate B0:\n");
      logmvcand(out_cand->pred_vector[PRED_B0]);
    }
  }


  // --- A0 ---

  int xA0 = xP-1;
  int yA0 = yP+nPbH;

  bool availableA0;

  if (xP>>log2_parallel_merge_level == xB1>>log2_parallel_merge_level &&
      yP>>log2_parallel_merge_level == yB1>>log2_parallel_merge_level) {
    availableA0 = false;
    logtrace(LogMotion,"spatial merging candidate A0: below parallel merge level\n");
  }
  else {
    availableA0 = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xA0,yA0);
    if (!availableA0) logtrace(LogMotion,"spatial merging candidate A0: unavailable\n");
  }

  if (!availableA0) {
    out_cand->available[PRED_A0] = 0;
    reset_pred_vector(&out_cand->pred_vector[PRED_A0]);
  }
  else {
    out_cand->available[PRED_A0] = 1;
    out_cand->pred_vector[PRED_A0] = *get_mv_info(ctx,xA0,yA0);

    if (availableA1 &&
        equal_cand_MV(&out_cand->pred_vector[PRED_A1],
                      &out_cand->pred_vector[PRED_A0])) {
      out_cand->available[PRED_A0] = 0;
      logtrace(LogMotion,"spatial merging candidate A0: redundant to A1\n");
    }
    else {
      logtrace(LogMotion,"spatial merging candidate A0:\n");
      logmvcand(out_cand->pred_vector[PRED_A0]);
    }
  }


  // --- B2 ---

  int xB2 = xP-1;
  int yB2 = yP-1;

  bool availableB2;

  if (out_cand->available[PRED_A0] && out_cand->available[PRED_A1] &&
      out_cand->available[PRED_B0] && out_cand->available[PRED_B1]) {
    availableB2 = false;
    logtrace(LogMotion,"spatial merging candidate B2: ignore\n");
  }
  else if (xP>>log2_parallel_merge_level == xB1>>log2_parallel_merge_level &&
           yP>>log2_parallel_merge_level == yB1>>log2_parallel_merge_level) {
    availableB2 = false;
    logtrace(LogMotion,"spatial merging candidate B2: below parallel merge level\n");
  }
  else {
    availableB2 = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xB2,yB2);
    if (!availableB2) logtrace(LogMotion,"spatial merging candidate B2: unavailable\n");
  }

  if (!availableB2) {
    out_cand->available[PRED_B2] = 0;
    reset_pred_vector(&out_cand->pred_vector[PRED_B2]);
  }
  else {
    out_cand->available[PRED_B2] = 1;
    out_cand->pred_vector[PRED_B2] = *get_mv_info(ctx,xB2,yB2);

    if (availableB1 &&
        equal_cand_MV(&out_cand->pred_vector[PRED_B1],
                      &out_cand->pred_vector[PRED_B2])) {
      out_cand->available[PRED_B2] = 0;
      logtrace(LogMotion,"spatial merging candidate B2: redundant to B1\n");
    }
    else if (availableA1 &&
             equal_cand_MV(&out_cand->pred_vector[PRED_A1],
                           &out_cand->pred_vector[PRED_B2])) {
      out_cand->available[PRED_B2] = 0;
      logtrace(LogMotion,"spatial merging candidate B2: redundant to A1\n");
    }
    else {
      logtrace(LogMotion,"spatial merging candidate B0:\n");
      logmvcand(out_cand->pred_vector[PRED_B0]);
    }
  }
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


void scale_mv(MotionVector* out_mv, MotionVector mv, int colDist, int currDist)
{
  int td = Clip3(-128,127, colDist);
  int tb = Clip3(-128,127, currDist);

  int tx = (16384 + (abs_value(td)>>1)) / td;
  int distScaleFactor = Clip3(-4096,4095, (tb*tx+32)>>6);
  out_mv->x = Clip3(-32768,32767,
                    Sign(distScaleFactor*mv.x)*((abs_value(distScaleFactor*mv.x)+127)>>8));
  out_mv->y = Clip3(-32768,32767,
                    Sign(distScaleFactor*mv.y)*((abs_value(distScaleFactor*mv.y)+127)>>8));
}


// (L1003) 8.5.3.2.8

void derive_collocated_motion_vectors(decoder_context* ctx,
                                      slice_segment_header* shdr,
                                      int xP,int yP,
                                      int colPic,
                                      int xColPb,int yColPb,
                                      int refIdxLX, int X,
                                      MotionVector* out_mvLXCol,
                                      uint8_t* out_availableFlagLXCol)
{
  logtrace(LogMotion,"derive_collocated_motion_vectors %d;%d\n",xP,yP);

  // TODO: has to get pred_mode from reference picture
  enum PredMode predMode = get_img_pred_mode(ctx, &ctx->dpb[colPic], xColPb,yColPb);

  if (predMode == MODE_INTRA) {
    out_mvLXCol->x = 0;
    out_mvLXCol->y = 0;
    *out_availableFlagLXCol = 0;
    return;
  }
  else {
    logtrace(LogMotion,"colPic:%d (POC=%d) X:%d refIdxLX:%d refpiclist:%d\n",
             colPic,
             ctx->dpb[colPic].PicOrderCntVal,
             X,refIdxLX,shdr->RefPicList[X][refIdxLX]);

    const de265_image* colImg = &ctx->dpb[colPic];
    const PredVectorInfo* mvi = get_img_mv_info(ctx,colImg,xColPb,yColPb);
    int listCol;
    int refIdxCol;
    MotionVector mvCol;

    logtrace(LogMotion,"read MVI %d;%d:\n",xColPb,yColPb);
    logmvcand(*mvi);

    if (mvi->predFlag[0]==0) {
      mvCol = mvi->mv[1];
      refIdxCol = mvi->refIdx[1];
      listCol = 1;
    }
    else {
      if (mvi->predFlag[1]==0) {
        mvCol = mvi->mv[0];
        refIdxCol = mvi->refIdx[0];
        listCol = 0;
      }
      else {
        assert(0); // both predFlag[]s are 1
      }
    }

    *out_availableFlagLXCol = 1;

    bool isLongTerm = false; // TODO
    int colDist  = colImg->PicOrderCntVal - colImg->RefPicList_POC[listCol][refIdxCol];
    int currDist = ctx->img->PicOrderCntVal - ctx->img->RefPicList_POC[listCol][refIdxLX];

    logtrace(LogMotion,"COLPOCDIFF %d %d [%d %d / %d %d]\n",colDist, currDist,
             colImg->PicOrderCntVal, colImg->RefPicList_POC[listCol][refIdxCol],
             ctx->img->PicOrderCntVal, ctx->img->RefPicList_POC[listCol][refIdxLX]
             );

    if (isLongTerm || colDist == currDist) {
      *out_mvLXCol = mvCol;
    }
    else {
      scale_mv(out_mvLXCol, mvCol, colDist, currDist);
      logtrace(LogMotion,"scale: %d;%d to %d;%d\n",
               mvCol.x,mvCol.y, out_mvLXCol->x,out_mvLXCol->y);
    }
  }
}


// 8.5.3.1.7
void derive_temporal_luma_vector_prediction(decoder_context* ctx,
                                            slice_segment_header* shdr,
                                            int xP,int yP,
                                            int nPbW,int nPbH,
                                            int refIdxL, int X,
                                            MotionVector* out_mvLXCol,
                                            uint8_t*      out_availableFlagLXCol)
{

  if (shdr->slice_temporal_mvp_enabled_flag == 0) {
    out_mvLXCol->x = 0;
    out_mvLXCol->y = 0;
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
      xColPb = xColBr & ~0x0F; // reduce resolution of collocated motion-vectors to 16 pixels grid
      yColPb = yColBr & ~0x0F;

      derive_collocated_motion_vectors(ctx,shdr, xP,yP, colPic, xColPb,yColPb, refIdxL, X,
                                       out_mvLXCol, out_availableFlagLXCol);
    }
  else
    {
      out_mvLXCol->x = 0;
      out_mvLXCol->y = 0;
      *out_availableFlagLXCol = 0;
    }


  if (*out_availableFlagLXCol==0) {

    int xColCtr = xP+(nPbW>>1);
    int yColCtr = yP+(nPbH>>1);

    xColPb = xColCtr & ~0x0F; // reduce resolution of collocated motion-vectors to 16 pixels grid
    yColPb = yColCtr & ~0x0F;

    derive_collocated_motion_vectors(ctx,shdr, xP,yP, colPic, xColPb,yColPb, refIdxL, X,
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

  if (xP==112 && yP==96) {
    //raise(SIGINT);
  }

  MotionVector mvCol[2];
  uint8_t availableFlagLCol;
  derive_temporal_luma_vector_prediction(ctx,shdr, xP,yP,nPbW,nPbH, refIdxCol[0],0, &mvCol[0],
                                         &availableFlagLCol);
  /*
  derive_temporal_luma_vector_prediction(ctx,shdr, xP,yP,nPbW,nPbH, refIdxCol[1],1, &mvCol[1],
                                         &availableFlagLCol);
  */

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


  logtrace(LogMotion,"mergeCandList:\n");
  for (int i=0;i<shdr->MaxNumMergeCand;i++)
    {
      logtrace(LogMotion, " %d:%s\n", i, i==merge_idx ? " SELECTED":"");
      logmvcand(mergeCandList[i]);
    }

  // 9.

  if (out_vi->lum.predFlag[0] && out_vi->lum.predFlag[1] && nPbW+nPbH==12) {
    out_vi->lum.refIdx[1] = -1;
    out_vi->lum.predFlag[1] = 0;
  }
}


// 8.5.3.1.6
MotionVector derive_spatial_luma_vector_prediction(const decoder_context* ctx,
                                                   const slice_segment_header* shdr,
                                                   int xC,int yC,int nCS,int xP,int yP,
                                                   int nPbW,int nPbH, int X,
                                                   int refIdxLX, int partIdx,
                                                   uint8_t out_availableFlagLXN[2],
                                                   MotionVector out_mvLXN[2])
{
  int isScaledFlagLX = 0;

  const int A=0;
  const int B=1;

  // --- A ---

  // 1.

  int xA[2], yA[2];
  xA[0] = xP-1;
  yA[0] = yP + nPbH;
  xA[1] = xA[0];
  yA[1] = yA[0]-1;

  // 2.

  out_availableFlagLXN[A] = 0;
  out_mvLXN[A].x = 0;
  out_mvLXN[A].y = 0;

  // 3. / 4.

  bool availableA[2];
  availableA[0] = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xA[0],yA[0]);
  availableA[1] = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xA[1],yA[1]);

  // 5.

  if (availableA[0] || availableA[1]) {
    isScaledFlagLX = 1;
  }

  // 6.  test A0 and A1  (Ak)

  int refIdxA;

  for (int k=0;k<=1;k++) {
    if (availableA[k] &&
        out_availableFlagLXN[A]==0 &&
        get_pred_mode(ctx,xA[k],yA[k]) != MODE_INTRA) {

      int Y=1-X;
      
      const PredVectorInfo* vi = get_mv_info(ctx, xA[k],yA[k]);
      if (vi->predFlag[X] &&
          ctx->dpb[ shdr->RefPicList[X][ vi->refIdx[X] ] ].PicOrderCntVal ==
          ctx->dpb[ shdr->RefPicList[X][ refIdxLX ] ].PicOrderCntVal) {
        //vi->refIdx[X] == refIdxLX) {
        out_availableFlagLXN[A]=1;
        out_mvLXN[A] = vi->mv[X];
        refIdxA = vi->refIdx[X];
      }
      else if (vi->predFlag[Y] &&
               ctx->dpb[ shdr->RefPicList[Y][ vi->refIdx[Y] ] ].PicOrderCntVal ==
               ctx->dpb[ shdr->RefPicList[X][ refIdxLX ] ].PicOrderCntVal) {
        out_availableFlagLXN[A]=1;
        out_mvLXN[A] = vi->mv[Y];
        refIdxA = vi->refIdx[Y];
      }
    }
  }

  // 7.

  for (int k=0 ; k<=1 && out_availableFlagLXN[A]==0 ; k++) {
    int refPicList;

    if (availableA[k] &&
        get_pred_mode(ctx,xA[k],yA[k]) != MODE_INTRA) {

      int Y=1-X;
      
      const PredVectorInfo* vi = get_mv_info(ctx, xA[k],yA[k]);
      if (vi->predFlag[X]==1 &&
          true) { // TODO: long-term references
        out_availableFlagLXN[A]=1;
        out_mvLXN[A] = vi->mv[X];
        refIdxA = vi->refIdx[X];
        refPicList = X;
      }
      else if (vi->predFlag[Y]==1 &&
               true) { // TODO: long-term references
        out_availableFlagLXN[A]=1;
        out_mvLXN[A] = vi->mv[Y];
        refIdxA = vi->refIdx[Y];
        refPicList = Y;
      }
    }

    if (out_availableFlagLXN[A]==1) {
      de265_image* refPicA = &ctx->dpb[ shdr->RefPicList[refPicList][refIdxA ] ];
      de265_image* refPicX = &ctx->dpb[ shdr->RefPicList[X         ][refIdxLX] ];
      if (refPicA->PicState == UsedForShortTermReference &&
          refPicX->PicState == UsedForShortTermReference) {

        int distA = ctx->img->PicOrderCntVal - refPicA->PicOrderCntVal;
        int distX = ctx->img->PicOrderCntVal - refPicX->PicOrderCntVal;

        scale_mv(&out_mvLXN[A], out_mvLXN[A], distA, distX);
      }
    }
  }


  // --- B ---

  // 1.

  int xB[3], yB[3];
  xB[0] = xP+nPbW;
  yB[0] = yP-1;
  xB[1] = xB[0]-1;
  yB[1] = yP-1;
  xB[2] = xP-1;
  yB[2] = yP-1;

  // 2.

  out_availableFlagLXN[B] = 0;
  out_mvLXN[B].x = 0;
  out_mvLXN[B].y = 0;

  // 3. test B0,B1,B2 (Bk)

  int refIdxB;

  bool availableB[3];
  for (int k=0;k<3;k++) {
    availableB[k] = available_pred_blk(ctx, xC,yC, nCS, xP,yP, nPbW,nPbH,partIdx, xB[k],yB[k]);

    if (availableB[k] && out_availableFlagLXN[B]==0) {
      
      int Y=1-X;
      
      const PredVectorInfo* vi = get_mv_info(ctx, xB[k],yB[k]);
      if (vi->predFlag[X] &&
          ctx->dpb[ shdr->RefPicList[X][ vi->refIdx[X] ] ].PicOrderCntVal ==
          ctx->dpb[ shdr->RefPicList[X][ refIdxLX ] ].PicOrderCntVal) {
        //vi->refIdx[X] == refIdxLX) {
        out_availableFlagLXN[B]=1;
        out_mvLXN[B] = vi->mv[X];
        refIdxB = vi->refIdx[X];
      }
      else if (vi->predFlag[Y] &&
               ctx->dpb[ shdr->RefPicList[Y][ vi->refIdx[Y] ] ].PicOrderCntVal ==
               ctx->dpb[ shdr->RefPicList[X][ refIdxLX ] ].PicOrderCntVal) {
        out_availableFlagLXN[B]=1;
        out_mvLXN[B] = vi->mv[Y];
        refIdxB = vi->refIdx[Y];
      }
    }
  }

  // 4.

  if (isScaledFlagLX==0 &&
      out_availableFlagLXN[B]) {
    out_availableFlagLXN[A]=1;
    out_mvLXN[A] = out_mvLXN[B];
    refIdxA = refIdxB;
  }

  // 5.

  if (isScaledFlagLX==0) {
    out_availableFlagLXN[B]=0;

    for (int k=0 ; k<=2 && out_availableFlagLXN[B]==0 ; k++) {
      int refPicList;

      if (availableB[k]) {
        // get_pred_mode(ctx,xB[k],yB[k]) != MODE_INTRA) {

        int Y=1-X;
      
        const PredVectorInfo* vi = get_mv_info(ctx, xB[k],yB[k]);
        if (vi->predFlag[X]==1 &&
            true) { // TODO: long-term references
          out_availableFlagLXN[B]=1;
          out_mvLXN[B] = vi->mv[X];
          refIdxB = vi->refIdx[X];
          refPicList = X;
        }
        else if (vi->predFlag[Y]==1 &&
                 true) { // TODO: long-term references
          out_availableFlagLXN[B]=1;
          out_mvLXN[B] = vi->mv[Y];
          refIdxB = vi->refIdx[Y];
          refPicList = Y;
        }
      }

      if (out_availableFlagLXN[B]==1) {
        de265_image* refPicB = &ctx->dpb[ shdr->RefPicList[refPicList][refIdxB ] ];
        de265_image* refPicX = &ctx->dpb[ shdr->RefPicList[X         ][refIdxLX] ];
        if (refPicB->PicOrderCntVal != refPicX->PicOrderCntVal &&
            refPicB->PicState == UsedForShortTermReference &&
            refPicX->PicState == UsedForShortTermReference) {

          int distB = ctx->img->PicOrderCntVal - refPicB->PicOrderCntVal;
          int distX = ctx->img->PicOrderCntVal - refPicX->PicOrderCntVal;

          scale_mv(&out_mvLXN[B], out_mvLXN[B], distB, distX);
        }
      }
    }
  }
}

// 8.5.3.1.5
MotionVector luma_motion_vector_prediction(const decoder_context* ctx,
                                           const slice_segment_header* shdr,
                                           int xC,int yC,int nCS,int xP,int yP,
                                           int nPbW,int nPbH, int l,
                                           int refIdx, int partIdx)
{
  // 8.5.3.1.6: derive two spatial vector predictors A (0) and B (1)

  bool availableFlagLXN[2];
  MotionVector mvLXN[2];

  if (xP==68 && yP==16 && ctx->img->PicOrderCntVal==638) {
    //raise(SIGINT);
  }

  derive_spatial_luma_vector_prediction(ctx, shdr, xC,yC, nCS, xP,yP, nPbW,nPbH, l, refIdx, partIdx,
                                        availableFlagLXN, mvLXN);
  
  // 8.5.3.1.7: if we only have one spatial vector or both spatial vectors are the same,
  // derive a temporal predictor

  bool availableFlagLXCol;
  MotionVector mvLXCol;


  if (availableFlagLXN[0] &&
      availableFlagLXN[1] &&
      (mvLXN[0].x != mvLXN[1].x || mvLXN[0].y != mvLXN[1].y)) {
    availableFlagLXCol = 0;
  }
  else {
    derive_temporal_luma_vector_prediction(ctx, shdr, xP,yP, nPbW,nPbH, refIdx,l,
                                           &mvLXCol, &availableFlagLXCol);
  }


  // --- build candidate vector list with exactly two entries ---

  int numMVPCandLX=0;

  // spatial predictor A

  MotionVector mvpList[3];
  if (availableFlagLXN[0])
    {
      mvpList[numMVPCandLX++] = mvLXN[0];
    }

  // spatial predictor B (if not same as A)

  if (availableFlagLXN[1] &&
      (!availableFlagLXN[0] || // in case A in not available, but mvLXA initialized to same as mvLXB
        (mvLXN[0].x != mvLXN[1].x || mvLXN[0].y != mvLXN[1].y)))
    {
      mvpList[numMVPCandLX++] = mvLXN[1];
    }

  // temporal predictor

  if (availableFlagLXCol)
    {
      mvpList[numMVPCandLX++] = mvLXCol;
    }

  // fill with zero predictors

  while (numMVPCandLX<2) {
    mvpList[numMVPCandLX].x = 0;
    mvpList[numMVPCandLX].y = 0;
    numMVPCandLX++;
  }


  // select predictor according to mvp_lX_flag

  return mvpList[ get_mvp_flag(ctx,xP,yP,l) ];
}

void logMV(int x0,int y0,int nPbW,int nPbH, const char* mode,const VectorInfo* mv)
{
  logtrace(LogMotion,
           "*MV %d;%d [%d;%d] %s: (%d) %d;%d @%d\n", x0,y0,nPbW,nPbH,mode,
           mv->lum.predFlag[0], mv->lum.mv[0].x,mv->lum.mv[0].y, mv->lum.refIdx[0]);

  /*
  logtrace(LogMotion,
           "MV %d;%d [%d;%d] %s: (%d) %d;%d @%d   (%d) %d;%d @%d\n", x0,y0,nPbW,nPbH,mode,
           mv->lum.predFlag[0], mv->lum.mv[0].x,mv->lum.mv[0].y, mv->lum.refIdx[0],
           mv->lum.predFlag[1], mv->lum.mv[1].x,mv->lum.mv[1].y, mv->lum.refIdx[1]);
  */
}



// 8.5.3.1
void motion_vectors_and_ref_indices(decoder_context* ctx,
                                    slice_segment_header* shdr,
                                    int xC,int yC, int xB,int yB, int nCS, int nPbW,int nPbH, int partIdx,
                                    VectorInfo* out_vi)
{
  int xP = xC+xB;
  int yP = yC+yB;

  enum PredMode predMode = get_pred_mode(ctx, xC,yC);

  if (predMode == MODE_SKIP ||
      (predMode == MODE_INTER && get_merge_flag(ctx, xP,yP)))
    {
      derive_luma_motion_merge_mode(ctx,shdr, xC,yC, xP,yP, nCS,nPbW,nPbH, partIdx, out_vi);

      logMV(xP,yP,nPbW,nPbH, "merge_mode", out_vi);
    }
  else {
    int mvdL[2][2];
    MotionVector mvpL[2];

    for (int l=0;l<2;l++) {
      // 1.

      enum InterPredIdc inter_pred_idc = get_inter_pred_idc(ctx,xP,yP,l);

      if (inter_pred_idc == PRED_BI ||
          (inter_pred_idc == PRED_L0 && l==0) ||
          (inter_pred_idc == PRED_L1 && l==1)) {
        out_vi->lum.refIdx[l] = get_ref_idx(ctx,xP,yP,l);
        out_vi->lum.predFlag[l] = 1;
      }
      else {
        out_vi->lum.refIdx[l] = -1;
        out_vi->lum.predFlag[l] = 0;
      }

      // 2.

      mvdL[l][0] = get_mvd_x(ctx,xP,yP,l);
      mvdL[l][1] = get_mvd_y(ctx,xP,yP,l);


      if (out_vi->lum.predFlag[l]) {
        // 3.

        mvpL[l] = luma_motion_vector_prediction(ctx,shdr,xC,yC,nCS,xP,yP, nPbW,nPbH, l,
                                                out_vi->lum.refIdx[l], partIdx);

        // 4.

        int32_t x = (mvpL[l].x + mvdL[l][0] + 0x10000) & 0xFFFF;
        int32_t y = (mvpL[l].y + mvdL[l][1] + 0x10000) & 0xFFFF;

        out_vi->lum.mv[l].x = (x>=0x8000) ? x-0x10000 : x;
        out_vi->lum.mv[l].y = (y>=0x8000) ? y-0x10000 : y;
      }
    }

    logMV(xP,yP,nPbW,nPbH, "mvp", out_vi);
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
  motion_vectors_and_ref_indices(ctx,shdr, xC,yC, xB,yB, nCS, nPbW,nPbH, partIdx, &vi);

  // 2.

  generate_inter_prediction_samples(ctx,shdr, xC,yC, xB,yB, nCS, nPbW,nPbH, &vi);


  set_mv_info(ctx,xC+xB,yC+yB,nPbW,nPbH, &vi.lum);
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

  case PART_2NxN:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,0,     nCS_L, nCS_L,nCS1L, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, 0,nCS1L, nCS_L, nCS_L,nCS1L, 1);
    break;

  case PART_Nx2N:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,    0, nCS_L, nCS1L,nCS_L, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, nCS1L,0, nCS_L, nCS1L,nCS_L, 1);
    break;

  case PART_2NxnU:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,0,        nCS_L, nCS_L,nCS1L>>1, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, 0,nCS1L>>1, nCS_L, nCS_L,nCS1L + (nCS1L>>1), 1);
    break;

  case PART_2NxnD:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,0,                  nCS_L, nCS_L,nCS1L + (nCS1L>>1), 0);
    decode_prediction_unit(ctx,shdr,xC,yC, 0,nCS1L + (nCS1L>>1), nCS_L, nCS_L,nCS1L>>1, 1);
    break;

  case PART_nLx2N:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,       0, nCS_L, nCS1L>>1,          nCS_L, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, nCS1L>>1,0, nCS_L, nCS1L + (nCS1L>>1),nCS_L, 1);
    break;

  case PART_nRx2N:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,                 0, nCS_L, nCS1L + (nCS1L>>1),nCS_L, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, nCS1L + (nCS1L>>1),0, nCS_L, nCS1L>>1,nCS_L, 1);
    break;

  case PART_NxN:
    decode_prediction_unit(ctx,shdr,xC,yC, 0,    0,     nCS_L, nCS1L,nCS1L, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, nCS1L,0,     nCS_L, nCS1L,nCS1L, 1);
    decode_prediction_unit(ctx,shdr,xC,yC, 0,    nCS1L, nCS_L, nCS1L,nCS1L, 0);
    decode_prediction_unit(ctx,shdr,xC,yC, nCS1L,nCS1L, nCS_L, nCS1L,nCS1L, 1);
    break;

  default:
    assert(false); // TODO
  }
}
