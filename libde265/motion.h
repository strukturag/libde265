/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
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

#ifndef DE265_MOTION_H
#define DE265_MOTION_H

#include <stdint.h>


typedef struct
{
  int16_t x,y;
} MotionVector;


typedef struct
{
  uint8_t predFlag[2];  // which of the two vectors is actually used
   int8_t   refIdx[2];
  MotionVector  mv[2];
} PredVectorInfo;



typedef struct
{
  // array indices
  // important! order like shown in 8.5.3.1.1
  enum {
    PRED_A1  = 0,
    PRED_B1  = 1,
    PRED_B0  = 2,
    PRED_A0  = 3,
    PRED_B2  = 4
  };

  uint8_t          available[5];
  PredVectorInfo pred_vector[5];
} SpatialMergingCandidates;




void derive_spatial_merging_candidates(const class de265_image* img,
                                       int xC, int yC, int nCS, int xP, int yP,
                                       uint8_t singleMCLFlag,
                                       int nPbW, int nPbH,
                                       int partIdx,
                                       SpatialMergingCandidates* out_cand);



void decode_prediction_unit(struct thread_context* shdr,
                            int xC,int yC, int xB,int yB, int nCS, int nPbW,int nPbH, int partIdx);

void inter_prediction(struct decoder_context* ctx,struct slice_segment_header* shdr,
                      int xC,int yC, int log2CbSize);

#endif
