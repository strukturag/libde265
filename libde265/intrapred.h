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

#ifndef DE265_INTRAPRED_H
#define DE265_INTRAPRED_H

#include "libde265/decctx.h"

extern const int intraPredAngle_table[1+34];


/* Fill the three intra-pred-mode candidates into candModeList.
   This is computed for the intra block at position (x,y).
   If we happen to know also the PUidx for this position (sps.getPUIndexRS()), it can be
   given to the function. If it is not known, you can use the next function below.
   availableA/B is the output of check_CTB_available().
 */
void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3],
                                 int x,int y, int PUidx,
                                 bool availableA, // left
                                 bool availableB, // top
                                 const image* img);


inline void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3], int x,int y,
                                 bool availableA, // left
                                 bool availableB, // top
                                 const image* img)
{
  int PUidx = img->get_sps().getPUIndexRS(x,y);
  fillIntraPredModeCandidates(candModeList, x,y, PUidx, availableA,availableB, img);
}


void fill_intraPredMode_candidates_from_tree(enum IntraPredMode candModeList[3],
                                             int x,int y,
                                             bool availableA, // left
                                             bool availableB, // top
                                             const class CTBTreeMatrix& ctbs,
                                             const seq_parameter_set* sps);



/* Return value >= 0 -> use mpm_idx(return value)
   else              -> use rem_intra(-return value-1)

   This function may modify the candModeList !
 */
int find_intra_pred_mode(enum IntraPredMode mode,
                         enum IntraPredMode candModeList[3]);

void list_chroma_pred_candidates(enum IntraPredMode chroma_mode[5],
                                 enum IntraPredMode luma_mode);

int get_intra_scan_idx(int log2TrafoSize, enum IntraPredMode intraPredMode, int cIdx,
                       const seq_parameter_set* sps);

enum IntraPredMode lumaPredMode_to_chromaPredMode(enum IntraPredMode luma,
                                                  enum IntraChromaPredMode chroma);

void decode_intra_prediction(image* img,
                             int xB0,int yB0,
                             enum IntraPredMode intraPredMode,
                             int nT, int cIdx);

void decode_intra_prediction_from_tree(const image* img,
                                       const class enc_tb* tb,
                                       const class CTBTreeMatrix& ctbs,
                                       const class seq_parameter_set& sps,
                                       int cIdx);

// TODO: remove this
template <class pixel_t> void decode_intra_prediction(image* img,
                                                      int xB0,int yB0,
                                                      enum IntraPredMode intraPredMode,
                                                      pixel_t* dst, int nT, int cIdx);

#endif
