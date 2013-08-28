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

#ifndef DE265_REFPIC_H
#define DE265_REFPIC_H

#include "libde265/bitstream.h"


#define MAX_NUM_REF_PICS 16

typedef struct {
  int NumDeltaPocs;
  int NumNegativePics;
  int NumPositivePics;

  int DeltaPocS0[MAX_NUM_REF_PICS]; // sorted in decreasing order (e.g. -1, -2, -4, -7, ...)
  int DeltaPocS1[MAX_NUM_REF_PICS]; // sorted in ascending order (e.g. 1, 2, 4, 7)

  char UsedByCurrPicS0[MAX_NUM_REF_PICS];
  char UsedByCurrPicS1[MAX_NUM_REF_PICS];

  int NumPoc_withoutLongterm;
} ref_pic_set;

//void alloc_ref_pic_set(ref_pic_set*, int max_dec_pic_buffering);
//void free_ref_pic_set(ref_pic_set*);

void dump_short_term_ref_pic_set(ref_pic_set*);
void dump_compact_short_term_ref_pic_set(ref_pic_set* set, int range);

void read_short_term_ref_pic_set(bitreader* br, ref_pic_set* sets, int idxRps, int num_short_term_ref_pic_sets);

#endif
