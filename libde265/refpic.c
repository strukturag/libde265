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

#include "refpic.h"
#include "util.h"

#include <assert.h>
#include <stdlib.h>
#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#else
# include <alloca.h>
#endif


static void compute_NumPoc(ref_pic_set* rpset)
{
  rpset->NumPocTotalCurr = 0;
  
  for (int i=0; i<rpset->NumNegativePics; i++)
    if (rpset->UsedByCurrPicS0[i])
      rpset->NumPocTotalCurr++;

  for (int i=0; i<rpset->NumPositivePics; i++)
    if (rpset->UsedByCurrPicS1[i])
      rpset->NumPocTotalCurr++;

  /*
  for (int i = 0; i < num_long_term_sps + num_long_term_pics; i++ )
            if( UsedByCurrPicLt[i] )
              NumPocTotalCurr++
                }
  */
}


void read_short_term_ref_pic_set(bitreader* br, ref_pic_set* sets, int idxRps, int num_short_term_ref_pic_sets)
{
  char inter_ref_pic_set_prediction_flag=0;

  if (idxRps != 0) {
    inter_ref_pic_set_prediction_flag = get_bits(br,1);
  }

  if (inter_ref_pic_set_prediction_flag) {

    int delta_idx;
    if (idxRps == num_short_term_ref_pic_sets) {
      delta_idx = get_uvlc(br)+1;
    } else {
      delta_idx = 1;
    }

    int RIdx = idxRps - delta_idx;
    assert(RIdx>=0);

    int delta_rps_sign = get_bits(br,1);
    int abs_delta_rps  = get_uvlc(br)+1;
    int DeltaRPS = (delta_rps_sign ? -abs_delta_rps : abs_delta_rps);

    // bits are stored in this order:
    // - all bits for negative Pocs (forward),
    // - then all bits for positive Pocs (forward),
    // - then bits for '0'

    logtrace(LogHeaders,"predicted from %d with delta %d\n",RIdx,DeltaRPS);

    char *const used_by_curr_pic_flag = (char *)alloca((sets[RIdx].NumDeltaPocs+1) * sizeof(char));
    char *const use_delta_flag = (char *)alloca((sets[RIdx].NumDeltaPocs+1) * sizeof(char));

    for (int j=0;j<=sets[RIdx].NumDeltaPocs;j++) {
      used_by_curr_pic_flag[j] = get_bits(br,1);
      if (!used_by_curr_pic_flag[j]) {
        use_delta_flag[j] = get_bits(br,1);
      } else {
        use_delta_flag[j] = 1;
      }
    }

    logtrace(LogHeaders,"flags: ");
    for (int j=0;j<=sets[RIdx].NumDeltaPocs;j++) {
      logtrace(LogHeaders,"%d ", use_delta_flag[j]);
    }
    logtrace(LogHeaders,"\n");

    int nNegativeRIdx = sets[RIdx].NumNegativePics;
    int nDeltaPocsRIdx= sets[RIdx].NumDeltaPocs;

    // --- update list 0 (negative Poc) ---
    // Iterate through all Pocs in decreasing value order (positive reverse, 0, negative forward).

    int i=0;
    for (int j=sets[RIdx].NumPositivePics-1;j>=0;j--) {
      int dPoc = sets[RIdx].DeltaPocS1[j] + DeltaRPS;
      if (dPoc<0 && use_delta_flag[nNegativeRIdx+j]) {
        sets[idxRps].DeltaPocS0[i] = dPoc;
        sets[idxRps].UsedByCurrPicS0[i] = used_by_curr_pic_flag[nNegativeRIdx+j];
        i++;
      }
    }

    if (DeltaRPS<0 && use_delta_flag[nDeltaPocsRIdx]) {
      sets[idxRps].DeltaPocS0[i] = DeltaRPS;
      sets[idxRps].UsedByCurrPicS0[i] = used_by_curr_pic_flag[nDeltaPocsRIdx];
      i++;
    }

    for (int j=0;j<nNegativeRIdx;j++) {
      int dPoc = sets[RIdx].DeltaPocS0[j] + DeltaRPS;
      if (dPoc<0 && use_delta_flag[j]) {
        sets[idxRps].DeltaPocS0[i] = dPoc;
        sets[idxRps].UsedByCurrPicS0[i] = used_by_curr_pic_flag[j];
        i++;
      }
    }

    sets[idxRps].NumNegativePics = i;


    // --- update list 1 (positive Poc) ---
    // Iterate through all Pocs in increasing value order (negative reverse, 0, positive forward)

    i=0;
    for (int j=sets[RIdx].NumNegativePics-1;j>=0;j--) {
      int dPoc = sets[RIdx].DeltaPocS0[j] + DeltaRPS;
      if (dPoc>0 && use_delta_flag[j]) {
        sets[idxRps].DeltaPocS1[i] = dPoc;
        sets[idxRps].UsedByCurrPicS1[i] = used_by_curr_pic_flag[j];
        i++;
      }
    }

    if (DeltaRPS>0 && use_delta_flag[nDeltaPocsRIdx]) {
      sets[idxRps].DeltaPocS1[i] = DeltaRPS;
      sets[idxRps].UsedByCurrPicS1[i] = used_by_curr_pic_flag[nDeltaPocsRIdx];
      i++;
    }

    for (int j=0;j<sets[RIdx].NumPositivePics;j++) {
      int dPoc = sets[RIdx].DeltaPocS1[j] + DeltaRPS;
      if (dPoc>0 && use_delta_flag[nNegativeRIdx+j]) {
        sets[idxRps].DeltaPocS1[i] = dPoc;
        sets[idxRps].UsedByCurrPicS1[i] = used_by_curr_pic_flag[nNegativeRIdx+j];
        i++;
      }
    }

    sets[idxRps].NumPositivePics = i;

    sets[idxRps].NumDeltaPocs = sets[idxRps].NumNegativePics + sets[idxRps].NumPositivePics;

  } else {
    int num_negative_pics = get_uvlc(br);
    int num_positive_pics = get_uvlc(br);

    assert(num_negative_pics + num_positive_pics <= MAX_NUM_REF_PICS);


    sets[idxRps].NumNegativePics = num_negative_pics;
    sets[idxRps].NumPositivePics = num_positive_pics;
    sets[idxRps].NumDeltaPocs = num_positive_pics + num_negative_pics;

    int lastPocS=0;
    for (int i=0;i<num_negative_pics;i++) {
      int  delta_poc_s0 = get_uvlc(br)+1;
      char used_by_curr_pic_s0_flag = get_bits(br,1);

      logtrace(LogHeaders,"neg: %d %d\n", delta_poc_s0, used_by_curr_pic_s0_flag);

      sets[idxRps].DeltaPocS0[i]      = lastPocS - delta_poc_s0;
      sets[idxRps].UsedByCurrPicS0[i] = used_by_curr_pic_s0_flag;
      lastPocS = sets[idxRps].DeltaPocS0[i];
    }

    lastPocS=0;
    for (int i=0;i<num_positive_pics;i++) {
      int  delta_poc_s1 = get_uvlc(br)+1;
      char used_by_curr_pic_s1_flag = get_bits(br,1);

      logtrace(LogHeaders,"pos: %d %d\n", delta_poc_s1, used_by_curr_pic_s1_flag);

      sets[idxRps].DeltaPocS1[i]      = lastPocS + delta_poc_s1;
      sets[idxRps].UsedByCurrPicS1[i] = used_by_curr_pic_s1_flag;
      lastPocS = sets[idxRps].DeltaPocS1[i];
    }
  }


  compute_NumPoc(&sets[idxRps]);
}


/*
void alloc_ref_pic_set(ref_pic_set* set, int max_dec_pic_buffering)
{
  set->NumDeltaPocs = 0;
  set->NumNegativePics = 0;
  set->NumPositivePics = 0;

  set->DeltaPocS0 = calloc(max_dec_pic_buffering+1,sizeof(int));
  set->DeltaPocS1 = calloc(max_dec_pic_buffering+1,sizeof(int));
  set->UsedByCurrPicS0 = calloc(max_dec_pic_buffering+1,1);
  set->UsedByCurrPicS1 = calloc(max_dec_pic_buffering+1,1);
}

void free_ref_pic_set(ref_pic_set* set)
{
  free(set->DeltaPocS0);
  free(set->DeltaPocS1);
  free(set->UsedByCurrPicS0);
  free(set->UsedByCurrPicS1);

  set->UsedByCurrPicS0 = NULL;
  set->UsedByCurrPicS1 = NULL;
}
*/

void dump_short_term_ref_pic_set(ref_pic_set* set)
{
  logtrace(LogHeaders,"NumDeltaPocs: %d [-:%d +:%d]\n", set->NumDeltaPocs,
         set->NumNegativePics, set->NumPositivePics);

  logtrace(LogHeaders,"DeltaPocS0:");
  for (int i=0;i<set->NumNegativePics;i++) {
    if (i) { logtrace(LogHeaders,","); }
    logtrace(LogHeaders," %d/%d",set->DeltaPocS0[i],set->UsedByCurrPicS0[i]);
  }
  logtrace(LogHeaders,"\n");

  logtrace(LogHeaders,"DeltaPocS1:");
  for (int i=0;i<set->NumPositivePics;i++) {
    if (i) { logtrace(LogHeaders,","); }
    logtrace(LogHeaders," %d/%d",set->DeltaPocS1[i],set->UsedByCurrPicS1[i]);
  }
  logtrace(LogHeaders,"\n");
}


void dump_compact_short_term_ref_pic_set(ref_pic_set* set, int range)
{
  char *const log = (char *)alloca((range+1+range+1) * sizeof(char));
  log[2*range+1] = 0;
  for (int i=0;i<2*range+1;i++) log[i]='.';
  log[range]='|';

  for (int i=set->NumNegativePics-1;i>=0;i--) {
    int n = set->DeltaPocS0[i];
    if (n>=-range) {
      if (set->UsedByCurrPicS0[i]) log[n+range] = 'X';
      else log[n+range] = 'o';
    } else { loginfo(LogHeaders,"*%d ",n); }
  }

  for (int i=set->NumPositivePics-1;i>=0;i--) {
    int n = set->DeltaPocS1[i];
    if (n<=range) {
      if (set->UsedByCurrPicS1[i]) log[n+range] = 'X';
      else log[n+range] = 'o';
    } else { loginfo(LogHeaders,"*%d ",n); }
  }

  loginfo(LogHeaders,"*%s\n",log);
}
