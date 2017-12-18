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

#include "refpic.h"
#include "decctx.h"
#include "util.h"

#include <assert.h>
#include <stdlib.h>
#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#elif defined(HAVE_ALLOCA_H)
# include <alloca.h>
#endif

ref_pic_set::ref_pic_set()
{
  reset();
}


void ref_pic_set::addRefL0(int temporal_offset, image_mode mode)
{
  assert(NumNegativePics < MAX_NUM_REF_PICS);
  assert(temporal_offset < 0);

  DeltaPocS0[NumNegativePics] = temporal_offset;
  bool used = (mode == image_mode::used);
  UsedByCurrPicS0[NumNegativePics] = used;

  NumNegativePics++;
  NumDeltaPocs++;

  if (used) {
    NumPocTotalCurr_shortterm_only++;
  }
}


void ref_pic_set::addRefL1(int temporal_offset, image_mode mode)
{
  assert(NumPositivePics < MAX_NUM_REF_PICS);
  assert(temporal_offset > 0);

  DeltaPocS1[NumPositivePics] = temporal_offset;
  bool used = (mode == image_mode::used);
  UsedByCurrPicS1[NumPositivePics] = used;

  NumPositivePics++;
  NumDeltaPocs++;

  if (used) {
    NumPocTotalCurr_shortterm_only++;
  }
}


void ref_pic_set::addRef(int temporal_offset, image_mode mode)
{
  if (temporal_offset < 0) {
    addRefL0(temporal_offset, mode);
  }
  else {
    addRefL1(temporal_offset, mode);
  }
}


void ref_pic_set::reset()
{
  NumNegativePics = 0;
  NumPositivePics = 0;
  NumDeltaPocs = 0;
  NumPocTotalCurr_shortterm_only = 0;

  for (int i=0;i<MAX_NUM_REF_PICS;i++) {
    DeltaPocS0[i] = 0;
    DeltaPocS1[i] = 0;

    UsedByCurrPicS0[i] = 0;
    UsedByCurrPicS1[i] = 0;
  }
}


void ref_pic_set::compute_derived_values()
{
  NumPocTotalCurr_shortterm_only = 0;

  for (int i=0; i<NumNegativePics; i++)
    if (UsedByCurrPicS0[i])
      NumPocTotalCurr_shortterm_only++;

  for (int i=0; i<NumPositivePics; i++)
    if (UsedByCurrPicS1[i])
      NumPocTotalCurr_shortterm_only++;

  NumDeltaPocs = NumNegativePics + NumPositivePics;


  /*
    NOTE: this is done when reading the slice header.
    The value numPocTotalCurr is then stored in the slice header.

  for (int i = 0; i < num_long_term_sps + num_long_term_pics; i++ )
            if( UsedByCurrPicLt[i] )
              NumPocTotalCurr++
                }
  */
}


/* A ref-pic-set is coded either
   - as a list of the relative POC deltas themselves, or
   - by shifting an existing ref-pic-set by some number of frames
   When shifting an existing set, the frame 0 is also shifted as an additional reference frame.
   When coding the ref-pic-sets in the SPS, predicition is always from the previous set.
   In the slice header, the ref-pic-set can use any previous set as reference.
 */
de265_error ref_pic_set::read(error_queue* errqueue,
                              const seq_parameter_set* sps,
                              bitreader* br,
                              int idxRps,  // index of the set to be read
                              const std::vector<ref_pic_set>& sets, // previously read sets
                              bool sliceRefPicSet) // is this in the slice header?
{
  // --- is this set coded in prediction mode (not possible for the first set)

  char inter_ref_pic_set_prediction_flag;

  if (idxRps != 0) {
    inter_ref_pic_set_prediction_flag = get_bits(br,1);
  }
  else {
    // first set cannot use prediction
    inter_ref_pic_set_prediction_flag = 0;
  }



  if (inter_ref_pic_set_prediction_flag) {
    /* Only for the last ref_pic_set (that's the one coded in the slice header),
       we can specify relative to which reference set we code the set. */

    int delta_idx;
    if (sliceRefPicSet) { // idxRps == num_short_term_ref_pic_sets) {
      if (!get_uvlc(br, &delta_idx)) {
        return DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE;
      }

      if (delta_idx>=idxRps) {
        return DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE;
      }

      delta_idx++;
    } else {
      delta_idx = 1;
    }

    int RIdx = idxRps - delta_idx; // this is our source set, which we will modify
    assert(RIdx>=0);

    int delta_rps_sign = get_bits(br,1);
    int abs_delta_rps;
    if (!get_uvlc(br, &abs_delta_rps)) {
      return DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE;
    }
    abs_delta_rps++;
    int DeltaRPS = (delta_rps_sign ? -abs_delta_rps : abs_delta_rps);

    // bits are stored in this order:
    // - all bits for negative Pocs (forward),
    // - then all bits for positive Pocs (forward),
    // - then bits for '0', shifting of the current picture
    // in total, these are 'nDeltaPocsRIdx'+1 bits

    logtrace(LogHeaders,"predicted from %d with delta %d\n",RIdx,DeltaRPS);

    int nDeltaPocsRIdx= sets[RIdx].NumDeltaPocs; // size of source set
    char *const used_by_curr_pic_flag = (char *)alloca((nDeltaPocsRIdx+1) * sizeof(char));
    char *const use_delta_flag = (char *)alloca((nDeltaPocsRIdx+1) * sizeof(char));

    for (int j=0;j<=nDeltaPocsRIdx;j++) {
      used_by_curr_pic_flag[j] = get_bits(br,1);
      if (used_by_curr_pic_flag[j]) {
        use_delta_flag[j] = 1;  // if this frame is used, we also have to apply the delta
      } else {
        use_delta_flag[j] = get_bits(br,1);  // otherwise, it is only optionally included
      }
    }

    logtrace(LogHeaders,"flags: ");
    for (int j=0;j<=nDeltaPocsRIdx;j++) {
      logtrace(LogHeaders,"%d ", use_delta_flag[j]);
    }
    logtrace(LogHeaders,"\n");

    int nNegativeRIdx = sets[RIdx].NumNegativePics;
    int nPositiveRIdx = sets[RIdx].NumPositivePics;

    // --- update list 0 (negative Poc) ---
    // Iterate through all Pocs in decreasing value order (positive reverse, 0, negative forward).

    int i=0; // target index

    // positive list
    for (int j=nPositiveRIdx-1;j>=0;j--) {
      assert(RIdx >= 0 && RIdx < sets.size());
      assert(j>=0 && j < MAX_NUM_REF_PICS);

      int dPoc = sets[RIdx].DeltaPocS1[j] + DeltaRPS; // new delta
      if (dPoc<0 && use_delta_flag[nNegativeRIdx+j]) {
        if (i>= MAX_NUM_REF_PICS) {
          return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
        }

        DeltaPocS0[i] = dPoc;
        UsedByCurrPicS0[i] = used_by_curr_pic_flag[nNegativeRIdx+j];
        i++;
      }
    }

    // frame 0
    if (DeltaRPS<0 && use_delta_flag[nDeltaPocsRIdx]) {
      if (i>= MAX_NUM_REF_PICS) {
        return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
      }

      DeltaPocS0[i] = DeltaRPS;
      UsedByCurrPicS0[i] = used_by_curr_pic_flag[nDeltaPocsRIdx];
      i++;
    }

    // negative list
    for (int j=0;j<nNegativeRIdx;j++) {
      int dPoc = sets[RIdx].DeltaPocS0[j] + DeltaRPS;
      if (dPoc<0 && use_delta_flag[j]) {
        if (i>= MAX_NUM_REF_PICS) {
          return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
        }

        DeltaPocS0[i] = dPoc;
        UsedByCurrPicS0[i] = used_by_curr_pic_flag[j];
        i++;
      }
    }

    NumNegativePics = i;


    // --- update list 1 (positive Poc) ---
    // Iterate through all Pocs in increasing value order (negative reverse, 0, positive forward)

    i=0; // target index

    // negative list
    for (int j=nNegativeRIdx-1;j>=0;j--) {
      int dPoc = sets[RIdx].DeltaPocS0[j] + DeltaRPS;
      if (dPoc>0 && use_delta_flag[j]) {
        if (i>= MAX_NUM_REF_PICS) {
          return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
        }

        DeltaPocS1[i] = dPoc;
        UsedByCurrPicS1[i] = used_by_curr_pic_flag[j];
        i++;
      }
    }

    // frame 0
    if (DeltaRPS>0 && use_delta_flag[nDeltaPocsRIdx]) {
      if (i>= MAX_NUM_REF_PICS) {
        return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
      }

      DeltaPocS1[i] = DeltaRPS;
      UsedByCurrPicS1[i] = used_by_curr_pic_flag[nDeltaPocsRIdx];
      i++;
    }

    // positive list
    for (int j=0;j<nPositiveRIdx;j++) {
      int dPoc = sets[RIdx].DeltaPocS1[j] + DeltaRPS;
      if (dPoc>0 && use_delta_flag[nNegativeRIdx+j]) {
        if (i>= MAX_NUM_REF_PICS) {
          return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
        }

        DeltaPocS1[i] = dPoc;
        UsedByCurrPicS1[i] = used_by_curr_pic_flag[nNegativeRIdx+j];
        i++;
      }
    }

    NumPositivePics = i;

  } else {

    // --- first, read the number of past and future frames in this set ---

    int num_negative_pics;
    int num_positive_pics;

    // total number of reference pictures may not exceed buffer capacity
    if (!get_uvlc(br, &num_negative_pics) ||
        !get_uvlc(br, &num_positive_pics) ||
        num_negative_pics + num_positive_pics >
        sps->sps_max_dec_pic_buffering[ sps->sps_max_sub_layers-1 ]) {

      NumNegativePics = 0;
      NumPositivePics = 0;
      NumDeltaPocs = 0;
      NumPocTotalCurr_shortterm_only = 0;

      errqueue->add_warning(DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED, false);
      return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
    }

    if (num_negative_pics > MAX_NUM_REF_PICS ||
        num_positive_pics > MAX_NUM_REF_PICS) {
      errqueue->add_warning(DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED, false);
      return DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED;
    }

    NumNegativePics = num_negative_pics;
    NumPositivePics = num_positive_pics;

    // --- now, read the deltas between the reference frames to fill the lists ---

    // past frames

    int lastPocS=0;
    for (int i=0;i<num_negative_pics;i++) {
      int  delta_poc_s0;
      if (!get_uvlc(br, &delta_poc_s0)) {
          return DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE;
      }

      delta_poc_s0++;
      char used_by_curr_pic_s0_flag = get_bits(br,1);

      DeltaPocS0[i]      = lastPocS - delta_poc_s0;
      UsedByCurrPicS0[i] = used_by_curr_pic_s0_flag;
      lastPocS = DeltaPocS0[i];
    }

    // future frames

    lastPocS=0;
    for (int i=0;i<num_positive_pics;i++) {
      int delta_poc_s1;
      if (!get_uvlc(br, &delta_poc_s1)) {
        return DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE;
      }

      delta_poc_s1++;
      char used_by_curr_pic_s1_flag = get_bits(br,1);

      DeltaPocS1[i]      = lastPocS + delta_poc_s1;
      UsedByCurrPicS1[i] = used_by_curr_pic_s1_flag;
      lastPocS = DeltaPocS1[i];
    }
  }


  compute_derived_values();

  return DE265_OK;
}


bool ref_pic_set::write_nopred(const seq_parameter_set* sps,
                               CABAC_encoder& out,
                               int idxRps,  // index of the set to be written
                               const std::vector<ref_pic_set>& sets, // previously read sets
                               bool sliceRefPicSet) const // is this in the slice header?
{
  if (idxRps != 0) {
    // inter_ref_pic_set_prediction_flag
    out.write_bit(0);
  }


  // --- first, write the number of past and future frames in this set ---

  out.write_uvlc(NumNegativePics);
  out.write_uvlc(NumPositivePics);

  // --- now, write the deltas between the reference frames to fill the lists ---

  // past frames

  int lastPocS=0;
  for (int i=0;i<NumNegativePics;i++) {
    int  delta_poc_s0 = lastPocS - DeltaPocS0[i];
    char used_by_curr_pic_s0_flag = UsedByCurrPicS0[i];

    assert(delta_poc_s0 >= 1);
    out.write_uvlc(delta_poc_s0-1);
    out.write_bit(used_by_curr_pic_s0_flag);
    lastPocS = DeltaPocS0[i];
  }

  // future frames

  lastPocS=0;
  for (int i=0;i<NumPositivePics;i++) {
    int  delta_poc_s1 = DeltaPocS1[i] - lastPocS;
    char used_by_curr_pic_s1_flag = UsedByCurrPicS1[i];

    assert(delta_poc_s1 >= 1);
    out.write_uvlc(delta_poc_s1-1);
    out.write_bit(used_by_curr_pic_s1_flag);
    lastPocS = DeltaPocS1[i];
  }

  return true;
}


bool ref_pic_set::write(const seq_parameter_set* sps,
                        CABAC_encoder& out,
                        int idxRps,  // index of the set to be read
                        const std::vector<ref_pic_set>& sets, // previously read sets
                        bool sliceRefPicSet) const // is this in the slice header?
{
  // TODO: currently, we never use prediction
  return write_nopred(sps, out, idxRps, sets, sliceRefPicSet);
}


std::string ref_pic_set::dump() const
{
  std::stringstream sstr;

  log2sstr(sstr,"NumDeltaPocs: %d [-:%d +:%d]\n", NumDeltaPocs,
           NumNegativePics, NumPositivePics);

  log2sstr(sstr,"DeltaPocS0:");
  for (int i=0;i<NumNegativePics;i++) {
    if (i) { sstr << ','; }
    log2sstr(sstr," %d/%d",DeltaPocS0[i],UsedByCurrPicS0[i]);
  }
  sstr << '\n';

  sstr << "DeltaPocS1:";
  for (int i=0;i<NumPositivePics;i++) {
    if (i) { sstr << ','; }
    log2sstr(sstr," %d/%d",DeltaPocS1[i],UsedByCurrPicS1[i]);
  }
  log2sstr(sstr,"\n");

  return sstr.str();
}


std::string ref_pic_set::dump_compact(int range) const
{
  std::stringstream sstr;

  char *const log = (char *)alloca((range+1+range+1) * sizeof(char));
  log[2*range+1] = 0;
  for (int i=0;i<2*range+1;i++) log[i]='.';
  log[range]='|';

  for (int i=NumNegativePics-1;i>=0;i--) {
    int n = DeltaPocS0[i];
    if (n>=-range) {
      if (UsedByCurrPicS0[i]) log[n+range] = 'X';
      else log[n+range] = 'o';
    } else { log2sstr(sstr,"*%d%c ",n, UsedByCurrPicS0[i] ? 'X':'o'); }
  }

  for (int i=NumPositivePics-1;i>=0;i--) {
    int n = DeltaPocS1[i];
    if (n<=range) {
      if (UsedByCurrPicS1[i]) log[n+range] = 'X';
      else log[n+range] = 'o';
    } else { log2sstr(sstr,"*%d%c ",n, UsedByCurrPicS1[i] ? 'X':'o'); }
  }

  log2sstr(sstr,"*%s\n",log);

  return sstr.str();
}
