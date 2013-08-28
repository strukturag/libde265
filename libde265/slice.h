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

#ifndef DE265_SLICE_H
#define DE265_SLICE_H

#include "libde265/cabac.h"


#define SLICE_TYPE_B 0
#define SLICE_TYPE_P 1
#define SLICE_TYPE_I 2

enum PartMode
  {
    PART_2Nx2N = 0,
    PART_2NxN  = 1,
    PART_Nx2N  = 2,
    PART_NxN   = 3,
    PART_2NxnU = 4,
    PART_2NXnD = 5,
    PART_nLx2N = 6,
    PART_nRx2N = 7
  };

enum PredMode
  {
    MODE_INTRA, MODE_INTER, MODE_SKIP
  };

enum IntraPredMode
  {
    INTRA_PLANAR = 0,
    INTRA_DC = 1,
    INTRA_ANGULAR_2 = 2,    INTRA_ANGULAR_3 = 3,    INTRA_ANGULAR_4 = 4,    INTRA_ANGULAR_5 = 5,
    INTRA_ANGULAR_6 = 6,    INTRA_ANGULAR_7 = 7,    INTRA_ANGULAR_8 = 8,    INTRA_ANGULAR_9 = 9,
    INTRA_ANGULAR_10 = 10,  INTRA_ANGULAR_11 = 11,  INTRA_ANGULAR_12 = 12,  INTRA_ANGULAR_13 = 13,
    INTRA_ANGULAR_14 = 14,  INTRA_ANGULAR_15 = 15,  INTRA_ANGULAR_16 = 16,  INTRA_ANGULAR_17 = 17,
    INTRA_ANGULAR_18 = 18,  INTRA_ANGULAR_19 = 19,  INTRA_ANGULAR_20 = 20,  INTRA_ANGULAR_21 = 21,
    INTRA_ANGULAR_22 = 22,  INTRA_ANGULAR_23 = 23,  INTRA_ANGULAR_24 = 24,  INTRA_ANGULAR_25 = 25,
    INTRA_ANGULAR_26 = 26,  INTRA_ANGULAR_27 = 27,  INTRA_ANGULAR_28 = 28,  INTRA_ANGULAR_29 = 29,
    INTRA_ANGULAR_30 = 30,  INTRA_ANGULAR_31 = 31,  INTRA_ANGULAR_32 = 32,  INTRA_ANGULAR_33 = 33,
    INTRA_ANGULAR_34 = 34,
    INTRA_CHROMA_EQ_LUMA = 100  // chroma := luma
  };

typedef struct {
  int slice_index; // index through all slices in a picture

  char first_slice_segment_in_pic_flag;
  char no_output_of_prior_pics_flag;
  int  slice_pic_parameter_set_id;
  char dependent_slice_segment_flag;
  int  slice_segment_address;

  int  slice_type;
  char pic_output_flag;
  char colour_plane_id;
  int  slice_pic_order_cnt_lsb;
  char short_term_ref_pic_set_sps_flag;
  //short_term_ref_pic_set(num_short_term_ref_pic_sets)
  int  short_term_ref_pic_set_idx;
  int  num_long_term_sps;
  int  num_long_term_pics;

  //int lt_idx_sps[i];
  //int poc_lsb_lt[i];
  //char used_by_curr_pic_lt_flag[i];

  //char delta_poc_msb_present_flag[i];
  //int delta_poc_msb_cycle_lt[i];

  char slice_temporal_mvp_enabled_flag;
  char slice_sao_luma_flag;
  char slice_sao_chroma_flag;

  char num_ref_idx_active_override_flag;
  int  num_ref_idx_l0_active;
  int  num_ref_idx_l1_active;

  //ref_pic_lists_modification()
  char mvd_l1_zero_flag;
  char cabac_init_flag;
  char collocated_from_l0_flag;
  int  collocated_ref_idx;

  //pred_weight_table()
  int  five_minus_max_num_merge_cand;
  int  slice_qp_delta;

  int  slice_cb_qp_offset;
  int  slice_cr_qp_offset;

  char deblocking_filter_override_flag;
  char slice_deblocking_filter_disabled_flag;
  int  slice_beta_offset; // = pps->beta_offset if undefined
  int  slice_tc_offset;   // = pps->tc_offset if undefined

  char slice_loop_filter_across_slices_enabled_flag;

  int  num_entry_point_offsets;
  int  offset_len;
  // int entry_point_offset[i]

  int  slice_segment_header_extension_length;


  // --- derived data ---

  int SliceAddrRS; // current value, this is also set into the CTB-info array in the decoder context

  int CtbAddrInRS;
  int CtbAddrInTS;
  int SliceQPY;

  int initType;

  int IsCuQpDeltaCoded;
  int CuQpDelta;

  int cu_transquant_bypass_flag;

  int CurrRpsIdx;


  // --- decoder runtime data ---

  int currentQPY;
  int lastQPYinPreviousQG;

  int qPYPrime, qPCbPrime, qPCrPrime;

  CABAC_decoder cabac_decoder;

  context_model sao_merge_flag_model[3];
  context_model sao_type_idx_model[3];
  context_model split_flag_model[9];
  context_model part_mode_model[9];
  context_model prev_intra_luma_pred_flag_model[3];
  context_model intra_chroma_pred_mode_model[3];
  context_model cbf_luma_model[8];
  context_model cbf_chroma_model[12];
  context_model split_transform_flag_model[9];
  context_model last_significant_coefficient_x_prefix_model[54];
  context_model last_significant_coefficient_y_prefix_model[54];
  context_model coded_sub_block_flag_model[12];
  context_model significant_coeff_flag_model[126];
  context_model coeff_abs_level_greater1_flag_model[72];
  context_model coeff_abs_level_greater2_flag_model[18];
  context_model cu_qp_delta_abs_model[6];
  context_model transform_skip_flag_model[6];

} slice_segment_header;


typedef struct {
  // TODO: we could combine SaoTypeIdx and SaoEoClass into one byte to make the struct 16 bytes only

  unsigned char SaoTypeIdx; // use with (SaoTypeIdx>>(2*cIdx)) & 0x3
  unsigned char SaoEoClass; // use with (SaoTypeIdx>>(2*cIdx)) & 0x3

  uint8_t sao_band_position[3];
  int8_t  saoOffsetVal[3][4]; // index with [][idx-1] as saoOffsetVal[][0]==0 always  
} sao_info;

#endif
