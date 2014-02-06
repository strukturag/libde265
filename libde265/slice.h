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
#include "libde265/de265.h"
#include "libde265/util.h"


#define MAX_CTB_ROWS   68  // enough for 4K @ 32 pixel CTBs, but TODO: make this dynamic
#define MAX_ENTRY_POINTS    MAX_CTB_ROWS
#define MAX_THREAD_CONTEXTS MAX_CTB_ROWS
#define MAX_REF_PIC_LIST (14+1)

#define SLICE_TYPE_B 0
#define SLICE_TYPE_P 1
#define SLICE_TYPE_I 2


/*
        2Nx2N           2NxN             Nx2N            NxN          
      +-------+       +-------+       +---+---+       +---+---+
      |       |       |       |       |   |   |       |   |   |
      |       |       |_______|       |   |   |       |___|___|
      |       |       |       |       |   |   |       |   |   |
      |       |       |       |       |   |   |       |   |   |
      +-------+       +-------+       +---+---+       +---+---+

        2NxnU           2NxnD           nLx2N           nRx2N        
      +-------+       +-------+       +-+-----+       +-----+-+
      |_______|       |       |       | |     |       |     | |
      |       |       |       |       | |     |       |     | |
      |       |       |_______|       | |     |       |     | |
      |       |       |       |       | |     |       |     | |
      +-------+       +-------+       +-+-----+       +-----+-+

      - AMP only if CU size > min CU size -> minimum PU size = CUsize/2
      - NxN only if size >= 16x16 (-> minimum block size = 8x8)
      - minimum block size for Bi-Pred is 8x8 (wikipedia: Coding_tree_unit)
*/
enum PartMode
  {
    PART_2Nx2N = 0,
    PART_2NxN  = 1,
    PART_Nx2N  = 2,
    PART_NxN   = 3,
    PART_2NxnU = 4,
    PART_2NxnD = 5,
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

enum InterPredIdc
  {
    PRED_L0=0,
    PRED_L1=1,
    PRED_BI=2
  };

enum context_model_indices {
  CONTEXT_MODEL_SAO_MERGE_FLAG = 0,
  CONTEXT_MODEL_SAO_TYPE_IDX   = CONTEXT_MODEL_SAO_MERGE_FLAG +3,
  CONTEXT_MODEL_SPLIT_CU_FLAG  = CONTEXT_MODEL_SAO_TYPE_IDX + 3,
  CONTEXT_MODEL_CU_SKIP_FLAG   = CONTEXT_MODEL_SPLIT_CU_FLAG + 9,
  CONTEXT_MODEL_PART_MODE      = CONTEXT_MODEL_CU_SKIP_FLAG + 6,
  CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG = CONTEXT_MODEL_PART_MODE + 9,
  CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE    = CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG + 3,
  CONTEXT_MODEL_CBF_LUMA                  = CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE + 3,
  CONTEXT_MODEL_CBF_CHROMA                = CONTEXT_MODEL_CBF_LUMA + 8,
  CONTEXT_MODEL_SPLIT_TRANSFORM_FLAG      = CONTEXT_MODEL_CBF_CHROMA + 12,
  CONTEXT_MODEL_LAST_SIGNIFICANT_COEFFICIENT_X_PREFIX = CONTEXT_MODEL_SPLIT_TRANSFORM_FLAG + 9,
  CONTEXT_MODEL_LAST_SIGNIFICANT_COEFFICIENT_Y_PREFIX = CONTEXT_MODEL_LAST_SIGNIFICANT_COEFFICIENT_X_PREFIX + 54,
  CONTEXT_MODEL_CODED_SUB_BLOCK_FLAG          = CONTEXT_MODEL_LAST_SIGNIFICANT_COEFFICIENT_Y_PREFIX + 54,
  CONTEXT_MODEL_SIGNIFICANT_COEFF_FLAG        = CONTEXT_MODEL_CODED_SUB_BLOCK_FLAG + 12,
  CONTEXT_MODEL_COEFF_ABS_LEVEL_GREATER1_FLAG = CONTEXT_MODEL_SIGNIFICANT_COEFF_FLAG + 42,
  CONTEXT_MODEL_COEFF_ABS_LEVEL_GREATER2_FLAG = CONTEXT_MODEL_COEFF_ABS_LEVEL_GREATER1_FLAG + 72,
  CONTEXT_MODEL_CU_QP_DELTA_ABS        = CONTEXT_MODEL_COEFF_ABS_LEVEL_GREATER2_FLAG + 18,
  CONTEXT_MODEL_TRANSFORM_SKIP_FLAG    = CONTEXT_MODEL_CU_QP_DELTA_ABS + 6,
  CONTEXT_MODEL_MERGE_FLAG             = CONTEXT_MODEL_TRANSFORM_SKIP_FLAG + 6,
  CONTEXT_MODEL_MERGE_IDX              = CONTEXT_MODEL_MERGE_FLAG + 2,
  CONTEXT_MODEL_PRED_MODE_FLAG         = CONTEXT_MODEL_MERGE_IDX + 2,
  CONTEXT_MODEL_ABS_MVD_GREATER01_FLAG = CONTEXT_MODEL_PRED_MODE_FLAG + 2,
  CONTEXT_MODEL_MVP_LX_FLAG            = CONTEXT_MODEL_ABS_MVD_GREATER01_FLAG + 4,
  CONTEXT_MODEL_RQT_ROOT_CBF           = CONTEXT_MODEL_MVP_LX_FLAG + 2,
  CONTEXT_MODEL_REF_IDX_LX             = CONTEXT_MODEL_RQT_ROOT_CBF + 2,
  CONTEXT_MODEL_INTER_PRED_IDC         = CONTEXT_MODEL_REF_IDX_LX + 4,
  CONTEXT_MODEL_TABLE_LENGTH           = CONTEXT_MODEL_INTER_PRED_IDC + 10
};


struct slice_segment_header;

typedef struct thread_context
{
  int SliceAddrRS; // current value, this is also set into the CTB-info array in the decoder context

  int CtbAddrInRS;
  int CtbAddrInTS;


  // motion vectors

  int8_t  refIdx[2];
  int16_t mvd[2][2]; // only in top left position
  uint8_t merge_flag;
  uint8_t merge_idx;
  uint8_t mvp_lX_flag[2];
  uint8_t inter_pred_idc; // enum InterPredIdc


  // prediction

  enum IntraPredMode IntraPredModeC; // chroma intra-prediction mode for current CB


  // residual data

  uint8_t transform_skip_flag[3];

  ALIGNED_16(int16_t) coeffBuf[32*32]; // alignment required for SSE code !

  int16_t coeffList[3][32*32];
  int16_t coeffPos[3][32*32];
  int16_t nCoeff[3];


  // quantization

  int currentQPY;
  int currentQG_x, currentQG_y;
  int lastQPYinPreviousQG;

  int qPYPrime, qPCbPrime, qPCrPrime;

  CABAC_decoder cabac_decoder;

  context_model ctx_model[CONTEXT_MODEL_TABLE_LENGTH];
  context_model ctx_model_wpp_storage[CONTEXT_MODEL_TABLE_LENGTH];

  struct decoder_context* decctx;
  struct slice_segment_header* shdr;
} thread_context;


typedef struct slice_segment_header {
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

  char ref_pic_list_modification_flag_l0;
  char ref_pic_list_modification_flag_l1;
  int list_entry_l0[1]; // TODO
  int list_entry_l1[1]; // TODO

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
  int entry_point_offset[MAX_ENTRY_POINTS];

  int  slice_segment_header_extension_length;


  // --- derived data ---

  int SliceQPY;

  int initType;

  int IsCuQpDeltaCoded;
  int CuQpDelta;

  int cu_transquant_bypass_flag;

  int CurrRpsIdx;
  int MaxNumMergeCand;

  int RefPicList[2][MAX_REF_PIC_LIST];


  // --- decoder runtime data ---

  struct thread_context thread_context[MAX_THREAD_CONTEXTS];
} slice_segment_header;



typedef struct {
  // TODO: we could combine SaoTypeIdx and SaoEoClass into one byte to make the struct 16 bytes only

  unsigned char SaoTypeIdx; // use with (SaoTypeIdx>>(2*cIdx)) & 0x3
  unsigned char SaoEoClass; // use with (SaoTypeIdx>>(2*cIdx)) & 0x3

  uint8_t sao_band_position[3];
  int8_t  saoOffsetVal[3][4]; // index with [][idx-1] as saoOffsetVal[][0]==0 always  
} sao_info;


#endif
