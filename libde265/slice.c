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

#include "slice.h"
#include "slice_func.h"
#include "motion_func.h"
#include "util.h"
#include "scan.h"
#include "intrapred.h"
#include "transform.h"

#include <assert.h>
#include <string.h>
#ifdef _MSC_VER
# include <malloc.h>
#else
# include <alloca.h>
#endif


void read_coding_tree_unit(decoder_context* ctx, slice_segment_header* shdr);

void read_coding_quadtree(decoder_context* ctx,
                          slice_segment_header* shdr,
                          int xCtb, int yCtb, 
                          int Log2CtbSizeY,
                          int ctDepth);

int check_CTB_available(decoder_context* ctx,
                        slice_segment_header* shdr,
                        int xC,int yC, int xN,int yN);



void read_slice_segment_header(bitreader* br, slice_segment_header* shdr, decoder_context* ctx)
{
  // set defaults

  shdr->dependent_slice_segment_flag = 0;


  // read bitstream

  shdr->first_slice_segment_in_pic_flag = get_bits(br,1);

  if (ctx->RapPicFlag) { // TODO: is this still correct ? Should we drop RapPicFlag ?
    shdr->no_output_of_prior_pics_flag = get_bits(br,1);
  }

  shdr->slice_pic_parameter_set_id = get_uvlc(br);

  pic_parameter_set* pps = &ctx->pps[shdr->slice_pic_parameter_set_id];
  assert(pps->pps_read); // TODO: error handling

  seq_parameter_set* sps = &ctx->sps[pps->seq_parameter_set_id];
  assert(sps->sps_read); // TODO: error handling

  if (!shdr->first_slice_segment_in_pic_flag) {
    if (pps->dependent_slice_segments_enabled_flag) {
      shdr->dependent_slice_segment_flag = get_bits(br,1);
    } else {
      shdr->dependent_slice_segment_flag = 0;
    }

    shdr->slice_segment_address = get_bits(br, ceil_log2(sps->PicSizeInCtbsY));
  } else {
    shdr->dependent_slice_segment_flag = 0;
    shdr->slice_segment_address = 0;
  }


  if (!shdr->dependent_slice_segment_flag) {

    for (int i=0; i<pps->num_extra_slice_header_bits; i++) {
      //slice_reserved_undetermined_flag[i]
      skip_bits(br,1); 
    }

    shdr->slice_type = get_uvlc(br);

    if (pps->output_flag_present_flag) {
      shdr->pic_output_flag = get_bits(br,1);
    }
    else {
      shdr->pic_output_flag = 1;
    }

    if (sps->separate_colour_plane_flag == 1) {
      shdr->colour_plane_id = get_bits(br,1);
    }

    if (ctx->nal_unit_type != NAL_UNIT_IDR_W_RADL &&
        ctx->nal_unit_type != NAL_UNIT_IDR_N_LP) {
      shdr->slice_pic_order_cnt_lsb = get_bits(br, sps->log2_max_pic_order_cnt_lsb);
      shdr->short_term_ref_pic_set_sps_flag = get_bits(br,1);

      if (!shdr->short_term_ref_pic_set_sps_flag) {
        read_short_term_ref_pic_set(br, ctx->ref_pic_sets,
                                    sps->num_short_term_ref_pic_sets,
                                    sps->num_short_term_ref_pic_sets);

        shdr->CurrRpsIdx = sps->num_short_term_ref_pic_sets;
      }
      else {
        int nBits = ceil_log2(sps->num_short_term_ref_pic_sets);
        if (nBits>0) shdr->short_term_ref_pic_set_idx = get_bits(br,nBits);
        else         shdr->short_term_ref_pic_set_idx = 0;

        shdr->CurrRpsIdx = shdr->short_term_ref_pic_set_idx;
      }

      if (sps->long_term_ref_pics_present_flag) {
        assert(false);
        /*
          if( num_long_term_ref_pics_sps > 0 )
          num_long_term_sps
          num_long_term_pics
          for( i = 0; i < num_long_term_sps + num_long_term_pics; i++ ) {
          if( i < num_long_term_sps )
          lt_idx_sps[i]
          else {
          poc_lsb_lt[i]
          used_by_curr_pic_lt_flag[i]
          }
          delta_poc_msb_present_flag[i]
          if( delta_poc_msb_present_flag[i] )
          delta_poc_msb_cycle_lt[i]
          }
        */
      }

      if (sps->sps_temporal_mvp_enabled_flag) {
        shdr->slice_temporal_mvp_enabled_flag = get_bits(br,1);
      }
      else {
        shdr->slice_temporal_mvp_enabled_flag = 0;
      }
    }


    if (sps->sample_adaptive_offset_enabled_flag) {
      shdr->slice_sao_luma_flag   = get_bits(br,1);
      shdr->slice_sao_chroma_flag = get_bits(br,1);
    }
    else {
      shdr->slice_sao_luma_flag   = 0;
      shdr->slice_sao_chroma_flag = 0;
    }

    if (shdr->slice_type == SLICE_TYPE_P  ||
        shdr->slice_type == SLICE_TYPE_B) {
      shdr->num_ref_idx_active_override_flag = get_bits(br,1);
      if (shdr->num_ref_idx_active_override_flag) {
        shdr->num_ref_idx_l0_active = get_uvlc(br) +1;
        if (shdr->slice_type == SLICE_TYPE_B) {
          shdr->num_ref_idx_l1_active = get_uvlc(br) +1;
        }
      }
      else {
        shdr->num_ref_idx_l0_active = pps->num_ref_idx_l0_default_active;
        shdr->num_ref_idx_l1_active = pps->num_ref_idx_l1_default_active;
      }

      int NumPocTotalCurr = ctx->ref_pic_sets[shdr->CurrRpsIdx].NumPocTotalCurr;
      // TODO: add number of longterm images

      if (pps->lists_modification_present_flag && NumPocTotalCurr > 1) {
        assert(false);
        /*
          ref_pic_lists_modification()
        */
      }
      else {
        shdr->ref_pic_list_modification_flag_l0 = 0;
        shdr->ref_pic_list_modification_flag_l1 = 0;
      }

      if (shdr->slice_type == SLICE_TYPE_B) {
        shdr->mvd_l1_zero_flag = get_bits(br,1);
      }

      if (pps->cabac_init_present_flag) {
        shdr->cabac_init_flag = get_bits(br,1);
      }
      else {
        shdr->cabac_init_flag = 0;
      }

      if (shdr->slice_temporal_mvp_enabled_flag) {
        if (shdr->slice_type == SLICE_TYPE_B)
          shdr->collocated_from_l0_flag = get_bits(br,1);
        else
          shdr->collocated_from_l0_flag = 1;

        if (( shdr->collocated_from_l0_flag && shdr->num_ref_idx_l0_active > 1) ||
            (!shdr->collocated_from_l0_flag && shdr->num_ref_idx_l1_active > 1)) {
          shdr->collocated_ref_idx = get_uvlc(br);
        }
        else {
          shdr->collocated_ref_idx = 0;
        }
      }

      if ((pps->weighted_pred_flag   && shdr->slice_type == SLICE_TYPE_P) ||
          (pps->weighted_bipred_flag && shdr->slice_type == SLICE_TYPE_B)) {
        //pred_weight_table()
        assert(false);
      }

      shdr->five_minus_max_num_merge_cand = get_uvlc(br);
      shdr->MaxNumMergeCand = 5-shdr->five_minus_max_num_merge_cand;
    }
    
    shdr->slice_qp_delta = get_svlc(br);
    //logtrace(LogSlice,"slice_qp_delta: %d\n",shdr->slice_qp_delta);

    if (pps->pps_slice_chroma_qp_offsets_present_flag) {
      shdr->slice_cb_qp_offset = get_svlc(br);
      shdr->slice_cr_qp_offset = get_svlc(br);
    }
    else {
      shdr->slice_cb_qp_offset = 0;
      shdr->slice_cr_qp_offset = 0;
    }

    if (pps->deblocking_filter_override_enabled_flag) {
      shdr->deblocking_filter_override_flag = get_bits(br,1);
    }
    else {
      shdr->deblocking_filter_override_flag = 0;
    }

    shdr->slice_beta_offset = pps->beta_offset;
    shdr->slice_tc_offset   = pps->tc_offset;

    if (shdr->deblocking_filter_override_flag) {
      shdr->slice_deblocking_filter_disabled_flag = get_bits(br,1);
      if (!shdr->slice_deblocking_filter_disabled_flag) {
        shdr->slice_beta_offset = get_svlc(br)*2;
        shdr->slice_tc_offset   = get_svlc(br)*2;
      }
    }
    else {
      shdr->slice_deblocking_filter_disabled_flag = pps->pic_disable_deblocking_filter_flag;
    }

    if (pps->pps_loop_filter_across_slices_enabled_flag  &&
        (shdr->slice_sao_luma_flag || shdr->slice_sao_chroma_flag ||
         !shdr->slice_deblocking_filter_disabled_flag )) {
      shdr->slice_loop_filter_across_slices_enabled_flag = get_bits(br,1);
    }
  }

  if (pps->tiles_enabled_flag || pps->entropy_coding_sync_enabled_flag ) {
    shdr->num_entry_point_offsets = get_uvlc(br);

    if (shdr->num_entry_point_offsets > 0) {
      shdr->offset_len = get_uvlc(br) +1;

      for (int i=0; i<shdr->num_entry_point_offsets; i++) {
        {
          assert(false);
          //entry_point_offset[i]
        }
      }
    }
  }

  if (pps->slice_segment_header_extension_present_flag) {
    shdr->slice_segment_header_extension_length = get_uvlc(br);
    
    for (int i=0; i<shdr->slice_segment_header_extension_length; i++) {
      //slice_segment_header_extension_data_byte[i]
      get_bits(br,8);
    }
  }

  //byte_alignment();
  //skip_to_byte_boundary(br);


  // --- init variables ---

  if (shdr->dependent_slice_segment_flag==0) {
    shdr->SliceAddrRS = shdr->slice_segment_address;
  } else {
    shdr->SliceAddrRS = pps->CtbAddrTStoRS[ pps->CtbAddrRStoTS[shdr->slice_segment_address] -1 ];
  }

  shdr->CtbAddrInTS = shdr->slice_segment_address;
  shdr->CtbAddrInRS = shdr->CtbAddrInTS; // TODO (page 46)

  shdr->SliceQPY = pps->pic_init_qp + shdr->slice_qp_delta;

  switch (shdr->slice_type)
    {
    case SLICE_TYPE_I: shdr->initType = 0; break;
    case SLICE_TYPE_P: shdr->initType = shdr->cabac_init_flag ? 2 : 1; break;
    case SLICE_TYPE_B: shdr->initType = shdr->cabac_init_flag ? 1 : 2; break;
    }
}



//-----------------------------------------------------------------------


void dump_slice_segment_header(const slice_segment_header* shdr, const decoder_context* ctx)
{
#define LOG(...) loginfo(LogHeaders, __VA_ARGS__)

  const pic_parameter_set* pps = &ctx->pps[shdr->slice_pic_parameter_set_id];
  assert(pps->pps_read); // TODO: error handling

  const seq_parameter_set* sps = &ctx->sps[pps->seq_parameter_set_id];
  assert(sps->sps_read); // TODO: error handling


  LOG("----------------- SLICE -----------------\n");
  LOG("first_slice_segment_in_pic_flag        : %d\n", shdr->first_slice_segment_in_pic_flag);
  if (ctx->nal_unit_type >= NAL_UNIT_BLA_W_LP &&
      ctx->nal_unit_type <= NAL_UNIT_RSV_IRAP_VCL23) {
    LOG("no_output_of_prior_pics_flag           : %d\n", shdr->no_output_of_prior_pics_flag);
  }

  LOG("slice_pic_parameter_set_id             : %d\n", shdr->slice_pic_parameter_set_id);

  if (!shdr->first_slice_segment_in_pic_flag) {
    if (pps->dependent_slice_segments_enabled_flag) {
      LOG("dependent_slice_segment_flag         : %d\n", shdr->dependent_slice_segment_flag);
    }
    LOG("slice_segment_address                : %d\n", shdr->slice_segment_address);
  }

  if (!shdr->dependent_slice_segment_flag) {
    //for (int i=0; i<pps->num_extra_slice_header_bits; i++) {
    //slice_reserved_flag[i]

    LOG("slice_type                           : %c\n",
        shdr->slice_type == 0 ? 'B' :
        shdr->slice_type == 1 ? 'P' : 'I');

    if (pps->output_flag_present_flag) {
      LOG("pic_output_flag                      : %d\n", shdr->pic_output_flag);
    }

    if (sps->separate_colour_plane_flag == 1) {
      LOG("colour_plane_id                      : %d\n", shdr->colour_plane_id);
    }

    if (ctx->nal_unit_type != NAL_UNIT_IDR_W_RADL &&
        ctx->nal_unit_type != NAL_UNIT_IDR_N_LP) {
      LOG("slice_pic_order_cnt_lsb              : %d\n", shdr->slice_pic_order_cnt_lsb);
      LOG("short_term_ref_pic_set_sps_flag      : %d\n", shdr->short_term_ref_pic_set_sps_flag);

      if (!shdr->short_term_ref_pic_set_sps_flag) {
        // TODO: DUMP short_term_ref_pic_set(num_short_term_ref_pic_sets)
      }
      else if (sps->num_short_term_ref_pic_sets > 1) {
        LOG("short_term_ref_pic_set_idx           : %d\n", shdr->short_term_ref_pic_set_idx);
      }

      if (sps->long_term_ref_pics_present_flag) {
        if (sps->num_long_term_ref_pics_sps > 0) {
          LOG("num_long_term_sps                        : %d\n", shdr->num_long_term_sps);
        }

        LOG("num_long_term_pics                       : %d\n", shdr->num_long_term_pics);
          
        for (int i=0; i<shdr->num_long_term_sps + shdr->num_long_term_pics; i++) {
          if (i < shdr->num_long_term_sps) {
            if (sps->num_long_term_ref_pics_sps > 1) {
              // TODO lt_idx_sps[i]
            }
          } else {
            // TODO poc_lsb_lt[i]
            // TODO used_by_curr_pic_lt_flag[i]
          }
          // TODO delta_poc_msb_present_flag[i]
          // TODO if( delta_poc_msb_present_flag[i] )
          // TODO delta_poc_msb_cycle_lt[i]
        }
      }

      if (sps->sps_temporal_mvp_enabled_flag) {
        LOG("slice_temporal_mvp_enabled_flag : %d\n", shdr->slice_temporal_mvp_enabled_flag);
      }
    }
      

    if (sps->sample_adaptive_offset_enabled_flag) {
      LOG("slice_sao_luma_flag             : %d\n", shdr->slice_sao_luma_flag);
      LOG("slice_sao_chroma_flag           : %d\n", shdr->slice_sao_chroma_flag);
    }


    if (shdr->slice_type == SLICE_TYPE_P || shdr->slice_type == SLICE_TYPE_B) {
      LOG("num_ref_idx_active_override_flag : %d\n", shdr->num_ref_idx_active_override_flag);

      LOG("num_ref_idx_l0_active          : %d\n", shdr->num_ref_idx_l0_active);

      if (shdr->slice_type == SLICE_TYPE_B) {
        LOG("num_ref_idx_l1_active          : %d\n", shdr->num_ref_idx_l1_active);
      }

      int NumPocTotalCurr = ctx->ref_pic_sets[shdr->CurrRpsIdx].NumPocTotalCurr;
      // TODO: add number of longterm images

      if (pps->lists_modification_present_flag && NumPocTotalCurr > 1)
        {
          assert(false);
          //ref_pic_lists_modification()
        }

      if (shdr->slice_type == SLICE_TYPE_B) {
        LOG("mvd_l1_zero_flag               : %d\n", shdr->mvd_l1_zero_flag);
      }
      
      LOG("cabac_init_flag                : %d\n", shdr->cabac_init_flag);

      if (shdr->slice_temporal_mvp_enabled_flag) {
        LOG("collocated_from_l0_flag        : %d\n", shdr->collocated_from_l0_flag);
        LOG("collocated_ref_idx             : %d\n", shdr->collocated_ref_idx);
      }

      if ((pps->weighted_pred_flag   && shdr->slice_type == SLICE_TYPE_P) ||
          (pps->weighted_bipred_flag && shdr->slice_type == SLICE_TYPE_B))
        {
          assert(false);
          //pred_weight_table()
        }

      LOG("five_minus_max_num_merge_cand  : %d\n", shdr->five_minus_max_num_merge_cand);
    }


    LOG("slice_qp_delta         : %d\n", shdr->slice_qp_delta);
    if (pps->pps_slice_chroma_qp_offsets_present_flag) {
      LOG("slice_cb_qp_offset     : %d\n", shdr->slice_cb_qp_offset);
      LOG("slice_cr_qp_offset     : %d\n", shdr->slice_cr_qp_offset);
    }

    if (pps->deblocking_filter_override_enabled_flag) {
      LOG("deblocking_filter_override_flag : %d\n", shdr->deblocking_filter_override_flag);
    }

    LOG("slice_deblocking_filter_disabled_flag : %d %s\n",
        shdr->slice_deblocking_filter_disabled_flag,
        (shdr->deblocking_filter_override_flag ? "(override)" : "(from pps)"));

    if (shdr->deblocking_filter_override_flag) {

      if (!shdr->slice_deblocking_filter_disabled_flag) {
        LOG("slice_beta_offset  : %d\n", shdr->slice_beta_offset);
        LOG("slice_tc_offset    : %d\n", shdr->slice_tc_offset);
      }
    }

    if (pps->pps_loop_filter_across_slices_enabled_flag  &&
        (shdr->slice_sao_luma_flag || shdr->slice_sao_chroma_flag ||
         !shdr->slice_deblocking_filter_disabled_flag)) {
      LOG("slice_loop_filter_across_slices_enabled_flag : %d\n",
          shdr->slice_loop_filter_across_slices_enabled_flag);
    }
  }

  if (pps->tiles_enabled_flag || pps->entropy_coding_sync_enabled_flag) {
    LOG("num_entry_point_offsets    : %d\n", shdr->num_entry_point_offsets);

    if (shdr->num_entry_point_offsets > 0) {
      LOG("offset_len                 : %d\n", shdr->offset_len);

      for (int i=0; i<shdr->num_entry_point_offsets; i++) {
        // TODO DUMP entry_point_offset_minus1[i]
      }
    }
  }

  /*
    if( slice_segment_header_extension_present_flag ) {
    slice_segment_header_extension_length
    for( i = 0; i < slice_segment_header_extension_length; i++) 
    slice_segment_header_extension_data_byte[i]
    }
    byte_alignment()
    }
  */

#undef LOG
}





void set_initValue(decoder_context* ctx, slice_segment_header* shdr,
                   context_model* model, int initValue)
{
  int slopeIdx = initValue >> 4;
  int intersecIdx = initValue & 0xF;
  int m = slopeIdx*5 - 45;
  int n = (intersecIdx<<3) - 16;
  int preCtxState = Clip3(1,126, ((m*Clip3(0,51, shdr->SliceQPY))>>4)+n);
  
  logtrace(LogSlice,"QP=%d slopeIdx=%d intersecIdx=%d m=%d n=%d\n",shdr->SliceQPY,slopeIdx,intersecIdx,m,n);
  
  model->MPSbit=(preCtxState<=63) ? 0 : 1;
  model->state = model->MPSbit ? (preCtxState-64) : (63-preCtxState);
  //model->state = model->MPSbit ? (preCtxState-64) : preCtxState; // HACK
}


static const int initValue_split_cu_flag[9] = { 139,141,157, 107,139,126, 107,139,126 };
static const int initValue_cu_skip_flag[6] = { 197,185,201, 197,185,201 };
static const int initValue_part_mode[9] = { 184,154,139, 154,154,154, 139,154,154 };
static const int initValue_prev_intra_luma_pred_flag[3] = { 184,154,183 };
static const int initValue_intra_chroma_pred_mode[3] = { 63,152,152 };
static const int initValue_cbf_luma[8] = { 111,141,153,111,153,111 };
static const int initValue_cbf_chroma[12] = { 94,138,182,154,149,107,167,154,149,92,167,154 };
static const int initValue_split_transform_flag[12] = { 153,138,138, 124,138,94, 224,167,122 }; // FIX712
  //const static int initValue_split_transform_flag[12] = { 224,167,122, 124,138,94, 153,138,138 };
static const int initValue_last_significant_coefficient_prefix[54] = {
    110,110,124,125,140,153,125,127,140,109,111,143,127,111, 79,108,123, 63,
    125,110, 94,110, 95, 79,125,111,110, 78,110,111,111, 95, 94,108,123,108,
    125,110,124,110, 95, 94,125,111,111, 79,125,126,111,111, 79,108,123, 93
  };
static const int initValue_coded_sub_block_flag[12] = { 91,171,134,141,121,140,61,154,121,140,61,154 };
static const int initValue_significant_coeff_flag[126] = {
    111,  111,  125,  110,  110,   94,  124,  108,  124,  107,  125,  141,  179,  153,  125,  107,
    125,  141,  179,  153,  125,  107,  125,  141,  179,  153,  125,  140,  139,  182,  182,  152,
    136,  152,  136,  153,  136,  139,  111,  136,  139,  111,  155,  154,  139,  153,  139,  123,
    123,   63,  153,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  166,
    183,  140,  136,  153,  154,  170,  153,  123,  123,  107,  121,  107,  121,  167,  151,  183,
    140,  151,  183,  140,  170,  154,  139,  153,  139,  123,  123,   63,  124,  166,  183,  140,
    136,  153,  154,  166,  183,  140,  136,  153,  154,  166,  183,  140,  136,  153,  154,  170,
    153,  138,  138,  122,  121,  122,  121,  167,  151,  183,  140,  151,  183,  140
  };
static const int initValue_coeff_abs_level_greater1_flag[72] = {
    140, 92,137,138,140,152,138,139,153, 74,149, 92,139,107,122,152,
    140,179,166,182,140,227,122,197,154,196,196,167,154,152,167,182,
    182,134,149,136,153,121,136,137,169,194,166,167,154,167,137,182,
    154,196,167,167,154,152,167,182,182,134,149,136,153,121,136,122,
    169,208,166,167,154,152,167,182
  };
static const int initValue_coeff_abs_level_greater2_flag[18] = {
    138,153,136,167,152,152,107,167, 91,122,107,167,
    107,167, 91,107,107,167
  };
static const int initValue_sao_merge_leftUp_flag[3] = { 153,153,153 };
static const int initValue_sao_type_idx_lumaChroma_flag[3] = { 200,185,160 };
static const int initValue_cu_qp_delta_abs[6] = { 154,154,154,154,154,154 };
static const int initValue_transform_skip_flag[6] = { 139,139,139,139,139,139 };
static const int initValue_merge_flag[2] = { 110,154 };
static const int initValue_merge_idx[2] = { 122,137 };
static const int initValue_pred_mode_flag[2] = { 149,134 };
static const int initValue_abs_mvd_greater01_flag[2] = { 140,198,169,198 };
static const int initValue_mvp_lx_flag[2] = { 168,168 };
static const int initValue_rqt_root_cbf[2] = { 79,79 };
static const int initValue_ref_idx_lX[2] = { 153,153,153,153 };

void init_sao_merge_leftUp_flag_context(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<3;i++)
    {
      set_initValue(ctx,shdr,
                    &shdr->sao_merge_flag_model[i],
                    initValue_sao_merge_leftUp_flag[i]);

      logtrace(LogSlice,"sao_merge_leftUp_flag context[%d] = state:%d MPS:%d\n",
             i,shdr->sao_merge_flag_model[i].state,shdr->sao_merge_flag_model[i].MPSbit);
    }
}


void init_sao_type_idx_lumaChroma_context(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<3;i++)
    {
      set_initValue(ctx,shdr,
                    &shdr->sao_type_idx_model[i],
                    initValue_sao_type_idx_lumaChroma_flag[i]);

      logtrace(LogSlice,"sao_type_idx_lumaChroma context[%d] = state:%d MPS:%d\n",
             i,shdr->sao_type_idx_model[i].state,shdr->sao_type_idx_model[i].MPSbit);
    }
}


void init_transform_skip_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<6;i++)
    {
      set_initValue(ctx,shdr,
                    &shdr->transform_skip_flag_model[i],
                    initValue_transform_skip_flag[i]);

      logtrace(LogSlice,"transform_skip_flag context[%d] = state:%d MPS:%d\n",i,
             shdr->transform_skip_flag_model[i].state,
             shdr->transform_skip_flag_model[i].MPSbit);
    }
}


void init_cu_qp_delta_abs(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<6;i++)
    {
      set_initValue(ctx,shdr,
                    &shdr->cu_qp_delta_abs_model[i],
                    initValue_cu_qp_delta_abs[i]);

      logtrace(LogSlice,"cu_qp_delta_abs context[%d] = state:%d MPS:%d\n",
             i,shdr->cu_qp_delta_abs_model[i].state,shdr->cu_qp_delta_abs_model[i].MPSbit);
    }
}


void init_split_cu_context(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<9;i++)
    {
      set_initValue(ctx,shdr, &shdr->split_flag_model[i], initValue_split_cu_flag[i]);

      logtrace(LogSlice,"split_cu_flag context[%d] = state:%d MPS:%d\n",
             i,shdr->split_flag_model[i].state,shdr->split_flag_model[i].MPSbit);
    }
}


void init_cu_skip_flag_context(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<6;i++)
    {
      set_initValue(ctx,shdr, &shdr->cu_skip_flag_model[i], initValue_cu_skip_flag[i]);

      logtrace(LogSlice,"cu_skip_flag context[%d] = state:%d MPS:%d\n",
             i,shdr->cu_skip_flag_model[i].state,shdr->cu_skip_flag_model[i].MPSbit);
    }
}


void init_part_mode_context(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<9;i++)
    {
      set_initValue(ctx,shdr, &shdr->part_mode_model[i], initValue_part_mode[i]);

      logtrace(LogSlice,"part_mode context[%d] = (%d) %d %d\n",i,initValue_part_mode[i],
             shdr->part_mode_model[i].state,shdr->part_mode_model[i].MPSbit);
    }
}


void init_prev_intra_luma_pred_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<3;i++)
    {
      set_initValue(ctx,shdr, &shdr->prev_intra_luma_pred_flag_model[i], initValue_prev_intra_luma_pred_flag[i]);

      logtrace(LogSlice,"prev_intra_luma_pred_flag context[%d] = %d %d\n",
             i,
             shdr->prev_intra_luma_pred_flag_model[i].state,
             shdr->prev_intra_luma_pred_flag_model[i].MPSbit);
    }
}

void init_intra_chroma_pred_mode(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<3;i++)
    {
      set_initValue(ctx,shdr, &shdr->intra_chroma_pred_mode_model[i], initValue_intra_chroma_pred_mode[i]);
    }
}

void init_cbf_chroma(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<12;i++)
    {
      set_initValue(ctx,shdr, &shdr->cbf_chroma_model[i], initValue_cbf_chroma[i]);
    }
}

void init_cbf_luma(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<8;i++)
    {
      set_initValue(ctx,shdr, &shdr->cbf_luma_model[i], initValue_cbf_luma[i]);
    }
}

void init_split_transform_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<9;i++)
    {
      set_initValue(ctx,shdr, &shdr->split_transform_flag_model[i],
                    initValue_split_transform_flag[i]);

      logtrace(LogSlice,"split_transform_flag context[%d] = %d %d\n",i,
             shdr->split_transform_flag_model[i].state,
             shdr->split_transform_flag_model[i].MPSbit);
    }
}

void init_last_significant_coefficient_prefix(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<54;i++)
    {
      set_initValue(ctx,shdr, &shdr->last_significant_coefficient_x_prefix_model[i],
                    initValue_last_significant_coefficient_prefix[i]);
      set_initValue(ctx,shdr, &shdr->last_significant_coefficient_y_prefix_model[i],
                    initValue_last_significant_coefficient_prefix[i]);
    }
}

void init_coded_sub_block_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<12;i++)
    {
      set_initValue(ctx,shdr, &shdr->coded_sub_block_flag_model[i], initValue_coded_sub_block_flag[i]);
    }
}

void init_significant_coeff_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<126;i++)
    {
      set_initValue(ctx,shdr, &shdr->significant_coeff_flag_model[i], initValue_significant_coeff_flag[i]);
    }
}


void init_coeff_abs_level_greater1_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<72;i++)
    {
      set_initValue(ctx,shdr, &shdr->coeff_abs_level_greater1_flag_model[i],
                    initValue_coeff_abs_level_greater1_flag[i]);
    }
}

void init_coeff_abs_level_greater2_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<18;i++)
    {
      set_initValue(ctx,shdr, &shdr->coeff_abs_level_greater2_flag_model[i],
                    initValue_coeff_abs_level_greater2_flag[i]);
    }
}

void init_merge_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<2;i++)
    {
      set_initValue(ctx,shdr, &shdr->merge_flag_model[i], initValue_merge_flag[i]);
    }
}

void init_merge_idx(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<2;i++)
    {
      set_initValue(ctx,shdr, &shdr->merge_idx_model[i], initValue_merge_idx[i]);
    }
}

void init_pred_mode_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<2;i++)
    {
      set_initValue(ctx,shdr, &shdr->pred_mode_flag_model[i], initValue_pred_mode_flag[i]);
    }
}

void init_abs_mvd_greater01_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<4;i++)
    {
      set_initValue(ctx,shdr,
                    &shdr->abs_mvd_greater01_flag_model[i],
                    initValue_abs_mvd_greater01_flag[i]);
    }
}

void init_mvp_lx_flag(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<2;i++)
    {
      set_initValue(ctx,shdr, &shdr->mvp_lx_flag_model[i], initValue_mvp_lx_flag[i]);
    }
}

void init_rqt_root_cbf(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<2;i++)
    {
      set_initValue(ctx,shdr,
                    &shdr->rqt_root_cbf_model[i],
                    initValue_rqt_root_cbf[i]);
    }
}

void init_ref_idx_lX(decoder_context* ctx, slice_segment_header* shdr)
{
  for (int i=0;i<4;i++)
    {
      set_initValue(ctx,shdr, &shdr->ref_idx_lX_model[i], initValue_ref_idx_lX[i]);
    }
}


int decode_transform_skip_flag(decoder_context* ctx,
                               slice_segment_header* shdr,
                               int cIdx)
{
  int context;
  if (cIdx==0) { context =   shdr->initType; }
  else         { context = 3+shdr->initType; }

  logtrace(LogSlice,"# transform_skip_flag (context=%d)\n",context);

  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->transform_skip_flag_model[context]);
  return bit;
}


int decode_sao_merge_flag(decoder_context* ctx,
                          slice_segment_header* shdr)
{
  logtrace(LogSlice,"# sao_merge_left/up_flag\n");
  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->sao_merge_flag_model[shdr->initType]);
  return bit;
}



int decode_sao_type_idx(decoder_context* ctx,
                             slice_segment_header* shdr)
{
  logtrace(LogSlice,"# sao_type_idx_luma/chroma\n");
  int bit0 = decode_CABAC_bit(&shdr->cabac_decoder,
                              &shdr->sao_type_idx_model[shdr->initType]);
  if (bit0==0) {
    return 0;
  }
  else {
    int bit1 = decode_CABAC_bypass(&shdr->cabac_decoder);
    if (bit1==0) {
      return 1;
    }
    else {
      return 2;
    }
  }
}


int decode_sao_offset_abs(decoder_context* ctx,
                          slice_segment_header* shdr)
{
  logtrace(LogSlice,"# sao_offset_abs\n");
  int bitDepth = 8;
  int cMax = (1<<(min(bitDepth,10)-5))-1;
  int value = decode_CABAC_TU_bypass(&shdr->cabac_decoder, cMax);
  return value;
}


int decode_sao_class(decoder_context* ctx,
                     slice_segment_header* shdr)
{
  logtrace(LogSlice,"# sao_class\n");
  int value = decode_CABAC_FL_bypass(&shdr->cabac_decoder, 2);
  return value;
}


int decode_sao_offset_sign(decoder_context* ctx,
                     slice_segment_header* shdr)
{
  logtrace(LogSlice,"# sao_offset_sign\n");
  int value = decode_CABAC_bypass(&shdr->cabac_decoder);
  return value;
}


int decode_sao_band_position(decoder_context* ctx,
                             slice_segment_header* shdr)
{
  logtrace(LogSlice,"# sao_band_position\n");
  int value = decode_CABAC_FL_bypass(&shdr->cabac_decoder,5);
  return value;
}


int decode_split_cu_flag(decoder_context* ctx,
                         slice_segment_header* shdr,
                         int x0, int y0, int ctDepth)
{
  // check if neighbors are available

  int availableL = check_CTB_available(ctx,shdr, x0,y0, x0-1,y0);
  int availableA = check_CTB_available(ctx,shdr, x0,y0, x0,y0-1);

  int condL = 0;
  int condA = 0;

  if (availableL && get_ctDepth(ctx,x0-1,y0) > ctDepth) condL=1;
  if (availableA && get_ctDepth(ctx,x0,y0-1) > ctDepth) condA=1;

  int contextOffset = condL + condA;
  int context = 3*shdr->initType + contextOffset;

  // decode bit

  logtrace(LogSlice,"# split_cu_flag context=%d R=%x\n", context, shdr->cabac_decoder.range);

  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->split_flag_model[context]);

  logtrace(LogSlice,"> split_cu_flag R=%x, ctx=%d, bit=%d\n", shdr->cabac_decoder.range,context,bit);

  return bit;
}


int decode_cu_skip_flag(decoder_context* ctx,
                        slice_segment_header* shdr,
                        int x0, int y0, int ctDepth)
{
  // check if neighbors are available

  int availableL = check_CTB_available(ctx,shdr, x0,y0, x0-1,y0);
  int availableA = check_CTB_available(ctx,shdr, x0,y0, x0,y0-1);

  int condL = 0;
  int condA = 0;

  if (availableL && get_cu_skip_flag(ctx,x0-1,y0)) condL=1;
  if (availableA && get_cu_skip_flag(ctx,x0,y0-1)) condA=1;

  int contextOffset = condL + condA;
  int context = 3*(shdr->initType-1) + contextOffset;

  // decode bit

  logtrace(LogSlice,"# cu_skip_flag context=%d R=%x\n", context, shdr->cabac_decoder.range);

  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->cu_skip_flag_model[context]);

  logtrace(LogSlice,"> cu_skip_flag R=%x, ctx=%d, bit=%d\n", shdr->cabac_decoder.range,context,bit);

  return bit;
}


enum PartMode decode_part_mode(decoder_context* ctx,
                               slice_segment_header* shdr,
                               enum PredMode pred_mode, int cLog2CbSize)
{
  if (pred_mode == MODE_INTRA) {
    logtrace(LogSlice,"# part_mode (INTRA)\n");

    int ctxIdxOffset;
    switch (shdr->slice_type) {
    case SLICE_TYPE_I: ctxIdxOffset=0; break;
    case SLICE_TYPE_P: ctxIdxOffset=1; break;
    case SLICE_TYPE_B:
    default:           ctxIdxOffset=1; break;
    }

    int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset]);

    logtrace(LogSlice,"> %s\n",bit ? "2Nx2N" : "NxN");

    return bit ? PART_2Nx2N : PART_NxN;
  }
  else {
    int ctxIdxOffset = (shdr->slice_type==SLICE_TYPE_P) ? 1 : 5;

    int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset]);
    if (bit) { return PART_2Nx2N; }

    if (cLog2CbSize > ctx->current_sps->Log2MinCbSizeY) {
      if (!ctx->current_sps->amp_enabled_flag) {
        bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset+1]);
        return bit ? PART_2NxN : PART_Nx2N;
      }
      else {
        int bit2 = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset+1]);
        int bit3 = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset+2]);
        if (bit3 &&  bit2) return PART_2NxN;
        if (bit3 && !bit2) return PART_Nx2N;

        int bit4 = decode_CABAC_bypass(&shdr->cabac_decoder);
        if ( bit2 &&  bit4) return PART_2NxnD;
        if ( bit2 && !bit4) return PART_2NxnU;
        if (!bit2 && !bit4) return PART_nLx2N;
        if (!bit2 &&  bit4) return PART_nRx2N;
      }
    }
    else {
      // TODO, we could save one if here when first decoding the next bin and then
      // checkcLog2CbSize==3 when it is '0'

      if (cLog2CbSize==3) {
        bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset+1]);
        return bit ? PART_2NxN : PART_Nx2N;
      }
      else {
        int bit1 = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset+1]);
        if (bit1) return PART_2NxN;

        int bit2 = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->part_mode_model[ctxIdxOffset+2]);
        return bit2 ? PART_Nx2N : PART_NxN;
      }
    }
  }
}


int decode_prev_intra_luma_pred_flag(decoder_context* ctx,
                                     slice_segment_header* shdr)
{
  logtrace(LogSlice,"# prev_intra_luma_pred_flag\n");
  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->prev_intra_luma_pred_flag_model[shdr->initType]);
  return bit;
}


int decode_mpm_idx(decoder_context* ctx,
                   slice_segment_header* shdr)
{
  logtrace(LogSlice,"# mpm_idx (TU:2)\n");
  int mpm = decode_CABAC_TU_bypass(&shdr->cabac_decoder, 2);
  logtrace(LogSlice,"> mpm_idx = %d\n",mpm);
  return mpm;
}


int decode_rem_intra_luma_pred_mode(decoder_context* ctx,
                                    slice_segment_header* shdr)
{
  logtrace(LogSlice,"# rem_intra_luma_pred_mode (5 bits)\n");
  return decode_CABAC_FL_bypass(&shdr->cabac_decoder, 5);
}


int decode_intra_chroma_pred_mode(decoder_context* ctx,
                                  slice_segment_header* shdr)
{
  logtrace(LogSlice,"# intra_chroma_pred_mode\n");

  int prefix = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->intra_chroma_pred_mode_model[ shdr->initType ]);

  int mode;
  if (prefix==0) {
    mode=4;
  }
  else {
    mode = decode_CABAC_FL_bypass(&shdr->cabac_decoder, 2);
  }

  logtrace(LogSlice,"> intra_chroma_pred_mode = %d\n",mode);

  return mode;
}


int decode_split_transform_flag(decoder_context* ctx,
                                slice_segment_header* shdr,
                                int log2TrafoSize)
{
  logtrace(LogSlice,"# split_transform_flag (log2TrafoSize=%d)\n",log2TrafoSize);

  int context = 3*shdr->initType + 5-log2TrafoSize;

  logtrace(LogSlice,"# context: %d\n",context);

  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->split_transform_flag_model[context]);
  return bit;
}


int decode_cbf_chroma(decoder_context* ctx,
                      slice_segment_header* shdr,
                      int trafoDepth)
{
  logtrace(LogSlice,"# cbf_chroma\n");

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->cbf_chroma_model[shdr->initType*4 + trafoDepth]);

  return bit;
}


int decode_cbf_luma(decoder_context* ctx,
                    slice_segment_header* shdr,
                    int trafoDepth)
{
  logtrace(LogSlice,"# cbf_luma\n");

  int context = shdr->initType*2;
  if (trafoDepth==0) context++;

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->cbf_luma_model[context]);

  logtrace(LogSlice,"> cbf_luma = %d\n",bit);

  return bit;
}


int decode_coded_sub_block_flag(decoder_context* ctx,
                                slice_segment_header* shdr,
                                int cIdx,int sbWidth, int xS,int yS,
                                const uint8_t* coded_sub_block_flag)
{
  logtrace(LogSlice,"# coded_sub_block_flag\n");

  int context = shdr->initType*4;
  context += 0;

  int csbfCtx = 0;
  if (xS<sbWidth-1) {
    csbfCtx += coded_sub_block_flag[xS+1 +yS*sbWidth];
  }
  if (yS<sbWidth-1) {
    csbfCtx += coded_sub_block_flag[xS +(yS+1)*sbWidth];
  }
  //coded_sub_block_flag[S.x+S.y*sbWidth] = decode_coded_sub_block_flag(ctx,shdr, cIdx,sbWidth,S.x,S.y, coded_sub_block_flag);

  int ctxIdxInc;
  if (cIdx==0) {
    ctxIdxInc = csbfCtx<1 ? csbfCtx : 1;
  } else {
    ctxIdxInc = 2 + (csbfCtx<1 ? csbfCtx : 1);
  }

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->coded_sub_block_flag_model[context + ctxIdxInc]);

  return bit;
}


int decode_cu_qp_delta_abs(decoder_context* ctx,
                           slice_segment_header* shdr)
{
  logtrace(LogSlice,"# cu_qp_delta_abs\n");

  int context = shdr->initType*2;

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->cu_qp_delta_abs_model[context + 0]);
  if (bit==0) {
    return 0;
  }

  int prefix=1;
  for (int i=0;i<4;i++) {
    bit = decode_CABAC_bit(&shdr->cabac_decoder,
                           &shdr->cu_qp_delta_abs_model[context + 1]);
    if (bit==0) { break; }
    else { prefix++; }
  }

  if (prefix==5) {
    int value = decode_CABAC_EGk_bypass(&shdr->cabac_decoder, 0);
    return value + 4;
  }
  else {
    return prefix;
  }
}

        
int decode_last_significant_coeff_prefix(decoder_context* ctx,
                                         slice_segment_header* shdr,
                                         int log2TrafoSize,
                                         int cIdx,
                                         context_model* model)
{
  logtrace(LogSlice,"# last_significant_coeff_prefix log2TrafoSize:%d cIdx:%d\n",log2TrafoSize,cIdx);

  int cMax = (log2TrafoSize<<1)-1;

  int ctxOffset, ctxShift;
  if (cIdx==0) {
    ctxOffset = 3*(log2TrafoSize-2) + ((log2TrafoSize-1)>>2);
    ctxShift  = (log2TrafoSize+1)>>2;
  }
  else {
    ctxOffset = 15;
    ctxShift  = log2TrafoSize-2;
  }

  int context = 18 * shdr->initType + ctxOffset;

  int binIdx;
  int value = cMax;
  for (binIdx=0;binIdx<cMax;binIdx++)
    {
      int ctxIdxInc = (binIdx >> ctxShift);

      logtrace(LogSlice,"context: %d+%d\n",context,ctxIdxInc);

      int bit = decode_CABAC_bit(&shdr->cabac_decoder, &model[context + ctxIdxInc]);
      if (bit==0) {
        value=binIdx;
        break;
      }
    }

  logtrace(LogSlice,"> last_significant_coeff_prefix: %d\n", value);

  return value;
}


static const int ctxIdxMap[15] = {
  0,1,4,5,2,3,4,5,6,6,8,8,7,7,8
};

int decode_significant_coeff_flag(decoder_context* ctx,
                                  slice_segment_header* shdr,
                                  int xC,int yC,
                                  const uint8_t* coded_sub_block_flag,
                                  int sbWidth,
                                  int cIdx,
                                  int scanIdx)
{
  logtrace(LogSlice,"# significant_coeff_flag (xC:%d yC:%d sbWidth:%d cIdx:%d scanIdx:%d)\n",
         xC,yC,sbWidth,cIdx,scanIdx);

  int sigCtx;

  // if log2TrafoSize==2
  if (sbWidth==1) {
    sigCtx = ctxIdxMap[(yC<<2) + xC];
  }
  else if (xC+yC==0) {
    sigCtx = 0;
  }
  else {
    int xS = xC>>2;
    int yS = yC>>2;
    int prevCsbf = 0;
    if (xS < sbWidth-1) { prevCsbf += coded_sub_block_flag[xS+1  +yS*sbWidth];    }
    if (yS < sbWidth-1) { prevCsbf += coded_sub_block_flag[xS+(1+yS)*sbWidth]<<1; }

    int xP = xC & 3;
    int yP = yC & 3;

    logtrace(LogSlice,"posInSubset: %d,%d\n",xP,yP);
    logtrace(LogSlice,"prevCsbf: %d\n",prevCsbf);

    switch (prevCsbf) {
    case 0:
      sigCtx = (xP+yP==0) ? 2 : (xP+yP<3) ? 1 : 0;
      break;
    case 1:
      sigCtx = (yP==0) ? 2 : (yP==1) ? 1 : 0;
      break;
    case 2:
      sigCtx = (xP==0) ? 2 : (xP==1) ? 1 : 0;
      break;
    default:
      sigCtx = 2;
      break;
    }

    logtrace(LogSlice,"a) sigCtx=%d\n",sigCtx);

    if (cIdx==0) {
      if (xS+yS > 0) sigCtx+=3;

      logtrace(LogSlice,"b) sigCtx=%d\n",sigCtx);

      // if log2TrafoSize==3
      if (sbWidth==2) {
        sigCtx += (scanIdx==0) ? 9 : 15;
      } else {
        sigCtx += 21;
      }

      logtrace(LogSlice,"c) sigCtx=%d\n",sigCtx);
    }
    else {
      // if log2TrafoSize==3
      if (sbWidth==2) {
        sigCtx+=9;
      }
      else {
        sigCtx+=12;
      }
    }
  }

  int ctxIdxInc;
  if (cIdx==0) { ctxIdxInc=sigCtx; }
  else         { ctxIdxInc=27+sigCtx; }

  int context = shdr->initType*42 + ctxIdxInc;
  logtrace(LogSlice,"context: %d\n",context);

  int bit = decode_CABAC_bit(&shdr->cabac_decoder, &shdr->significant_coeff_flag_model[context]);
  return bit;
}


int decode_coeff_abs_level_greater1(decoder_context* ctx,
                                    slice_segment_header* shdr,
                                    int cIdx, int i,int n,
                                    bool firstCoeffInSubblock,
                                    bool firstSubblock,
                                    int  lastSubblock_greater1Ctx,
                                    int* lastInvocation_greater1Ctx,
                                    int* lastInvocation_coeff_abs_level_greater1_flag,
                                    int* lastInvocation_ctxSet, int c1)
{
  logtrace(LogSlice,"# coeff_abs_level_greater1\n");

  logtrace(LogSlice,"  cIdx:%d i:%d n:%d firstCoeffInSB:%d firstSB:%d lastSB>1:%d last>1Ctx:%d lastLev>1:%d lastCtxSet:%d\n", cIdx,i,n,firstCoeffInSubblock,firstSubblock,lastSubblock_greater1Ctx,
         *lastInvocation_greater1Ctx,
         *lastInvocation_coeff_abs_level_greater1_flag,
         *lastInvocation_ctxSet);

  int context = shdr->initType*24;

  int lastGreater1Ctx;
  int greater1Ctx;
  int ctxSet;

  logtrace(LogSlice,"c1: %d\n",c1);

  if (firstCoeffInSubblock) {
    // block with real DC -> ctx 0
    if (i==0 || cIdx>0) { ctxSet=0; }
    else { ctxSet=2; }

    if (firstSubblock) { lastGreater1Ctx=1; }
    else { lastGreater1Ctx = lastSubblock_greater1Ctx; }

    if (lastGreater1Ctx==0) { ctxSet++; }

    logtrace(LogSlice,"ctxSet: %d\n",ctxSet);

    greater1Ctx=1;
  }
  else { // !firstCoeffInSubblock
    ctxSet = *lastInvocation_ctxSet;
    logtrace(LogSlice,"ctxSet (old): %d\n",ctxSet);

    greater1Ctx = *lastInvocation_greater1Ctx;
    if (greater1Ctx>0) {
      int lastGreater1Flag=*lastInvocation_coeff_abs_level_greater1_flag;
      if (lastGreater1Flag==1) greater1Ctx=0;
      else { /*if (greater1Ctx>0)*/ greater1Ctx++; }
    }
  }

  ctxSet = c1; // use HM algo

  int ctxIdxInc = (ctxSet*4) + (greater1Ctx>=3 ? 3 : greater1Ctx);

  if (cIdx>0) { ctxIdxInc+=16; }

  context += ctxIdxInc;

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->coeff_abs_level_greater1_flag_model[context]);

  *lastInvocation_greater1Ctx = greater1Ctx;
  *lastInvocation_coeff_abs_level_greater1_flag = bit;
  *lastInvocation_ctxSet = ctxSet;

  return bit;
}


int decode_coeff_abs_level_greater2(decoder_context* ctx,
                                    slice_segment_header* shdr,
                                    int cIdx, // int i,int n,
                                    int ctxSet)
{
  logtrace(LogSlice,"# coeff_abs_level_greater2\n");

  int context = shdr->initType*6;
  int ctxIdxInc;

  ctxIdxInc = ctxSet;
  if (cIdx>0) ctxIdxInc+=4;

  context += ctxIdxInc;

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->coeff_abs_level_greater2_flag_model[context]);

  return bit;
}


int decode_coeff_abs_level_remaining(decoder_context* ctx,
                                     slice_segment_header* shdr,
                                     int cRiceParam, int cTRMax)
{
  logtrace(LogSlice,"# decode_coeff_abs_level_remaining\n");

  int prefix = decode_CABAC_TR_bypass(&shdr->cabac_decoder,cRiceParam,cTRMax);

  if (prefix==cTRMax) {
    int value = decode_CABAC_EGk_bypass(&shdr->cabac_decoder,cRiceParam+1);
    value += cTRMax;
    return value;
  }
  else {
    return prefix;
  }
}


int decode_coeff_abs_level_remaining_HM(decoder_context* ctx,
                                        slice_segment_header* shdr,
                                        int param)
{
  logtrace(LogSlice,"# decode_coeff_abs_level_remaining_HM\n");

  int prefix=0;
  int codeword=0;
  do {
    prefix++;
    codeword = decode_CABAC_bypass(&shdr->cabac_decoder);
  }
  while (codeword);
  codeword = 1-codeword;
  prefix -= codeword;
  codeword=0;

  int value;

  if (prefix < /* COEF_REMAIN_BIN_REDUCTION */ 3) {
    codeword = decode_CABAC_FL_bypass(&shdr->cabac_decoder, param);
    value = (prefix<<param) + codeword;
  }
  else {
    codeword = decode_CABAC_FL_bypass(&shdr->cabac_decoder, prefix-3+param);
    value = (((1<<(prefix-3))+3-1)<<param)+codeword;
  }

  return value;
}


int decode_merge_flag(decoder_context* ctx,
                      slice_segment_header* shdr)
{
  logtrace(LogSlice,"# merge_flag\n");

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->merge_flag_model[shdr->initType-1]);

  return bit;
}


int decode_merge_idx(decoder_context* ctx,
                     slice_segment_header* shdr)
{
  logtrace(LogSlice,"# merge_idx\n");

  int idx = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->merge_idx_model[shdr->initType-1]);

  if (idx==0) {
    // nothing
  }
  else {
    idx=1;

    while (decode_CABAC_bypass(&shdr->cabac_decoder)) {
      idx++;

      if (idx==shdr->MaxNumMergeCand-1) {
        break;
      }
    }
  }

  logtrace(LogSlice,"> merge_idx = %d\n",idx);

  return idx;
}


int decode_pred_mode_flag(decoder_context* ctx,
                          slice_segment_header* shdr)
{
  logtrace(LogSlice,"# pred_mode_flag\n");

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->pred_mode_flag_model[shdr->initType-1]);

  return bit;
}

int decode_mvp_lx_flag(decoder_context* ctx,
                       slice_segment_header* shdr)
{
  logtrace(LogSlice,"# mvp_lx_flag\n");

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->mvp_lx_flag_model[shdr->initType-1]);

  return bit;
}

int decode_rqt_root_cbf(decoder_context* ctx,
                        slice_segment_header* shdr)
{
  logtrace(LogSlice,"# rqt_root_cbf\n");

  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->rqt_root_cbf_model[shdr->initType-1]);

  return bit;
}

int decode_ref_idx_lX(decoder_context* ctx,
                      slice_segment_header* shdr, int numRefIdxLXActive)
{
  logtrace(LogSlice,"# ref_idx_lX\n");

  int cMax = numRefIdxLXActive-1;

  if (cMax==0) {
    logtrace(LogSlice,"> ref_idx = 0 (cMax==0)\n");
    return 0;
  } // do check for single reference frame here

  int ctxIdxOffset = (shdr->initType-1)*2;
  int bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->ref_idx_lX_model[ctxIdxOffset+0]);

  int idx=0;

  while (bit) {
    idx++;
    if (idx==cMax) { break; }

    if (idx==1) {
      bit = decode_CABAC_bit(&shdr->cabac_decoder,
                             &shdr->ref_idx_lX_model[ctxIdxOffset+1]);
    }
    else {
      bit = decode_CABAC_bypass(&shdr->cabac_decoder);
    }
  }

  /*
  if (bit==0) {
    idx = 0;
  }
  else if (cMax==1) {
    idx = 1;
  }
  else {
    bit = decode_CABAC_bit(&shdr->cabac_decoder,
                           &shdr->ref_idx_lX_model[ctxIdxOffset+1]);

    if (bit==0) {
      idx=1;
    }
    else if (cMax==2) 
    idx=1;

    while (decode_CABAC_bypass(&shdr->cabac_decoder)) {
      idx++;

      if (idx==shdr->MaxNumMergeCand-1) {
        break;
      }
    }
  }
  */

  logtrace(LogSlice,"> ref_idx = %d\n",idx);

  return idx;
}



int read_slice_segment_data(decoder_context* ctx, slice_segment_header* shdr)
{
  init_sao_merge_leftUp_flag_context(ctx, shdr);
  init_sao_type_idx_lumaChroma_context(ctx, shdr);
  init_split_cu_context(ctx, shdr);
  init_cu_skip_flag_context(ctx, shdr);
  init_part_mode_context(ctx, shdr);
  init_prev_intra_luma_pred_flag(ctx, shdr);
  init_intra_chroma_pred_mode(ctx, shdr);
  init_cbf_luma(ctx, shdr);
  init_cbf_chroma(ctx, shdr);
  init_split_transform_flag(ctx, shdr);
  init_last_significant_coefficient_prefix(ctx, shdr);
  init_coded_sub_block_flag(ctx, shdr);
  init_significant_coeff_flag(ctx, shdr);
  init_coeff_abs_level_greater1_flag(ctx, shdr);
  init_coeff_abs_level_greater2_flag(ctx, shdr);
  init_cu_qp_delta_abs(ctx, shdr);
  init_transform_skip_flag(ctx, shdr);
  init_merge_flag(ctx, shdr);
  init_merge_idx(ctx, shdr);
  init_pred_mode_flag(ctx, shdr);
  init_abs_mvd_greater01_flag(ctx, shdr);
  init_mvp_lx_flag(ctx,shdr);
  init_rqt_root_cbf(ctx,shdr);
  init_ref_idx_lX(ctx,shdr);


  int end_of_slice_segment_flag;

  do {
    read_coding_tree_unit(ctx, shdr);
    end_of_slice_segment_flag = decode_CABAC_term_bit(&shdr->cabac_decoder);

    logtrace(LogSlice,"read CTB %d -> end=%d %d\n",shdr->CtbAddrInTS, end_of_slice_segment_flag,
           ctx->current_sps->PicSizeInCtbsY);

    shdr->CtbAddrInTS++;
    shdr->CtbAddrInRS = shdr->CtbAddrInTS; // TODO (page 46)

    if (shdr->CtbAddrInRS==ctx->current_sps->PicSizeInCtbsY &&
        end_of_slice_segment_flag == false) {

      // image full, but end_of_slice_segment_flag not set, this cannot be correct...

      return DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA;
    }

    // TODO (page 46)

  } while (!end_of_slice_segment_flag);

  return DE265_OK;
}


void read_sao(decoder_context* ctx, slice_segment_header* shdr, int xCtb,int yCtb,
              int CtbAddrInSliceSeg)
{
  logtrace(LogSlice,"# read_sao(%d,%d)\n",xCtb,yCtb);

  sao_info saoinfo;
  memset(&saoinfo,0,sizeof(sao_info));
  logtrace(LogSlice,"sizeof saoinfo: %d\n",sizeof(sao_info));


  char sao_merge_left_flag = 0;
  char sao_merge_up_flag = 0;

  if (xCtb>0) {
    char leftCtbInSliceSeg = (CtbAddrInSliceSeg>0);
    char leftCtbInTile = true; // TODO TILES

    if (leftCtbInSliceSeg && leftCtbInTile) {
      sao_merge_left_flag = decode_sao_merge_flag(ctx,shdr);
      logtrace(LogSlice,"sao_merge_left_flag: %d\n",sao_merge_left_flag);
    }
  }

  if (yCtb>0 && sao_merge_left_flag==0) {
    logtrace(LogSlice,"CtbAddrInRS:%d PicWidthInCtbsY:%d slice_segment_address:%d\n",
           shdr->CtbAddrInRS,
           ctx->current_sps->PicWidthInCtbsY,
           shdr->slice_segment_address);
    char upCtbInSliceSeg = (shdr->CtbAddrInRS - ctx->current_sps->PicWidthInCtbsY) >= shdr->slice_segment_address;
    char upCtbInTile = true; // TODO TILES

    if (upCtbInSliceSeg && upCtbInTile) {
      sao_merge_up_flag = decode_sao_merge_flag(ctx,shdr);
      logtrace(LogSlice,"sao_merge_up_flag: %d\n",sao_merge_up_flag);
    }
  }

  if (!sao_merge_up_flag && !sao_merge_left_flag) {
    for (int cIdx=0; cIdx<3; cIdx++) {
      if ((shdr->slice_sao_luma_flag && cIdx==0) ||
          (shdr->slice_sao_chroma_flag && cIdx>0)) {

        uint8_t SaoTypeIdx = 0;

        if (cIdx==0) {
          char sao_type_idx_luma = decode_sao_type_idx(ctx,shdr);
          logtrace(LogSlice,"sao_type_idx_luma: %d\n", sao_type_idx_luma);
          saoinfo.SaoTypeIdx = SaoTypeIdx = sao_type_idx_luma;
        }
        else if (cIdx==1) {
          char sao_type_idx_chroma = decode_sao_type_idx(ctx,shdr);
          logtrace(LogSlice,"sao_type_idx_chroma: %d\n", sao_type_idx_chroma);
          SaoTypeIdx = sao_type_idx_chroma;
          saoinfo.SaoTypeIdx |= SaoTypeIdx<<(2*1);
          saoinfo.SaoTypeIdx |= SaoTypeIdx<<(2*2);  // set for both chroma components
        }
        else {
          // SaoTypeIdx = 0

          SaoTypeIdx = (saoinfo.SaoTypeIdx >> (2*cIdx)) & 0x3;
        }

        if (SaoTypeIdx != 0) {
          for (int i=0;i<4;i++) {
            saoinfo.saoOffsetVal[cIdx][i] = decode_sao_offset_abs(ctx,shdr);
            logtrace(LogSlice,"saoOffsetVal[%d][%d] = %d\n",cIdx,i, saoinfo.saoOffsetVal[cIdx][i]);
          }

          int sign[4];
          if (SaoTypeIdx==1) {
            for (int i=0;i<4;i++) {
              if (saoinfo.saoOffsetVal[cIdx][i] != 0) {
                sign[i] = decode_sao_offset_sign(ctx,shdr) ? -1 : 1;
              }
            }

            saoinfo.sao_band_position[cIdx] = decode_sao_band_position(ctx,shdr);
          }
          else {
            uint8_t SaoEoClass = 0;

            sign[0] = sign[1] =  1;
            sign[2] = sign[3] = -1;

            if (cIdx==0) {
              saoinfo.SaoEoClass = SaoEoClass = decode_sao_class(ctx,shdr);
            }
            else if (cIdx==1) {
              SaoEoClass = decode_sao_class(ctx,shdr);
              saoinfo.SaoEoClass |= SaoEoClass << (2*1);
              saoinfo.SaoEoClass |= SaoEoClass << (2*2);
            }

            logtrace(LogSlice,"SaoEoClass[%d] = %d\n",cIdx,SaoEoClass);
          }

          int bitDepth = (cIdx==0 ?
                          ctx->current_sps->BitDepth_Y :
                          ctx->current_sps->BitDepth_C);
          int shift = bitDepth-min(bitDepth,10);

          for (int i=0;i<4;i++) {
            saoinfo.saoOffsetVal[cIdx][i] = sign[i]*(saoinfo.saoOffsetVal[cIdx][i] << shift);
          }
        }
      }
    }

    set_sao_info(ctx,xCtb,yCtb,  &saoinfo);
  }


  if (sao_merge_left_flag) {
    set_sao_info(ctx,xCtb,yCtb,  get_sao_info(ctx,xCtb-1,yCtb));
  }

  if (sao_merge_up_flag) {
    set_sao_info(ctx,xCtb,yCtb,  get_sao_info(ctx,xCtb,yCtb-1));
  }
}


void read_coding_tree_unit(decoder_context* ctx, slice_segment_header* shdr)
{
  seq_parameter_set* sps = ctx->current_sps;

  int xCtb = (shdr->CtbAddrInRS % sps->PicWidthInCtbsY);
  int yCtb = (shdr->CtbAddrInRS / sps->PicWidthInCtbsY);
  int xCtbPixels = xCtb << sps->Log2CtbSizeY;
  int yCtbPixels = yCtb << sps->Log2CtbSizeY;

  set_SliceAddrRS(ctx, xCtb, yCtb,
                  shdr->SliceAddrRS);

  set_SliceHeaderIndex(ctx,xCtbPixels,yCtbPixels, shdr->slice_index);


  int CtbAddrInSliceSeg = shdr->CtbAddrInRS - shdr->slice_segment_address;

  if (shdr->slice_sao_luma_flag || shdr->slice_sao_chroma_flag)
    {
      read_sao(ctx,shdr, xCtb,yCtb, CtbAddrInSliceSeg);
    }

  read_coding_quadtree(ctx,shdr, xCtbPixels, yCtbPixels, sps->Log2CtbSizeY, 0);
}


int luma_pos_to_ctbAddrRS(decoder_context* ctx, int x,int y)
{
  int ctbX = x >> (ctx->current_sps->Log2CtbSizeY);
  int ctbY = y >> (ctx->current_sps->Log2CtbSizeY);

  return ctbY * ctx->current_sps->PicWidthInCtbsY + ctbX;
}


int check_CTB_available(decoder_context* ctx,
                        slice_segment_header* shdr,
                        int xC,int yC, int xN,int yN)
{
  // check whether neighbor is outside of frame

  if (xN < 0 || yN < 0) { return 0; }
  if (xN >= ctx->current_sps->pic_width_in_luma_samples)  { return 0; }
  if (yN >= ctx->current_sps->pic_height_in_luma_samples) { return 0; }


  //int ctbAddrRS = luma_pos_to_ctbAddrRS(ctx, xC,yC);
  //int ctbAddrTS = ctx->current_pps->CtbAddrRStoTS[ ctbAddrRS ];

  // TODO: check if this is correct (6.4.1)

  int neighbor_ctbAddrRS = luma_pos_to_ctbAddrRS(ctx, xN,yN);
  int neighbor_ctbAddrTS = ctx->current_pps->CtbAddrRStoTS[ neighbor_ctbAddrRS ];


  // check whether neighbor is in the same slice

  int first_ctb_in_slice_TS = ctx->current_pps->CtbAddrRStoTS[ shdr->slice_segment_address ];

  if (neighbor_ctbAddrTS < first_ctb_in_slice_TS) {
    return 0;
  }

  return 1;
}


int residual_coding(decoder_context* ctx,
                    slice_segment_header* shdr,
                    int x0, int y0,
                    int log2TrafoSize,
                    int cIdx)
{
  logtrace(LogSlice,"- residual_coding x0:%d y0:%d log2TrafoSize:%d cIdx:%d\n",x0,y0,log2TrafoSize,cIdx);

  //seq_parameter_set* sps = ctx->current_sps;


  shdr->cu_transquant_bypass_flag=0; // TODO

  if (ctx->current_pps->transform_skip_enabled_flag &&
      !shdr->cu_transquant_bypass_flag &&
      (log2TrafoSize==2))
    {
      int transform_skip_flag = decode_transform_skip_flag(ctx,shdr,cIdx);
      if (transform_skip_flag) {
        set_transform_skip_flag(ctx,x0,y0,cIdx);
      }
    }

  int last_significant_coeff_x_prefix =
    decode_last_significant_coeff_prefix(ctx,shdr,log2TrafoSize,cIdx,
                                         shdr->last_significant_coefficient_x_prefix_model);

  int last_significant_coeff_y_prefix =
    decode_last_significant_coeff_prefix(ctx,shdr,log2TrafoSize,cIdx,
                                         shdr->last_significant_coefficient_y_prefix_model);


  int LastSignificantCoeffX;
  if (last_significant_coeff_x_prefix > 3) {
    int nBits = (last_significant_coeff_x_prefix>>1)-1;
    int last_significant_coeff_x_suffix = decode_CABAC_FL_bypass(&shdr->cabac_decoder,nBits);

    LastSignificantCoeffX = (1<<nBits) *
      (2+(last_significant_coeff_x_prefix & 1)) + last_significant_coeff_x_suffix;
  }
  else {
    LastSignificantCoeffX = last_significant_coeff_x_prefix;
  }

  int LastSignificantCoeffY;
  if (last_significant_coeff_y_prefix > 3) {
    int nBits = (last_significant_coeff_y_prefix>>1)-1;
    int last_significant_coeff_y_suffix = decode_CABAC_FL_bypass(&shdr->cabac_decoder,nBits);

    LastSignificantCoeffY = (1<<nBits) *
      (2+(last_significant_coeff_y_prefix & 1)) + last_significant_coeff_y_suffix;
  }
  else {
    LastSignificantCoeffY = last_significant_coeff_y_prefix;
  }


  int IntraPredMode = get_IntraPredMode(ctx,x0,y0);
  logtrace(LogSlice,"IntraPredMode[%d,%d] = %d\n",x0,y0,IntraPredMode);


  // --- find last sub block and last scan pos ---

  int lastScanPos = 16;
  int lastSubBlock = (1<<(log2TrafoSize-2)) * (1<<(log2TrafoSize-2)) -1;

  

  int scanIdx;

  enum PredMode PredMode = MODE_INTRA; // HACK (TODO: take from decctx)

#if 0
  // process from Draft-10, does not seem to fit to HM9.1
  if (PredMode == MODE_INTRA &&
      (log2TrafoSize==2 || (log2TrafoSize==3 && cIdx==0))) {
    if (IntraPredMode>= 6 && IntraPredMode<=14) scanIdx=2;
    else if (IntraPredMode>=22 && IntraPredMode<=30) scanIdx=1;
    else scanIdx=0;
  }
  else {
    scanIdx=0;
  }
#else
  // scanIdx derived as by HM9.1

  if (PredMode == MODE_INTRA) {
    int pred;

    if (cIdx==0) {
      pred = IntraPredMode;

      if (log2TrafoSize==2 || log2TrafoSize==3) {
        if (pred>= 6 && pred<=14) scanIdx=2;
        else if (pred>=22 && pred<=30) scanIdx=1;
        else scanIdx=0;
      }
      else { scanIdx=0; }
    }
    else {
      pred = get_IntraPredModeC(ctx,x0,y0);

      if (log2TrafoSize==1 || log2TrafoSize==2) {
        if (pred>= 6 && pred<=14) scanIdx=2;
        else if (pred>=22 && pred<=30) scanIdx=1;
        else scanIdx=0;
      }
      else { scanIdx=0; }
    }

    logtrace(LogSlice,"pred: %d -> scan: %d\n",pred,scanIdx);
  }
  else {
    assert(0);
    scanIdx=0;
  }
#endif

  // HM 9 only ?
  if (scanIdx==2) {
    int t = LastSignificantCoeffX;
    LastSignificantCoeffX = LastSignificantCoeffY;
    LastSignificantCoeffY = t;
  }

  logtrace(LogSlice,"LastSignificantCoeff: x=%d;y=%d\n",LastSignificantCoeffX,LastSignificantCoeffY);

  const position* ScanOrderSub = get_scan_order(log2TrafoSize-2, scanIdx);
  const position* ScanOrderPos = get_scan_order(2, scanIdx);

  logtrace(LogSlice,"ScanOrderPos: ");
  for (int n=0;n<4*4;n++)
    logtrace(LogSlice,"*%d,%d ", ScanOrderPos[n].x, ScanOrderPos[n].y);
  logtrace(LogSlice,"*\n");


  int xC,yC;
  do {
    if (lastScanPos==0) {
      lastScanPos=16;
      lastSubBlock--;
    }
    lastScanPos--;

    position S = ScanOrderSub[lastSubBlock];
    xC = (S.x<<2) + ScanOrderPos[lastScanPos].x;
    yC = (S.y<<2) + ScanOrderPos[lastScanPos].y;

  } while ( (xC != LastSignificantCoeffX) || (yC != LastSignificantCoeffY));

  logtrace(LogSlice,"lastScanPos=%d lastSubBlock=%d\n",lastScanPos,lastSubBlock);


  int sbWidth = 1<<(log2TrafoSize-2);
  uint8_t *const coded_sub_block_flag = (uint8_t *)alloca((sbWidth*sbWidth) * sizeof(uint8_t));
  memset(coded_sub_block_flag,0,sbWidth*sbWidth);

  int  c1 = 1;
  bool firstSubblock = true;           // for coeff_abs_level_greater1_flag context model
  int  lastSubblock_greater1Ctx=false; /* for coeff_abs_level_greater1_flag context model
                                          (initialization not strictly needed)
                                       */

  int16_t TransCoeffLevel[32][32];
  memset(TransCoeffLevel,0, sizeof(uint16_t)*32*32); // actually, we only need [1<<log2TrafoSize][1<<log2TrafoSize] = (1<<(2*log2TrafoSize))


  int  lastInvocation_greater1Ctx;
  int  lastInvocation_coeff_abs_level_greater1_flag;
  int  lastInvocation_ctxSet;

  for (int i=lastSubBlock;i>=0;i--) {
    position S = ScanOrderSub[i];
    int inferSbDcSigCoeffFlag=0;

    logtrace(LogSlice,"sub block scan idx: %d\n",i);

    uint8_t significant_coeff_flag[4][4];
    memset(significant_coeff_flag, 0, 4*4/*sbWidth*sbWidth*/);

    if ((i<lastSubBlock) && (i>0)) {
      coded_sub_block_flag[S.x+S.y*sbWidth] = decode_coded_sub_block_flag(ctx,shdr, cIdx,sbWidth,S.x,S.y, coded_sub_block_flag);
      inferSbDcSigCoeffFlag=1;
    }
    else {
      if (S.x==0 && S.y==0) {
        coded_sub_block_flag[S.x+S.y*sbWidth]=1;
      }
      else if (S.x==LastSignificantCoeffX>>2 && S.y==LastSignificantCoeffY>>2) {
        coded_sub_block_flag[S.x+S.y*sbWidth]=1;
      }
    }


    bool hasNonZero = false;

    for (int n= (i==lastSubBlock) ? lastScanPos-1 : 15 ;
         n>=0 ; n--) {
      xC = (S.x<<2) + ScanOrderPos[n].x;
      yC = (S.y<<2) + ScanOrderPos[n].y;
      int subX = ScanOrderPos[n].x;
      int subY = ScanOrderPos[n].y;

      logtrace(LogSlice,"n=%d , S.x=%d S.y=%d\n",n,S.x,S.y);

      if (coded_sub_block_flag[S.x+S.y*sbWidth] && (n>0 || !inferSbDcSigCoeffFlag)) {
        significant_coeff_flag[subY][subX] = decode_significant_coeff_flag(ctx,shdr, xC,yC, coded_sub_block_flag,sbWidth, cIdx,scanIdx);
        if (significant_coeff_flag[subY][subX]) {
          hasNonZero=true;
          inferSbDcSigCoeffFlag = 0;
        }
      }
      else {
        /*
          if (xC==LastSignificantCoeffX &&
          yC==LastSignificantCoeffY) {
          significant_coeff_flag[yC][xC]=1;
          }
          else*/
        if (((xC&3)==0 && (yC&3)==0) &&
            inferSbDcSigCoeffFlag==1 &&
            coded_sub_block_flag[S.x+S.y*sbWidth]==1) {
          significant_coeff_flag[subY][subX]=1;
          hasNonZero=true;
        }
        else {
          significant_coeff_flag[subY][subX]=0;
        }
      }
    }

    if (i==lastSubBlock) {
      significant_coeff_flag[LastSignificantCoeffY%4][LastSignificantCoeffX%4] = 1;
      hasNonZero=true;
    }



    logtrace(LogSlice,"significant_coeff_flags:\n");
    for (int y=0;y<4;y++) {
      logtrace(LogSlice,"  ");
      for (int x=0;x<4;x++) {
        logtrace(LogSlice,"*%d ",significant_coeff_flag[y][x]);
      }
      logtrace(LogSlice,"*\n");
    }


    int firstSigScanPos=16;
    int lastSigScanPos=-1;
    int numGreater1Flag=0;
    int lastGreater1ScanPos=-1;

    uint8_t coeff_abs_level_greater1_flag[16];
    uint8_t coeff_abs_level_greater2_flag[16];
    uint8_t coeff_sign_flag[16];

    bool firstCoeffInSubblock = true;

    memset(coeff_abs_level_greater1_flag,0,16);
    memset(coeff_abs_level_greater2_flag,0,16);
    memset(coeff_sign_flag,0,16); // TODO check: do we need this?

    if (hasNonZero) {
      int ctxSet;
      if (i==0 || cIdx>0) { ctxSet=0; }
      else { ctxSet=2; }

      if (c1==0) { ctxSet++; }
      c1=1;

      for (int n=15;n>=0;n--) {
        xC = (S.x<<2) + ScanOrderPos[n].x;
        yC = (S.y<<2) + ScanOrderPos[n].y;
        int subX = ScanOrderPos[n].x;
        int subY = ScanOrderPos[n].y;

        if (significant_coeff_flag[subY][subX]) {
          if (numGreater1Flag<8) {
            coeff_abs_level_greater1_flag[n] =
              decode_coeff_abs_level_greater1(ctx,shdr, cIdx,i,n,
                                              firstCoeffInSubblock,
                                              firstSubblock,
                                              lastSubblock_greater1Ctx,
                                              &lastInvocation_greater1Ctx,
                                              &lastInvocation_coeff_abs_level_greater1_flag,
                                              &lastInvocation_ctxSet, ctxSet);
            numGreater1Flag++;

            if (coeff_abs_level_greater1_flag[n]) {
              c1=0;
            }
            else if (c1<3 && c1>0) {
              c1++;
            }

            if (coeff_abs_level_greater1_flag[n] && lastGreater1ScanPos == -1) {
              lastGreater1ScanPos=n;
            }
          }

          if (lastSigScanPos==-1) {
            lastSigScanPos=n;
          }

          firstSigScanPos=n;

          firstCoeffInSubblock = false;
        }
      }
    }

    firstSubblock = false;
    lastSubblock_greater1Ctx = lastInvocation_greater1Ctx;


    logtrace(LogSlice,"lastGreater1ScanPos=%d\n",lastGreater1ScanPos);

    int signHidden = (lastSigScanPos-firstSigScanPos > 3 &&
                      !shdr->cu_transquant_bypass_flag);

    if (lastGreater1ScanPos != -1) {
      coeff_abs_level_greater2_flag[lastGreater1ScanPos] =
        decode_coeff_abs_level_greater2(ctx,shdr,cIdx, lastInvocation_ctxSet);
    }


    // --- decode coefficient signs ---

    for (int n=15;n>=0;n--) {
      xC = (S.x<<2) + ScanOrderPos[n].x;
      yC = (S.y<<2) + ScanOrderPos[n].y;
      int subX = ScanOrderPos[n].x;
      int subY = ScanOrderPos[n].y;

      if (significant_coeff_flag[subY][subX]) {
        if (!ctx->current_pps->sign_data_hiding_flag ||
            !signHidden ||
            n != firstSigScanPos) {
          coeff_sign_flag[n] = decode_CABAC_bypass(&shdr->cabac_decoder);
          logtrace(LogSlice,"sign[%d] = %d\n", n, coeff_sign_flag[n]);
        }
      }
    }


    // --- decode coefficient value ---

    int numSigCoeff=0;
    int sumAbsLevel=0;

    int coeff_abs_level_remaining[16];
    memset(coeff_abs_level_remaining,0,16*sizeof(int));

    int cLastAbsLevel  = 0;
    int cLastRiceParam = 0;

    int uiGoRiceParam=0;

    for (int n=15;n>=0;n--) {
      xC = (S.x<<2) + ScanOrderPos[n].x;
      yC = (S.y<<2) + ScanOrderPos[n].y;
      int subX = ScanOrderPos[n].x;
      int subY = ScanOrderPos[n].y;

      logtrace(LogSlice,"read coefficient %d (%d,%d) [full blk scan pos: %d]\n",n,xC,yC,
             (yC+subY*4)*4+(xC+subX*4));

      if (significant_coeff_flag[subY][subX]) {
        int baseLevel = 1 + coeff_abs_level_greater1_flag[n] + coeff_abs_level_greater2_flag[n];

        logtrace(LogSlice,"baseLevel=%d\n",baseLevel);

        int checkLevel;
        if (numSigCoeff<8) {
          if (n==lastGreater1ScanPos) {
            checkLevel=3;
          }
          else {
            checkLevel=2;
          }
        }
        else {
          checkLevel=1;
        }

        logtrace(LogSlice,"checkLevel=%d\n",checkLevel);

        logtrace(LogSlice,"cLastAbsLevel=%d   cLastRiceParam=%d\n",cLastAbsLevel,cLastRiceParam);

#if 0
        int cRiceParam = cLastRiceParam;
        cRiceParam += cLastAbsLevel > (3*(1<<cLastRiceParam)) ? 1 : 0;
        if (cRiceParam>4) cRiceParam=4;

        int cTRMax = 4<<cRiceParam;

        if (baseLevel==checkLevel) {
          coeff_abs_level_remaining[n] =
            decode_coeff_abs_level_remaining(ctx,shdr, cRiceParam,cTRMax);
        }


        cLastAbsLevel = baseLevel + coeff_abs_level_remaining[n];
        cLastRiceParam= cRiceParam;
#else
        if (baseLevel==checkLevel) {
          coeff_abs_level_remaining[n] =
            decode_coeff_abs_level_remaining_HM(ctx,shdr, uiGoRiceParam);

          if (baseLevel + coeff_abs_level_remaining[n] > 3*(1<<uiGoRiceParam)) {
            uiGoRiceParam++;
            if (uiGoRiceParam>4) uiGoRiceParam=4;
          }
        }
#endif


        TransCoeffLevel[xC][yC] = (baseLevel + coeff_abs_level_remaining[n]);
        if (coeff_sign_flag[n]) {
          TransCoeffLevel[xC][yC] = -TransCoeffLevel[xC][yC];
        }

        if (ctx->current_pps->sign_data_hiding_flag && signHidden) {
          sumAbsLevel += baseLevel + coeff_abs_level_remaining[n];

          if (n==firstSigScanPos && (sumAbsLevel & 1)) {
            TransCoeffLevel[xC][yC] = -TransCoeffLevel[xC][yC];
          }
        }

        numSigCoeff++;
      }
      else {
        cLastAbsLevel = 0;
        cLastRiceParam= 0;
      }
    }

  }



  //int nSubBlocks = (1<<(log2TrafoSize-2)) * (1<<(log2TrafoSize-2)) -1;

  int xB = x0;
  int yB = y0;
  if (cIdx>0) { xB/=2; yB/=2; }

  logtrace(LogSlice,"coefficients [cIdx=%d,at %d,%d] ----------------------------------------\n",cIdx,xB,yB);
  for (int y=0;y<(1<<log2TrafoSize);y++) {
    logtrace(LogSlice,"  ");
    for (int x=0;x<(1<<log2TrafoSize);x++) {
      logtrace(LogSlice,"*%3d ", TransCoeffLevel[x][y]);
    }
    logtrace(LogSlice,"*\n");
  }

  int16_t* coeff;
  int      coeffStride;
  get_coeff_plane(ctx,cIdx, &coeff,&coeffStride);

  if ((1<<log2TrafoSize)+yB > ctx->img->height) {
    return DE265_ERROR_COEFFICIENT_OUT_OF_IMAGE_BOUNDS;
  }

  for (int y=0;y<(1<<log2TrafoSize);y++)
    for (int x=0;x<(1<<log2TrafoSize);x++) {
      coeff[x+xB+(y+yB)*coeffStride] = TransCoeffLevel[x][y];
    }

  return DE265_OK;
}


int read_transform_unit(decoder_context* ctx,
                        slice_segment_header* shdr,
                        int x0, int y0,
                        int xBase, int yBase,
                        int log2TrafoSize,
                        int trafoDepth,
                        int blkIdx,
                        int cbf_luma, int cbf_cb, int cbf_cr)
{
  logtrace(LogSlice,"- read_transform_unit x0:%d y0:%d xBase:%d yBase:%d cbf:%d:%d:%d\n",
         x0,y0,xBase,yBase, cbf_luma, cbf_cb, cbf_cr);

  assert(cbf_cb != -1);
  assert(cbf_cr != -1);
  assert(cbf_luma != -1);

  if (cbf_luma || cbf_cb || cbf_cr)
    {
      if (ctx->current_pps->cu_qp_delta_enabled_flag &&
          !shdr->IsCuQpDeltaCoded) {

        int cu_qp_delta_abs = decode_cu_qp_delta_abs(ctx,shdr);
        int cu_qp_delta_sign=0;
        if (cu_qp_delta_abs) {
          cu_qp_delta_sign = decode_CABAC_bypass(&shdr->cabac_decoder);
        }

        shdr->IsCuQpDeltaCoded = 1;
        shdr->CuQpDelta = cu_qp_delta_abs*(1-2*cu_qp_delta_sign);
      }

      int err;
      if (cbf_luma) {
        if ((err=residual_coding(ctx,shdr,x0,y0,log2TrafoSize,0)) != DE265_OK) return err;
      }

      if (log2TrafoSize>2) {
        if (cbf_cb) {
          if ((err=residual_coding(ctx,shdr,x0,y0,log2TrafoSize-1,1)) != DE265_OK) return err;
        }

        if (cbf_cr) {
          if ((err=residual_coding(ctx,shdr,x0,y0,log2TrafoSize-1,2)) != DE265_OK) return err;
        }
      }
      else if (blkIdx==3) {
        if (cbf_cb) {
          if ((err=residual_coding(ctx,shdr,xBase,yBase,log2TrafoSize,1)) != DE265_OK) return err;
        }

        if (cbf_cr) {
          if ((err=residual_coding(ctx,shdr,xBase,yBase,log2TrafoSize,2)) != DE265_OK) return err;
        }
      }
    }

  return DE265_OK;
}


void read_transform_tree(decoder_context* ctx,
                         slice_segment_header* shdr,
                         int x0, int y0,
                         int xBase, int yBase,
                         int log2TrafoSize,
                         int trafoDepth,
                         int blkIdx,
                         int MaxTrafoDepth,
                         int IntraSplitFlag)
{
  logtrace(LogSlice,"- read_transform_tree x0:%d y0:%d xBase:%d yBase:%d "
         "log2TrafoSize:%d trafoDepth:%d MaxTrafoDepth:%d\n",
         x0,y0,xBase,yBase,log2TrafoSize,trafoDepth,MaxTrafoDepth);

  enum PredMode PredMode = get_pred_mode(ctx,x0,y0);

  int split_transform_flag;
  
  int interSplitFlag=0; // TODO

  if (log2TrafoSize <= ctx->current_sps->Log2MaxTrafoSize &&
      log2TrafoSize >  ctx->current_sps->Log2MinTrafoSize &&
      trafoDepth < MaxTrafoDepth &&
      !(IntraSplitFlag && trafoDepth==0))
    {
      split_transform_flag = decode_split_transform_flag(ctx,shdr, log2TrafoSize);
    }
  else
    {
      split_transform_flag = (log2TrafoSize > ctx->current_sps->Log2MaxTrafoSize ||
                              (IntraSplitFlag==1 && trafoDepth==0) ||
                              interSplitFlag==1) ? 1:0;
    }


  if (split_transform_flag) {
    logtrace(LogSlice,"set_split_transform_flag(%d,%d, %d)\n",x0,y0,trafoDepth);
    set_split_transform_flag(ctx,x0,y0,trafoDepth);
  }


  int cbf_cb=-1;
  int cbf_cr=-1;

  if (log2TrafoSize>2) {
    if (trafoDepth==0 || get_cbf_cb(ctx,xBase,yBase,trafoDepth-1)) {
      cbf_cb = decode_cbf_chroma(ctx,shdr,trafoDepth);
    }

    if (trafoDepth==0 || get_cbf_cr(ctx,xBase,yBase,trafoDepth-1)) {
      cbf_cr = decode_cbf_chroma(ctx,shdr,trafoDepth);
    }
  }


  // cbf_cr/cbf_cb not present in bitstream -> induce values

  if (cbf_cb<0) {
    if (trafoDepth>0 && log2TrafoSize==2) {
      cbf_cb = get_cbf_cb(ctx,xBase,yBase,trafoDepth-1);
    } else {
      cbf_cb=0;
    }
  }

  if (cbf_cb) {
    set_cbf_cb(ctx,x0,y0, trafoDepth);
  }
  logtrace(LogSlice,"check cbf_cb[%d;%d;%d]: %d\n", xBase,yBase,trafoDepth,
         get_cbf_cb(ctx,xBase,yBase,trafoDepth));

  if (cbf_cr<0) {
    if (trafoDepth>0 && log2TrafoSize==2) {
      cbf_cr = get_cbf_cr(ctx,xBase,yBase,trafoDepth-1);
    } else {
      cbf_cr=0;
    }
  }

  if (cbf_cr) {
    set_cbf_cr(ctx,x0,y0, trafoDepth);
  }
  logtrace(LogSlice,"check cbf_cr[%d;%d;%d]: %d\n", xBase,yBase,trafoDepth,
         get_cbf_cr(ctx,xBase,yBase,trafoDepth));

  if (split_transform_flag) {
    int x1 = x0 + (1<<(log2TrafoSize-1));
    int y1 = y0 + (1<<(log2TrafoSize-1));

    logtrace(LogSlice,"transform split.\n");

    read_transform_tree(ctx,shdr, x0,y0, x0,y0, log2TrafoSize-1, trafoDepth+1, 0,
                        MaxTrafoDepth,IntraSplitFlag);
    read_transform_tree(ctx,shdr, x1,y0, x0,y0, log2TrafoSize-1, trafoDepth+1, 1,
                        MaxTrafoDepth,IntraSplitFlag);
    read_transform_tree(ctx,shdr, x0,y1, x0,y0, log2TrafoSize-1, trafoDepth+1, 2,
                        MaxTrafoDepth,IntraSplitFlag);
    read_transform_tree(ctx,shdr, x1,y1, x0,y0, log2TrafoSize-1, trafoDepth+1, 3,
                        MaxTrafoDepth,IntraSplitFlag);
  }
  else {
    int cbf_luma=1;

    if (PredMode==MODE_INTRA || trafoDepth!=0 || cbf_cb || cbf_cr) {
      cbf_luma = decode_cbf_luma(ctx,shdr,trafoDepth);
    }

    read_transform_unit(ctx,shdr, x0,y0,xBase,yBase, log2TrafoSize,trafoDepth, blkIdx,
                        cbf_luma, cbf_cb, cbf_cr);
  }
}


static const char* part_mode_name(enum PartMode pm)
{
  switch (pm) {
  case PART_2Nx2N: return "2Nx2N";
  case PART_2NxN:  return "2NxN";
  case PART_Nx2N:  return "Nx2N";
  case PART_NxN:   return "NxN";
  case PART_2NxnU: return "2NxnU";
  case PART_2NxnD: return "2NxnD";
  case PART_nLx2N: return "nLx2N";
  case PART_nRx2N: return "nRx2N";
  }
}


void read_mvd_coding(decoder_context* ctx,
                     slice_segment_header* shdr,
                     int x0,int y0, int refList)
{
  int ctxIdxOffset = (shdr->slice_type == SLICE_TYPE_P) ? 0 : 2;

  int abs_mvd_greater0_flag[2];
  abs_mvd_greater0_flag[0] = decode_CABAC_bit(&shdr->cabac_decoder,
                                              &shdr->abs_mvd_greater01_flag_model[ctxIdxOffset+0]);
  abs_mvd_greater0_flag[1] = decode_CABAC_bit(&shdr->cabac_decoder,
                                              &shdr->abs_mvd_greater01_flag_model[ctxIdxOffset+0]);

  int abs_mvd_greater1_flag[2];
  if (abs_mvd_greater0_flag[0]) {
    abs_mvd_greater1_flag[0] = decode_CABAC_bit(&shdr->cabac_decoder,
                                                &shdr->abs_mvd_greater01_flag_model[ctxIdxOffset+1]);
  }
  else {
    abs_mvd_greater1_flag[0]=0;
  }

  if (abs_mvd_greater0_flag[1]) {
    abs_mvd_greater1_flag[1] = decode_CABAC_bit(&shdr->cabac_decoder,
                                                &shdr->abs_mvd_greater01_flag_model[ctxIdxOffset+1]);
  }
  else {
    abs_mvd_greater1_flag[1]=0;
  }


  int abs_mvd_minus2[2];
  int mvd_sign_flag[2];
  int value[2];

  for (int c=0;c<2;c++) {
    if (abs_mvd_greater0_flag[c]) {
      if (abs_mvd_greater1_flag[c]) {
        abs_mvd_minus2[c] = decode_CABAC_EGk_bypass(&shdr->cabac_decoder, 1);
      }
      else {
        abs_mvd_minus2[c] = abs_mvd_greater1_flag[c] -1;
      }

      mvd_sign_flag[c] = decode_CABAC_bypass(&shdr->cabac_decoder);

      value[c] = abs_mvd_minus2[c]+2;
      if (mvd_sign_flag[c]) { value[c] = -value[c]; }
    }
    else {
      value[c] = 0;
    }
  }

  set_mvd(ctx, x0,y0, refList, value[0],value[1]);

  logtrace(LogSlice, "MVD[%d;%d|%d] = %d;%d\n",x0,y0,refList, value[0],value[1]);
}


void read_prediction_unit_SKIP(decoder_context* ctx,
                               slice_segment_header* shdr,
                               int x0, int y0,
                               int nPbW, int nPbH)
{
  int merge_idx = 0;
  if (shdr->MaxNumMergeCand>1) {
    merge_idx = decode_merge_idx(ctx,shdr);
  }

  set_merge_idx(ctx,x0,y0, nPbW,nPbH, merge_idx);
  set_merge_flag(ctx,x0,y0,nPbW,nPbH, true);

  logtrace(LogSlice,"prediction skip 2Nx2N, merge_idx: %d\n",merge_idx);
}


void read_prediction_unit(decoder_context* ctx,
                          slice_segment_header* shdr,
                          int x0, int y0,
                          int nPbW, int nPbH)
{
  logtrace(LogSlice,"read_prediction_unit %d;%d %dx%d\n",x0,y0,nPbW,nPbH);

  int merge_flag = decode_merge_flag(ctx,shdr);
  set_merge_flag(ctx,x0,y0,nPbW,nPbH, merge_flag);

  if (merge_flag) {
    int merge_idx = 0;

    if (shdr->MaxNumMergeCand>1) {
      merge_idx = decode_merge_idx(ctx,shdr);
    }

    logtrace(LogSlice,"prediction unit %d,%d, merge mode, index: %d\n",x0,y0,merge_idx);

    set_merge_idx(ctx,x0,y0, nPbW,nPbH, merge_idx);
  }
  else { // no merge flag
    enum InterPredIdc inter_pred_idc;

    if (shdr->slice_type==SLICE_TYPE_B) {
      assert(0); // TODO: inter_pred_idc
    }
    else {
      inter_pred_idc = PRED_L0;
    }

    set_inter_pred_idc(ctx,x0,y0,0, inter_pred_idc);

    if (inter_pred_idc != PRED_L1) {
      int ref_idx_l0 = decode_ref_idx_lX(ctx,shdr, shdr->num_ref_idx_l0_active);

      // NOTE: case for only one reference frame is handles in decode_ref_idx_lX()
      set_ref_idx(ctx,x0,y0,nPbW,nPbH,0, ref_idx_l0);

      read_mvd_coding(ctx,shdr,x0,y0, 0);

      int mvp_l0_flag = decode_mvp_lx_flag(ctx,shdr); // l0
      set_mvp_flag(ctx,x0,y0,nPbW,nPbH,0, mvp_l0_flag);

      logtrace(LogSlice,"prediction unit %d,%d, L0, mvp_l0_flag:%d\n",
               x0,y0, mvp_l0_flag);
    }

    if (inter_pred_idc != PRED_L0) {
      assert(0);
    }
  }
}


void read_coding_unit(decoder_context* ctx,
                      slice_segment_header* shdr,
                      int x0, int y0,
                      int log2CbSize,
                      int ctDepth)
{
  logtrace(LogSlice,"- read_coding_unit %d;%d cbsize:%d\n",x0,y0,1<<log2CbSize);

  set_log2CbSize(ctx, x0,y0, log2CbSize);

  int nCbS = 1<<log2CbSize; // number of coding block samples


  //enum PredMode PredMode = MODE_INTRA; // TODO: HACK for intra only decoder
  //set_pred_mode(ctx,x0,y0,log2CbSize, MODE_INTRA); // HACK, TODO: decode and set correct values


  const seq_parameter_set* sps = ctx->current_sps;


  if (ctx->current_pps->transquant_bypass_enable_flag)
    {
      assert(false); // TODO
    }

  uint8_t cu_skip_flag = 0;
  if (shdr->slice_type != SLICE_TYPE_I) {
    cu_skip_flag = decode_cu_skip_flag(ctx,shdr,x0,y0,ctDepth);
  }

  set_cu_skip_flag(ctx,x0,y0,log2CbSize, cu_skip_flag);

  int IntraSplitFlag = 0;

  enum PredMode cuPredMode;

  if (cu_skip_flag) {
    read_prediction_unit_SKIP(ctx,shdr,x0,y0,nCbS,nCbS);

    set_PartMode(ctx, x0,y0, PART_2Nx2N); // TODO: not sure if we need this
    set_pred_mode(ctx,x0,y0,log2CbSize, MODE_SKIP);
    cuPredMode = MODE_SKIP;

    logtrace(LogSlice,"CU pred mode: SKIP\n");
  }
  else /* not skipped */ {
    if (shdr->slice_type != SLICE_TYPE_I) {
      int pred_mode_flag = decode_pred_mode_flag(ctx,shdr);
      cuPredMode = pred_mode_flag ? MODE_INTRA : MODE_INTER;
    }
    else {
      cuPredMode = MODE_INTRA;
    }

    set_pred_mode(ctx,x0,y0,log2CbSize, cuPredMode);

    logtrace(LogSlice,"CU pred mode: %s\n", cuPredMode==MODE_INTRA ? "INTRA" : "INTER");


    enum PartMode PartMode;

    if (cuPredMode != MODE_INTRA ||
        log2CbSize == sps->Log2MinCbSizeY) {
      PartMode = decode_part_mode(ctx,shdr, cuPredMode, log2CbSize);

      if (PartMode==PART_NxN && cuPredMode==MODE_INTRA) {
        IntraSplitFlag=1;
      }
    } else {
      PartMode = PART_2Nx2N;
    }

    set_PartMode(ctx, x0,y0, PartMode);  // currently not required for decoding (but for visualization)

    logtrace(LogSlice, "PartMode: %s\n", part_mode_name(PartMode));


    if (cuPredMode == MODE_INTRA) {
      assert(!sps->pcm_enabled_flag); // TODO

      if (false) {
      }
      else {
        int pbOffset = (PartMode == PART_NxN) ? (nCbS/2) : nCbS;
        int log2IntraPredSize = (PartMode == PART_NxN) ? (log2CbSize-1) : log2CbSize;

        logtrace(LogSlice,"nCbS:%d pbOffset:%d\n",nCbS,pbOffset);

        int prev_intra_luma_pred_flag[4];

        int idx=0;
        for (int j=0;j<nCbS;j+=pbOffset)
          for (int i=0;i<nCbS;i+=pbOffset)
            {
              prev_intra_luma_pred_flag[idx++] = decode_prev_intra_luma_pred_flag(ctx,shdr);
            }

        int mpm_idx[4], rem_intra_luma_pred_mode[4];
        idx=0;

        for (int j=0;j<nCbS;j+=pbOffset)
          for (int i=0;i<nCbS;i+=pbOffset)
            {
              if (prev_intra_luma_pred_flag[idx]) {
                mpm_idx[idx] = decode_mpm_idx(ctx,shdr);
              }
              else {
                rem_intra_luma_pred_mode[idx] = decode_rem_intra_luma_pred_mode(ctx,shdr);
              }


              int x = x0+i;
              int y = y0+j;

              // --- find intra prediction mode ---

              int IntraPredMode;

              int availableA = check_CTB_available(ctx, shdr, x,y, x-1,y);
              int availableB = check_CTB_available(ctx, shdr, x,y, x,y-1);

              // block on left side


              enum IntraPredMode candIntraPredModeA, candIntraPredModeB;
              if (availableA==false) {
                candIntraPredModeA=INTRA_DC;
              }
              else if (get_pred_mode(ctx, x-1,y) != MODE_INTRA) { // || TODO: pcm_flag (page 110)
                candIntraPredModeA=INTRA_DC;
              }
              else {
                candIntraPredModeA = get_IntraPredMode(ctx, x-1,y);
              }

              // block above

              if (availableB==false) {
                candIntraPredModeB=INTRA_DC;
              }
              else if (get_pred_mode(ctx, x,y-1) != MODE_INTRA) { // || TODO: pcm_flag (page 110)
                candIntraPredModeB=INTRA_DC;
              }
              else if (y-1 < ((y >> sps->Log2CtbSizeY) << sps->Log2CtbSizeY)) {
                candIntraPredModeB=INTRA_DC;
              }
              else {
                candIntraPredModeB = get_IntraPredMode(ctx, x,y-1);
              }

              // build candidate list

              int candModeList[3];

              logtrace(LogSlice,"availableA:%d candA:%d & availableB:%d candB:%d\n",
                     availableA, candIntraPredModeA,
                     availableB, candIntraPredModeB);

              if (candIntraPredModeA == candIntraPredModeB) {
                if (candIntraPredModeA < 2) {
                  candModeList[0] = INTRA_PLANAR;
                  candModeList[1] = INTRA_DC;
                  candModeList[2] = INTRA_ANGULAR_26; 
                }
                else {
                  candModeList[0] = candIntraPredModeA;
                  candModeList[1] = 2 + ((candIntraPredModeA-2 -1 +32) % 32);
                  candModeList[2] = 2 + ((candIntraPredModeA-2 +1    ) % 32);
                }
              }
              else {
                candModeList[0] = candIntraPredModeA;
                candModeList[1] = candIntraPredModeB;

                if (candIntraPredModeA != INTRA_PLANAR &&
                    candIntraPredModeB != INTRA_PLANAR) {
                  candModeList[2] = INTRA_PLANAR;
                }
                else if (candIntraPredModeA != INTRA_DC &&
                         candIntraPredModeB != INTRA_DC) {
                  candModeList[2] = INTRA_DC;
                }
                else {
                  candModeList[2] = INTRA_ANGULAR_26; 
                }
              }

              for (int i=0;i<3;i++)
                logtrace(LogSlice,"candModeList[%d] = %d\n", i, candModeList[i]);

              if (prev_intra_luma_pred_flag[idx]==1) {
                IntraPredMode = candModeList[ mpm_idx[idx] ];
              }
              else {
                // sort candModeList

                if (candModeList[0] > candModeList[1]) {
                  int t = candModeList[0]; candModeList[0]=candModeList[1]; candModeList[1]=t;
                }
                if (candModeList[0] > candModeList[2]) {
                  int t = candModeList[0]; candModeList[0]=candModeList[2]; candModeList[2]=t;
                }
                if (candModeList[1] > candModeList[2]) {
                  int t = candModeList[1]; candModeList[1]=candModeList[2]; candModeList[2]=t;
                }

                // skip modes in the list
                // (we have 35 modes. skipping the 3 in the list gives us 32, which can be selected by 5 bits)
                IntraPredMode = rem_intra_luma_pred_mode[idx];
                for (int n=0;n<=2;n++) {
                  if (IntraPredMode >= candModeList[n]) { IntraPredMode++; }
                }
              }

              logtrace(LogSlice,"IntraPredMode[%d][%d] = %d (log2blk:%d)\n",x,y,IntraPredMode, log2IntraPredSize);

              set_IntraPredMode(ctx,x,y, log2IntraPredSize,(enum IntraPredMode)IntraPredMode);
              
              idx++;
            }


        // set chroma intra prediction mode

        int intra_chroma_pred_mode = decode_intra_chroma_pred_mode(ctx,shdr);

        int IntraPredMode = get_IntraPredMode(ctx,x0,y0);
        logtrace(LogSlice,"IntraPredMode: %d\n",IntraPredMode);

        int IntraPredModeC;
        if (intra_chroma_pred_mode==4) {
          IntraPredModeC = IntraPredMode;
        }
        else {
          static enum IntraPredMode IntraPredModeCCand[4] = {
            INTRA_PLANAR,
            INTRA_ANGULAR_26, // vertical
            INTRA_ANGULAR_10, // horizontal
            INTRA_DC
          };

          IntraPredModeC = IntraPredModeCCand[intra_chroma_pred_mode];
          if (IntraPredModeC == IntraPredMode) {
            IntraPredModeC = INTRA_ANGULAR_34;
          }
        }

        logtrace(LogSlice,"IntraPredModeC[%d][%d]: %d\n",x0,y0,IntraPredModeC);

        set_IntraPredModeC(ctx,x0,y0, log2CbSize, (enum IntraPredMode)IntraPredModeC);
      }
    }
    else {
      if (PartMode == PART_2Nx2N) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS,nCbS);
      }
      else if (PartMode == PART_2NxN) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS,nCbS/2);
        read_prediction_unit(ctx,shdr,x0,y0+nCbS/2,nCbS,nCbS/2);
      }
      else if (PartMode == PART_Nx2N) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS/2,nCbS);
        read_prediction_unit(ctx,shdr,x0+nCbS/2,y0,nCbS/2,nCbS);
      }
      else if (PartMode == PART_2NxnU) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS,nCbS/4);
        read_prediction_unit(ctx,shdr,x0,y0+nCbS/4,nCbS,nCbS*3/4);
      }
      else if (PartMode == PART_2NxnD) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS,nCbS*3/4);
        read_prediction_unit(ctx,shdr,x0,y0+nCbS*3/4,nCbS,nCbS/4);
      }
      else if (PartMode == PART_nLx2N) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS/4,nCbS);
        read_prediction_unit(ctx,shdr,x0+nCbS/4,y0,nCbS*3/4,nCbS);
      }
      else if (PartMode == PART_nRx2N) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS*3/4,nCbS);
        read_prediction_unit(ctx,shdr,x0+nCbS*3/4,y0,nCbS/4,nCbS);
      }
      else if (PartMode == PART_NxN) {
        read_prediction_unit(ctx,shdr,x0,y0,nCbS/2,nCbS/2);
        read_prediction_unit(ctx,shdr,x0+nCbS/2,y0,nCbS/2,nCbS/2);
        read_prediction_unit(ctx,shdr,x0,y0+nCbS/2,nCbS/2,nCbS/2);
        read_prediction_unit(ctx,shdr,x0+nCbS/2,y0+nCbS/2,nCbS/2,nCbS/2);
      }
      else {
        assert(0); // undefined PartMode
      }
    }


    if (!false) { // !pcm
      bool rqt_root_cbf;

      bool merge_flag=get_merge_flag(ctx,x0,y0);

      if (cuPredMode != MODE_INTRA &&
          !(PartMode == PART_2Nx2N && merge_flag)) {

        rqt_root_cbf = decode_rqt_root_cbf(ctx,shdr);
      }
      else {
        rqt_root_cbf = true;
      }

      if (rqt_root_cbf) {
        int MaxTrafoDepth;

        if (cuPredMode==MODE_INTRA) {
          MaxTrafoDepth = ctx->current_sps->max_transform_hierarchy_depth_intra + IntraSplitFlag;
        }
        else {
          MaxTrafoDepth = ctx->current_sps->max_transform_hierarchy_depth_inter;
        }

        logtrace(LogSlice,"MaxTrafoDepth: %d\n",MaxTrafoDepth);

        read_transform_tree(ctx,shdr, x0,y0, x0,y0, log2CbSize, 0,0, MaxTrafoDepth, IntraSplitFlag);
      }
    }
  }


  // --- decode CU ---

  logtrace(LogSlice,"--- decodeCU (%d;%d size %d) ---\n",x0,y0,1<<log2CbSize);

  int nS = 1 << log2CbSize;

  // (8.4.1) decoding process for CUs coded in intra prediction mode

  if (cuPredMode == MODE_INTRA) {
    decode_quantization_parameters(ctx,shdr, x0,y0);

    if (false) { // pcm_flag (8.4.1)
      // TODO
    } else {
      if (IntraSplitFlag==0) {
        logtrace(LogSlice,"IntraSplitFlag==0\n");
        logtrace(LogSlice,"get_IntraPredMode(%d,%d)=%d\n",x0,y0,get_IntraPredMode(ctx,x0,y0));

        decode_intra_block(ctx,shdr,0,
                           x0,y0,
                           log2CbSize,0,
                           get_IntraPredMode(ctx,x0,y0));
      } else {
        // luma

        for (int blkIdx=0; blkIdx<=3; blkIdx++) {
          int xBS = x0 + (nS>>1)*(blkIdx % 2);
          int yBS = y0 + (nS>>1)*(blkIdx / 2);

          logtrace(LogSlice,"IntraSplitFlag==1\n");
          logtrace(LogSlice,"get_IntraPredMode(%d,%d)=%d\n",xBS,yBS,get_IntraPredMode(ctx,xBS,yBS));

          decode_intra_block(ctx,shdr,0,
                             xBS,yBS,
                             log2CbSize-1,1,
                             get_IntraPredMode(ctx,xBS,yBS));
        }
      }

      // chroma

      logtrace(LogSlice,"get_IntraPredModeC(%d,%d)=%d\n",x0,y0,get_IntraPredModeC(ctx,x0,y0));

      decode_intra_block(ctx,shdr,1,
                         x0/2,y0/2,
                         log2CbSize-1,0,
                         get_IntraPredModeC(ctx,x0,y0));
      decode_intra_block(ctx,shdr,2,
                         x0/2,y0/2,
                         log2CbSize-1,0,
                         get_IntraPredModeC(ctx,x0,y0));
    }

  }
  else { // cuPredMode == MODE_INTER / MODE_SKIP
    decode_quantization_parameters(ctx,shdr, x0,y0);

    inter_prediction(ctx,shdr, x0,y0, log2CbSize);
  }


  {
    uint8_t* image;
    int stride;
    get_image_plane(ctx, 0 /*cIdx*/,  &image, &stride);
    for (int y=0;y<40;y++)
      {
        for (int x=0;x<40;x++)
          logtrace(LogSlice,"*%02x ", image[x+y*stride]);

        logtrace(LogSlice,"*\n");
      }
  }

  // write for debugging
  //write_picture(ctx);
}


void read_coding_quadtree(decoder_context* ctx,
                          slice_segment_header* shdr,
                          int x0, int y0,
                          int log2CbSize,
                          int ctDepth)
{
  logtrace(LogSlice,"- read_coding_quadtree %d;%d cbsize:%d depth:%d\n",x0,y0,1<<log2CbSize,ctDepth);

  seq_parameter_set* sps = ctx->current_sps;

  int split_flag;

  // We only send a split flag if CU is larger than minimum size and
  // completely contained within the image area.
  // If it is partly outside the image area and not at minimum size,
  // it is split. If already at minimum size, it is not split further.
  if (x0+(1<<log2CbSize) <= sps->pic_width_in_luma_samples &&
      y0+(1<<log2CbSize) <= sps->pic_height_in_luma_samples &&
      log2CbSize > sps->Log2MinCbSizeY) {
    split_flag = decode_split_cu_flag(ctx, shdr, x0,y0, ctDepth);
  } else {
    if (log2CbSize > sps->Log2MinCbSizeY) { split_flag=1; }
    else                                  { split_flag=0; }
  }


  if (ctx->current_pps->cu_qp_delta_enabled_flag &&
      log2CbSize >= ctx->current_pps->Log2MinCuQpDeltaSize)
    {
      shdr->IsCuQpDeltaCoded = 0;
      shdr->CuQpDelta = 0;
    }
  else
    {
      shdr->CuQpDelta = 0; // TODO check: is this the right place to set to default value ?
    }

  if (split_flag) {
    int x1 = x0 + (1<<(log2CbSize-1));
    int y1 = y0 + (1<<(log2CbSize-1));

    read_coding_quadtree(ctx,shdr,x0,y0, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples)
      read_coding_quadtree(ctx,shdr,x1,y0, log2CbSize-1, ctDepth+1);

    if (y1<sps->pic_height_in_luma_samples)
      read_coding_quadtree(ctx,shdr,x0,y1, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples &&
        y1<sps->pic_height_in_luma_samples)
      read_coding_quadtree(ctx,shdr,x1,y1, log2CbSize-1, ctDepth+1);
  }
  else {
    // set ctDepth of this CU

    set_ctDepth(ctx, x0,y0, log2CbSize, ctDepth);

    read_coding_unit(ctx,shdr, x0,y0, log2CbSize, ctDepth);
  }

  logtrace(LogSlice,"-\n");
  //debug_dump_cb_info(ctx);
}
