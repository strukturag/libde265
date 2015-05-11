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

#include "vui.h"
#include "vps.h"

de265_error video_usability_information::read(bitreader* reader,
                                              int sps_max_sub_layers_minus1)
{
  aspect_ratio_info_present_flag = get_bits(reader,1);
  if (aspect_ratio_info_present_flag) {
    int code = get_bits(reader,8);
    if (code <= 16) aspect_ratio_idc = (SAR_Inidcator) code;
    else if (code < 255 ) aspect_ratio_idc = SAR_RESERVED;
    else aspect_ratio_idc = SAR_EXTENDED;
    if (aspect_ratio_idc == SAR_EXTENDED) {
      sar_width  = get_bits(reader,16);
      sar_height = get_bits(reader,16);
    }
  }

  overscan_info_present_flag = get_bits(reader,1);
  if (overscan_info_present_flag) {
    overscan_appropriate_flag = get_bits(reader,1);
  }

  video_signal_type_present_flag = get_bits(reader,1);
  if (video_signal_type_present_flag) {
    video_format = get_bits(reader,3);
    video_full_range_flag = get_bits(reader,1);
    colour_description_present_flag = get_bits(reader,1);
    if (colour_description_present_flag) {
      colour_primaries = get_bits(reader,8);
      transfer_characteristics = get_bits(reader,8);
      matrix_coeffs = get_bits(reader,8);
    }
  }

  chroma_loc_info_present_flag = get_bits(reader,1);
  if( chroma_loc_info_present_flag ) {
    chroma_sample_loc_type_top_field = get_uvlc(reader);
    chroma_sample_loc_type_bottom_field = get_uvlc(reader);
  }

  neutral_chroma_indication_flag = get_bits(reader,1);
  field_seq_flag = get_bits(reader,1);
  frame_field_info_present_flag = get_bits(reader,1);
  default_display_window_flag = get_bits(reader,1);
  if( default_display_window_flag ) {
    def_disp_win_left_offset = get_uvlc(reader);
    def_disp_win_right_offset = get_uvlc(reader);
    def_disp_win_top_offset = get_uvlc(reader);
    def_disp_win_bottom_offset = get_uvlc(reader);
  }

  vui_timing_info_present_flag = get_bits(reader,1);
  if( vui_timing_info_present_flag ) {
    vui_num_units_in_tick = get_bits(reader,32);
    vui_time_scale = get_bits(reader,32);
    vui_poc_proportional_to_timing_flag = get_bits(reader,1);
    if (vui_poc_proportional_to_timing_flag) {
      vui_num_ticks_poc_diff_one_minus1 = get_uvlc(reader);
    }
    vui_hrd_parameters_present_flag = get_bits(reader,1);
    if (vui_hrd_parameters_present_flag) {
      de265_error err = hrd.read(reader, true, sps_max_sub_layers_minus1 );
      if (err != DE265_OK) return err;
    }
  }

  bitstream_restriction_flag = get_bits(reader,1);
  if( bitstream_restriction_flag ) {
    tiles_fixed_structure_flag = get_bits(reader,1);
    motion_vectors_over_pic_boundaries_flag = get_bits(reader,1);
    restricted_ref_pic_lists_flag = get_bits(reader,1);
    min_spatial_segmentation_idc = get_uvlc(reader);
    max_bytes_per_pic_denom = get_uvlc(reader);
    max_bits_per_min_cu_denom = get_uvlc(reader);
    log2_max_mv_length_horizontal = get_uvlc(reader);
    log2_max_mv_length_vertical = get_uvlc(reader);
  }

  return DE265_OK;
}

de265_error vps_vui_video_signal_info::read_video_signal_info(bitreader* reader)
{
  video_vps_format = get_bits(reader,3);
  video_full_range_vps_flag = get_bits(reader,1);
  colour_primaries_vps = get_bits(reader,8);
  transfer_characteristics_vps = get_bits(reader,8);
  matrix_coeffs_vps = get_bits(reader,8);

  return DE265_OK;
}

de265_error vps_vui_bsp_hrd_params::read_vps_vui_bsp_hrd_params(bitreader* reader,
                                                                video_parameter_set* vps)
{
  video_parameter_set_extension *vps_ext = &vps->vps_extension;

  vps_num_add_hrd_params = get_uvlc(reader);
  for( int i = vps->vps_num_hrd_parameters;
        i < vps->vps_num_hrd_parameters + vps_num_add_hrd_params; i++ ) {
    if (i > 0) {
      cprms_add_present_flag[ i ] = get_bits(reader,1);
    }
    num_sub_layer_hrd_minus1[i] = get_uvlc(reader);

    hrd_params[i].read(reader, cprms_add_present_flag[i], num_sub_layer_hrd_minus1[i]);
  }

  if( vps->vps_num_hrd_parameters + vps_num_add_hrd_params > 0 ) {
    for( int h = 1; h < vps_ext->NumOutputLayerSets; h++ ) {
      num_signalled_partitioning_schemes[ h ] = get_uvlc(reader);
      for( int j = 1; j < num_signalled_partitioning_schemes[ h ] + 1; j++ ) {
        num_partitions_in_scheme_minus1[ h ][ j ] = get_uvlc(reader);
        for( int k = 0; k <= num_partitions_in_scheme_minus1[ h ][ j ]; k++ ) {
          for( int r = 0; r < vps_ext->NumLayersInIdList[ vps_ext->OlsIdxToLsIdx[ h ] ]; r++ ) {
            layer_included_in_partition_flag[ h ][ j ][ k ][ r ] = get_bits(reader,1);
          }
        }
      }

      for( int i = 0; i < num_signalled_partitioning_schemes[ h ] + 1; i++ ) {
        for( int t = 0; t <= vps_ext->MaxSubLayersInLayerSetMinus1[ vps_ext->OlsIdxToLsIdx[ h ] ]; t++ ) {
          num_bsp_schedules_minus1[ h ][ i ][ t ] = get_uvlc(reader);
          for (int j = 0; j <= num_bsp_schedules_minus1[h][i][t]; j++) {
            for( int k = 0; k <= num_partitions_in_scheme_minus1[ h ][ i ]; k++ ) {
              if( vps->vps_num_hrd_parameters + vps_num_add_hrd_params > 1 ) {
                int nr_bits = ceil_log2( vps->vps_num_hrd_parameters + vps_num_add_hrd_params );
                bsp_hrd_idx[ h ][ i ][ t ][ j ][ k ] = get_bits(reader,nr_bits);
              }
              bsp_sched_idx[ h ][ i ][ t ][ j ][ k ] = get_uvlc(reader);
            }
          }
        }
      }
    }
  }

  return DE265_OK;
}

de265_error vps_vui::read_vps_vui( bitreader* reader,
                                   video_parameter_set* vps)
{
  video_parameter_set_extension * vps_ext = &vps->vps_extension;

  cross_layer_pic_type_aligned_flag = get_bits(reader,1);
  cross_layer_irap_aligned_flag = vps_ext->vps_vui_present_flag;
  if (!cross_layer_pic_type_aligned_flag) {
    cross_layer_irap_aligned_flag = get_bits(reader,1);
  }
  if (cross_layer_irap_aligned_flag) {
all_layers_idr_aligned_flag = get_bits(reader, 1);
  }
  bit_rate_present_vps_flag = get_bits(reader, 1);
  pic_rate_present_vps_flag = get_bits(reader, 1);
  if (bit_rate_present_vps_flag || pic_rate_present_vps_flag) {
    for (int i = vps->vps_base_layer_internal_flag ? 0 : 1; i < vps_ext->NumLayerSets; i++) {
      for (int j = 0; j <= vps_ext->MaxSubLayersInLayerSetMinus1[i]; j++) {
        if (bit_rate_present_vps_flag) {
          bit_rate_present_flag[i][j] = get_bits(reader, 1);
        }
        if (pic_rate_present_vps_flag) {
          pic_rate_present_flag[i][j] = get_bits(reader, 1);
        }
        if (bit_rate_present_flag[i][j]) {
          avg_bit_rate[i][j] = get_bits(reader, 16);
          max_bit_rate[i][j] = get_bits(reader, 16);
        }
        if (pic_rate_present_flag[i][j]) {
          constant_pic_rate_idc[i][j] = get_bits(reader, 2);
          avg_pic_rate[i][j] = get_bits(reader, 16);
        }
      }
    }
  }

  video_signal_info_idx_present_flag = get_bits(reader, 1);
  if (video_signal_info_idx_present_flag) {
    vps_num_video_signal_info_minus1 = get_bits(reader, 4);
  }
  for (int i = 0; i <= vps_num_video_signal_info_minus1; i++) {
    video_signal_info[i].read_video_signal_info(reader);
  }

  if (video_signal_info_idx_present_flag && vps_num_video_signal_info_minus1 > 0) {
    for (int i = vps->vps_base_layer_internal_flag ? 0 : 1; i <= vps->MaxLayersMinus1; i++) {
      vps_video_signal_info_idx[i] = get_bits(reader, 4);
    }
  }

  tiles_not_in_use_flag = get_bits(reader, 1);

  if (!tiles_not_in_use_flag) {
    for (int i = vps->vps_base_layer_internal_flag ? 0 : 1; i <= vps->MaxLayersMinus1; i++) {
      tiles_in_use_flag[i] = get_bits(reader, 1);
      if (tiles_in_use_flag[i]) {
        loop_filter_not_across_tiles_flag[i] = get_bits(reader, 1);
      }
    }
    for (int i = vps->vps_base_layer_internal_flag ? 1 : 2; i <= vps->MaxLayersMinus1; i++) {
      for (int j = 0; j < vps_ext->NumDirectRefLayers[vps_ext->layer_id_in_nuh[i]]; j++) {
        int layerIdx = vps_ext->LayerIdxInVps[vps_ext->IdDirectRefLayer[vps_ext->layer_id_in_nuh[i]][j]];
        if (tiles_in_use_flag[i] && tiles_in_use_flag[layerIdx]) {
          tile_boundaries_aligned_flag[i][j] = get_bits(reader, 1);
        }
      }
    }
  }

  wpp_not_in_use_flag = get_bits(reader, 1);
  if (!wpp_not_in_use_flag) {
    for (int i = vps->vps_base_layer_internal_flag ? 0 : 1; i <= vps->MaxLayersMinus1; i++) {
      wpp_in_use_flag[i] = get_bits(reader, 1);
    }
  }
  single_layer_for_non_irap_flag = get_bits(reader, 1);
  higher_layer_irap_skip_flag = get_bits(reader, 1);
  ilp_restricted_ref_layers_flag = get_bits(reader, 1);

  if (ilp_restricted_ref_layers_flag) {
    for (int i = 1; i <= vps->MaxLayersMinus1; i++) {
      for (int j = 0; j < vps_ext->NumDirectRefLayers[vps_ext->layer_id_in_nuh[i]]; j++) {
        if (vps->vps_base_layer_internal_flag ||
          vps_ext->IdDirectRefLayer[vps_ext->layer_id_in_nuh[i]][j] > 0) {
          min_spatial_segment_offset_plus1[i][j] = get_uvlc(reader);
          if (min_spatial_segment_offset_plus1[i][j] > 0) {
            ctu_based_offset_enabled_flag[i][j] = get_bits(reader, 1);
            if (ctu_based_offset_enabled_flag[i][j]) {
              min_horizontal_ctu_offset_plus1[i][j] = get_uvlc(reader);
            }
          }
        }
      }
    }
  }

  vps_vui_bsp_hrd_present_flag = get_bits(reader, 1);
  if (vps_vui_bsp_hrd_present_flag) {
    bsp_hrd_params.read_vps_vui_bsp_hrd_params(reader, vps);
  }

  for (int i = 1; i <= vps->MaxLayersMinus1; i++) {
    if (vps_ext->NumDirectRefLayers[vps_ext->layer_id_in_nuh[i]] == 0) {
      base_layer_parameter_set_compatibility_flag[i] = get_bits(reader, 1);
    }
  }

  return DE265_OK;
}

de265_error hrd_parameters::read( bitreader* reader,
                                  bool commonInfPresentFlag,
                                  int maxNumSubLayersMinus1)
{
  if (commonInfPresentFlag) {
    nal_hrd_parameters_present_flag = get_bits(reader,1);
    vcl_hrd_parameters_present_flag = get_bits(reader,1);
    if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
      sub_pic_hrd_params_present_flag = get_bits(reader,1);
      if (sub_pic_hrd_params_present_flag) {
          tick_divisor_minus2 = get_bits(reader,8);
          du_cpb_removal_delay_increment_length_minus1 = get_bits(reader,5);
          sub_pic_cpb_params_in_pic_timing_sei_flag = get_bits(reader,1);
          dpb_output_delay_du_length_minus1 = get_bits(reader,5);
      }
      bit_rate_scale = get_bits(reader,4);
      cpb_size_scale = get_bits(reader,4);
      if (sub_pic_hrd_params_present_flag) {
        cpb_size_du_scale = get_bits(reader,4);
      }
      initial_cpb_removal_delay_length_minus1 = get_bits(reader,5);
      au_cpb_removal_delay_length_minus1 = get_bits(reader,5);
      dpb_output_delay_length_minus1 = get_bits(reader,5);
    }
  }

  for( int i = 0; i <= maxNumSubLayersMinus1; i++ ) {
    fixed_pic_rate_general_flag[ i ] = get_bits(reader,1);
    if (!fixed_pic_rate_general_flag[i]) {
      fixed_pic_rate_within_cvs_flag[ i ] = get_bits(reader,1);
    }
    if (fixed_pic_rate_within_cvs_flag[i]) {
      elemental_duration_in_tc_minus1[i] = get_uvlc(reader);
    }
    else {
      low_delay_hrd_flag[ i ] = get_bits(reader,1);
    }
    if (!low_delay_hrd_flag[i]) {
      cpb_cnt_minus1[i] = get_uvlc(reader);
    }
    if (nal_hrd_parameters_present_flag) {
      sub_layer_hrd[i].read(reader, this, i);
    }
    if (vcl_hrd_parameters_present_flag) {
      sub_layer_hrd[i].read(reader, this, i);
    }
  }

  return DE265_OK;
}

de265_error sub_layer_hrd_parameters::read( bitreader* reader,
                                            hrd_parameters *hrd,
                                            int subLayerId)
{
  int CpbCnt = hrd->cpb_cnt_minus1[subLayerId];
  for( int i = 0; i <= CpbCnt; i++ ) {
    bit_rate_value_minus1[ i ] = get_uvlc(reader);
    cpb_size_value_minus1[ i ] = get_uvlc(reader);
    if( hrd->sub_pic_hrd_params_present_flag ) {
      cpb_size_du_value_minus1[ i ] = get_uvlc(reader);
      bit_rate_du_value_minus1[ i ] = get_uvlc(reader);
    }
    cbr_flag[ i ] = get_bits(reader,1);
  }

  return DE265_OK;
}
