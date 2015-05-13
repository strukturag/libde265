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

#ifndef DE265_VUI_H
#define DE265_VUI_H

#include "libde265/bitstream.h"
#include "libde265/de265.h"
#include "libde265/util.h"

// Structs defined in vps.h
struct video_parameter_set;
struct video_parameter_set_extension;
struct hrd_parameters;

enum SAR_Inidcator {
  UNSPECIFIED = 0,
  SAR_1_1 = 1,
  SAR_12_11 = 2,
  SAR_10_11 = 3,
  SAR_16_11 = 4,
  SAR_40_33 = 5,
  SAR_24_11 = 6,
  SAR_20_11 = 7,
  SAR_32_11 = 8,
  SAR_80_33 = 9,
  SAR_18_11 = 10,
  SAR_15_11 = 11,
  SAR_64_33 = 12,
  SAR_160_99 = 13,
  SAR_4_3 = 14,
  SAR_3_2 = 15,
  SAR_2_1 = 16,
  SAR_RESERVED = 17,
  SAR_EXTENDED = 255
};

struct hrd_parameters;
struct sub_layer_hrd_parameters {
  de265_error read( bitreader* reader, hrd_parameters *hrd, int subLayerId);

  int_1d  bit_rate_value_minus1;
  int_1d  cpb_size_value_minus1;
  int_1d  cpb_size_du_value_minus1;
  int_1d  bit_rate_du_value_minus1;
  bool_1d cbr_flag;
};

struct hrd_parameters {
  de265_error read(bitreader* reader, bool commonInfPresentFlag, int maxNumSubLayersMinus1);

  bool commonInfPresentFlag;

  // Common info
  bool nal_hrd_parameters_present_flag;
  bool vcl_hrd_parameters_present_flag;
  bool sub_pic_hrd_params_present_flag;
  int  tick_divisor_minus2;
  int  du_cpb_removal_delay_increment_length_minus1;
  bool sub_pic_cpb_params_in_pic_timing_sei_flag;
  int  dpb_output_delay_du_length_minus1;
  int  bit_rate_scale;
  int  cpb_size_scale;
  int  cpb_size_du_scale;
  int  initial_cpb_removal_delay_length_minus1;
  int  au_cpb_removal_delay_length_minus1;
  int  dpb_output_delay_length_minus1;
  // end common info

  bool_1d  fixed_pic_rate_general_flag;
  bool_1d  fixed_pic_rate_within_cvs_flag;
  int_1d   elemental_duration_in_tc_minus1;
  bool_1d  low_delay_hrd_flag;
  int_1d   cpb_cnt_minus1;

  std::map<int, sub_layer_hrd_parameters> sub_layer_hrd;
};

// Video usability information as defined in Annex E
// See Annex E JCTVC-R1013_v6
struct video_usability_information {
  de265_error read(bitreader *reader, int sps_max_sub_layers_minus1);

  bool aspect_ratio_info_present_flag;
  SAR_Inidcator aspect_ratio_idc;

  int sar_width;
  int sar_height;
  bool overscan_info_present_flag;
  bool overscan_appropriate_flag;
  bool video_signal_type_present_flag;
  int video_format;
  bool video_full_range_flag;
  bool colour_description_present_flag;
  int colour_primaries;
  int transfer_characteristics;
  int matrix_coeffs;
  bool chroma_loc_info_present_flag;
  int chroma_sample_loc_type_top_field;
  int chroma_sample_loc_type_bottom_field;
  bool neutral_chroma_indication_flag;
  bool field_seq_flag;
  bool frame_field_info_present_flag;
  bool default_display_window_flag;

  int def_disp_win_left_offset;
  int def_disp_win_right_offset;
  int def_disp_win_top_offset;
  int def_disp_win_bottom_offset;
  bool vui_timing_info_present_flag;
  int vui_num_units_in_tick;
  int vui_time_scale;
  bool vui_poc_proportional_to_timing_flag;
  int vui_num_ticks_poc_diff_one_minus1;
  bool vui_hrd_parameters_present_flag;

  hrd_parameters hrd;

  bool bitstream_restriction_flag;
  bool tiles_fixed_structure_flag;
  bool motion_vectors_over_pic_boundaries_flag;
  bool restricted_ref_pic_lists_flag;
  int min_spatial_segmentation_idc;
  int max_bytes_per_pic_denom;
  int max_bits_per_min_cu_denom;
  int log2_max_mv_length_horizontal;
  int log2_max_mv_length_vertical;
};

// VPS VUI, vps_vui_bsp_hrd_params, and vps_vui_video_signal_info as defined
// int annex F (multi-layer extensions).
// See Annex F JCTVC-R1013_v6
struct vps_vui_bsp_hrd_params {
  de265_error read_vps_vui_bsp_hrd_params(bitreader* reader, video_parameter_set* vps);

  int  vps_num_add_hrd_params;

  bool_1d cprms_add_present_flag;
  int_1d  num_sub_layer_hrd_minus1;

  std::map<int, hrd_parameters> hrd_params;

  int_1d  num_signalled_partitioning_schemes;
  int_2d  num_partitions_in_scheme_minus1;
  int_4d  layer_included_in_partition_flag;
  int_3d  num_bsp_schedules_minus1;
  int_5d  bsp_hrd_idx;
  int_5d  bsp_sched_idx;
};

struct vps_vui_video_signal_info {
  de265_error read_video_signal_info(bitreader* reader);

  int  video_vps_format;
  bool video_full_range_vps_flag;
  int  colour_primaries_vps;
  int  transfer_characteristics_vps;
  int  matrix_coeffs_vps;
};

struct vps_vui {
  de265_error read_vps_vui( bitreader* reader, video_parameter_set* vps);

  bool    cross_layer_pic_type_aligned_flag;
  bool    cross_layer_irap_aligned_flag;
  bool    all_layers_idr_aligned_flag;
  bool    bit_rate_present_vps_flag;
  bool    pic_rate_present_vps_flag;
  bool_2d bit_rate_present_flag;
  bool_2d pic_rate_present_flag;
  int_2d  avg_bit_rate;
  int_2d  max_bit_rate;
  int_2d  constant_pic_rate_idc;
  int_2d  avg_pic_rate;
  bool    video_signal_info_idx_present_flag;
  int     vps_num_video_signal_info_minus1;

  std::map<int, vps_vui_video_signal_info> video_signal_info;

  int  vps_video_signal_info_idx[8];
  bool tiles_not_in_use_flag;
  bool tiles_in_use_flag[8];
  bool loop_filter_not_across_tiles_flag[8];
  bool tile_boundaries_aligned_flag[8][8];
  bool wpp_not_in_use_flag;
  bool wpp_in_use_flag[8];
  bool single_layer_for_non_irap_flag;
  bool higher_layer_irap_skip_flag;
  bool ilp_restricted_ref_layers_flag;
  int  min_spatial_segment_offset_plus1[8][8];
  bool ctu_based_offset_enabled_flag   [8][8];
  int  min_horizontal_ctu_offset_plus1 [8][8];
  bool vps_vui_bsp_hrd_present_flag;

  vps_vui_bsp_hrd_params bsp_hrd_params;

  bool base_layer_parameter_set_compatibility_flag[8];
};

#endif
