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

#ifndef DE265_VPS_EXTENSION_H
#define DE265_VPS_EXTENSION_H

#include "de265.h"
#include "vui.h"
#include "vps.h"

struct video_parameter_set;
struct decoded_picture_buffer_size_table {
  de265_error read(bitreader* reader, video_parameter_set *vps);

  bool_1d  sub_layer_flag_info_present_flag;
  bool_2d  sub_layer_dpb_info_present_flag;
  int_3d   max_vps_dec_pic_buffering_minus1;
  int_2d   max_vps_num_reorder_pics;
  int_2d   max_vps_latency_increase_plus1;
};

struct conformance_window {
  int conf_win_vps_left_offset;
  int conf_win_vps_right_offset;
  int conf_win_vps_top_offset;
  int conf_win_vps_bottom_offset;
};

struct rep_format {
  de265_error read(bitreader* reader);

  int  pic_width_vps_in_luma_samples;
  int  pic_height_vps_in_luma_samples;
  bool chroma_and_bit_depth_vps_present_flag;

  de265_chroma chroma_format_vps_idc;

  bool separate_colour_plane_vps_flag;
  int  bit_depth_vps_luma_minus8;
  int  bit_depth_vps_chroma_minus8;
  bool conformance_window_vps_flag;

  conformance_window m_conformanceWindowVps;
};

class profile_tier_level;
struct video_parameter_set;
struct video_parameter_set_extension{
  de265_error read(bitreader* reader, video_parameter_set *vps);

  // Parameters of the vps_extension
  std::map<int, profile_tier_level> vps_ext_PTL;

  bool     splitting_flag;
  bool     scalability_mask_flag[16];
  int      dimension_id_len_minus1[16];
  bool     vps_nuh_layer_id_present_flag;
  int      layer_id_in_nuh[8];
  int      dimension_id[8][16];
  int      view_id_len;
  int      view_id_val[8];
  bool     direct_dependency_flag[8][8];
  int      num_add_layer_sets;
  int_2d   highest_layer_idx_plus1;
  bool     vps_sub_layers_max_minus1_present_flag;
  int      sub_layers_vps_max_minus1[8];
  bool     max_tid_ref_present_flag;
  int      max_tid_il_ref_pics_plus1[7][8];
  bool     default_ref_layers_active_flag;
  int      vps_num_profile_tier_level_minus1;
  bool_1d  vps_profile_present_flag;
  
  int      num_add_olss;
  int      default_output_layer_idc;
  int_1d   layer_set_idx_for_ols_minus1;
  bool_2d  output_layer_flag;

  int_2d   profile_tier_level_idx;

  bool_1d  alt_output_layer_flag;
  int      vps_num_rep_formats_minus1;

  std::map<int, rep_format> vps_ext_rep_format;

  bool   rep_format_idx_present_flag;
  int    vps_rep_format_idx[16];
  bool   max_one_active_ref_layer_flag;
  bool   vps_poc_lsb_aligned_flag;
  bool   poc_lsb_not_present_flag[8];

  decoded_picture_buffer_size_table dpb_size_table;
  
  int   direct_dep_type_len_minus2;
  bool  direct_dependency_all_layers_flag;
  int   direct_dependency_all_layers_type;
  int   direct_dependency_type[8][8];
  int   vps_non_vui_extension_length;
  bool  vps_vui_present_flag;

  vps_vui vui;

  /// Variables calculated from the read parameters when parsing the vps_extension
  int_1d   LayerIdxInVps;
  int      NumLayerSets;
  int_1d   MaxSubLayersInLayerSetMinus1;
  bool_2d  DependencyFlag;
  int_1d   NumDirectRefLayers;
  int_1d   NumRefLayers;
  int_1d   NumPredictedLayers;
  int_2d   IdDirectRefLayer;
  int_2d   IdRefLayer;
  int_2d   IdPredictedLayer;
  int_1d   OlsIdxToLsIdx;
  bool_2d  NecessaryLayerFlag;
  bool_2d  OutputLayerFlag;
  int_1d   NumNecessaryLayers;
  int_1d   NumOutputLayersInOutputLayerSet;
  int_1d   OlsHighestOutputLayerId;
  int      NumOutputLayerSets;
  int_1d   NumLayersInIdList;
  int_2d   LayerSetLayerIdList;

};

#endif