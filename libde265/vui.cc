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
#include "decctx.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define READ_VLC_OFFSET(variable, vlctype, offset)   \
  if ((vlc = get_ ## vlctype(br)) == UVLC_ERROR) {   \
    errqueue->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);  \
    return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE; \
  } \
  variable = vlc + offset;

#define READ_VLC(variable, vlctype)  READ_VLC_OFFSET(variable,vlctype,0)


#define NUM_SAR_PRESETS 17

static uint16_t sar_presets[NUM_SAR_PRESETS+1][2] = {
  { 0,0 },
  { 1,1 },
  { 12,11 },
  { 10,11 },
  { 16,11 },
  { 40,33 },
  { 24,11 },
  { 20,11 },
  { 32,11 },
  { 80,33 },
  { 18,11 },
  { 15,11 },
  { 64,33 },
  { 160,99 },
  { 4,3 },
  { 3,2 },
  { 2,1 }
};

#define EXTENDED_SAR 255


video_usability_information::video_usability_information()
{
}


de265_error video_usability_information::read(error_queue* errqueue, bitreader* br,
                                              const seq_parameter_set* sps)
{
  int vlc;


  // --- sample aspect ratio (SAR) ---

  aspect_ratio_info_present_flag = get_bits(br,1);
  if (aspect_ratio_info_present_flag) {
    int aspect_ratio_idc = get_bits(br,8);
    if (aspect_ratio_idc <= NUM_SAR_PRESETS) {
      sar_width  = sar_presets[aspect_ratio_idc][0];
      sar_height = sar_presets[aspect_ratio_idc][1];
    }
    else if (aspect_ratio_idc == EXTENDED_SAR) {
      sar_width  = get_bits(br,16);
      sar_height = get_bits(br,16);
    }
    else {
      sar_width  = 0;
      sar_height = 0;
    }
  }
  else {
    sar_width  = 0;
    sar_height = 0;
  }


  // --- overscan ---

  overscan_info_present_flag = get_bits(br,1);
  if (overscan_info_present_flag) {
    overscan_appropriate_flag = get_bits(br,1);
  }


  // --- video signal type ---

  { // defaults
    video_format = VideoFormat_Unspecified;
    video_full_range_flag = false;
    colour_primaries = 2;
    transfer_characteristics = 2;
    matrix_coeffs = 2;
  }

  video_signal_type_present_flag = get_bits(br,1);
  if (video_signal_type_present_flag) {
    int video_format_idc = get_bits(br,3);
    if (video_format_idc > 5) {
      video_format_idc = VideoFormat_Unspecified;
    }
    video_format = (VideoFormat)video_format_idc;

    video_full_range_flag = get_bits(br,1);

    colour_description_present_flag = get_bits(br,1);
    if (colour_description_present_flag) {
      colour_primaries = get_bits(br,8);
      if (colour_primaries == 0 ||
          colour_primaries == 3 ||
          colour_primaries >= 11) {
        colour_primaries = 2;
      }

      transfer_characteristics = get_bits(br,8);
      if (transfer_characteristics == 0 ||
          transfer_characteristics == 3 ||
          transfer_characteristics >= 18) {
        transfer_characteristics = 2;
      }

      matrix_coeffs = get_bits(br,8);
      if (matrix_coeffs == 0 ||
          matrix_coeffs >= 11) {
        matrix_coeffs = 2;
      }
    }
  }


  // --- chroma / interlaced ---

  chroma_loc_info_present_flag = get_bits(br,1);
  if (chroma_loc_info_present_flag) {
    READ_VLC(chroma_sample_loc_type_top_field,    uvlc);
    READ_VLC(chroma_sample_loc_type_bottom_field, uvlc);
  }
  else {
    chroma_sample_loc_type_top_field    = 0;
    chroma_sample_loc_type_bottom_field = 0;
  }

  neutral_chroma_indication_flag = get_bits(br,1);
  field_seq_flag                 = get_bits(br,1);
  frame_field_info_present_flag  = get_bits(br,1);


  // --- default display window ---

  default_display_window_flag = get_bits(br,1);
  if (default_display_window_flag) {
    READ_VLC(def_disp_win_left_offset  ,uvlc);
    READ_VLC(def_disp_win_right_offset ,uvlc);
    READ_VLC(def_disp_win_top_offset   ,uvlc);
    READ_VLC(def_disp_win_bottom_offset,uvlc);
  }
  else {
    def_disp_win_left_offset  =0;
    def_disp_win_right_offset =0;
    def_disp_win_top_offset   =0;
    def_disp_win_bottom_offset=0;
  }


  // --- timing ---

  vui_timing_info_present_flag = get_bits(br,1);
  if (vui_timing_info_present_flag) {
    vui_num_units_in_tick = get_bits(br,32);
    vui_time_scale        = get_bits(br,32);
  }

  vui_poc_proportional_to_timing_flag = get_bits(br,1);
  READ_VLC_OFFSET(vui_num_ticks_poc_diff_one, uvlc, 1);


  // --- hrd parameters ---

  vui_hrd_parameters_present_flag = get_bits(br,1);
  if (vui_hrd_parameters_present_flag) {
    return DE265_ERROR_NOT_IMPLEMENTED_YET;
    //hrd_parameters vui_hrd_parameters;
  }


  // --- bitstream restriction ---

  bitstream_restriction_flag = get_bits(br,1);
  if (bitstream_restriction_flag) {
    tiles_fixed_structure_flag = get_bits(br,1);
    motion_vectors_over_pic_boundaries_flag = get_bits(br,1);
    restricted_ref_pic_lists_flag = get_bits(br,1);

    READ_VLC(min_spatial_segmentation_idc, uvlc);
    if (min_spatial_segmentation_idc > 4095) {
      errqueue->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);
      min_spatial_segmentation_idc = 0;
    }

    READ_VLC(max_bytes_per_pic_denom, uvlc);
    if (max_bytes_per_pic_denom > 16) {
      errqueue->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);
      max_bytes_per_pic_denom = 2;
    }

    READ_VLC(max_bits_per_min_cu_denom, uvlc);
    if (max_bits_per_min_cu_denom > 16) {
      errqueue->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);
      max_bits_per_min_cu_denom = 1;
    }

    READ_VLC(log2_max_mv_length_horizontal, uvlc);
    if (log2_max_mv_length_horizontal > 15) {
      errqueue->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);
      log2_max_mv_length_horizontal = 15;
    }

    READ_VLC(log2_max_mv_length_vertical, uvlc);
    if (log2_max_mv_length_vertical > 15) {
      errqueue->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);
      log2_max_mv_length_vertical = 15;
    }
  }
  else {
    tiles_fixed_structure_flag = false;
    motion_vectors_over_pic_boundaries_flag = true;
    restricted_ref_pic_lists_flag = false; // NOTE: default not specified in standard 2014/10

    min_spatial_segmentation_idc = 0;
    max_bytes_per_pic_denom   = 2;
    max_bits_per_min_cu_denom = 1;
    log2_max_mv_length_horizontal = 15;
    log2_max_mv_length_vertical   = 15;
  }

  //vui_read = true;

  return DE265_OK;
}


void video_usability_information::dump(int fd) const
{
  //#if (_MSC_VER >= 1500)
  //#define LOG0(t) loginfo(LogHeaders, t)
  //#define LOG1(t,d) loginfo(LogHeaders, t,d)
  //#define LOG2(t,d1,d2) loginfo(LogHeaders, t,d1,d2)
  //#define LOG3(t,d1,d2,d3) loginfo(LogHeaders, t,d1,d2,d3)

  FILE* fh;
  if (fd==1) fh=stdout;
  else if (fd==2) fh=stderr;
  else { return; }

#define LOG0(t) log2fh(fh, t)
#define LOG1(t,d) log2fh(fh, t,d)
#define LOG2(t,d1,d2) log2fh(fh, t,d1,d2)
#define LOG3(t,d1,d2,d3) log2fh(fh, t,d1,d2,d3)

#if 0
  LOG0("----------------- SPS -----------------\n");
  LOG1("video_parameter_set_id  : %d\n", video_parameter_set_id);
  LOG1("sps_max_sub_layers      : %d\n", sps_max_sub_layers);
  LOG1("sps_temporal_id_nesting_flag : %d\n", sps_temporal_id_nesting_flag);

  profile_tier_level_.dump(sps_max_sub_layers, fh);

  LOG1("seq_parameter_set_id    : %d\n", seq_parameter_set_id);
  LOG2("chroma_format_idc       : %d (%s)\n", chroma_format_idc,
       chroma_format_idc == 1 ? "4:2:0" :
       chroma_format_idc == 2 ? "4:2:2" :
       chroma_format_idc == 3 ? "4:4:4" : "unknown");

  if (chroma_format_idc == 3) {
    LOG1("separate_colour_plane_flag : %d\n", separate_colour_plane_flag);
  }

  LOG1("pic_width_in_luma_samples  : %d\n", pic_width_in_luma_samples);
  LOG1("pic_height_in_luma_samples : %d\n", pic_height_in_luma_samples);
  LOG1("conformance_window_flag    : %d\n", conformance_window_flag);

  if (conformance_window_flag) {
    LOG1("conf_win_left_offset  : %d\n", conf_win_left_offset);
    LOG1("conf_win_right_offset : %d\n", conf_win_right_offset);
    LOG1("conf_win_top_offset   : %d\n", conf_win_top_offset);
    LOG1("conf_win_bottom_offset: %d\n", conf_win_bottom_offset);
  }

  LOG1("bit_depth_luma   : %d\n", bit_depth_luma);
  LOG1("bit_depth_chroma : %d\n", bit_depth_chroma);

  LOG1("log2_max_pic_order_cnt_lsb : %d\n", log2_max_pic_order_cnt_lsb);
  LOG1("sps_sub_layer_ordering_info_present_flag : %d\n", sps_sub_layer_ordering_info_present_flag);

  int firstLayer = (sps_sub_layer_ordering_info_present_flag ?
                    0 : sps_max_sub_layers-1 );

  for (int i=firstLayer ; i <= sps_max_sub_layers-1; i++ ) {
    LOG1("Layer %d\n",i);
    LOG1("  sps_max_dec_pic_buffering      : %d\n", sps_max_dec_pic_buffering[i]);
    LOG1("  sps_max_num_reorder_pics       : %d\n", sps_max_num_reorder_pics[i]);
    LOG1("  sps_max_latency_increase_plus1 : %d\n", sps_max_latency_increase_plus1[i]);
  }

  LOG1("log2_min_luma_coding_block_size : %d\n", log2_min_luma_coding_block_size);
  LOG1("log2_diff_max_min_luma_coding_block_size : %d\n",log2_diff_max_min_luma_coding_block_size);
  LOG1("log2_min_transform_block_size   : %d\n", log2_min_transform_block_size);
  LOG1("log2_diff_max_min_transform_block_size : %d\n", log2_diff_max_min_transform_block_size);
  LOG1("max_transform_hierarchy_depth_inter : %d\n", max_transform_hierarchy_depth_inter);
  LOG1("max_transform_hierarchy_depth_intra : %d\n", max_transform_hierarchy_depth_intra);
  LOG1("scaling_list_enable_flag : %d\n", scaling_list_enable_flag);

  if (scaling_list_enable_flag) {

    LOG1("sps_scaling_list_data_present_flag : %d\n", sps_scaling_list_data_present_flag);
    if (sps_scaling_list_data_present_flag) {

      LOG0("scaling list logging output not implemented");
      //assert(0);
      //scaling_list_data()
    }
  }

  LOG1("amp_enabled_flag                    : %d\n", amp_enabled_flag);
  LOG1("sample_adaptive_offset_enabled_flag : %d\n", sample_adaptive_offset_enabled_flag);
  LOG1("pcm_enabled_flag                    : %d\n", pcm_enabled_flag);

  if (pcm_enabled_flag) {
    LOG1("pcm_sample_bit_depth_luma     : %d\n", pcm_sample_bit_depth_luma);
    LOG1("pcm_sample_bit_depth_chroma   : %d\n", pcm_sample_bit_depth_chroma);
    LOG1("log2_min_pcm_luma_coding_block_size : %d\n", log2_min_pcm_luma_coding_block_size);
    LOG1("log2_diff_max_min_pcm_luma_coding_block_size : %d\n", log2_diff_max_min_pcm_luma_coding_block_size);
    LOG1("pcm_loop_filter_disable_flag  : %d\n", pcm_loop_filter_disable_flag);
  }

  LOG1("num_short_term_ref_pic_sets : %d\n", ref_pic_sets.size());

  for (int i = 0; i < ref_pic_sets.size(); i++) {
    LOG1("ref_pic_set[ %2d ]: ",i);
    dump_compact_short_term_ref_pic_set(&ref_pic_sets[i], 16, fh);
  }

  LOG1("long_term_ref_pics_present_flag : %d\n", long_term_ref_pics_present_flag);

  if (long_term_ref_pics_present_flag) {

    LOG1("num_long_term_ref_pics_sps : %d\n", num_long_term_ref_pics_sps);

    for (int i = 0; i < num_long_term_ref_pics_sps; i++ ) {
      LOG3("lt_ref_pic_poc_lsb_sps[%d] : %d   (used_by_curr_pic_lt_sps_flag=%d)\n",
           i, lt_ref_pic_poc_lsb_sps[i], used_by_curr_pic_lt_sps_flag[i]);
    }
  }

  LOG1("sps_temporal_mvp_enabled_flag      : %d\n", sps_temporal_mvp_enabled_flag);
  LOG1("strong_intra_smoothing_enable_flag : %d\n", strong_intra_smoothing_enable_flag);
  LOG1("vui_parameters_present_flag        : %d\n", vui_parameters_present_flag);

  LOG1("sps_extension_present_flag    : %d\n", sps_extension_present_flag);
  LOG1("sps_range_extension_flag      : %d\n", sps_range_extension_flag);
  LOG1("sps_multilayer_extension_flag : %d\n", sps_multilayer_extension_flag);
  LOG1("sps_extension_6bits           : %d\n", sps_extension_6bits);

  LOG1("CtbSizeY     : %d\n", CtbSizeY);
  LOG1("MinCbSizeY   : %d\n", MinCbSizeY);
  LOG1("MaxCbSizeY   : %d\n", 1<<(log2_min_luma_coding_block_size + log2_diff_max_min_luma_coding_block_size));
  LOG1("MinTBSizeY   : %d\n", 1<<log2_min_transform_block_size);
  LOG1("MaxTBSizeY   : %d\n", 1<<(log2_min_transform_block_size + log2_diff_max_min_transform_block_size));

  LOG1("PicWidthInCtbsY         : %d\n", PicWidthInCtbsY);
  LOG1("PicHeightInCtbsY        : %d\n", PicHeightInCtbsY);
  LOG1("SubWidthC               : %d\n", SubWidthC);
  LOG1("SubHeightC              : %d\n", SubHeightC);

  if (sps_range_extension_flag) {
    range_extension.dump(fd);
  }

  return;

  if (vui_parameters_present_flag) {
    assert(false);
    /*
      vui_parameters()

        sps_extension_flag
        u(1)
        if( sps_extension_flag )

          while( more_rbsp_data() )

            sps_extension_data_flag
              u(1)
              rbsp_trailing_bits()
    */
  }
#endif

#undef LOG0
#undef LOG1
#undef LOG2
#undef LOG3
  //#endif
}
