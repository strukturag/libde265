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
#include <sstream>

#define READ_VLC_OFFSET(variable, vlctype, offset)   \
  if ((vlc = get_ ## vlctype(br)) == UVLC_ERROR) {   \
    return DE265_WARNING_INVALID_VUI_PARAMETER;      \
  } \
  variable = vlc + offset;

#define READ_VLC(variable, vlctype)  READ_VLC_OFFSET(variable,vlctype,0)

#define LOG(...) log2sstr(sstr, __VA_ARGS__)


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


const char* get_video_format_name(enum VideoFormat format)
{
  switch (format) {
  case VideoFormat_Component: return "component";
  case VideoFormat_PAL:       return "PAL";
  case VideoFormat_NTSC:      return "NTSC";
  case VideoFormat_SECAM:     return "SECAM";
  case VideoFormat_MAC:       return "MAC";
  default:                    return "unspecified";
  }
}


video_usability_information::video_usability_information()
{
  aspect_ratio_info_present_flag = false;
  sar_width  = 0;
  sar_height = 0;


  // --- overscan ---

  overscan_info_present_flag = false;
  overscan_appropriate_flag  = false;


  // --- video signal type ---

  video_signal_type_present_flag = false;
  video_format = VideoFormat_Unspecified;
  video_full_range_flag = false;
  colour_description_present_flag = false;
  colour_primaries = 2;
  transfer_characteristics = 2;
  matrix_coeffs = 2;

  // --- chroma / interlaced ---

  chroma_loc_info_present_flag = false;
  chroma_sample_loc_type_top_field    = 0;
  chroma_sample_loc_type_bottom_field = 0;

  neutral_chroma_indication_flag = false;
  field_seq_flag = false;
  frame_field_info_present_flag = false;

  // --- default display window ---

  default_display_window_flag = false;
  def_disp_win_left_offset   = 0;
  def_disp_win_right_offset  = 0;
  def_disp_win_top_offset    = 0;
  def_disp_win_bottom_offset = 0;


  // --- timing ---

  vui_timing_info_present_flag = false;
  vui_num_units_in_tick = 0;
  vui_time_scale = 0;

  vui_poc_proportional_to_timing_flag = false;
  vui_num_ticks_poc_diff_one = 1;


  // --- hrd parameters ---

  vui_hrd_parameters_present_flag = false;
  //hrd_parameters vui_hrd_parameters;


  // --- bitstream restriction ---

  bitstream_restriction_flag = false;
  tiles_fixed_structure_flag = false;
  motion_vectors_over_pic_boundaries_flag = true;
  restricted_ref_pic_lists_flag = false;
  min_spatial_segmentation_idc = 0;
  max_bytes_per_pic_denom   = 2;
  max_bits_per_min_cu_denom = 1;
  log2_max_mv_length_horizontal = 15;
  log2_max_mv_length_vertical   = 15;
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
  if (vui_poc_proportional_to_timing_flag) {
    READ_VLC_OFFSET(vui_num_ticks_poc_diff_one, uvlc, 1);
  }


  // --- hrd parameters ---

  vui_hrd_parameters_present_flag = get_bits(br,1);
  if (vui_hrd_parameters_present_flag) {
    de265_error err = vui_hrd_parameters.read(errqueue, br, sps,
                                              true, sps->sps_max_sub_layers);
  }


  // --- bitstream restriction ---

  bitstream_restriction_flag = get_bits(br,1);
  if (bitstream_restriction_flag) {
    tiles_fixed_structure_flag = get_bits(br,1);
    motion_vectors_over_pic_boundaries_flag = get_bits(br,1);
    restricted_ref_pic_lists_flag = get_bits(br,1);

    READ_VLC(min_spatial_segmentation_idc, uvlc);
    if (min_spatial_segmentation_idc > 4095) {
      errqueue->add_warning(DE265_WARNING_INVALID_VUI_PARAMETER, false);
      min_spatial_segmentation_idc = 0;
    }

    READ_VLC(max_bytes_per_pic_denom, uvlc);
    if (max_bytes_per_pic_denom > 16) {
      errqueue->add_warning(DE265_WARNING_INVALID_VUI_PARAMETER, false);
      max_bytes_per_pic_denom = 2;
    }

    READ_VLC(max_bits_per_min_cu_denom, uvlc);
    if (max_bits_per_min_cu_denom > 16) {
      errqueue->add_warning(DE265_WARNING_INVALID_VUI_PARAMETER, false);
      max_bits_per_min_cu_denom = 1;
    }

    READ_VLC(log2_max_mv_length_horizontal, uvlc);
    if (log2_max_mv_length_horizontal > 15) {
      errqueue->add_warning(DE265_WARNING_INVALID_VUI_PARAMETER, false);
      log2_max_mv_length_horizontal = 15;
    }

    READ_VLC(log2_max_mv_length_vertical, uvlc);
    if (log2_max_mv_length_vertical > 15) {
      errqueue->add_warning(DE265_WARNING_INVALID_VUI_PARAMETER, false);
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


std::string video_usability_information::dump() const
{
  std::stringstream sstr;

  LOG("----------------- VUI -----------------\n");
  LOG("sample aspect ratio        : %d:%d\n", sar_width,sar_height);
  LOG("overscan_info_present_flag : %d\n", overscan_info_present_flag);
  LOG("overscan_appropriate_flag  : %d\n", overscan_appropriate_flag);

  LOG("video_signal_type_present_flag: %d\n", video_signal_type_present_flag);
  if (video_signal_type_present_flag) {
    LOG("  video_format                : %s\n", get_video_format_name(video_format));
    LOG("  video_full_range_flag       : %d\n", video_full_range_flag);
    LOG("  colour_description_present_flag : %d\n", colour_description_present_flag);
    LOG("  colour_primaries            : %d\n", colour_primaries);
    LOG("  transfer_characteristics    : %d\n", transfer_characteristics);
    LOG("  matrix_coeffs               : %d\n", matrix_coeffs);
  }

  LOG("chroma_loc_info_present_flag: %d\n", chroma_loc_info_present_flag);
  if (chroma_loc_info_present_flag) {
    LOG("  chroma_sample_loc_type_top_field   : %d\n", chroma_sample_loc_type_top_field);
    LOG("  chroma_sample_loc_type_bottom_field: %d\n", chroma_sample_loc_type_bottom_field);
  }

  LOG("neutral_chroma_indication_flag: %d\n", neutral_chroma_indication_flag);
  LOG("field_seq_flag                : %d\n", field_seq_flag);
  LOG("frame_field_info_present_flag : %d\n", frame_field_info_present_flag);

  LOG("default_display_window_flag   : %d\n", default_display_window_flag);
  LOG("  def_disp_win_left_offset    : %d\n", def_disp_win_left_offset);
  LOG("  def_disp_win_right_offset   : %d\n", def_disp_win_right_offset);
  LOG("  def_disp_win_top_offset     : %d\n", def_disp_win_top_offset);
  LOG("  def_disp_win_bottom_offset  : %d\n", def_disp_win_bottom_offset);

  LOG("vui_timing_info_present_flag  : %d\n", vui_timing_info_present_flag);
  if (vui_timing_info_present_flag) {
    LOG("  vui_num_units_in_tick       : %d\n", vui_num_units_in_tick);
    LOG("  vui_time_scale              : %d\n", vui_time_scale);
  }

  LOG("vui_poc_proportional_to_timing_flag : %d\n", vui_poc_proportional_to_timing_flag);
  LOG("vui_num_ticks_poc_diff_one          : %d\n", vui_num_ticks_poc_diff_one);

  LOG("vui_hrd_parameters_present_flag : %d\n", vui_hrd_parameters_present_flag);
  if (vui_hrd_parameters_present_flag) {
    //hrd_parameters vui_hrd_parameters;
  }


  // --- bitstream restriction ---

  LOG("bitstream_restriction_flag         : %d\n", bitstream_restriction_flag);
  if (bitstream_restriction_flag) {
    LOG("  tiles_fixed_structure_flag       : %d\n", tiles_fixed_structure_flag);
    LOG("  motion_vectors_over_pic_boundaries_flag : %d\n", motion_vectors_over_pic_boundaries_flag);
    LOG("  restricted_ref_pic_lists_flag    : %d\n", restricted_ref_pic_lists_flag);
    LOG("  min_spatial_segmentation_idc     : %d\n", min_spatial_segmentation_idc);
    LOG("  max_bytes_per_pic_denom          : %d\n", max_bytes_per_pic_denom);
    LOG("  max_bits_per_min_cu_denom        : %d\n", max_bits_per_min_cu_denom);
    LOG("  log2_max_mv_length_horizontal    : %d\n", log2_max_mv_length_horizontal);
    LOG("  log2_max_mv_length_vertical      : %d\n", log2_max_mv_length_vertical);
  }

  return sstr.str();
}


de265_error read_sub_layer_hrd_parameters(std::vector<sub_layer_hrd_parameters>& output,
                                          bitreader* br, int CpbCnt,
                                          bool sub_pic_hrd_params_present_flag)
{
  output.clear();

  int vlc;

  for (int i=0; i<=CpbCnt; i++) {
    sub_layer_hrd_parameters p;

    READ_VLC_OFFSET(p.bit_rate_value, uvlc, 1);
    READ_VLC_OFFSET(p.cpb_size_value, uvlc, 1);

    if (sub_pic_hrd_params_present_flag) {
      READ_VLC_OFFSET(p.cpb_size_du_value, uvlc, 1);
      READ_VLC_OFFSET(p.bit_rate_du_value, uvlc, 1);
    }

    p.cbr_flag = get_bits(br,1);

    output.push_back(p);
  }

  return DE265_OK;
}


hrd_parameters::hrd_parameters()
{
}


de265_error hrd_parameters::read(error_queue* errqueue, bitreader* br, const seq_parameter_set* sps,
                                 bool commonInfPresentFlag, int maxNumSubLayers)
{
  int vlc;

  if (commonInfPresentFlag) {
    nal_hrd_parameters_present_flag = get_bits(br,1);
    vcl_hrd_parameters_present_flag = get_bits(br,1);

    if (nal_hrd_parameters_present_flag ||
        vcl_hrd_parameters_present_flag) {

      sub_pic_hrd_params_present_flag = get_bits(br,1);

      if (sub_pic_hrd_params_present_flag) {
        tick_divisor = get_bits(br,8) + 2;
        du_cpb_removal_delay_increment_length = get_bits(br,5) + 1;
        sub_pic_cpb_params_in_pic_timing_sei_flag = get_bits(br,1);
        dpb_output_delay_du_length = get_bits(br,5) + 1;
      }

      bit_rate_scale = get_bits(br,4);
      cpb_size_scale = get_bits(br,4);

      if (sub_pic_hrd_params_present_flag) {
        cpb_size_du_scale = get_bits(br,4);
      }

      initial_cpb_removal_delay_length = get_bits(br,5) + 1;
      au_cpb_removal_delay_length = get_bits(br,5) + 1;
      dpb_output_delay_length = get_bits(br,5) + 1;
    }
  }

  for (int i=0; i<=maxNumSubLayers-1; i++) {
    hrd_sub_layer_parameters param;
    param.fixed_pic_rate_general_flag = get_bits(br,1);
    if (!param.fixed_pic_rate_general_flag) {
      param.fixed_pic_rate_within_cvs_flag = get_bits(br,1);
    }
    else {
      param.fixed_pic_rate_within_cvs_flag = false;
    }

    if (param.fixed_pic_rate_within_cvs_flag) {
      READ_VLC_OFFSET(param.element_duration_in_tc, uvlc, 1);
      param.low_delay_hrd_flag = false;
    }
    else {
      param.low_delay_hrd_flag = get_bits(br,1);
    }

    if (!param.low_delay_hrd_flag) {
      READ_VLC_OFFSET(param.cpb_cnt, uvlc, 1);
    }

    if (nal_hrd_parameters_present_flag) {
      de265_error err = read_sub_layer_hrd_parameters(param.nal_hrd, br, param.cpb_cnt,
                                                      sub_pic_hrd_params_present_flag);
      if (err) {
        return err;
      }
    }

    if (vcl_hrd_parameters_present_flag) {
      de265_error err = read_sub_layer_hrd_parameters(param.vcl_hrd, br, param.cpb_cnt,
                                                      sub_pic_hrd_params_present_flag);
      if (err) {
        return err;
      }
    }


    sublayer_parameters.push_back(param);
  }

  return DE265_OK;
}
