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

#include "sps.h"
#include "util.h"
#include "scan.h"
#include "decctx.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define READ_VLC_OFFSET(variable, vlctype, offset)   \
  if (!get_ ## vlctype(br, &vlc)) {   \
    return DE265_WARNING_INVALID_SPS_PARAMETER; \
  } \
  variable = vlc + offset;

#define READ_VLC(variable, vlctype)  READ_VLC_OFFSET(variable,vlctype,0)

#define LOG(...) log2sstr(sstr, __VA_ARGS__)


static int SubWidthC_tab[]  = { 1,2,2,1 };
static int SubHeightC_tab[] = { 1,2,1,1 };


// TODO if (!check_high(ctx, vlc, 15)) return false;
// TODO if (!check_ulvc(ctx, vlc)) return false;


sps_range_extension::sps_range_extension()
{
}


seq_parameter_set::seq_parameter_set()
{
  // TODO: this is dangerous
  //memset(this,0,sizeof(seq_parameter_set));

  sps_read = false;
  //ref_pic_sets = NULL;

  set_defaults();
}


seq_parameter_set::~seq_parameter_set()
{
  //free(ref_pic_sets);
}


void seq_parameter_set::set_defaults()
{
  video_parameter_set_id = 0;
  sps_max_sub_layers = 1;
  sps_temporal_id_nesting_flag = 1;

  profile_tier_level_.general.set_defaults(Profile_Main, 6,2); // TODO

  seq_parameter_set_id = 0;
  chroma_format_idc = de265_chroma_420;
  ChromaArrayType = (de265_chroma)chroma_format_idc;

  separate_colour_plane_flag = 0;
  pic_width_in_luma_samples = 0;
  pic_height_in_luma_samples = 0;
  conformance_window_flag = 0;

  conf_win_left_offset   = 0;
  conf_win_right_offset  = 0;
  conf_win_top_offset    = 0;
  conf_win_bottom_offset = 0;

  bit_depth_luma  =8;
  bit_depth_chroma=8;

  log2_max_pic_order_cnt_lsb = 8;
  sps_sub_layer_ordering_info_present_flag = 0;

  sps_max_dec_pic_buffering[0] = 1;
  sps_max_num_reorder_pics[0]  = 0;
  sps_max_latency_increase_plus1[0] = 0;

  set_CB_size_range(16,64);
  set_TB_size_range(8,16);
  max_transform_hierarchy_depth_inter = 1;
  max_transform_hierarchy_depth_intra = 1;

  scaling_list_enable_flag = 0;
  sps_scaling_list_data_present_flag = 0;

  // TODO struct scaling_list_data scaling_list;

  amp_enabled_flag = 0;
  sample_adaptive_offset_enabled_flag = 0;
  pcm_enabled_flag = 0;

  pcm_sample_bit_depth_luma = 8;
  pcm_sample_bit_depth_chroma = 8;
  // TODO log2_min_pcm_luma_coding_block_size;
  // TODO log2_diff_max_min_pcm_luma_coding_block_size;
  pcm_loop_filter_disable_flag = 1;

  // num_short_term_ref_pic_sets = 0;
  // std::vector<ref_pic_set> ref_pic_sets; // [0 ; num_short_term_ref_pic_set (<=MAX_REF_PIC_SETS) )
  ref_pic_sets.clear();

  long_term_ref_pics_present_flag = 0;

  num_long_term_ref_pics_sps = 0;

  /* TODO
  int  lt_ref_pic_poc_lsb_sps[MAX_NUM_LT_REF_PICS_SPS];
  char used_by_curr_pic_lt_sps_flag[MAX_NUM_LT_REF_PICS_SPS];
  */

  sps_temporal_mvp_enabled_flag = 0;
  strong_intra_smoothing_enable_flag = 0;
  vui_parameters_present_flag = 0;

  /*
    if( vui_parameters_present_flag )
      vui_parameters()
  */

  sps_extension_present_flag = 0;
  sps_range_extension_flag = 0;
  sps_multilayer_extension_flag = 0;
  sps_extension_6bits = 0;
}


void seq_parameter_set::set_coded_resolution(int w,int h)
{
  assert(w>0);
  assert(h>0);

  pic_width_in_luma_samples  = align_up_power_of_two(w, MinCbSizeY);
  pic_height_in_luma_samples = align_up_power_of_two(h, MinCbSizeY);

  derive_picture_sizes_in_blocks();
}


void seq_parameter_set::set_conformance_window_offsets(int left, int right, int top, int bottom)
{
  conf_win_left_offset = left;
  conf_win_right_offset = right;
  conf_win_top_offset = top;
  conf_win_bottom_offset = bottom;

  conformance_window_flag = (left != 0 || right != 0 || top != 0 || bottom != 0);
}


bool seq_parameter_set::set_video_resolution(int w,int h)
{
  set_coded_resolution(align_up_power_of_two(w, MinCbSizeY),
                       align_up_power_of_two(h, MinCbSizeY) );


  int right_offset  = pic_width_in_luma_samples  - w;
  int bottom_offset = pic_height_in_luma_samples - h;

  set_conformance_window_offsets(0, right_offset >> SubWidthC,
                                 0, bottom_offset >> SubHeightC);

  if (SubWidthC ==2 && (right_offset & 1) ||
      SubHeightC==2 && (bottom_offset & 1)) {
    return false;
  }
  else {
    return true;
  }
}


void seq_parameter_set::set_bit_depths(int luma, int chroma)
{
  assert(luma   >= 8 && luma   <= 16);
  assert(chroma >= 8 && chroma <= 16);

  bit_depth_luma = luma;
  bit_depth_chroma = chroma;

  derive_bitdepth_parameters();
}


void seq_parameter_set::set_nBits_for_POC(int nBits)
{
  assert(nBits >= 4 && nBits <= 16);

  log2_max_pic_order_cnt_lsb = nBits;
  MaxPicOrderCntLsb = 1<<nBits;
}


void seq_parameter_set::set_CB_size_range_log2(int mini,int maxi)
{
  assert(maxi >= mini);
  assert(mini >= 3); // minimum: 8x8
  assert(maxi <= 6); // maximum: 64x64

  log2_min_luma_coding_block_size = mini;
  log2_diff_max_min_luma_coding_block_size = maxi-mini;

  derive_block_sizes();
  derive_picture_sizes_in_blocks();
}


void seq_parameter_set::set_TB_size_range_log2(int mini,int maxi)
{
  assert(maxi >= mini);
  assert(mini >= 2); // minimum: 4x4
  assert(maxi <= 5); // maximum: 32x32

  log2_min_transform_block_size = mini;
  log2_diff_max_min_transform_block_size = maxi-mini;

  derive_block_sizes();
  derive_picture_sizes_in_blocks();
}


void seq_parameter_set::set_CB_size_range(int minSize, int maxSize)
{
  assert(isPowerOf2(minSize));
  assert(isPowerOf2(maxSize));
  assert(maxSize >= minSize);

  set_CB_size_range_log2(Log2(minSize),
                         Log2(maxSize));
}


void seq_parameter_set::set_TB_size_range(int minSize, int maxSize)
{
  assert(isPowerOf2(minSize));
  assert(isPowerOf2(maxSize));

  set_TB_size_range_log2(Log2(minSize),
                         Log2(maxSize));
}


void seq_parameter_set::set_PCM_size_range(int minSize, int maxSize)
{
  assert(isPowerOf2(minSize));
  assert(isPowerOf2(maxSize));
  assert(maxSize >= minSize);

  int minSize_log2 = Log2(minSize);
  int maxSize_log2 = Log2(maxSize);

  log2_min_pcm_luma_coding_block_size = minSize_log2;
  log2_diff_max_min_pcm_luma_coding_block_size = maxSize_log2 - minSize_log2;

  derive_pcm_block_sizes();
}







de265_error seq_parameter_set::read(bitreader* br, error_queue* errqueue)
{
  int vlc;

  video_parameter_set_id = get_bits(br,4);
  sps_max_sub_layers     = get_bits(br,3) +1;
  if (sps_max_sub_layers>7) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }

  sps_temporal_id_nesting_flag = get_bits(br,1);

  profile_tier_level_.read(br, sps_max_sub_layers);

  READ_VLC(seq_parameter_set_id, uvlc);
  if (seq_parameter_set_id >= DE265_MAX_SPS_SETS) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }


  // --- decode chroma type ---

  READ_VLC(vlc, uvlc);
  chroma_format_idc = (de265_chroma)vlc;

  if (chroma_format_idc == 3) {
    separate_colour_plane_flag = get_bits(br,1);
  }
  else {
    separate_colour_plane_flag = false;
  }

  if (chroma_format_idc<0 ||
      chroma_format_idc>3) {
    return DE265_WARNING_INVALID_CHROMA_FORMAT;
  }


  // --- picture size ---

  READ_VLC(pic_width_in_luma_samples,  uvlc);
  READ_VLC(pic_height_in_luma_samples, uvlc);

  if (pic_width_in_luma_samples  == 0 ||
      pic_height_in_luma_samples == 0) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }

  if (pic_width_in_luma_samples > MAX_PICTURE_WIDTH ||
      pic_height_in_luma_samples> MAX_PICTURE_HEIGHT) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }

  conformance_window_flag = get_bits(br,1);

  if (conformance_window_flag) {
    READ_VLC(conf_win_left_offset,  uvlc);
    READ_VLC(conf_win_right_offset, uvlc);
    READ_VLC(conf_win_top_offset,   uvlc);
    READ_VLC(conf_win_bottom_offset,uvlc);
  }
  else {
    conf_win_left_offset  = 0;
    conf_win_right_offset = 0;
    conf_win_top_offset   = 0;
    conf_win_bottom_offset= 0;
  }

  READ_VLC_OFFSET(bit_depth_luma,  uvlc, 8);
  READ_VLC_OFFSET(bit_depth_chroma,uvlc, 8);

  READ_VLC_OFFSET(log2_max_pic_order_cnt_lsb, uvlc, 4);
  MaxPicOrderCntLsb = 1<<(log2_max_pic_order_cnt_lsb);


  // --- sub_layer_ordering_info ---

  sps_sub_layer_ordering_info_present_flag = get_bits(br,1);

  int firstLayer = (sps_sub_layer_ordering_info_present_flag ?
                    0 : sps_max_sub_layers-1 );

  for (int i=firstLayer ; i <= sps_max_sub_layers-1; i++ ) {

    // sps_max_dec_pic_buffering[i]

    if (!get_uvlc(br, &vlc) ||
        vlc+1 > MAX_NUM_REF_PICS) {
      return DE265_WARNING_INVALID_SPS_PARAMETER;
    }

    sps_max_dec_pic_buffering[i] = vlc+1;

    // sps_max_num_reorder_pics[i]

    READ_VLC(sps_max_num_reorder_pics[i], uvlc);


    // sps_max_latency_increase[i]

    READ_VLC(sps_max_latency_increase_plus1[i], uvlc);

    SpsMaxLatencyPictures[i] = (sps_max_num_reorder_pics[i] +
                                sps_max_latency_increase_plus1[i]-1);
  }

  // copy info to all layers if only specified once

  if (sps_sub_layer_ordering_info_present_flag) {
    int ref = sps_max_sub_layers-1;
    assert(ref<7);

    for (int i=0 ; i < sps_max_sub_layers-1; i++ ) {
      sps_max_dec_pic_buffering[i] = sps_max_dec_pic_buffering[ref];
      sps_max_num_reorder_pics[i]  = sps_max_num_reorder_pics[ref];
      sps_max_latency_increase_plus1[i]  = sps_max_latency_increase_plus1[ref];
    }
  }


  READ_VLC_OFFSET(log2_min_luma_coding_block_size, uvlc, 3);
  READ_VLC       (log2_diff_max_min_luma_coding_block_size, uvlc);
  READ_VLC_OFFSET(log2_min_transform_block_size, uvlc, 2);
  READ_VLC(log2_diff_max_min_transform_block_size, uvlc);
  READ_VLC(max_transform_hierarchy_depth_inter, uvlc);
  READ_VLC(max_transform_hierarchy_depth_intra, uvlc);

  if (log2_min_luma_coding_block_size > 6) { return DE265_WARNING_INVALID_SPS_PARAMETER; }
  if (log2_min_luma_coding_block_size + log2_diff_max_min_luma_coding_block_size > 6) { return DE265_WARNING_INVALID_SPS_PARAMETER; }
  if (log2_min_transform_block_size > 5) { return DE265_WARNING_INVALID_SPS_PARAMETER; }
  if (log2_min_transform_block_size + log2_diff_max_min_transform_block_size > 5) { return DE265_WARNING_INVALID_SPS_PARAMETER; }

  scaling_list_enable_flag = get_bits(br,1);

  if (scaling_list_enable_flag) {

    sps_scaling_list_data_present_flag = get_bits(br,1);
    if (sps_scaling_list_data_present_flag) {

      de265_error err;
      if ((err=scaling_list.read(br,this, false)) != DE265_OK) {
        return err;
      }
    }
    else {
      scaling_list.set_default_scaling_lists();
    }
  }

  amp_enabled_flag = get_bits(br,1);
  sample_adaptive_offset_enabled_flag = get_bits(br,1);
  pcm_enabled_flag = get_bits(br,1);
  if (pcm_enabled_flag) {
    pcm_sample_bit_depth_luma = get_bits(br,4)+1;
    pcm_sample_bit_depth_chroma = get_bits(br,4)+1;
    READ_VLC_OFFSET(log2_min_pcm_luma_coding_block_size, uvlc, 3);
    READ_VLC(log2_diff_max_min_pcm_luma_coding_block_size, uvlc);
    pcm_loop_filter_disable_flag = get_bits(br,1);
  }
  else {
    pcm_sample_bit_depth_luma = 0;
    pcm_sample_bit_depth_chroma = 0;
    log2_min_pcm_luma_coding_block_size = 0;
    log2_diff_max_min_pcm_luma_coding_block_size = 0;
    pcm_loop_filter_disable_flag = 0;
  }

  int num_short_term_ref_pic_sets;
  READ_VLC(num_short_term_ref_pic_sets, uvlc);
  if (num_short_term_ref_pic_sets < 0 ||
      num_short_term_ref_pic_sets > 64) {
    return DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE;
  }

  // --- allocate reference pic set ---

  // we do not allocate the ref-pic-set for the slice header here, but in the slice header itself

  ref_pic_sets.resize(num_short_term_ref_pic_sets);

  for (int i = 0; i < num_short_term_ref_pic_sets; i++) {

    de265_error err = ref_pic_sets[i].read(errqueue,this,br,
                                           i,
                                           ref_pic_sets,
                                           false);

    if (err) {
      return err;
    }

    // dump_short_term_ref_pic_set(&(*ref_pic_sets)[i], fh);
  }

  long_term_ref_pics_present_flag = get_bits(br,1);

  if (long_term_ref_pics_present_flag) {

    READ_VLC(num_long_term_ref_pics_sps, uvlc);
    if (num_long_term_ref_pics_sps > MAX_NUM_LT_REF_PICS_SPS) {
      return DE265_WARNING_INVALID_SPS_PARAMETER;
    }

    for (int i = 0; i < num_long_term_ref_pics_sps; i++ ) {
      lt_ref_pic_poc_lsb_sps[i] = get_bits(br, log2_max_pic_order_cnt_lsb);
      used_by_curr_pic_lt_sps_flag[i] = get_bits(br,1);
    }
  }
  else {
    num_long_term_ref_pics_sps = 0; // NOTE: missing definition in standard !
  }

  sps_temporal_mvp_enabled_flag = get_bits(br,1);
  strong_intra_smoothing_enable_flag = get_bits(br,1);

  vui_parameters_present_flag = get_bits(br,1);
  if (vui_parameters_present_flag) {
    de265_error err = vui.read(errqueue, br, this);
    if (err) {
      return err;
    }
  }


  sps_extension_present_flag = get_bits(br,1);
  if (sps_extension_present_flag) {
    sps_range_extension_flag = get_bits(br,1);
    sps_multilayer_extension_flag = get_bits(br,1);
    sps_extension_6bits = get_bits(br,6);
  }
  else {
    sps_range_extension_flag = 0;
  }

  if (sps_range_extension_flag) {
    de265_error err = range_extension.read(errqueue, br);
    if (err != DE265_OK) { return err; }
  }

  /*
  sps_extension_flag = get_bits(br,1);
  if (sps_extension_flag) {
    assert(false);
  }
  */


  de265_error err = compute_all_derived_values();
  if (err != DE265_OK) { return err; }

  if (!check_parameters_for_consistency()) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }

  sps_read = true;

  return DE265_OK;
}


void seq_parameter_set::set_chroma(de265_chroma chroma)
{
  chroma_format_idc = chroma;

  derive_chroma_parameters();
}

void seq_parameter_set::set_separate_colour_planes(bool flag)
{
  separate_colour_plane_flag = flag;

  derive_chroma_parameters();
}


void seq_parameter_set::enable_range_extension(bool flag)
{
  sps_range_extension_flag = flag;

  // if we disable the range extension, reset all its values
  if (flag==false) {
    range_extension = sps_range_extension();
  }

  derive_bitdepth_parameters();
}


de265_error seq_parameter_set::derive_chroma_parameters()
{
  if (separate_colour_plane_flag &&
      chroma_format_idc != de265_chroma_444) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }


  if (separate_colour_plane_flag) {
    ChromaArrayType = de265_chroma_mono;
  }
  else {
    ChromaArrayType = chroma_format_idc;
  }

  SubWidthC  = SubWidthC_tab [ChromaArrayType];
  SubHeightC = SubHeightC_tab[ChromaArrayType];

  return DE265_OK;
}


void seq_parameter_set::derive_bitdepth_parameters()
{
  BitDepth_Y   = bit_depth_luma;
  QpBdOffset_Y = 6*(bit_depth_luma-8);
  BitDepth_C   = bit_depth_chroma;
  QpBdOffset_C = 6*(bit_depth_chroma-8);


  // --- weighted prediction offsets

  if (range_extension.high_precision_offsets_enabled_flag) {
    WpOffsetBdShiftY = 0;
    WpOffsetBdShiftC = 0;
    WpOffsetHalfRangeY = 1 << (BitDepth_Y - 1);
    WpOffsetHalfRangeC = 1 << (BitDepth_C - 1);
  }
  else {
    WpOffsetBdShiftY = ( BitDepth_Y - 8 );
    WpOffsetBdShiftC = ( BitDepth_C - 8 );
    WpOffsetHalfRangeY = 1 << 7;
    WpOffsetHalfRangeC = 1 << 7;
  }
}


void seq_parameter_set::derive_block_sizes()
{
  Log2MinCbSizeY = log2_min_luma_coding_block_size;
  Log2CtbSizeY = Log2MinCbSizeY + log2_diff_max_min_luma_coding_block_size;
  MinCbSizeY = 1 << Log2MinCbSizeY;
  CtbSizeY = 1 << Log2CtbSizeY;

  if (chroma_format_idc==0 || separate_colour_plane_flag) {
    // chroma CTB size is undefined for monochrome content
    CtbWidthC  = 0;
    CtbHeightC = 0;
  }
  else {
    CtbWidthC  = CtbSizeY / SubWidthC;
    CtbHeightC = CtbSizeY / SubHeightC;
  }

  Log2MinTrafoSize = log2_min_transform_block_size;
  Log2MaxTrafoSize = log2_min_transform_block_size + log2_diff_max_min_transform_block_size;

  Log2MinPUSize = Log2MinCbSizeY-1;
}


void seq_parameter_set::derive_picture_sizes_in_blocks()
{
  if (MinCbSizeY != 0) {
    PicWidthInMinCbsY = ceil_div(pic_width_in_luma_samples, MinCbSizeY);
    PicHeightInMinCbsY = ceil_div(pic_height_in_luma_samples, MinCbSizeY);
  }

  if (CtbSizeY != 0) {
    PicWidthInCtbsY   = ceil_div(pic_width_in_luma_samples, CtbSizeY);
    PicHeightInCtbsY   = ceil_div(pic_height_in_luma_samples,CtbSizeY);
  }

  PicSizeInMinCbsY   = PicWidthInMinCbsY * PicHeightInMinCbsY;
  PicSizeInCtbsY = PicWidthInCtbsY * PicHeightInCtbsY;
  PicSizeInSamplesY = pic_width_in_luma_samples * pic_height_in_luma_samples;

  PicWidthInMinPUs  = PicWidthInCtbsY  << (Log2CtbSizeY - Log2MinPUSize);
  PicHeightInMinPUs = PicHeightInCtbsY << (Log2CtbSizeY - Log2MinPUSize);

  // the following are not in the standard
  PicWidthInTbsY  = PicWidthInCtbsY  << (Log2CtbSizeY - Log2MinTrafoSize);
  PicHeightInTbsY = PicHeightInCtbsY << (Log2CtbSizeY - Log2MinTrafoSize);
  PicSizeInTbsY = PicWidthInTbsY * PicHeightInTbsY;
}


void seq_parameter_set::derive_pcm_block_sizes()
{
  Log2MinIpcmCbSizeY = log2_min_pcm_luma_coding_block_size;
  Log2MaxIpcmCbSizeY = (log2_min_pcm_luma_coding_block_size +
                        log2_diff_max_min_pcm_luma_coding_block_size);
}


bool seq_parameter_set::check_parameters_for_consistency() const
{
  // separate_colour_plane_flag may only be true for chroma 4:4:4

  if (separate_colour_plane_flag == true &&
      chroma_format_idc != de265_chroma_444) {
    return false;
  }



  // conformance window must be at least one pixel in size

  if (conf_win_left_offset + conf_win_right_offset >= pic_width_in_luma_samples) {
    return false;
  }

  if (conf_win_top_offset + conf_win_bottom_offset >= pic_height_in_luma_samples) {
    return false;
  }


  // check block sizes

  if (Log2MaxTrafoSize > CtbSizeY) {
    return false;
  }

  if (Log2MinCbSizeY < Log2MinTrafoSize) {
    return false;
  }


  // --- check max transform depths

  // depths larger than the full span (CTB size -> min trafo size) are invalid

  if (max_transform_hierarchy_depth_inter > Log2CtbSizeY - Log2MinTrafoSize) {
    return false;
  }

  if (max_transform_hierarchy_depth_intra > Log2CtbSizeY - Log2MinTrafoSize) {
    return false;
  }



  // --- check SPS sanity ---

  if (pic_width_in_luma_samples  % MinCbSizeY != 0 ||
      pic_height_in_luma_samples % MinCbSizeY != 0) {
    return false;
  }

  if (Log2MinTrafoSize > Log2MinCbSizeY) {
    return false;
  }

  if (Log2MaxTrafoSize > libde265_min(Log2CtbSizeY,5)) {
    return false;
  }


  if (BitDepth_Y < 8 || BitDepth_Y > 16) {
    return false;
  }

  if (BitDepth_C < 8 || BitDepth_C > 16) {
    return false;
  }

  return true;
}


de265_error seq_parameter_set::compute_all_derived_values()
{
  de265_error err;

  if ( (err=derive_chroma_parameters()) ) {
    return err;
  }

  derive_bitdepth_parameters();
  derive_block_sizes();
  derive_picture_sizes_in_blocks();
  derive_pcm_block_sizes();

  return DE265_OK;
}



std::string seq_parameter_set::dump() const
{
  std::stringstream sstr;

  LOG("----------------- SPS -----------------\n");
  LOG("video_parameter_set_id  : %d\n", video_parameter_set_id);
  LOG("sps_max_sub_layers      : %d\n", sps_max_sub_layers);
  LOG("sps_temporal_id_nesting_flag : %d\n", sps_temporal_id_nesting_flag);

  sstr << profile_tier_level_.dump(sps_max_sub_layers);

  LOG("seq_parameter_set_id    : %d\n", seq_parameter_set_id);
  LOG("chroma_format_idc       : %d (%s)\n", chroma_format_idc,
      chroma_format_idc == 0 ? "monochrome" :
      chroma_format_idc == 1 ? "4:2:0" :
      chroma_format_idc == 2 ? "4:2:2" :
      chroma_format_idc == 3 ? "4:4:4" : "unknown");

  if (chroma_format_idc == 3) {
    LOG("separate_colour_plane_flag : %d\n", separate_colour_plane_flag);
  }

  LOG("pic_width_in_luma_samples  : %d\n", pic_width_in_luma_samples);
  LOG("pic_height_in_luma_samples : %d\n", pic_height_in_luma_samples);
  LOG("conformance_window_flag    : %d\n", conformance_window_flag);

  if (conformance_window_flag) {
    LOG("conf_win_left_offset  : %d\n", conf_win_left_offset);
    LOG("conf_win_right_offset : %d\n", conf_win_right_offset);
    LOG("conf_win_top_offset   : %d\n", conf_win_top_offset);
    LOG("conf_win_bottom_offset: %d\n", conf_win_bottom_offset);
  }

  LOG("bit_depth_luma   : %d\n", bit_depth_luma);
  LOG("bit_depth_chroma : %d\n", bit_depth_chroma);

  LOG("log2_max_pic_order_cnt_lsb : %d\n", log2_max_pic_order_cnt_lsb);
  LOG("sps_sub_layer_ordering_info_present_flag : %d\n", sps_sub_layer_ordering_info_present_flag);

  int firstLayer = (sps_sub_layer_ordering_info_present_flag ?
                    0 : sps_max_sub_layers-1 );

  for (int i=firstLayer ; i <= sps_max_sub_layers-1; i++ ) {
    LOG("Layer %d\n",i);
    LOG("  sps_max_dec_pic_buffering      : %d\n", sps_max_dec_pic_buffering[i]);
    LOG("  sps_max_num_reorder_pics       : %d\n", sps_max_num_reorder_pics[i]);
    LOG("  sps_max_latency_increase_plus1 : %d\n", sps_max_latency_increase_plus1[i]);
  }

  LOG("log2_min_luma_coding_block_size : %d\n", log2_min_luma_coding_block_size);
  LOG("log2_diff_max_min_luma_coding_block_size : %d\n",log2_diff_max_min_luma_coding_block_size);
  LOG("log2_min_transform_block_size   : %d\n", log2_min_transform_block_size);
  LOG("log2_diff_max_min_transform_block_size : %d\n", log2_diff_max_min_transform_block_size);
  LOG("max_transform_hierarchy_depth_inter : %d\n", max_transform_hierarchy_depth_inter);
  LOG("max_transform_hierarchy_depth_intra : %d\n", max_transform_hierarchy_depth_intra);
  LOG("scaling_list_enable_flag : %d\n", scaling_list_enable_flag);

  if (scaling_list_enable_flag) {

    LOG("sps_scaling_list_data_present_flag : %d\n", sps_scaling_list_data_present_flag);
    if (sps_scaling_list_data_present_flag) {

      LOG("scaling list logging output not implemented");
      //assert(0);
      //scaling_list_data()
    }
  }

  LOG("amp_enabled_flag                    : %d\n", amp_enabled_flag);
  LOG("sample_adaptive_offset_enabled_flag : %d\n", sample_adaptive_offset_enabled_flag);
  LOG("pcm_enabled_flag                    : %d\n", pcm_enabled_flag);

  if (pcm_enabled_flag) {
    LOG("pcm_sample_bit_depth_luma     : %d\n", pcm_sample_bit_depth_luma);
    LOG("pcm_sample_bit_depth_chroma   : %d\n", pcm_sample_bit_depth_chroma);
    LOG("log2_min_pcm_luma_coding_block_size : %d\n", log2_min_pcm_luma_coding_block_size);
    LOG("log2_diff_max_min_pcm_luma_coding_block_size : %d\n", log2_diff_max_min_pcm_luma_coding_block_size);
    LOG("pcm_loop_filter_disable_flag  : %d\n", pcm_loop_filter_disable_flag);
  }

  LOG("num_short_term_ref_pic_sets : %d\n", ref_pic_sets.size());

  for (int i = 0; i < ref_pic_sets.size(); i++) {
    LOG("ref_pic_set[ %2d ]: ",i);
    sstr << ref_pic_sets[i].dump_compact(16);
  }

  LOG("long_term_ref_pics_present_flag : %d\n", long_term_ref_pics_present_flag);

  if (long_term_ref_pics_present_flag) {

    LOG("num_long_term_ref_pics_sps : %d\n", num_long_term_ref_pics_sps);

    for (int i = 0; i < num_long_term_ref_pics_sps; i++ ) {
      LOG("lt_ref_pic_poc_lsb_sps[%d] : %d   (used_by_curr_pic_lt_sps_flag=%d)\n",
          i, lt_ref_pic_poc_lsb_sps[i], used_by_curr_pic_lt_sps_flag[i]);
    }
  }

  LOG("sps_temporal_mvp_enabled_flag      : %d\n", sps_temporal_mvp_enabled_flag);
  LOG("strong_intra_smoothing_enable_flag : %d\n", strong_intra_smoothing_enable_flag);
  LOG("vui_parameters_present_flag        : %d\n", vui_parameters_present_flag);

  LOG("sps_extension_present_flag    : %d\n", sps_extension_present_flag);
  LOG("sps_range_extension_flag      : %d\n", sps_range_extension_flag);
  LOG("sps_multilayer_extension_flag : %d\n", sps_multilayer_extension_flag);
  LOG("sps_extension_6bits           : %d\n", sps_extension_6bits);

  LOG("CtbSizeY     : %d\n", CtbSizeY);
  LOG("MinCbSizeY   : %d\n", MinCbSizeY);
  LOG("MaxCbSizeY   : %d\n", 1<<(log2_min_luma_coding_block_size + log2_diff_max_min_luma_coding_block_size));
  LOG("MinTBSizeY   : %d\n", 1<<log2_min_transform_block_size);
  LOG("MaxTBSizeY   : %d\n", 1<<(log2_min_transform_block_size + log2_diff_max_min_transform_block_size));

  LOG("PicWidthInCtbsY         : %d\n", PicWidthInCtbsY);
  LOG("PicHeightInCtbsY        : %d\n", PicHeightInCtbsY);
  LOG("SubWidthC               : %d\n", SubWidthC);
  LOG("SubHeightC              : %d\n", SubHeightC);

  if (sps_range_extension_flag) {
    sstr << range_extension.dump();
  }

  if (vui_parameters_present_flag) {
    sstr << vui.dump();
  }

  return sstr.str();
}


static uint8_t default_ScalingList_4x4[16] = {
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16
};

static uint8_t default_ScalingList_8x8_intra[64] = {
  16,16,16,16,16,16,16,16,
  16,16,17,16,17,16,17,18,
  17,18,18,17,18,21,19,20,
  21,20,19,21,24,22,22,24,
  24,22,22,24,25,25,27,30,
  27,25,25,29,31,35,35,31,
  29,36,41,44,41,36,47,54,
  54,47,65,70,65,88,88,115
};

static uint8_t default_ScalingList_8x8_inter[64] = {
  16,16,16,16,16,16,16,16,
  16,16,17,17,17,17,17,18,
  18,18,18,18,18,20,20,20,
  20,20,20,20,24,24,24,24,
  24,24,24,24,25,25,25,25,
  25,25,25,28,28,28,28,28,
  28,33,33,33,33,33,41,41,
  41,41,54,54,54,71,71,91
};


static void fill_scaling_factor(uint8_t* scalingFactors, const uint8_t* sclist, int sizeId)
{
  const position* scan;
  int width;
  int subWidth;

  switch (sizeId) {
  case 0:
    width=4;
    subWidth=1;
    scan = get_scan_order(2, 0 /* diag */);

    for (int i=0;i<4*4;i++) {
      scalingFactors[scan[i].x + width*scan[i].y] = sclist[i];
    }
    break;

  case 1:
    width=8;
    subWidth=1;
    scan = get_scan_order(3, 0 /* diag */);

    for (int i=0;i<8*8;i++) {
      scalingFactors[scan[i].x + width*scan[i].y] = sclist[i];
    }
    break;

  case 2:
    width=8;
    subWidth=2;
    scan = get_scan_order(3, 0 /* diag */);

    for (int i=0;i<8*8;i++) {
      for (int dy=0;dy<2;dy++)
        for (int dx=0;dx<2;dx++)
          {
            int x = 2*scan[i].x+dx;
            int y = 2*scan[i].y+dy;
            scalingFactors[x+width*subWidth*y] = sclist[i];
          }
    }
    break;

  case 3:
    width=8;
    subWidth=4;
    scan = get_scan_order(3, 0 /* diag */);

    for (int i=0;i<8*8;i++) {
      for (int dy=0;dy<4;dy++)
        for (int dx=0;dx<4;dx++)
          {
            int x = 4*scan[i].x+dx;
            int y = 4*scan[i].y+dy;
            scalingFactors[x+width*subWidth*y] = sclist[i];
          }
    }
    break;

  default:
    assert(0);
    break;
  }


  // --- dump matrix ---

#if 0
  for (int y=0;y<width;y++) {
    for (int x=0;x<width;x++)
      printf("%d,",scalingFactors[x*subWidth + width*subWidth*subWidth*y]);

    printf("\n");
  }
#endif
}


de265_error scaling_list_data::read(bitreader* br, const seq_parameter_set* sps, bool inPPS)
{
  int dc_coeff[4][6];

  for (int sizeId=0;sizeId<4;sizeId++) {
    int n = ((sizeId==3) ? 2 : 6);
    uint8_t scaling_list[6][32*32];

    for (int matrixId=0;matrixId<n;matrixId++) {
      uint8_t* curr_scaling_list = scaling_list[matrixId];
      int scaling_list_dc_coef;

      int canonicalMatrixId = matrixId;
      if (sizeId==3 && matrixId==1) { canonicalMatrixId=3; }


      //printf("----- matrix %d\n",matrixId);

      char scaling_list_pred_mode_flag = get_bits(br,1);
      if (!scaling_list_pred_mode_flag) {
        int scaling_list_pred_matrix_id_delta;
        if (!get_uvlc(br, &scaling_list_pred_matrix_id_delta) ||
            scaling_list_pred_matrix_id_delta > matrixId) {
          return DE265_WARNING_INVALID_SPS_PARAMETER;
        }

        //printf("scaling_list_pred_matrix_id_delta=%d\n", scaling_list_pred_matrix_id_delta);

        dc_coeff[sizeId][matrixId] = 16;
        scaling_list_dc_coef       = 16;

        if (scaling_list_pred_matrix_id_delta==0) {
          if (sizeId==0) {
            memcpy(curr_scaling_list, default_ScalingList_4x4, 16);
          }
          else {
            if (canonicalMatrixId<3)
              { memcpy(curr_scaling_list, default_ScalingList_8x8_intra,64); }
            else
              { memcpy(curr_scaling_list, default_ScalingList_8x8_inter,64); }
          }
        }
        else {
          // TODO: CHECK: for sizeID=3 and the second matrix, should we have delta=1 or delta=3 ?
          if (sizeId==3) { assert(scaling_list_pred_matrix_id_delta==1); }

          int mID = matrixId - scaling_list_pred_matrix_id_delta;

          int len = (sizeId == 0 ? 16 : 64);
          memcpy(curr_scaling_list, scaling_list[mID], len);

          scaling_list_dc_coef       = dc_coeff[sizeId][mID];
          dc_coeff[sizeId][matrixId] = dc_coeff[sizeId][mID];
        }
      }
      else {
        int nextCoef=8;
        int coefNum = (sizeId==0 ? 16 : 64);
        if (sizeId>1) {
          if (!get_svlc(br, &scaling_list_dc_coef) ||
              scaling_list_dc_coef < -7 ||
              scaling_list_dc_coef > 247) {
            return DE265_WARNING_INVALID_SPS_PARAMETER;
          }

          scaling_list_dc_coef += 8;
          nextCoef=scaling_list_dc_coef;
          dc_coeff[sizeId][matrixId] = scaling_list_dc_coef;
        }
        else {
          scaling_list_dc_coef = 16;
        }
        //printf("DC = %d\n",scaling_list_dc_coef);

        for (int i=0;i<coefNum;i++) {
          int scaling_list_delta_coef;
          if (!get_svlc(br, &scaling_list_delta_coef) ||
              scaling_list_delta_coef < -128 ||
              scaling_list_delta_coef >  127) {
            return DE265_WARNING_INVALID_SPS_PARAMETER;
          }

          nextCoef = (nextCoef + scaling_list_delta_coef + 256) % 256;
          curr_scaling_list[i] = nextCoef;
          //printf("curr %d = %d\n",i,nextCoef);
        }
      }


      // --- generate ScalingFactor arrays ---

      switch (sizeId) {
      case 0:
        fill_scaling_factor(&ScalingFactor_Size0[matrixId][0][0], curr_scaling_list, 0);
        break;

      case 1:
        fill_scaling_factor(&ScalingFactor_Size1[matrixId][0][0], curr_scaling_list, 1);
        break;

      case 2:
        fill_scaling_factor(&ScalingFactor_Size2[matrixId][0][0], curr_scaling_list, 2);
        ScalingFactor_Size2[matrixId][0][0] = scaling_list_dc_coef;
        //printf("DC coeff: %d\n", scaling_list_dc_coef);
        break;

      case 3:
        fill_scaling_factor(&ScalingFactor_Size3[matrixId][0][0], curr_scaling_list, 3);
        ScalingFactor_Size3[matrixId][0][0] = scaling_list_dc_coef;
        //printf("DC coeff: %d\n", scaling_list_dc_coef);
        break;
      }
    }
  }

  return DE265_OK;
}


de265_error scaling_list_data::write(CABAC_encoder& out, const seq_parameter_set* sps, bool inPPS)
{
  assert(false);
  // TODO

  return DE265_OK;
}


void scaling_list_data::set_default_scaling_lists()
{
  // 4x4

  for (int matrixId=0;matrixId<6;matrixId++) {
    fill_scaling_factor(&ScalingFactor_Size0[matrixId][0][0],
                        default_ScalingList_4x4, 0);
  }

  // 8x8

  for (int matrixId=0;matrixId<3;matrixId++) {
    fill_scaling_factor(&ScalingFactor_Size1[matrixId+0][0][0],
                        default_ScalingList_8x8_intra, 1);
    fill_scaling_factor(&ScalingFactor_Size1[matrixId+3][0][0],
                        default_ScalingList_8x8_inter, 1);
  }

  // 16x16

  for (int matrixId=0;matrixId<3;matrixId++) {
    fill_scaling_factor(&ScalingFactor_Size2[matrixId+0][0][0],
                        default_ScalingList_8x8_intra, 2);
    fill_scaling_factor(&ScalingFactor_Size2[matrixId+3][0][0],
                        default_ScalingList_8x8_inter, 2);
  }

  // 32x32

  fill_scaling_factor(&ScalingFactor_Size3[0][0][0],
                      default_ScalingList_8x8_intra, 3);
  fill_scaling_factor(&ScalingFactor_Size3[1][0][0],
                      default_ScalingList_8x8_inter, 3);
}


de265_error seq_parameter_set::write(CABAC_encoder& out)
{
  out.write_bits(video_parameter_set_id, 4);
  if (sps_max_sub_layers>7) {
    return DE265_WARNING_INVALID_SPS_PARAMETER;
  }
  out.write_bits(sps_max_sub_layers-1, 3);

  out.write_bit(sps_temporal_id_nesting_flag);

  profile_tier_level_.write(out, sps_max_sub_layers);

  out.write_uvlc(seq_parameter_set_id);


  // --- encode chroma type ---

  out.write_uvlc(chroma_format_idc);

  if (chroma_format_idc<0 ||
      chroma_format_idc>3) {
    return DE265_WARNING_INVALID_CHROMA_FORMAT;
  }

  if (chroma_format_idc == 3) {
    out.write_bit(separate_colour_plane_flag);
  }


  // --- picture size ---

  out.write_uvlc(pic_width_in_luma_samples);
  out.write_uvlc(pic_height_in_luma_samples);

  out.write_bit(conformance_window_flag);

  if (conformance_window_flag) {
    out.write_uvlc(conf_win_left_offset);
    out.write_uvlc(conf_win_right_offset);
    out.write_uvlc(conf_win_top_offset);
    out.write_uvlc(conf_win_bottom_offset);
  }


  out.write_uvlc(bit_depth_luma-8);
  out.write_uvlc(bit_depth_chroma-8);

  out.write_uvlc(log2_max_pic_order_cnt_lsb-4);


  // --- sub_layer_ordering_info ---

  out.write_bit(sps_sub_layer_ordering_info_present_flag);

  int firstLayer = (sps_sub_layer_ordering_info_present_flag ?
                    0 : sps_max_sub_layers-1 );

  for (int i=firstLayer ; i <= sps_max_sub_layers-1; i++ ) {

    // sps_max_dec_pic_buffering[i]

    if (sps_max_dec_pic_buffering[i] > MAX_NUM_REF_PICS) {
      return DE265_WARNING_INVALID_SPS_PARAMETER;
    }

    out.write_uvlc(sps_max_dec_pic_buffering[i]-1);

    // sps_max_num_reorder_pics[i]

    out.write_uvlc(sps_max_num_reorder_pics[i]);


    // sps_max_latency_increase[i]

    out.write_uvlc(sps_max_latency_increase_plus1[i]);
  }


  out.write_uvlc(log2_min_luma_coding_block_size-3);
  out.write_uvlc(log2_diff_max_min_luma_coding_block_size);
  out.write_uvlc(log2_min_transform_block_size-2);
  out.write_uvlc(log2_diff_max_min_transform_block_size);
  out.write_uvlc(max_transform_hierarchy_depth_inter);
  out.write_uvlc(max_transform_hierarchy_depth_intra);
  out.write_bit(scaling_list_enable_flag);

  if (scaling_list_enable_flag) {

    out.write_bit(sps_scaling_list_data_present_flag);
    if (sps_scaling_list_data_present_flag) {

      de265_error err;
      if ((err=scaling_list.write(out,this, false)) != DE265_OK) {
        return err;
      }
    }
  }

  out.write_bit(amp_enabled_flag);
  out.write_bit(sample_adaptive_offset_enabled_flag);
  out.write_bit(pcm_enabled_flag);
  if (pcm_enabled_flag) {
    out.write_bits(pcm_sample_bit_depth_luma  -1,4);
    out.write_bits(pcm_sample_bit_depth_chroma-1,4);
    out.write_uvlc(log2_min_pcm_luma_coding_block_size-3);
    out.write_uvlc(log2_diff_max_min_pcm_luma_coding_block_size);
    out.write_bit(pcm_loop_filter_disable_flag);
  }

  int num_short_term_ref_pic_sets = ref_pic_sets.size();
  if (num_short_term_ref_pic_sets < 0 ||
      num_short_term_ref_pic_sets > 64) {
    return DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE;
  }
  out.write_uvlc(num_short_term_ref_pic_sets);

  // --- allocate reference pic set ---

  // we do not allocate the ref-pic-set for the slice header here, but in the slice header itself

  for (int i = 0; i < num_short_term_ref_pic_sets; i++) {

    bool success = ref_pic_sets[i].write(this,out,
                                         i,
                                         ref_pic_sets,
                                         false);

    if (!success) {
      return DE265_WARNING_SPS_HEADER_INVALID;
    }

    // dump_short_term_ref_pic_set(&(*ref_pic_sets)[i], fh);
  }

  out.write_bit(long_term_ref_pics_present_flag);

  if (long_term_ref_pics_present_flag) {

    if (num_long_term_ref_pics_sps > MAX_NUM_LT_REF_PICS_SPS) {
      return DE265_WARNING_INVALID_SPS_PARAMETER;
    }
    out.write_uvlc(num_long_term_ref_pics_sps);

    for (int i = 0; i < num_long_term_ref_pics_sps; i++ ) {
      out.write_bits(lt_ref_pic_poc_lsb_sps[i], log2_max_pic_order_cnt_lsb);
      out.write_bit (used_by_curr_pic_lt_sps_flag[i]);
    }
  }

  out.write_bit(sps_temporal_mvp_enabled_flag);
  out.write_bit(strong_intra_smoothing_enable_flag);
  out.write_bit(vui_parameters_present_flag);

#if 0
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

  out.write_bit(sps_extension_present_flag);

#if 0
  if (sps_extension_flag) {
    assert(false);
  }
  check_rbsp_trailing_bits(br);
#endif

  // --- compute derived values ---

#if 0
  BitDepth_Y   = bit_depth_luma;
  QpBdOffset_Y = 6*(bit_depth_luma-8);
  BitDepth_C   = bit_depth_chroma;
  QpBdOffset_C = 6*(bit_depth_chroma-8);
  Log2MinCbSizeY = log2_min_luma_coding_block_size;
  Log2CtbSizeY = Log2MinCbSizeY + log2_diff_max_min_luma_coding_block_size;
  MinCbSizeY = 1 << Log2MinCbSizeY;
  CtbSizeY = 1 << Log2CtbSizeY;
  PicWidthInMinCbsY = pic_width_in_luma_samples / MinCbSizeY;
  PicWidthInCtbsY   = ceil_div(pic_width_in_luma_samples, CtbSizeY);
  PicHeightInMinCbsY = pic_height_in_luma_samples / MinCbSizeY;
  PicHeightInCtbsY   = ceil_div(pic_height_in_luma_samples,CtbSizeY);
  PicSizeInMinCbsY   = PicWidthInMinCbsY * PicHeightInMinCbsY;
  PicSizeInCtbsY = PicWidthInCtbsY * PicHeightInCtbsY;
  PicSizeInSamplesY = pic_width_in_luma_samples * pic_height_in_luma_samples;
  if (chroma_format_idc==0 || separate_colour_plane_flag) {
    CtbWidthC  = 0;
    CtbHeightC = 0;
  }
  else {
    CtbWidthC  = CtbSizeY / SubWidthC;
    CtbHeightC = CtbSizeY / SubHeightC;
  }
  Log2MinTrafoSize = log2_min_transform_block_size;
  Log2MaxTrafoSize = log2_min_transform_block_size + log2_diff_max_min_transform_block_size;
  Log2MinPUSize = Log2MinCbSizeY-1;
  PicWidthInMinPUs  = PicWidthInCtbsY  << (Log2CtbSizeY - Log2MinPUSize);
  PicHeightInMinPUs = PicHeightInCtbsY << (Log2CtbSizeY - Log2MinPUSize);
  Log2MinIpcmCbSizeY = log2_min_pcm_luma_coding_block_size;
  Log2MaxIpcmCbSizeY = (log2_min_pcm_luma_coding_block_size +
                        log2_diff_max_min_pcm_luma_coding_block_size);
  // the following are not in the standard
  PicWidthInTbsY  = PicWidthInCtbsY  << (Log2CtbSizeY - Log2MinTrafoSize);
  PicHeightInTbsY = PicHeightInCtbsY << (Log2CtbSizeY - Log2MinTrafoSize);
  PicSizeInTbsY = PicWidthInTbsY * PicHeightInTbsY;
  sps_read = true;
#endif

  return DE265_OK;
}


int seq_parameter_set::x_pixel_to_x_ctb(int x) const
{
  int ctbx = x >> Log2CtbSizeY;
  if (ctbx >= PicWidthInCtbsY)  ctbx = PicWidthInCtbsY-1;
  if (ctbx < 0) ctbx=0;
  return ctbx;
}


int seq_parameter_set::y_pixel_to_y_ctb(int y) const
{
  int ctby = y >> Log2CtbSizeY;
  if (ctby >= PicHeightInCtbsY) ctby = PicHeightInCtbsY-1;
  if (ctby < 0) ctby=0;
  return ctby;
}


de265_error sps_range_extension::read(error_queue* errqueue, bitreader* br)
{
  transform_skip_rotation_enabled_flag    = get_bits(br,1);
  transform_skip_context_enabled_flag     = get_bits(br,1);
  implicit_rdpcm_enabled_flag             = get_bits(br,1);
  explicit_rdpcm_enabled_flag             = get_bits(br,1);
  extended_precision_processing_flag      = get_bits(br,1);
  intra_smoothing_disabled_flag           = get_bits(br,1);
  high_precision_offsets_enabled_flag     = get_bits(br,1);
  persistent_rice_adaptation_enabled_flag = get_bits(br,1);
  cabac_bypass_alignment_enabled_flag     = get_bits(br,1);

  return DE265_OK;
}


std::string sps_range_extension::dump() const
{
  std::stringstream sstr;

  LOG("----------------- SPS-range-extension -----------------\n");
  LOG("transform_skip_rotation_enabled_flag    : %d\n", transform_skip_rotation_enabled_flag);
  LOG("transform_skip_context_enabled_flag     : %d\n", transform_skip_context_enabled_flag);
  LOG("implicit_rdpcm_enabled_flag             : %d\n", implicit_rdpcm_enabled_flag);
  LOG("explicit_rdpcm_enabled_flag             : %d\n", explicit_rdpcm_enabled_flag);
  LOG("extended_precision_processing_flag      : %d\n", extended_precision_processing_flag);
  LOG("intra_smoothing_disabled_flag           : %d\n", intra_smoothing_disabled_flag);
  LOG("high_precision_offsets_enabled_flag     : %d\n", high_precision_offsets_enabled_flag);
  LOG("persistent_rice_adaptation_enabled_flag : %d\n", persistent_rice_adaptation_enabled_flag);
  LOG("cabac_bypass_alignment_enabled_flag     : %d\n", cabac_bypass_alignment_enabled_flag);

  return sstr.str();
}
