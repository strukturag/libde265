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

#ifndef DE265_SPS_H
#define DE265_SPS_H

#include "libde265/vps.h"
#include "libde265/vui.h"
#include "libde265/bitstream.h"
#include "libde265/refpic.h"
#include "libde265/de265.h"
#include "libde265/cabac.h"

#include <vector>
#include <sstream>

class error_queue;


constexpr int MAX_NUM_LT_REF_PICS_SPS = 32;

// Maximum supported picture size.
// This is just a safety limit to prevent malicious streams from allocating excessive amounts of memory.
constexpr int MAX_PICTURE_WIDTH = 70000;
constexpr int MAX_PICTURE_HEIGHT = 70000;


class scaling_list_data
{
 public:
  de265_error read(bitreader*, const seq_parameter_set*, bool inPPS);
  de265_error write(CABAC_encoder& out, const seq_parameter_set* sps, bool inPPS);

  void set_default_scaling_lists();


  // structure size: approx. 4 kB

  uint8_t ScalingFactor_Size0[6][4][4];
  uint8_t ScalingFactor_Size1[6][8][8];
  uint8_t ScalingFactor_Size2[6][16][16];
  uint8_t ScalingFactor_Size3[2][32][32];
};


enum PresetSet {
  Preset_Default
};


class sps_range_extension
{
 public:
  sps_range_extension();

  uint8_t transform_skip_rotation_enabled_flag;
  uint8_t transform_skip_context_enabled_flag;
  uint8_t implicit_rdpcm_enabled_flag;
  uint8_t explicit_rdpcm_enabled_flag;
  uint8_t extended_precision_processing_flag;
  uint8_t intra_smoothing_disabled_flag;
  uint8_t high_precision_offsets_enabled_flag;
  uint8_t persistent_rice_adaptation_enabled_flag;
  uint8_t cabac_bypass_alignment_enabled_flag;

  de265_error read(error_queue*, bitreader*);
  std::string dump() const;
};


class seq_parameter_set {
public:
  seq_parameter_set();
  ~seq_parameter_set();

  de265_error read(error_queue*, bitreader*);
  de265_error write(error_queue*, CABAC_encoder&);

  std::string dump() const;

  // Check whether header parameters are consistent.
  bool check_parameters_for_consistency() const;

  void set_defaults(enum PresetSet = Preset_Default);
  void set_CB_log2size_range(int mini,int maxi);
  void set_TB_log2size_range(int mini,int maxi);
  void set_resolution(int w,int h);


  // --- chroma

  // This is the user relevant chroma type. Usually, this is equivalent to
  // the internal ChromaArrayType, except that ChromaArrayType is set to 'mono'
  // when separate_coulor_plane is activated.
  de265_chroma get_chroma() const { return chroma_format_idc; }

  int get_chroma_horizontal_subsampling() const {
    return (chroma_format_idc==de265_chroma_420 ||
            chroma_format_idc==de265_chroma_422) ? 2 : 1;
  }
  int get_chroma_vertical_subsampling() const {
    return (chroma_format_idc==de265_chroma_420) ? 2 : 1;
  }

  // Shift value to convert between luma and chroma coordinates.
  int get_chroma_shift_W(int cIdx) const { return cIdx ? SubWidthC -1 : 0; }
  int get_chroma_shift_H(int cIdx) const { return cIdx ? SubHeightC-1 : 0; }


  void set_chroma(de265_chroma);

  // Flag for coding images with separate colour planes (only allowed in conjunction with chroma 4:4:4)
  void set_separate_colour_planes(bool flag);




  // -----------------------------------

  // TODO: replace with 'sps_valid'
  bool sps_read; // whether the sps has been read from the bitstream


  char video_parameter_set_id;
  char sps_max_sub_layers;            // [1;7]
  char sps_temporal_id_nesting_flag;

  profile_tier_level profile_tier_level_;

  int seq_parameter_set_id;


  // --- chroma

  // Logical chroma format from bitstream. Differs from ChromaArrayType only for
  // separate_colour_plane_flag=true.
  de265_chroma chroma_format_idc;

  // If set, 4:4:4 images are coded as 3 separate colour planes. May not be used
  // for other chroma formats.
  bool separate_colour_plane_flag;

  // The internal chroma type. Should not be used by users of the library.
  de265_chroma ChromaArrayType;

  // Pixel subsampling factors
  int SubWidthC, SubHeightC;


  int  pic_width_in_luma_samples;
  int  pic_height_in_luma_samples;
  char conformance_window_flag;

  int conf_win_left_offset;
  int conf_win_right_offset;
  int conf_win_top_offset;
  int conf_win_bottom_offset;

  int bit_depth_luma;
  int bit_depth_chroma;

  int  log2_max_pic_order_cnt_lsb;
  char sps_sub_layer_ordering_info_present_flag;

  int sps_max_dec_pic_buffering[7]; // for each temporal layer
  int sps_max_num_reorder_pics[7];
  int sps_max_latency_increase_plus1[7];

  int  log2_min_luma_coding_block_size;             // smallest CB size [3;6]
  int  log2_diff_max_min_luma_coding_block_size;    // largest  CB size
  int  log2_min_transform_block_size;               // smallest TB size [2;5]
  int  log2_diff_max_min_transform_block_size;      // largest  TB size
  int  max_transform_hierarchy_depth_inter;
  int  max_transform_hierarchy_depth_intra;

  char scaling_list_enable_flag;
  char sps_scaling_list_data_present_flag; /* if not set, the default scaling lists will be set
                                              in scaling_list */

  scaling_list_data scaling_list;

  char amp_enabled_flag;
  char sample_adaptive_offset_enabled_flag;
  char pcm_enabled_flag;

  char pcm_sample_bit_depth_luma;
  char pcm_sample_bit_depth_chroma;
  int  log2_min_pcm_luma_coding_block_size;
  int  log2_diff_max_min_pcm_luma_coding_block_size;
  char pcm_loop_filter_disable_flag;

  int num_short_term_ref_pic_sets() const { return ref_pic_sets.size(); }
  std::vector<ref_pic_set> ref_pic_sets; // [0 ; num_short_term_ref_pic_set (<=MAX_REF_PIC_SETS) )

  char long_term_ref_pics_present_flag;

  int num_long_term_ref_pics_sps;

  int  lt_ref_pic_poc_lsb_sps[MAX_NUM_LT_REF_PICS_SPS];
  char used_by_curr_pic_lt_sps_flag[MAX_NUM_LT_REF_PICS_SPS];

  char sps_temporal_mvp_enabled_flag;
  char strong_intra_smoothing_enable_flag;

  char vui_parameters_present_flag;
  video_usability_information vui;

  char sps_extension_present_flag;
  char sps_range_extension_flag;
  char sps_multilayer_extension_flag;
  char sps_extension_6bits;

  sps_range_extension range_extension;

  /*
    if( sps_extension_flag )
    while( more_rbsp_data() )
    sps_extension_data_flag
    u(1)
    rbsp_trailing_bits()
  */


  // --- derived values ---

  de265_error compute_derived_values(bool sanitize_values = false);

  int BitDepth_Y;
  int QpBdOffset_Y;
  int BitDepth_C;
  int QpBdOffset_C;

  int MaxPicOrderCntLsb;

  int Log2MinCbSizeY;
  int Log2CtbSizeY;
  int MinCbSizeY;
  int CtbSizeY;
  int PicWidthInMinCbsY;
  int PicWidthInCtbsY;
  int PicHeightInMinCbsY;
  int PicHeightInCtbsY;
  int PicSizeInMinCbsY;
  int PicSizeInCtbsY;
  int PicSizeInSamplesY;

  int CtbWidthC, CtbHeightC;

  int PicWidthInTbsY; // not in standard
  int PicHeightInTbsY; // not in standard
  int PicSizeInTbsY; // not in standard

  int Log2MinTrafoSize;
  int Log2MaxTrafoSize;

  int Log2MinPUSize;
  int PicWidthInMinPUs;  // might be rounded up
  int PicHeightInMinPUs; // might be rounded up

  int Log2MinIpcmCbSizeY;
  int Log2MaxIpcmCbSizeY;

  int SpsMaxLatencyPictures[7]; // [temporal layer]

  uint8_t WpOffsetBdShiftY;
  uint8_t WpOffsetBdShiftC;
  int32_t WpOffsetHalfRangeY;
  int32_t WpOffsetHalfRangeC;


  int getPUIndexRS(int pixelX,int pixelY) const {
    return (pixelX>>Log2MinPUSize) + (pixelY>>Log2MinPUSize)*PicWidthInMinPUs;
  }

  int get_bit_depth(int cIdx) const {
    if (cIdx==0) return BitDepth_Y;
    else         return BitDepth_C;
  }


  // Convert pixel position to CTB index
  // Pixel positions outside the image are clipped to valid CTB positions in the image.
  int x_pixel_to_x_ctb(int x) const;
  int y_pixel_to_y_ctb(int y) const;

  // ------------------ setters ------------------

  void set_CB_size_range(int minSize, int maxSize);
  void set_TB_size_range(int minSize, int maxSize);
  void set_PCM_size_range(int minSize, int maxSize);

 private:

  // Compute ChromaArrayType and SubWidthC/SubHeightC.
  // Returns error when combination of chroma format and separate_colour_plane is invalid.
  de265_error derive_chroma_parameters();
};

#endif
