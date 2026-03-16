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

#ifndef DE265_PPS_H
#define DE265_PPS_H

#include "libde265/bitstream.h"
#include "libde265/sps.h" // for scaling list only

#include <vector>
#include <memory>

constexpr int DE265_MAX_TILE_COLUMNS = 10;
constexpr int DE265_MAX_TILE_ROWS    = 10;

class decoder_context;
class pic_parameter_set;


class pps_range_extension
{
 public:
  pps_range_extension() { reset(); }

  void reset();

  bool read(bitreader*, decoder_context*, const pic_parameter_set*);
  void dump(int fd) const;

  uint8_t log2_max_transform_skip_block_size;
  bool    cross_component_prediction_enabled_flag;
  bool    chroma_qp_offset_list_enabled_flag;
  uint8_t diff_cu_chroma_qp_offset_depth;
  uint8_t chroma_qp_offset_list_len;
  int8_t  cb_qp_offset_list[6];
  int8_t  cr_qp_offset_list[6];
  uint8_t log2_sao_offset_scale_luma;
  uint8_t log2_sao_offset_scale_chroma;
};


class pic_parameter_set {
public:
  pic_parameter_set();
  ~pic_parameter_set();

  void reset() { set_defaults(); }
  bool read(bitreader*, decoder_context*);
  bool write(error_queue*, CABAC_encoder&,
             const seq_parameter_set* sps);

  bool is_tile_start_CTB(int ctbX,int ctbY) const;
  void dump(int fd) const;


  void set_defaults(enum PresetSet = Preset_Default);

  bool pps_read; // whether this pps has been read from bitstream
  std::shared_ptr<const seq_parameter_set> sps;


  uint8_t pic_parameter_set_id;
  uint8_t seq_parameter_set_id;
  bool dependent_slice_segments_enabled_flag;
  bool sign_data_hiding_flag;
  bool cabac_init_present_flag;
  uint8_t num_ref_idx_l0_default_active; // [1;16]
  uint8_t num_ref_idx_l1_default_active; // [1;16]

  int pic_init_qp;
  bool constrained_intra_pred_flag;
  bool transform_skip_enabled_flag;

  // --- QP ---

  bool cu_qp_delta_enabled_flag;
  uint8_t diff_cu_qp_delta_depth;   // [ 0 ; log2_diff_max_min_luma_coding_block_size ]

  int  pic_cb_qp_offset;
  int  pic_cr_qp_offset;
  bool pps_slice_chroma_qp_offsets_present_flag;


  bool weighted_pred_flag;
  bool weighted_bipred_flag;
  bool output_flag_present_flag;
  bool transquant_bypass_enable_flag;
  bool entropy_coding_sync_enabled_flag;


  // --- tiles ---

  bool tiles_enabled_flag;
  uint8_t num_tile_columns;  // [1;PicWidthInCtbsY] max DE265_MAX_TILE_COLUMNS
  uint8_t num_tile_rows;     // [1;PicHeightInCtbsY] max DE265_MAX_TILE_ROWS
  bool uniform_spacing_flag;


  // --- ---

  bool loop_filter_across_tiles_enabled_flag;
  bool pps_loop_filter_across_slices_enabled_flag;
  bool deblocking_filter_control_present_flag;

  bool deblocking_filter_override_enabled_flag;
  bool pic_disable_deblocking_filter_flag;

  int8_t beta_offset;  // [-12;12]
  int8_t tc_offset;    // [-12;12]

  bool pic_scaling_list_data_present_flag;
  struct scaling_list_data scaling_list; // contains valid data if sps->scaling_list_enabled_flag set

  bool lists_modification_present_flag;
  uint8_t log2_parallel_merge_level; // [2 ; log2(max CB size)]
  uint8_t num_extra_slice_header_bits;
  bool slice_segment_header_extension_present_flag;
  bool pps_extension_flag;
  bool pps_range_extension_flag;
  bool pps_multilayer_extension_flag;
  uint8_t pps_extension_6bits;

  pps_range_extension range_extension;


  // --- derived values ---

  int Log2MinCuQpDeltaSize;
  int Log2MinCuChromaQpOffsetSize;
  int Log2MaxTransformSkipSize;

  int colWidth [ DE265_MAX_TILE_COLUMNS ];
  int rowHeight[ DE265_MAX_TILE_ROWS ];
  int colBd    [ DE265_MAX_TILE_COLUMNS+1 ];
  int rowBd    [ DE265_MAX_TILE_ROWS+1 ];

  std::vector<uint32_t> CtbAddrRStoTS; // #CTBs
  std::vector<uint32_t> CtbAddrTStoRS; // #CTBs
  std::vector<uint32_t> TileId;        // #CTBs  // index in tile-scan order
  std::vector<uint32_t> TileIdRS;      // #CTBs  // index in raster-scan order
  std::vector<uint32_t> MinTbAddrZS;   // #TBs   [x + y*PicWidthInTbsY]

  void set_derived_values(const seq_parameter_set* sps);
};

#endif
