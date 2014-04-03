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

#define DE265_MAX_TILE_COLUMNS 10
#define DE265_MAX_TILE_ROWS    10


typedef struct {
  bool pps_read; // whether this pps has been read from bitstream

  char pic_parameter_set_id;
  char seq_parameter_set_id;
  char dependent_slice_segments_enabled_flag;
  char sign_data_hiding_flag;
  char cabac_init_present_flag;
  char num_ref_idx_l0_default_active;
  char num_ref_idx_l1_default_active;

  int pic_init_qp;
  char constrained_intra_pred_flag;
  char transform_skip_enabled_flag;

  // if ( cu_qp_delta_enabled_flag )
  char cu_qp_delta_enabled_flag;
  int  diff_cu_qp_delta_depth;

  int  pic_cb_qp_offset;
  int  pic_cr_qp_offset;
  char pps_slice_chroma_qp_offsets_present_flag;
  char weighted_pred_flag;
  char weighted_bipred_flag;
  char output_flag_present_flag;
  char transquant_bypass_enable_flag;
  char entropy_coding_sync_enabled_flag;


  // --- tiles ---

  //if( tiles_enabled_flag ) {
  char tiles_enabled_flag;
  int  num_tile_columns;
  int  num_tile_rows;
  char uniform_spacing_flag;

  // derived values
  int colWidth [ DE265_MAX_TILE_COLUMNS ];
  int rowHeight[ DE265_MAX_TILE_ROWS ];
  int colBd    [ DE265_MAX_TILE_COLUMNS+1 ];
  int rowBd    [ DE265_MAX_TILE_ROWS+1 ];

  int* CtbAddrRStoTS; // #CTBs
  int* CtbAddrTStoRS; // #CTBs
  int* TileId;        // #CTBs
  int* TileIdRS;      // #CTBs
  int* MinTbAddrZS;   // #TBs   [x + y*PicWidthInTbsY]


  // --- QP ---

  int Log2MinCuQpDeltaSize;

  // --- ---

  char loop_filter_across_tiles_enabled_flag;

  char pps_loop_filter_across_slices_enabled_flag;
  char deblocking_filter_control_present_flag;

  char deblocking_filter_override_enabled_flag;
  char pic_disable_deblocking_filter_flag;

  int beta_offset;
  int tc_offset;

  char pic_scaling_list_data_present_flag;
  struct scaling_list_data scaling_list; // contains valid data if sps->scaling_list_enabled_flag set

  char lists_modification_present_flag;
  int log2_parallel_merge_level;
  char num_extra_slice_header_bits;
  char slice_segment_header_extension_present_flag;
  char pps_extension_flag;

} pic_parameter_set;

#endif
