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

#include "pps.h"
#include "decctx.h"
#include "util.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#else
# include <alloca.h>
#endif


pic_parameter_set::pic_parameter_set()
{
  reset();
}


pic_parameter_set::~pic_parameter_set()
{
}


void pic_parameter_set::set_defaults(enum PresetSet)
{
  pps_read = false;

  pic_parameter_set_id = 0;
  seq_parameter_set_id = 0;
  dependent_slice_segments_enabled_flag = 0;
  sign_data_hiding_flag = 0;
  cabac_init_present_flag = 0;
  num_ref_idx_l0_default_active = 1;
  num_ref_idx_l1_default_active = 1;

  pic_init_qp = 27;
  constrained_intra_pred_flag = 0;
  transform_skip_enabled_flag = 0;

  cu_qp_delta_enabled_flag = 0;
  diff_cu_qp_delta_depth = 0;

  pic_cb_qp_offset = 0;
  pic_cr_qp_offset = 0;
  pps_slice_chroma_qp_offsets_present_flag = 0;
  weighted_pred_flag  = 0;
  weighted_bipred_flag= 0;
  output_flag_present_flag = 0;
  transquant_bypass_enable_flag = 0;
  entropy_coding_sync_enabled_flag = 0;

  // --- tiles ---

  tiles_enabled_flag = 0;
  num_tile_columns = 1;
  num_tile_rows    = 1;
  uniform_spacing_flag = 1;


  // --- ---

  loop_filter_across_tiles_enabled_flag = 1;
  pps_loop_filter_across_slices_enabled_flag = 1;

  for (int i=0;i<DE265_MAX_TILE_COLUMNS;i++) { colWidth[i]=0; }
  for (int i=0;i<DE265_MAX_TILE_ROWS;i++)    { rowHeight[i]=0; }
  for (int i=0;i<=DE265_MAX_TILE_COLUMNS;i++) { colBd[i]=0; }
  for (int i=0;i<=DE265_MAX_TILE_ROWS;i++)    { rowBd[i]=0; }

  CtbAddrRStoTS.clear();
  CtbAddrTStoRS.clear();
  TileId.clear();
  TileIdRS.clear();
  MinTbAddrZS.clear();


  Log2MinCuQpDeltaSize = 0;

  deblocking_filter_control_present_flag = 0;
  deblocking_filter_override_enabled_flag = 0;
  pic_disable_deblocking_filter_flag = 0;

  beta_offset = 0;
  tc_offset   = 0;

  pic_scaling_list_data_present_flag = 0;
  // TODO struct scaling_list_data scaling_list;

  lists_modification_present_flag = 0;
  log2_parallel_merge_level = 2;

  num_extra_slice_header_bits = 0;
  slice_segment_header_extension_present_flag = 0;

  pps_extension_present_flag = false;
  pps_range_extension_flag = false;
  pps_multilayer_extension_flag = false;
}


bool pic_parameter_set::read(bitreader* br, decoder_context* ctx)
{
  reset();


  int uvlc;
  pic_parameter_set_id = uvlc = get_uvlc(br);
  if (uvlc >= DE265_MAX_PPS_SETS ||
      uvlc == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_PPS_REFERENCED, false);
    return false;
  }

  seq_parameter_set_id = uvlc = get_uvlc(br);
  if (uvlc >= DE265_MAX_PPS_SETS ||
      uvlc == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_SPS_REFERENCED, false);
    return false;
  }

  dependent_slice_segments_enabled_flag = get_bits(br,1);
  output_flag_present_flag = get_bits(br,1);
  num_extra_slice_header_bits = get_bits(br,3);
  sign_data_hiding_flag = get_bits(br,1);
  cabac_init_present_flag = get_bits(br,1);
  num_ref_idx_l0_default_active = uvlc = get_uvlc(br);
  if (uvlc == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  num_ref_idx_l0_default_active++;

  num_ref_idx_l1_default_active = uvlc = get_uvlc(br);
  if (uvlc == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  num_ref_idx_l1_default_active++;


  seq_parameter_set* sps = ctx->get_sps(seq_parameter_set_id);
  if (sps->sps_read==false) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_SPS_REFERENCED, false);
    return false;
  }

  if ((pic_init_qp = get_svlc(br)) == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  pic_init_qp += 26;

  constrained_intra_pred_flag = get_bits(br,1);
  transform_skip_enabled_flag = get_bits(br,1);
  cu_qp_delta_enabled_flag = get_bits(br,1);

  if (cu_qp_delta_enabled_flag) {
    if ((diff_cu_qp_delta_depth = get_uvlc(br)) == UVLC_ERROR) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
  } else {
    diff_cu_qp_delta_depth = 0;
  }

  if ((pic_cb_qp_offset = get_svlc(br)) == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  if ((pic_cr_qp_offset = get_svlc(br)) == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  pps_slice_chroma_qp_offsets_present_flag = get_bits(br,1);
  weighted_pred_flag = get_bits(br,1);
  weighted_bipred_flag = get_bits(br,1);
  transquant_bypass_enable_flag = get_bits(br,1);
  tiles_enabled_flag = get_bits(br,1);
  entropy_coding_sync_enabled_flag = get_bits(br,1);


  // --- tiles ---

  if (tiles_enabled_flag) {
    num_tile_columns = get_uvlc(br);
    if (num_tile_columns == UVLC_ERROR ||
	num_tile_columns+1 > DE265_MAX_TILE_COLUMNS) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    num_tile_columns++;

    num_tile_rows = get_uvlc(br);
    if (num_tile_rows == UVLC_ERROR ||
	num_tile_rows+1 > DE265_MAX_TILE_ROWS) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    num_tile_rows++;

    uniform_spacing_flag = get_bits(br,1);

    if (uniform_spacing_flag==false) {
      int lastColumnWidth = sps->PicWidthInCtbsY;
      int lastRowHeight   = sps->PicHeightInCtbsY;

      for (int i=0; i<num_tile_columns-1; i++)
        {
          colWidth[i] = get_uvlc(br);
          if (colWidth[i] == UVLC_ERROR) {
	    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
	    return false;
	  }
          colWidth[i]++;

          lastColumnWidth -= colWidth[i];
        }

      if (lastColumnWidth <= 0) {
        return false;
      }

      colWidth[num_tile_columns-1] = lastColumnWidth;

      for (int i=0; i<num_tile_rows-1; i++)
        {
          rowHeight[i] = get_uvlc(br);
          if (rowHeight[i] == UVLC_ERROR) {
	    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
	    return false;
	  }
          rowHeight[i]++;
          lastRowHeight -= rowHeight[i];
        }

      if (lastRowHeight <= 0) {
        return false;
      }


      rowHeight[num_tile_rows-1] = lastRowHeight;
    }

    loop_filter_across_tiles_enabled_flag = get_bits(br,1);

  } else {
    num_tile_columns = 1;
    num_tile_rows    = 1;
    uniform_spacing_flag = 1;
    loop_filter_across_tiles_enabled_flag = 0;
  }



  // END tiles



  beta_offset = 0; // default value
  tc_offset   = 0; // default value

  pps_loop_filter_across_slices_enabled_flag = get_bits(br,1);
  deblocking_filter_control_present_flag = get_bits(br,1);
  if (deblocking_filter_control_present_flag) {
    deblocking_filter_override_enabled_flag = get_bits(br,1);
    pic_disable_deblocking_filter_flag = get_bits(br,1);
    if (!pic_disable_deblocking_filter_flag) {
      beta_offset = get_svlc(br);
      if (beta_offset == UVLC_ERROR) {
	ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
	return false;
      }
      beta_offset *= 2;

      tc_offset   = get_svlc(br);
      if (tc_offset == UVLC_ERROR) {
	ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
	return false;
      }
      tc_offset   *= 2;
    }
  }
  else {
    deblocking_filter_override_enabled_flag = 0;
    pic_disable_deblocking_filter_flag = 0;
  }


  // --- scaling list ---

  pic_scaling_list_data_present_flag = get_bits(br,1);

  // check consistency: if scaling-lists are not enabled, pic_scalign_list_data_present_flag
  // must be FALSE
  if (sps->scaling_list_enable_flag==0 &&
      pic_scaling_list_data_present_flag != 0) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  if (pic_scaling_list_data_present_flag) {
    de265_error err = read_scaling_list(br, sps, &scaling_list, true);
    if (err != DE265_OK) {
      ctx->add_warning(err, false);
      return false;
    }
  }
  else {
    memcpy(&scaling_list, &sps->scaling_list, sizeof(scaling_list_data));
  }




  lists_modification_present_flag = get_bits(br,1);
  log2_parallel_merge_level = get_uvlc(br);
  if (log2_parallel_merge_level == UVLC_ERROR) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  log2_parallel_merge_level += 2;

  if (log2_parallel_merge_level-2 > sps->log2_min_luma_coding_block_size-3 +1 +
      sps->log2_diff_max_min_luma_coding_block_size) {
    return false;
  }

  slice_segment_header_extension_present_flag = get_bits(br,1);
  pps_extension_present_flag = get_bits(br,1);

  if (pps_extension_present_flag) {
    pps_range_extension_flag = get_bits(br,1);
    pps_multilayer_extension_flag = get_bits(br,1);
    pps_extension_6bits = get_bits(br,6);
  }
  else {
    pps_range_extension_flag = false;
    pps_multilayer_extension_flag = false;
    pps_extension_6bits = 0;
  }

  if (pps_range_extension_flag) {
    pps_range_ext.read(br, transform_skip_enabled_flag);
  }
  if (pps_multilayer_extension_flag) {
    pps_mult_ext.read(br);
  }
  if (pps_extension_6bits != 0) {
    // The remaining bits in the PPS are pps_extension_data_flag flags
  }

  set_derived_values(sps);

  pps_read = true;

  return true;
}


void pic_parameter_set::set_derived_values(const seq_parameter_set* sps)
{
  Log2MinCuQpDeltaSize = sps->Log2CtbSizeY - diff_cu_qp_delta_depth;

  if (uniform_spacing_flag) {

    // set columns widths

    int *const colPos = (int *)alloca((num_tile_columns+1) * sizeof(int));

    for (int i=0;i<=num_tile_columns;i++) {
      colPos[i] = i*sps->PicWidthInCtbsY / num_tile_columns;
    }
    for (int i=0;i<num_tile_columns;i++) {
      colWidth[i] = colPos[i+1] - colPos[i];
    }

    // set row heights

    int *const rowPos = (int *)alloca((num_tile_rows+1) * sizeof(int));

    for (int i=0;i<=num_tile_rows;i++) {
      rowPos[i] = i*sps->PicHeightInCtbsY / num_tile_rows;
    }
    for (int i=0;i<num_tile_rows;i++) {
      rowHeight[i] = rowPos[i+1] - rowPos[i];
    }
  }


  // set tile boundaries

  colBd[0]=0;
  for (int i=0;i<num_tile_columns;i++) {
    colBd[i+1] = colBd[i] + colWidth[i];
  }

  rowBd[0]=0;
  for (int i=0;i<num_tile_rows;i++) {
    rowBd[i+1] = rowBd[i] + rowHeight[i];
  }



  // alloc raster scan arrays

  CtbAddrRStoTS.resize(sps->PicSizeInCtbsY);
  CtbAddrTStoRS.resize(sps->PicSizeInCtbsY);
  TileId       .resize(sps->PicSizeInCtbsY);
  TileIdRS     .resize(sps->PicSizeInCtbsY);
  MinTbAddrZS  .resize(sps->PicSizeInTbsY );


  // raster scan (RS) <-> tile scan (TS) conversion

  for (int ctbAddrRS=0 ; ctbAddrRS < sps->PicSizeInCtbsY ; ctbAddrRS++)
    {
      int tbX = ctbAddrRS % sps->PicWidthInCtbsY;
      int tbY = ctbAddrRS / sps->PicWidthInCtbsY;
      int tileX=-1,tileY=-1;

      for (int i=0;i<num_tile_columns;i++)
        if (tbX >= colBd[i])
          tileX=i;

      for (int j=0;j<num_tile_rows;j++)
        if (tbY >= rowBd[j])
          tileY=j;

      CtbAddrRStoTS[ctbAddrRS] = 0;
      for (int i=0;i<tileX;i++)
        CtbAddrRStoTS[ctbAddrRS] += rowHeight[tileY]*colWidth[i];

      for (int j=0;j<tileY;j++)
        {
          //pps->CtbAddrRStoTS[ctbAddrRS] += (tbY - pps->rowBd[tileY])*pps->colWidth[tileX];
          //pps->CtbAddrRStoTS[ctbAddrRS] += tbX - pps->colBd[tileX];

          CtbAddrRStoTS[ctbAddrRS] += sps->PicWidthInCtbsY * rowHeight[j];
        }

      assert(tileX>=0 && tileY>=0);

      CtbAddrRStoTS[ctbAddrRS] += (tbY-rowBd[tileY])*colWidth[tileX];
      CtbAddrRStoTS[ctbAddrRS] +=  tbX - colBd[tileX];


      // inverse mapping

      CtbAddrTStoRS[ CtbAddrRStoTS[ctbAddrRS] ] = ctbAddrRS;
    }


#if 0
  logtrace(LogHeaders,"6.5.1 CtbAddrRSToTS\n");
  for (int y=0;y<sps->PicHeightInCtbsY;y++)
    {
      for (int x=0;x<sps->PicWidthInCtbsY;x++)
        {
          logtrace(LogHeaders,"%3d ", CtbAddrRStoTS[x + y*sps->PicWidthInCtbsY]);
        }

      logtrace(LogHeaders,"\n");
    }
#endif

  // tile id

  for (int j=0, tIdx=0 ; j<num_tile_rows ; j++)
    for (int i=0 ; i<num_tile_columns;i++)
      {
        for (int y=rowBd[j] ; y<rowBd[j+1] ; y++)
          for (int x=colBd[i] ; x<colBd[i+1] ; x++) {
            TileId  [ CtbAddrRStoTS[y*sps->PicWidthInCtbsY + x] ] = tIdx;
            TileIdRS[ y*sps->PicWidthInCtbsY + x ] = tIdx;

            //logtrace(LogHeaders,"tileID[%d,%d] = %d\n",x,y,pps->TileIdRS[ y*sps->PicWidthInCtbsY + x ]);
          }

        tIdx++;
      }

#if 0
  logtrace(LogHeaders,"Tile IDs RS:\n");
  for (int y=0;y<sps->PicHeightInCtbsY;y++) {
    for (int x=0;x<sps->PicWidthInCtbsY;x++) {
      logtrace(LogHeaders,"%2d ",TileIdRS[y*sps->PicWidthInCtbsY+x]);
    }
    logtrace(LogHeaders,"\n");
  }
#endif

  // 6.5.2 Z-scan order array initialization process

  for (int y=0;y<sps->PicHeightInTbsY;y++)
    for (int x=0;x<sps->PicWidthInTbsY;x++)
      {
        int tbX = (x<<sps->Log2MinTrafoSize)>>sps->Log2CtbSizeY;
        int tbY = (y<<sps->Log2MinTrafoSize)>>sps->Log2CtbSizeY;
        int ctbAddrRS = sps->PicWidthInCtbsY*tbY + tbX;

        MinTbAddrZS[x + y*sps->PicWidthInTbsY] = CtbAddrRStoTS[ctbAddrRS]
          << ((sps->Log2CtbSizeY-sps->Log2MinTrafoSize)*2);

        int p=0;
        for (int i=0 ; i<(sps->Log2CtbSizeY - sps->Log2MinTrafoSize) ; i++) {
          int m=1<<i;
          p += (m & x ? m*m : 0) + (m & y ? 2*m*m : 0);
        }

        MinTbAddrZS[x + y*sps->PicWidthInTbsY] += p;
      }


  // --- debug logging ---

  /*
    logtrace(LogHeaders,"6.5.2 Z-scan order array\n");
    for (int y=0;y<sps->PicHeightInTbsY;y++)
    {
    for (int x=0;x<sps->PicWidthInTbsY;x++)
    {
    logtrace(LogHeaders,"%4d ", pps->MinTbAddrZS[x + y*sps->PicWidthInTbsY]);
    }

    logtrace(LogHeaders,"\n");
    }

    for (int i=0;i<sps->PicSizeInTbsY;i++)
    {
    for (int y=0;y<sps->PicHeightInTbsY;y++)
    {
    for (int x=0;x<sps->PicWidthInTbsY;x++)
    {
    if (pps->MinTbAddrZS[x + y*sps->PicWidthInTbsY] == i) {
    logtrace(LogHeaders,"%d %d\n",x,y);
    }
    }
    }
    }
  */
}


bool pic_parameter_set::write(error_queue* errqueue, CABAC_encoder& out,
                              const seq_parameter_set* sps)
{
  if (pic_parameter_set_id >= DE265_MAX_PPS_SETS) {
    errqueue->add_warning(DE265_WARNING_NONEXISTING_PPS_REFERENCED, false);
    return false;
  }
  out.write_uvlc(pic_parameter_set_id);

  if (seq_parameter_set_id >= DE265_MAX_PPS_SETS) {
    errqueue->add_warning(DE265_WARNING_NONEXISTING_SPS_REFERENCED, false);
    return false;
  }
  out.write_uvlc(seq_parameter_set_id);

  out.write_bit(dependent_slice_segments_enabled_flag);
  out.write_bit(output_flag_present_flag);
  out.write_bits(num_extra_slice_header_bits,3);
  out.write_bit(sign_data_hiding_flag);
  out.write_bit(cabac_init_present_flag);
  out.write_uvlc(num_ref_idx_l0_default_active-1);
  out.write_uvlc(num_ref_idx_l1_default_active-1);

  out.write_svlc(pic_init_qp-26);

  out.write_bit(constrained_intra_pred_flag);
  out.write_bit(transform_skip_enabled_flag);
  out.write_bit(cu_qp_delta_enabled_flag);

  if (cu_qp_delta_enabled_flag) {
    out.write_uvlc(diff_cu_qp_delta_depth);
  }

  out.write_svlc(pic_cb_qp_offset);
  out.write_svlc(pic_cr_qp_offset);

  out.write_bit(pps_slice_chroma_qp_offsets_present_flag);
  out.write_bit(weighted_pred_flag);
  out.write_bit(weighted_bipred_flag);
  out.write_bit(transquant_bypass_enable_flag);
  out.write_bit(tiles_enabled_flag);
  out.write_bit(entropy_coding_sync_enabled_flag);


  // --- tiles ---

  if (tiles_enabled_flag) {
    if (num_tile_columns > DE265_MAX_TILE_COLUMNS) {
      errqueue->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    out.write_uvlc(num_tile_columns-1);

    if (num_tile_rows > DE265_MAX_TILE_ROWS) {
      errqueue->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    out.write_uvlc(num_tile_rows-1);

    out.write_bit(uniform_spacing_flag);

    if (uniform_spacing_flag==false) {
      for (int i=0; i<num_tile_columns-1; i++)
        {
          out.write_uvlc(colWidth[i]-1);
        }

      for (int i=0; i<num_tile_rows-1; i++)
        {
          out.write_uvlc(rowHeight[i]-1);
        }
    }

    out.write_bit(loop_filter_across_tiles_enabled_flag);
  }


  out.write_bit(pps_loop_filter_across_slices_enabled_flag);
  out.write_bit(deblocking_filter_control_present_flag);

  if (deblocking_filter_control_present_flag) {
    out.write_bit(deblocking_filter_override_enabled_flag);
    out.write_bit(pic_disable_deblocking_filter_flag);

    if (!pic_disable_deblocking_filter_flag) {
      out.write_svlc(beta_offset/2);
      out.write_svlc(tc_offset  /2);
    }
  }


  // --- scaling list ---

  out.write_bit(pic_scaling_list_data_present_flag);

  // check consistency: if scaling-lists are not enabled, pic_scalign_list_data_present_flag
  // must be FALSE
  if (sps->scaling_list_enable_flag==0 &&
      pic_scaling_list_data_present_flag != 0) {
    errqueue->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  if (pic_scaling_list_data_present_flag) {
    de265_error err = write_scaling_list(out, sps, &scaling_list, true);
    if (err != DE265_OK) {
      errqueue->add_warning(err, false);
      return false;
    }
  }



  out.write_bit(lists_modification_present_flag);
  out.write_uvlc(log2_parallel_merge_level-2);

  out.write_bit(slice_segment_header_extension_present_flag);
  out.write_bit(pps_extension_present_flag);

  if (pps_extension_present_flag) {
    //assert(false);
    /*
      while( more_rbsp_data() )

      pps_extension_data_flag
      u(1)
      rbsp_trailing_bits()

      }
    */
  }


  pps_read = true;

  return true;
}


void pic_parameter_set::dump(int fd) const
{
  FILE* fh;
  if (fd==1) fh=stdout;
  else if (fd==2) fh=stderr;
  else { return; }

#define LOG0(t) log2fh(fh, t)
#define LOG1(t,d) log2fh(fh, t,d)

  LOG0("----------------- PPS -----------------\n");
  LOG1("pic_parameter_set_id       : %d\n", pic_parameter_set_id);
  LOG1("seq_parameter_set_id       : %d\n", seq_parameter_set_id);
  LOG1("dependent_slice_segments_enabled_flag : %d\n", dependent_slice_segments_enabled_flag);
  LOG1("sign_data_hiding_flag      : %d\n", sign_data_hiding_flag);
  LOG1("cabac_init_present_flag    : %d\n", cabac_init_present_flag);
  LOG1("num_ref_idx_l0_default_active : %d\n", num_ref_idx_l0_default_active);
  LOG1("num_ref_idx_l1_default_active : %d\n", num_ref_idx_l1_default_active);

  LOG1("pic_init_qp                : %d\n", pic_init_qp);
  LOG1("constrained_intra_pred_flag: %d\n", constrained_intra_pred_flag);
  LOG1("transform_skip_enabled_flag: %d\n", transform_skip_enabled_flag);
  LOG1("cu_qp_delta_enabled_flag   : %d\n", cu_qp_delta_enabled_flag);

  if (cu_qp_delta_enabled_flag) {
    LOG1("diff_cu_qp_delta_depth     : %d\n", diff_cu_qp_delta_depth);
  }

  LOG1("pic_cb_qp_offset             : %d\n", pic_cb_qp_offset);
  LOG1("pic_cr_qp_offset             : %d\n", pic_cr_qp_offset);
  LOG1("pps_slice_chroma_qp_offsets_present_flag : %d\n", pps_slice_chroma_qp_offsets_present_flag);
  LOG1("weighted_pred_flag           : %d\n", weighted_pred_flag);
  LOG1("weighted_bipred_flag         : %d\n", weighted_bipred_flag);
  LOG1("output_flag_present_flag     : %d\n", output_flag_present_flag);
  LOG1("transquant_bypass_enable_flag: %d\n", transquant_bypass_enable_flag);
  LOG1("tiles_enabled_flag           : %d\n", tiles_enabled_flag);
  LOG1("entropy_coding_sync_enabled_flag: %d\n", entropy_coding_sync_enabled_flag);

  if (tiles_enabled_flag) {
    LOG1("num_tile_columns    : %d\n", num_tile_columns);
    LOG1("num_tile_rows       : %d\n", num_tile_rows);
    LOG1("uniform_spacing_flag: %d\n", uniform_spacing_flag);

    LOG0("tile column boundaries: ");
    for (int i=0;i<=num_tile_columns;i++) {
      LOG1("*%d ",colBd[i]);
    }
    LOG0("*\n");

    LOG0("tile row boundaries: ");
    for (int i=0;i<=num_tile_rows;i++) {
      LOG1("*%d ",rowBd[i]);
    }
    LOG0("*\n");

  //if( !uniform_spacing_flag ) {
  /*
            for( i = 0; i < num_tile_columns_minus1; i++ )

              column_width_minus1[i]
                ue(v)
                for( i = 0; i < num_tile_rows_minus1; i++ )

                  row_height_minus1[i]
                    ue(v)
                    }
  */

    LOG1("loop_filter_across_tiles_enabled_flag : %d\n", loop_filter_across_tiles_enabled_flag);
  }

  LOG1("pps_loop_filter_across_slices_enabled_flag: %d\n", pps_loop_filter_across_slices_enabled_flag);
  LOG1("deblocking_filter_control_present_flag: %d\n", deblocking_filter_control_present_flag);

  if (deblocking_filter_control_present_flag) {
    LOG1("deblocking_filter_override_enabled_flag: %d\n", deblocking_filter_override_enabled_flag);
    LOG1("pic_disable_deblocking_filter_flag: %d\n", pic_disable_deblocking_filter_flag);

    LOG1("beta_offset:  %d\n", beta_offset);
    LOG1("tc_offset:     %d\n", tc_offset);
  }

  LOG1("pic_scaling_list_data_present_flag: %d\n", pic_scaling_list_data_present_flag);
  if (pic_scaling_list_data_present_flag) {
    //scaling_list_data()
  }

  LOG1("lists_modification_present_flag: %d\n", lists_modification_present_flag);
  LOG1("log2_parallel_merge_level      : %d\n", log2_parallel_merge_level);
  LOG1("num_extra_slice_header_bits    : %d\n", num_extra_slice_header_bits);
  LOG1("slice_segment_header_extension_present_flag : %d\n", slice_segment_header_extension_present_flag);
  LOG1("pps_extension_flag : %d\n", pps_extension_present_flag);

  LOG1("Log2MinCuQpDeltaSize : %d\n", Log2MinCuQpDeltaSize);

#undef LOG0
#undef LOG1
}


bool pic_parameter_set::is_tile_start_CTB(int ctbX,int ctbY) const
{
  // fast check
  if (tiles_enabled_flag==0) {
    return ctbX == 0 && ctbY == 0;
  }

  for (int i=0;i<num_tile_columns;i++)
    if (colBd[i]==ctbX)
      {
        for (int k=0;k<num_tile_rows;k++)
          if (rowBd[k]==ctbY)
            {
              return true;
            }

        return false;
      }

  return false;
}

bool pic_parameter_set_range_extension::read(bitreader* reader, bool transform_skip_enabled_flag)
{
  if (transform_skip_enabled_flag) {
    log2_max_transform_skip_block_size_minus2 = get_uvlc(reader);
  }
  cross_component_prediction_enabled_flag = get_bits(reader,1);
  chroma_qp_offset_list_enabled_flag = get_bits(reader,1);
  if( chroma_qp_offset_list_enabled_flag ) {
    diff_cu_chroma_qp_offset_depth = get_uvlc(reader);
    chroma_qp_offset_list_len_minus1 = get_uvlc(reader);
    for( int i = 0; i  <=  chroma_qp_offset_list_len_minus1; i++ ) {
      cb_qp_offset_list[i] = get_svlc(reader);
      cr_qp_offset_list[i] = get_svlc(reader);
    }
  }
  log2_sao_offset_scale_luma = get_uvlc(reader);
  log2_sao_offset_scale_chroma = get_uvlc(reader);

  return true;
}

bool pps_multilayer_extension::read(bitreader* reader)
{
  poc_reset_info_present_flag = get_bits(reader,1);
  pps_infer_scaling_list_flag = get_bits(reader,1);
  if (pps_infer_scaling_list_flag) {
    pps_scaling_list_ref_layer_id = get_bits(reader,6);
  }
  num_ref_loc_offsets = get_uvlc(reader);
  for( int i = 0; i < num_ref_loc_offsets; i++ ) {
    ref_loc_offset_layer_id[ i ] = get_bits(reader,6);
    scaled_ref_layer_offset_present_flag[ i ] = get_bits(reader,1);
    if( scaled_ref_layer_offset_present_flag[ i ] ) {
      scaled_ref_layer_left_offset[ref_loc_offset_layer_id[ i ]] = get_svlc(reader);
      scaled_ref_layer_top_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
      scaled_ref_layer_right_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
      scaled_ref_layer_bottom_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
    }
    ref_region_offset_present_flag[ i ] = get_bits(reader,1);
    if( ref_region_offset_present_flag[ i ] ) {
      ref_region_left_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
      ref_region_top_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
      ref_region_right_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
      ref_region_bottom_offset[ ref_loc_offset_layer_id[ i ] ] = get_svlc(reader);
    }
    resample_phase_set_present_flag[ i ] = get_bits(reader,1);
    if( resample_phase_set_present_flag[ i ] ) {
      phase_hor_luma[ ref_loc_offset_layer_id[ i ] ] = get_uvlc(reader);
      phase_ver_luma[ ref_loc_offset_layer_id[ i ] ] = get_uvlc(reader);
      phase_hor_chroma[ ref_loc_offset_layer_id[ i ] ] = get_uvlc(reader) - 8;  // phase_hor_chroma_plus8
      phase_ver_chroma[ ref_loc_offset_layer_id[ i ] ] = get_uvlc(reader) - 8;  // phase_ver_chroma_plus8
    }
  }
  colour_mapping_enabled_flag = get_bits(reader,1);
  if (colour_mapping_enabled_flag) {
    cm_table.read(reader);
  }

  return true;
}

bool colour_mapping_table::read(bitreader* reader)
{
  num_cm_ref_layers_minus1 = get_uvlc(reader);
  for (int i = 0; i <= num_cm_ref_layers_minus1; i++) {
    cm_ref_layer_id[ i ] = get_bits(reader,6);
  }
  cm_octant_depth = get_bits(reader,2);
  cm_y_part_num_log2 = get_bits(reader,2);
  int PartNumY = 1 << cm_y_part_num_log2;
  luma_bit_depth_cm_input_minus8 = get_uvlc(reader);
  chroma_bit_depth_cm_input_minus8 = get_uvlc(reader);
  luma_bit_depth_cm_output_minus8 = get_uvlc(reader);
  chroma_bit_depth_cm_output_minus8 = get_uvlc(reader);
  cm_res_quant_bits = get_bits(reader,2);
  cm_delta_flc_bits_minus1 = get_bits(reader,2);

  BitDepthCmInputY = 8 + luma_bit_depth_cm_input_minus8;
  BitDepthCmOutputY = 8 + luma_bit_depth_cm_output_minus8;
  BitDepthCmInputC = 8 + chroma_bit_depth_cm_input_minus8;
  BitDepthCmOutputC = 8 + chroma_bit_depth_cm_output_minus8;
  CMThreshU = cm_adapt_threshold_u_delta + (1 << (BitDepthCmInputC - 1)); // (F 48)
  CMThreshV = cm_adapt_threshold_v_delta + (1 << (BitDepthCmInputC - 1)); // (F 49)

  int CMResLSBits = libde265_max( 0, ( 10 + BitDepthCmInputY - BitDepthCmOutputY - cm_res_quant_bits - ( cm_delta_flc_bits_minus1 + 1 ) ) );

  if( cm_octant_depth == 1 ) {
    cm_adapt_threshold_u_delta = get_svlc(reader);
    cm_adapt_threshold_v_delta = get_svlc(reader);
  }
  cm_octans.read(reader, 0, 0, 0, 0, 1 << cm_octant_depth, cm_octant_depth, PartNumY, CMResLSBits );

  return true;
}

bool colour_mapping_octants::read(bitreader* reader, int inpDepth, int idxY, int idxCb, int idxCr, int inpLength, int cm_octant_depth, int PartNumY, int CMResLSBits)
{
  _inpDepth = inpDepth;
  _idxY = idxY;
  _idxCb = idxCb;
  _idxCr = idxCr;
  _inpLength = inpLength;

  if( inpDepth < cm_octant_depth )
    split_octant_flag = get_bits(reader,1);
  else
    split_octant_flag = false;

  if (split_octant_flag) {
    for( int k = 0; k < 2; k++ )
      for( int m = 0; m < 2; m++ )
        for (int n = 0; n < 2; n++) {
          colour_mapping_octants new_oct;
          new_oct.read(reader, inpDepth+1, idxY + PartNumY * k * inpLength / 2,idxCb + m * inpLength / 2, idxCr + n * inpLength / 2, inpLength / 2, cm_octant_depth, PartNumY, CMResLSBits);
          sub_octans.push_back( new_oct );
        }
  }
  else {
    for( int i = 0; i < PartNumY; i++ ) {
      int idxShiftY = idxY + ( i << ( cm_octant_depth - inpDepth ) );
      for( int j = 0; j < 4; j++ ) {
        coded_res_flag[ j ] = get_bits(reader,1);
        if (coded_res_flag[j]) {
          for( int c = 0; c < 3; c++ ) {
            res_coeff_q[ j ][ c ] = get_uvlc(reader);
            int nr_bits = CMResLSBits;
            res_coeff_r[ j ][ c ] = get_bits(reader,nr_bits);
            if (res_coeff_q[j][c] || res_coeff_r[j][c]) {
              res_coeff_s[ j ][ c ] = get_bits(reader,1);
            }
          }
        }
      }
    }
  }

  // TODO: Rewrite.
  // TODO: Calculate LUT (see F.7.4.3.3.5)
  // LutY, LutCb, LutCr

  return true;
}
