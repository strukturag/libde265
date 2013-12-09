/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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
#include "pps_func.h"
#include "util.h"

#include <assert.h>
#include <stdlib.h>
#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#else
# include <alloca.h>
#endif


void read_pps(bitreader* br, pic_parameter_set* pps, decoder_context* ctx)
{
  pps->pic_parameter_set_id = get_uvlc(br);
  pps->seq_parameter_set_id = get_uvlc(br);
  pps->dependent_slice_segments_enabled_flag = get_bits(br,1);
  pps->output_flag_present_flag = get_bits(br,1);
  pps->num_extra_slice_header_bits = get_bits(br,3);
  pps->sign_data_hiding_flag = get_bits(br,1);
  pps->cabac_init_present_flag = get_bits(br,1);
  pps->num_ref_idx_l0_default_active = get_uvlc(br)+1;
  pps->num_ref_idx_l1_default_active = get_uvlc(br)+1;

  seq_parameter_set* sps = get_sps(ctx, pps->seq_parameter_set_id);

  pps->pic_init_qp = get_svlc(br)+26;
  pps->constrained_intra_pred_flag = get_bits(br,1);
  pps->transform_skip_enabled_flag = get_bits(br,1);
  pps->cu_qp_delta_enabled_flag = get_bits(br,1);

  if (pps->cu_qp_delta_enabled_flag) {
    pps->diff_cu_qp_delta_depth = get_uvlc(br);
  } else {
    pps->diff_cu_qp_delta_depth = 0;
  }

  pps->pic_cb_qp_offset = get_svlc(br);
  pps->pic_cr_qp_offset  = get_svlc(br);
  pps->pps_slice_chroma_qp_offsets_present_flag = get_bits(br,1);
  pps->weighted_pred_flag = get_bits(br,1);
  pps->weighted_bipred_flag = get_bits(br,1);
  pps->transquant_bypass_enable_flag = get_bits(br,1);
  pps->tiles_enabled_flag = get_bits(br,1);
  pps->entropy_coding_sync_enabled_flag = get_bits(br,1);


  // --- tiles ---

  if (pps->tiles_enabled_flag ) {
    pps->num_tile_columns = get_uvlc(br)+1;
    pps->num_tile_rows    = get_uvlc(br)+1;
    pps->uniform_spacing_flag = get_bits(br,1);

    assert(pps->num_tile_columns <= DE265_MAX_TILE_COLUMNS);
    assert(pps->num_tile_rows    <= DE265_MAX_TILE_ROWS);

    if (pps->uniform_spacing_flag==false) {
      int lastColumnWidth = sps->PicWidthInCtbsY;
      int lastRowHeight   = sps->PicHeightInCtbsY;

      for (int i=0; i<pps->num_tile_columns-1; i++)
        {
          pps->colWidth[i] = get_uvlc(br)+1;
          lastColumnWidth -= pps->colWidth[i];
        }

      pps->colWidth[pps->num_tile_columns-1] = lastColumnWidth;

      for (int i=0; i<pps->num_tile_rows-1; i++)
        {
          pps->rowHeight[i] = get_uvlc(br)+1;
          lastRowHeight -= pps->rowHeight[i];
        }

      pps->rowHeight[pps->num_tile_rows-1] = lastRowHeight;
    }

    pps->loop_filter_across_tiles_enabled_flag = get_bits(br,1);

  } else {
    pps->num_tile_columns = 1;
    pps->num_tile_rows    = 1;
    pps->uniform_spacing_flag = 1;
  }



  if (pps->uniform_spacing_flag) {

    // set columns widths

    int *const colPos = (int *)alloca((pps->num_tile_columns+1) * sizeof(int));

    for (int i=0;i<=pps->num_tile_columns;i++) {
      colPos[i] = i*sps->PicWidthInCtbsY / pps->num_tile_columns;
    }
    for (int i=0;i<pps->num_tile_columns;i++) {
      pps->colWidth[i] = colPos[i+1] - colPos[i];
    }

    // set row heights

    int *const rowPos = (int *)alloca((pps->num_tile_rows+1) * sizeof(int));

    for (int i=0;i<=pps->num_tile_rows;i++) {
      rowPos[i] = i*sps->PicHeightInCtbsY / pps->num_tile_rows;
    }
    for (int i=0;i<pps->num_tile_rows;i++) {
      pps->rowHeight[i] = rowPos[i+1] - rowPos[i];
    }
  }


  // set tile boundaries

  pps->colBd[0]=0;
  for (int i=0;i<pps->num_tile_columns;i++) {
    pps->colBd[i+1] = pps->colBd[i] + pps->colWidth[i];
  }

  pps->rowBd[0]=0;
  for (int i=0;i<pps->num_tile_rows;i++) {
    pps->rowBd[i+1] = pps->rowBd[i] + pps->rowHeight[i];
  }



  // alloc raster scan arrays

  if (pps->CtbAddrRStoTS) { free(pps->CtbAddrRStoTS); }
  if (pps->CtbAddrTStoRS) { free(pps->CtbAddrTStoRS); }
  if (pps->TileId) { free(pps->TileId); }
  if (pps->MinTbAddrZS) { free(pps->MinTbAddrZS); }

  pps->CtbAddrRStoTS = (int *)malloc( sizeof(int) * sps->PicSizeInCtbsY );
  pps->CtbAddrTStoRS = (int *)malloc( sizeof(int) * sps->PicSizeInCtbsY );
  pps->TileId        = (int *)malloc( sizeof(int) * sps->PicSizeInCtbsY );
  pps->MinTbAddrZS   = (int *)malloc( sizeof(int) * sps->PicSizeInTbsY  );


  // raster scan (RS) <-> tile scan (TS) conversion

  for (int ctbAddrRS=0 ; ctbAddrRS < sps->PicSizeInCtbsY ; ctbAddrRS++)
    {
      int tbX = ctbAddrRS % sps->PicWidthInCtbsY;
      int tbY = ctbAddrRS / sps->PicWidthInCtbsY;
      int tileX=-1,tileY=-1;

      for (int i=0;i<pps->num_tile_columns;i++)
        if (tbX >= pps->colBd[i])
          tileX=i;

      for (int j=0;j<pps->num_tile_rows;j++)
        if (tbY >= pps->rowBd[j])
          tileY=j;

      pps->CtbAddrRStoTS[ctbAddrRS] = 0;
      for (int i=0;i<tileX;i++)
        pps->CtbAddrRStoTS[ctbAddrRS] += pps->rowHeight[tileY]*pps->colWidth[i];

      for (int j=0;j<tileY;j++)
        {
          //pps->CtbAddrRStoTS[ctbAddrRS] += (tbY - pps->rowBd[tileY])*pps->colWidth[tileX];
          //pps->CtbAddrRStoTS[ctbAddrRS] += tbX - pps->colBd[tileX];

          pps->CtbAddrRStoTS[ctbAddrRS] += sps->PicWidthInCtbsY * pps->rowHeight[j];
        }

      assert(tileX>=0 && tileY>=0);

      pps->CtbAddrRStoTS[ctbAddrRS] += (tbY-pps->rowBd[tileY])*pps->colWidth[tileX];
      pps->CtbAddrRStoTS[ctbAddrRS] +=  tbX - pps->colBd[tileX];


      // inverse mapping

      pps->CtbAddrTStoRS[ pps->CtbAddrRStoTS[ctbAddrRS] ] = ctbAddrRS;
    }


  logtrace(LogHeaders,"6.5.1 CtbAddrRSToTS\n");
  for (int y=0;y<sps->PicHeightInCtbsY;y++)
    {
      for (int x=0;x<sps->PicWidthInCtbsY;x++)
        {
          logtrace(LogHeaders,"%3d ", pps->CtbAddrRStoTS[x + y*sps->PicWidthInCtbsY]);
        }

      logtrace(LogHeaders,"\n");
    }


  // tile id

  for (int j=0, tIdx=0 ; j<pps->num_tile_rows ; j++)
    for (int i=0 ; i<pps->num_tile_columns;i++)
      {
        for (int y=pps->rowBd[j] ; y<pps->rowBd[j+1] ; y++)
          for (int x=pps->colBd[j] ; x<pps->colBd[j+1] ; x++)
            pps->TileId[ pps->CtbAddrRStoTS[y*sps->PicWidthInCtbsY + x] ] = tIdx;

        tIdx++;
      }

  // 6.5.2 Z-scan order array initialization process

  for (int y=0;y<sps->PicHeightInTbsY;y++)
    for (int x=0;x<sps->PicWidthInTbsY;x++)
      {
        int tbX = (x<<sps->Log2MinTrafoSize)>>sps->Log2CtbSizeY;
        int tbY = (y<<sps->Log2MinTrafoSize)>>sps->Log2CtbSizeY;
        int ctbAddrRS = sps->PicWidthInCtbsY*tbY + tbX;

        pps->MinTbAddrZS[x + y*sps->PicWidthInTbsY] = pps->CtbAddrRStoTS[ctbAddrRS]
          << ((sps->Log2CtbSizeY-sps->Log2MinTrafoSize)*2);

        int p=0;
        for (int i=0 ; i<(sps->Log2CtbSizeY - sps->Log2MinTrafoSize) ; i++) {
          int m=1<<i;
          p += (m & x ? m*m : 0) + (m & y ? 2*m*m : 0);
        }

        pps->MinTbAddrZS[x + y*sps->PicWidthInTbsY] += p;
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

  // END tiles


  pps->Log2MinCuQpDeltaSize = sps->Log2CtbSizeY - pps->diff_cu_qp_delta_depth;


  pps->beta_offset = 0; // default value
  pps->tc_offset   = 0; // default value

  pps->pps_loop_filter_across_slices_enabled_flag = get_bits(br,1);
  pps->deblocking_filter_control_present_flag = get_bits(br,1);
  if (pps->deblocking_filter_control_present_flag) {
    pps->deblocking_filter_override_enabled_flag = get_bits(br,1);
    pps->pic_disable_deblocking_filter_flag = get_bits(br,1);
    if (!pps->pic_disable_deblocking_filter_flag) {
      pps->beta_offset = get_svlc(br)*2;
      pps->tc_offset   = get_svlc(br)*2;
    }
  }
  else {
    pps->deblocking_filter_override_enabled_flag = 0;
    pps->pic_disable_deblocking_filter_flag = 0;
  }

  pps->pic_scaling_list_data_present_flag = get_bits(br,1);
  if (pps->pic_scaling_list_data_present_flag) {
    assert(false);
    //scaling_list_data()
  }

  pps->lists_modification_present_flag = get_bits(br,1);
  pps->log2_parallel_merge_level = get_uvlc(br)+2;
  pps->slice_segment_header_extension_present_flag = get_bits(br,1);
  pps->pps_extension_flag = get_bits(br,1);

  if (pps->pps_extension_flag) {
    assert(false);
    /*
      while( more_rbsp_data() )

      pps_extension_data_flag
      u(1)
      rbsp_trailing_bits()

      }
    */
  }


  pps->pps_read = true;
}


void dump_pps(pic_parameter_set* pps)
{
#if (_MSC_VER >= 1500)
#define LOG(...) loginfo(LogHeaders, __VA_ARGS__)

  LOG("----------------- PPS -----------------\n");
  LOG("pic_parameter_set_id       : %d\n", pps->pic_parameter_set_id);
  LOG("seq_parameter_set_id       : %d\n", pps->seq_parameter_set_id);
  LOG("dependent_slice_segments_enabled_flag : %d\n", pps->dependent_slice_segments_enabled_flag);
  LOG("sign_data_hiding_flag      : %d\n", pps->sign_data_hiding_flag);
  LOG("cabac_init_present_flag    : %d\n", pps->cabac_init_present_flag);
  LOG("num_ref_idx_l0_default_active : %d\n", pps->num_ref_idx_l0_default_active);
  LOG("num_ref_idx_l1_default_active : %d\n", pps->num_ref_idx_l1_default_active);

  LOG("pic_init_qp                : %d\n", pps->pic_init_qp);
  LOG("constrained_intra_pred_flag: %d\n", pps->constrained_intra_pred_flag);
  LOG("transform_skip_enabled_flag: %d\n", pps->transform_skip_enabled_flag);
  LOG("cu_qp_delta_enabled_flag   : %d\n", pps->cu_qp_delta_enabled_flag);

  if (pps->cu_qp_delta_enabled_flag) {
    LOG("diff_cu_qp_delta_depth     : %d\n", pps->diff_cu_qp_delta_depth);
  }

  LOG("pic_cb_qp_offset             : %d\n", pps->pic_cb_qp_offset);
  LOG("pic_cr_qp_offset             : %d\n", pps->pic_cr_qp_offset);
  LOG("pps_slice_chroma_qp_offsets_present_flag : %d\n", pps->pps_slice_chroma_qp_offsets_present_flag);
  LOG("weighted_pred_flag           : %d\n", pps->weighted_pred_flag);
  LOG("weighted_bipred_flag         : %d\n", pps->weighted_bipred_flag);
  LOG("output_flag_present_flag     : %d\n", pps->output_flag_present_flag);
  LOG("transquant_bypass_enable_flag: %d\n", pps->transquant_bypass_enable_flag);
  LOG("tiles_enabled_flag           : %d\n", pps->tiles_enabled_flag);
  LOG("entropy_coding_sync_enabled_flag: %d\n", pps->entropy_coding_sync_enabled_flag);

  if (pps->tiles_enabled_flag) {
    LOG("num_tile_columns    : %d\n", pps->num_tile_columns);
    LOG("num_tile_rows       : %d\n", pps->num_tile_rows);
    LOG("uniform_spacing_flag: %d\n", pps->uniform_spacing_flag);

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

    LOG("loop_filter_across_tiles_enabled_flag : %d\n", pps->loop_filter_across_tiles_enabled_flag);
  }

  LOG("pps_loop_filter_across_slices_enabled_flag: %d\n", pps->pps_loop_filter_across_slices_enabled_flag);
  LOG("deblocking_filter_control_present_flag: %d\n", pps->deblocking_filter_control_present_flag);

  if (pps->deblocking_filter_control_present_flag) {
    LOG("deblocking_filter_override_enabled_flag: %d\n", pps->deblocking_filter_override_enabled_flag);
    LOG("pic_disable_deblocking_filter_flag: %d\n", pps->pic_disable_deblocking_filter_flag);

    LOG("beta_offset:  %d\n", pps->beta_offset);
    LOG("tc_offset:     %d\n", pps->tc_offset);
  }

  LOG("pic_scaling_list_data_present_flag: %d\n", pps->pic_scaling_list_data_present_flag);
  if (pps->pic_scaling_list_data_present_flag) {
    //scaling_list_data()
  }

  LOG("lists_modification_present_flag: %d\n", pps->lists_modification_present_flag);
  LOG("log2_parallel_merge_level      : %d\n", pps->log2_parallel_merge_level);
  LOG("num_extra_slice_header_bits    : %d\n", pps->num_extra_slice_header_bits);
  LOG("slice_segment_header_extension_present_flag : %d\n", pps->slice_segment_header_extension_present_flag);
  LOG("pps_extension_flag : %d\n", pps->pps_extension_flag);
#undef LOG
#endif
}


void init_pps(pic_parameter_set* pps)
{
  pps->CtbAddrRStoTS = NULL;
  pps->CtbAddrTStoRS = NULL;
  pps->TileId = NULL;
  pps->MinTbAddrZS = NULL;
}


void free_pps(pic_parameter_set* pps)
{
  if (pps->CtbAddrRStoTS) { free(pps->CtbAddrRStoTS); }
  if (pps->CtbAddrTStoRS) { free(pps->CtbAddrTStoRS); }
  if (pps->TileId) { free(pps->TileId); }
  if (pps->MinTbAddrZS) { free(pps->MinTbAddrZS); }
}


