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
#include <mutex>
#include <memory>
#include <atomic>
#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#elif defined(HAVE_ALLOCA_H)
# include <alloca.h>
#endif


void pps_range_extension::reset()
{
  log2_max_transform_skip_block_size = 2;
  cross_component_prediction_enabled_flag = false;
  chroma_qp_offset_list_enabled_flag = false;
  diff_cu_chroma_qp_offset_depth = 0;
  chroma_qp_offset_list_len = 0;
  log2_sao_offset_scale_luma = 0;
  log2_sao_offset_scale_chroma = 0;
}


bool pps_range_extension::read(bitreader* br, decoder_context* ctx, const pic_parameter_set* pps)
{
  const seq_parameter_set* sps = ctx->get_sps(pps->seq_parameter_set_id);

  uint32_t uvlc;

  if (pps->transform_skip_enabled_flag) {
    uvlc = br->get_uvlc();
    if (uvlc == UVLC_ERROR ||
        uvlc > static_cast<uint32_t>(sps->Log2MaxTrafoSize) - 2) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }

    log2_max_transform_skip_block_size = uvlc+2;
  }

  cross_component_prediction_enabled_flag = br->get_bits(1);
  // shall be 0 when ChromaArrayType is not 3 (Sec. 7.4.3.3.2)
  if (sps->ChromaArrayType != CHROMA_444 &&
      cross_component_prediction_enabled_flag) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
  }

  chroma_qp_offset_list_enabled_flag = br->get_bits(1);
  // shall be 0 when ChromaArrayType is 0 (mono) (Sec. 7.4.3.3.2)
  if (sps->ChromaArrayType == CHROMA_MONO &&
      chroma_qp_offset_list_enabled_flag) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
  }

  if (chroma_qp_offset_list_enabled_flag) {
    uvlc = br->get_uvlc();
    if (uvlc == UVLC_ERROR ||
        uvlc > static_cast<uint32_t>(sps->log2_diff_max_min_luma_coding_block_size)) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }

    diff_cu_chroma_qp_offset_depth = uvlc;


    uvlc = br->get_uvlc();
    if (uvlc == UVLC_ERROR ||
        uvlc > 5) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }

    chroma_qp_offset_list_len = uvlc+1;

    for (int i=0;i<chroma_qp_offset_list_len;i++) {
      int32_t svlc;
      svlc = br->get_svlc();
      if (svlc == SVLC_ERROR ||
          svlc < -12 || svlc > 12) {
        ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
        return false;
      }

      cb_qp_offset_list[i] = svlc;

      svlc = br->get_svlc();
      if (svlc == SVLC_ERROR ||
          svlc < -12 || svlc > 12) {
        ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
        return false;
      }

      cr_qp_offset_list[i] = svlc;
    }
  }


  uvlc = br->get_uvlc();
  if (uvlc == UVLC_ERROR ||
      uvlc > static_cast<uint32_t>(std::max(0, sps->BitDepth_Y-10))) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  log2_sao_offset_scale_luma = uvlc;

  uvlc = br->get_uvlc();
  if (uvlc == UVLC_ERROR ||
      uvlc > static_cast<uint32_t>(std::max(0, sps->BitDepth_C-10))) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  log2_sao_offset_scale_chroma = uvlc;

  return true;
}


void pps_range_extension::dump(int fd) const
{
  FILE* fh;
  if (fd==1) fh=stdout;
  else if (fd==2) fh=stderr;
  else { return; }

#define LOG0(t) log2fh(fh, t)
#define LOG1(t,d) log2fh(fh, t,d)
#define LOG2(t,d,e) log2fh(fh, t,d,e)

  LOG0("---------- PPS range-extension ----------\n");
  LOG1("log2_max_transform_skip_block_size      : %d\n", log2_max_transform_skip_block_size);
  LOG1("cross_component_prediction_enabled_flag : %d\n", cross_component_prediction_enabled_flag);
  LOG1("chroma_qp_offset_list_enabled_flag      : %d\n", chroma_qp_offset_list_enabled_flag);
  if (chroma_qp_offset_list_enabled_flag) {
    LOG1("diff_cu_chroma_qp_offset_depth          : %d\n", diff_cu_chroma_qp_offset_depth);
    LOG1("chroma_qp_offset_list_len               : %d\n", chroma_qp_offset_list_len);
    for (int i=0;i<chroma_qp_offset_list_len;i++) {
      LOG2("cb_qp_offset_list[%d]                    : %d\n", i,cb_qp_offset_list[i]);
      LOG2("cr_qp_offset_list[%d]                    : %d\n", i,cr_qp_offset_list[i]);
    }
  }

  LOG1("log2_sao_offset_scale_luma              : %d\n", log2_sao_offset_scale_luma);
  LOG1("log2_sao_offset_scale_chroma            : %d\n", log2_sao_offset_scale_chroma);
#undef LOG2
#undef LOG1
#undef LOG0
}





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
  sps = nullptr;

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

  scan.reset();


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
  pps_extension_flag = 0;

  pps_range_extension_flag = 0;
  pps_multilayer_extension_flag = 0;
  pps_extension_6bits = 0;

  range_extension.reset();
}


bool pic_parameter_set::read(bitreader* br, decoder_context* ctx)
{
  reset();


  uint32_t uvlc;
  uvlc = br->get_uvlc();
  if (uvlc == UVLC_ERROR || uvlc >= DE265_MAX_PPS_SETS) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_PPS_REFERENCED, false);
    return false;
  }
  pic_parameter_set_id = uvlc;

  uvlc = br->get_uvlc();
  if (uvlc == UVLC_ERROR || uvlc >= DE265_MAX_SPS_SETS) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_SPS_REFERENCED, false);
    return false;
  }
  seq_parameter_set_id = uvlc;

  dependent_slice_segments_enabled_flag = br->get_bits(1);
  output_flag_present_flag = br->get_bits(1);
  num_extra_slice_header_bits = br->get_bits(3);
  sign_data_hiding_flag = br->get_bits(1);
  cabac_init_present_flag = br->get_bits(1);
  uvlc = br->get_uvlc();
  if (uvlc == UVLC_ERROR || uvlc > 15) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  num_ref_idx_l0_default_active = uvlc + 1;

  uvlc = br->get_uvlc();
  if (uvlc == UVLC_ERROR || uvlc > 15) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  num_ref_idx_l1_default_active = uvlc + 1;


  if (!ctx->has_sps(seq_parameter_set_id)) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_SPS_REFERENCED, false);
    return false;
  }

  sps = ctx->get_shared_sps(seq_parameter_set_id);

  {
    int32_t svlc;
    // init_qp_minus26 shall be in [-(26 + QpBdOffset_Y), +25] (Sec. 7.4.3.3.1)
    if ((svlc = br->get_svlc()) == SVLC_ERROR ||
        svlc < -(26 + sps->QpBdOffset_Y) || svlc > 25) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    pic_init_qp = svlc + 26;
  }

  constrained_intra_pred_flag = br->get_bits(1);
  transform_skip_enabled_flag = br->get_bits(1);
  cu_qp_delta_enabled_flag = br->get_bits(1);

  if (cu_qp_delta_enabled_flag) {
    // diff_cu_qp_delta_depth shall be in [0, log2_diff_max_min_luma_coding_block_size] (Sec. 7.4.3.3.1)
    if ((uvlc = br->get_uvlc()) == UVLC_ERROR ||
        uvlc > sps->log2_diff_max_min_luma_coding_block_size) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    diff_cu_qp_delta_depth = uvlc;
  } else {
    diff_cu_qp_delta_depth = 0;
  }

  {
    int32_t svlc;
    if ((svlc = br->get_svlc()) == SVLC_ERROR) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    pic_cb_qp_offset = svlc;
  }

  {
    int32_t svlc;
    if ((svlc = br->get_svlc()) == SVLC_ERROR) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    pic_cr_qp_offset = svlc;
  }

  pps_slice_chroma_qp_offsets_present_flag = br->get_bits(1);
  weighted_pred_flag = br->get_bits(1);
  weighted_bipred_flag = br->get_bits(1);
  transquant_bypass_enable_flag = br->get_bits(1);
  tiles_enabled_flag = br->get_bits(1);
  entropy_coding_sync_enabled_flag = br->get_bits(1);


  // --- tiles ---

  if (tiles_enabled_flag) {
    if ((uvlc = br->get_uvlc()) == UVLC_ERROR ||
        uvlc + 1 > DE265_MAX_TILE_COLUMNS ||
        uvlc + 1 > sps->PicWidthInCtbsY) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    num_tile_columns = uvlc + 1;

    if ((uvlc = br->get_uvlc()) == UVLC_ERROR ||
        uvlc + 1 > DE265_MAX_TILE_ROWS ||
        uvlc + 1 > sps->PicHeightInCtbsY) {
      ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
      return false;
    }
    num_tile_rows = uvlc + 1;

    uniform_spacing_flag = br->get_bits(1);

    if (uniform_spacing_flag==false) {
      uint16_t lastColumnWidth = sps->PicWidthInCtbsY;
      uint16_t lastRowHeight   = sps->PicHeightInCtbsY;

      for (int i = 0; i < num_tile_columns - 1; i++) {
        if ((uvlc = br->get_uvlc()) == UVLC_ERROR ||
            uvlc + 1 >= lastColumnWidth) {
          ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
          return false;
        }

        colWidth[i] = uvlc + 1;

        lastColumnWidth -= colWidth[i];
      }

      colWidth[num_tile_columns - 1] = lastColumnWidth;

      for (int i = 0; i < num_tile_rows - 1; i++) {
        if ((uvlc = br->get_uvlc()) == UVLC_ERROR ||
            uvlc + 1 >= lastRowHeight) {
          ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
          return false;
        }
        rowHeight[i] = uvlc + 1;
        lastRowHeight -= rowHeight[i];
      }


      rowHeight[num_tile_rows-1] = lastRowHeight;
    }

    loop_filter_across_tiles_enabled_flag = br->get_bits(1);

  } else {
    num_tile_columns = 1;
    num_tile_rows    = 1;
    uniform_spacing_flag = 1;
    loop_filter_across_tiles_enabled_flag = 0;
  }



  // END tiles



  beta_offset = 0; // default value
  tc_offset   = 0; // default value

  pps_loop_filter_across_slices_enabled_flag = br->get_bits(1);
  deblocking_filter_control_present_flag = br->get_bits(1);
  if (deblocking_filter_control_present_flag) {
    deblocking_filter_override_enabled_flag = br->get_bits(1);
    pic_disable_deblocking_filter_flag = br->get_bits(1);
    if (!pic_disable_deblocking_filter_flag) {
      {
        int32_t svlc;
        // pps_beta_offset_div2 shall be in [-6, 6] (Sec. 7.4.3.3.1)
        if ((svlc = br->get_svlc()) == SVLC_ERROR || svlc < -6 || svlc > 6) {
	  ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
	  return false;
        }
        beta_offset = svlc * 2;

        // pps_tc_offset_div2 shall be in [-6, 6] (Sec. 7.4.3.3.1)
        if ((svlc = br->get_svlc()) == SVLC_ERROR || svlc < -6 || svlc > 6) {
	  ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
	  return false;
        }
        tc_offset = svlc * 2;
      }
    }
  }
  else {
    deblocking_filter_override_enabled_flag = 0;
    pic_disable_deblocking_filter_flag = 0;
  }


  // --- scaling list ---

  pic_scaling_list_data_present_flag = br->get_bits(1);

  // check consistency: if scaling-lists are not enabled, pic_scalign_list_data_present_flag
  // must be FALSE
  if (sps->scaling_list_enable_flag==0 &&
      pic_scaling_list_data_present_flag != 0) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }

  if (pic_scaling_list_data_present_flag) {
    de265_error err = read_scaling_list(br, sps.get(), &scaling_list, true);
    if (err != DE265_OK) {
      ctx->add_warning(err, false);
      return false;
    }
  }
  else {
    scaling_list = sps->scaling_list;
  }




  lists_modification_present_flag = br->get_bits(1);
  if ((uvlc = br->get_uvlc()) == UVLC_ERROR || uvlc > 4) {
    ctx->add_warning(DE265_WARNING_PPS_HEADER_INVALID, false);
    return false;
  }
  log2_parallel_merge_level = uvlc + 2;

  if (log2_parallel_merge_level-2 > sps->log2_min_luma_coding_block_size-3 +1 +
      sps->log2_diff_max_min_luma_coding_block_size) {
    return false;
  }

  slice_segment_header_extension_present_flag = br->get_bits(1);
  pps_extension_flag = br->get_bits(1);

  if (pps_extension_flag) {
    pps_range_extension_flag = br->get_bits(1);
    pps_multilayer_extension_flag = br->get_bits(1);
    pps_extension_6bits = br->get_bits(6);

    if (pps_range_extension_flag) {
      bool success = range_extension.read(br, ctx, this);
      if (!success) {
        return false;
      }
    }

    // Multilayer extension and the 6 reserved extension bits would carry
    // additional payload that we do not parse. Reject the stream.
    if (pps_multilayer_extension_flag || pps_extension_6bits) {
      ctx->add_warning(DE265_ERROR_NOT_IMPLEMENTED_YET, false);
      return false;
    }
  }


  set_derived_values(sps.get());

  pps_read = true;

  return true;
}


//----------------------------------------------------------------------------
// Library-scope cache for the geometry-derived scan tables (HEVC Sec. 6.5).
//
// The tables depend only on the picture/tile geometry. Many independent decoder
// contexts (e.g. libheif tile grids) decode images of the same geometry, so we
// compute the tables once and share them read-only via shared_ptr. A small LRU
// cache (a few distinct geometries) protected by a mutex serves concurrent
// decoders. The compute is done while holding the lock on purpose: a burst of
// contexts with the same new geometry then computes the tables exactly once
// (the others block briefly and pick up the cached result).
//----------------------------------------------------------------------------

namespace {

struct pps_scan_key {
  uint8_t  log2CtbSize;
  uint8_t  log2MinTrafo;
  uint16_t picWidthInCtbs, picHeightInCtbs;
  uint16_t picWidthInTbs,  picHeightInTbs;
  uint32_t picSizeInCtbs,  picSizeInTbs;
  uint16_t numTileCols,    numTileRows;
  uint16_t colBd[DE265_MAX_TILE_COLUMNS+1];
  uint16_t rowBd[DE265_MAX_TILE_ROWS+1];

  bool operator==(const pps_scan_key& o) const {
    if (log2CtbSize    != o.log2CtbSize    || log2MinTrafo   != o.log2MinTrafo   ||
        picWidthInCtbs != o.picWidthInCtbs || picHeightInCtbs!= o.picHeightInCtbs||
        picWidthInTbs  != o.picWidthInTbs  || picHeightInTbs != o.picHeightInTbs ||
        picSizeInCtbs  != o.picSizeInCtbs  || picSizeInTbs   != o.picSizeInTbs   ||
        numTileCols    != o.numTileCols    || numTileRows    != o.numTileRows) return false;
    for (int i=0;i<=numTileCols;i++) if (colBd[i]!=o.colBd[i]) return false;
    for (int i=0;i<=numTileRows;i++) if (rowBd[i]!=o.rowBd[i]) return false;
    return true;
  }
};

// Build the five scan tables from the geometry key (HEVC 6.5.1 + 6.5.2).
std::shared_ptr<const pps_scan_tables> compute_scan_tables(const pps_scan_key& k)
{
  std::shared_ptr<pps_scan_tables> t = std::make_shared<pps_scan_tables>();
  t->CtbAddrRStoTS.resize(k.picSizeInCtbs);
  t->CtbAddrTStoRS.resize(k.picSizeInCtbs);
  t->TileId       .resize(k.picSizeInCtbs);
  t->TileIdRS     .resize(k.picSizeInCtbs);
  t->MinTbAddrZS  .resize(k.picSizeInTbs);

  // 6.5.1 raster (RS) <-> tile scan (TS) conversion + tile-ID assignment.
  uint32_t ctbAddrTS = 0;
  uint32_t tIdx = 0;
  for (int tileY=0; tileY<k.numTileRows; tileY++) {
    for (int tileX=0; tileX<k.numTileCols; tileX++) {
      for (int y=k.rowBd[tileY]; y<k.rowBd[tileY+1]; y++) {
        for (int x=k.colBd[tileX]; x<k.colBd[tileX+1]; x++) {
          uint32_t ctbAddrRS = y * k.picWidthInCtbs + x;
          t->CtbAddrRStoTS[ctbAddrRS] = ctbAddrTS;
          t->CtbAddrTStoRS[ctbAddrTS] = ctbAddrRS;
          t->TileId  [ctbAddrTS] = tIdx;
          t->TileIdRS[ctbAddrRS] = tIdx;
          ctbAddrTS++;
        }
      }
      tIdx++;
    }
  }
  assert(ctbAddrTS == k.picSizeInCtbs);

  // 6.5.2 Z-scan order array initialization process.
  const int shift = k.log2CtbSize - k.log2MinTrafo;
  for (int y=0; y<k.picHeightInTbs; y++)
    for (int x=0; x<k.picWidthInTbs; x++) {
      int tbX = (x<<k.log2MinTrafo)>>k.log2CtbSize;
      int tbY = (y<<k.log2MinTrafo)>>k.log2CtbSize;
      int ctbAddrRS = k.picWidthInCtbs*tbY + tbX;

      uint32_t v = t->CtbAddrRStoTS[ctbAddrRS] << (shift*2);
      int p=0;
      for (int i=0;i<shift;i++) {
        int m=1<<i;
        p += (m & x ? m*m : 0) + (m & y ? 2*m*m : 0);
      }
      t->MinTbAddrZS[x + y*k.picWidthInTbs] = v + p;
    }

  return t;
}

class pps_scan_cache {
public:
  std::shared_ptr<const pps_scan_tables> get(const pps_scan_key& key) {
    std::lock_guard<std::mutex> lock(mMutex);

    for (size_t i=0; i<mEntries.size(); i++) {
      if (mEntries[i].key == key) {
        std::shared_ptr<const pps_scan_tables> tables = mEntries[i].tables;
        if (i != 0) {  // move-to-front (LRU)
          Entry e = mEntries[i];
          mEntries.erase(mEntries.begin()+i);
          mEntries.insert(mEntries.begin(), e);
        }
        return tables;
      }
    }

    // Miss: compute while holding the lock so that a burst of concurrent decoders
    // with the same new geometry computes the tables exactly once.
    std::shared_ptr<const pps_scan_tables> tables = compute_scan_tables(key);
    mEntries.insert(mEntries.begin(), Entry{key, tables});
    if (mEntries.size() > kMaxEntries) mEntries.pop_back();  // evict LRU
    return tables;
  }

private:
  static const size_t kMaxEntries = 3;
  struct Entry { pps_scan_key key; std::shared_ptr<const pps_scan_tables> tables; };
  std::mutex mMutex;
  std::vector<Entry> mEntries;
};

// Owned by the de265_init()/de265_free() lifecycle (see de265.cc). It is created
// and destroyed (under de265's init mutex) while no decoder is running, so it is
// read locklessly during decoding; the cache's own mutex guards concurrent get()
// calls. Atomic so the publish/read of the pointer is well-defined.
std::atomic<pps_scan_cache*> g_pps_scan_cache{nullptr};

std::shared_ptr<const pps_scan_tables> get_pps_scan_tables(const pps_scan_key& key)
{
  pps_scan_cache* cache = g_pps_scan_cache.load(std::memory_order_acquire);
  if (cache) return cache->get(key);
  return compute_scan_tables(key);  // library not initialized: compute without caching
}

} // namespace


void pps_scan_cache_init()
{
  if (!g_pps_scan_cache.load(std::memory_order_relaxed)) {
    g_pps_scan_cache.store(new pps_scan_cache(), std::memory_order_release);
  }
}

void pps_scan_cache_free()
{
  delete g_pps_scan_cache.exchange(nullptr, std::memory_order_acq_rel);
}


void pic_parameter_set::set_derived_values(const seq_parameter_set* sps)
{
  Log2MinCuQpDeltaSize = sps->Log2CtbSizeY - diff_cu_qp_delta_depth;

  Log2MinCuChromaQpOffsetSize = sps->Log2CtbSizeY - range_extension.diff_cu_chroma_qp_offset_depth;
  Log2MaxTransformSkipSize = range_extension.log2_max_transform_skip_block_size;

  if (uniform_spacing_flag) {

    // set columns widths

    int *const colPos = static_cast<int*>(alloca((num_tile_columns+1) * sizeof(int)));

    for (int i=0;i<=num_tile_columns;i++) {
      colPos[i] = i*sps->PicWidthInCtbsY / num_tile_columns;
    }
    for (int i=0;i<num_tile_columns;i++) {
      colWidth[i] = colPos[i+1] - colPos[i];
    }

    // set row heights

    int *const rowPos = static_cast<int*>(alloca((num_tile_rows+1) * sizeof(int)));

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



  // The derived scan tables (Sec. 6.5.1 + 6.5.2) depend only on the picture/tile
  // geometry computed above. Build the geometry key and fetch the shared tables
  // from the library-scope cache (computing+caching them on a miss). This avoids
  // recomputing the (potentially large) MinTbAddrZS table for every decoder
  // context when many contexts decode images of the same geometry.

  pps_scan_key key;
  memset(&key, 0, sizeof(key));   // zero padding/unused tile entries for clean compares
  key.log2CtbSize     = sps->Log2CtbSizeY;
  key.log2MinTrafo    = sps->Log2MinTrafoSize;
  key.picWidthInCtbs  = sps->PicWidthInCtbsY;
  key.picHeightInCtbs = sps->PicHeightInCtbsY;
  key.picWidthInTbs   = sps->PicWidthInTbsY;
  key.picHeightInTbs  = sps->PicHeightInTbsY;
  key.picSizeInCtbs   = sps->PicSizeInCtbsY;
  key.picSizeInTbs    = sps->PicSizeInTbsY;
  key.numTileCols     = num_tile_columns;
  key.numTileRows     = num_tile_rows;
  for (int i=0;i<=num_tile_columns;i++) key.colBd[i] = colBd[i];
  for (int i=0;i<=num_tile_rows;   i++) key.rowBd[i] = rowBd[i];

  scan = get_pps_scan_tables(key);
}


bool pic_parameter_set::write(error_queue* errqueue, CABAC_encoder& out,
                              const seq_parameter_set* sps)
{
  if (pic_parameter_set_id >= DE265_MAX_PPS_SETS) {
    errqueue->add_warning(DE265_WARNING_NONEXISTING_PPS_REFERENCED, false);
    return false;
  }
  out.write_uvlc(pic_parameter_set_id);

  if (seq_parameter_set_id >= DE265_MAX_SPS_SETS) {
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
  out.write_bit(pps_extension_flag);

  if (pps_extension_flag) {
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
    LOG1("tc_offset:    %d\n", tc_offset);
  }

  LOG1("pic_scaling_list_data_present_flag: %d\n", pic_scaling_list_data_present_flag);
  if (pic_scaling_list_data_present_flag) {
    //scaling_list_data()
  }

  LOG1("lists_modification_present_flag: %d\n", lists_modification_present_flag);
  LOG1("log2_parallel_merge_level      : %d\n", log2_parallel_merge_level);
  LOG1("num_extra_slice_header_bits    : %d\n", num_extra_slice_header_bits);
  LOG1("slice_segment_header_extension_present_flag : %d\n", slice_segment_header_extension_present_flag);
  LOG1("pps_extension_flag            : %d\n", pps_extension_flag);
  LOG1("pps_range_extension_flag      : %d\n", pps_range_extension_flag);
  LOG1("pps_multilayer_extension_flag : %d\n", pps_multilayer_extension_flag);
  LOG1("pps_extension_6bits           : %d\n", pps_extension_6bits);

  LOG1("Log2MinCuQpDeltaSize          : %d\n", Log2MinCuQpDeltaSize);
  LOG1("Log2MinCuChromaQpOffsetSize (RExt) : %d\n", Log2MinCuChromaQpOffsetSize);
  LOG1("Log2MaxTransformSkipSize    (RExt) : %d\n", Log2MaxTransformSkipSize);

#undef LOG0
#undef LOG1


  if (pps_range_extension_flag) {
    range_extension.dump(fd);
  }
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
