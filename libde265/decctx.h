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

#ifndef DE265_DECCTX_H
#define DE265_DECCTX_H

#include "libde265/vps.h"
#include "libde265/sps.h"
#include "libde265/pps.h"
#include "libde265/nal.h"
#include "libde265/slice.h"
#include "libde265/image.h"
#include "libde265/de265.h"

#define DE265_MAX_VPS_SETS 16
#define DE265_MAX_SPS_SETS 16
#define DE265_MAX_PPS_SETS 64
#define DE265_MAX_SLICES   64
#define DE265_IMAGE_OUTPUT_QUEUE_LEN 2


// split_cu_flag             CB (MinCbSizeY)
// skip_flag                 CB
// pcm_flag                  CB
// prev_intra_luma_pred_flag CB
// rem_intra_luma_pred_mode  CB
// mpm_idx                   CB
// intra_chroma_pred_mode    CB

// split_transform_flag      TU
// cbf_cb, cbf_cr, cbf_luma  TU[trafoDepth]
// transform_skip_flag       TU[cIdx]
// coded_sub_block_flag      TU
// significant_coeff_flag    TU


typedef struct {
  uint16_t SliceAddrRS;
  uint16_t SliceHeaderIndex; // index into array to slice header for this CTB

  sao_info saoInfo;
} CTB_info;


typedef struct {
  uint8_t       depth;
  uint8_t       cu_skip_flag;

  uint8_t split_cu_flag;
  // uint8_t pcm_flag;  // TODO
  uint8_t intra_chroma_pred_mode;
  uint8_t PartMode; // (enum PartMode)  set only in top-left of CB

  int8_t  QP_Y;

  uint8_t CB_size; // log2CbSize at top-left of CB, zero otherwise

  uint8_t PredMode; // (enum PredMode)
} CB_info;


typedef struct {
  uint16_t cbf_cb; // bitfield (1<<depth)
  uint16_t cbf_cr; // bitfield (1<<depth)
  uint16_t cbf_luma; // bitfield (1<<depth)

  uint8_t IntraPredMode;  // (enum IntraPredMode)
  uint8_t IntraPredModeC; // (enum IntraPredMode)

  uint8_t split_transform_flag;
  uint8_t transform_skip_flag;   // read bit (1<<cIdx)
  uint8_t coded_sub_block_flag;
  uint8_t significant_coeff_flag;
} TU_info;



#define DEBLOCK_FLAG_VERTI (1<<4)
#define DEBLOCK_FLAG_HORIZ (1<<5)
#define DEBLOCK_BS_MASK     0x03

typedef struct {
  uint8_t deblock_flags;
} deblock_info;



typedef struct {

  // --- parameters ---

  bool param_sei_check_hash;


  // --- internal data ---

  rbsp_buffer nal_data;
  int         input_push_state;

  video_parameter_set  vps[ DE265_MAX_VPS_SETS ];
  seq_parameter_set    sps[ DE265_MAX_SPS_SETS ];
  pic_parameter_set    pps[ DE265_MAX_PPS_SETS ];
  slice_segment_header slice[ DE265_MAX_SLICES ];
  int next_free_slice_index;

  ref_pic_set* ref_pic_sets;

  seq_parameter_set* current_sps;
  pic_parameter_set* current_pps;

  // --- currently decoded picture ---

  de265_image image_buffers[DE265_IMAGE_OUTPUT_QUEUE_LEN];
  int8_t image_output_queue[DE265_IMAGE_OUTPUT_QUEUE_LEN]; // -1: unused
  int8_t image_ref_count   [DE265_IMAGE_OUTPUT_QUEUE_LEN]; // 0: unused

  de265_image img; // TODO: make pointer inter image_buffers
  de265_image coeff; // transform coefficients

  CTB_info* ctb_info; // in raster scan
  CB_info*  cb_info; // in raster scan
  TU_info*  tu_info; // in raster scan
  deblock_info* deblk_info;

  int ctb_info_size;
  int cb_info_size;
  int tu_info_size;
  int deblk_info_size;

  int deblk_width;
  int deblk_height;

  // --- parameters derived from parameter sets ---

  // NAL

  uint8_t nal_unit_type;

  char IdrPicFlag;
  char RapPicFlag;

} decoder_context;


void init_decoder_context(decoder_context*);
void reset_decoder_context_for_new_picture(decoder_context* ctx);
void free_decoder_context(decoder_context*);

seq_parameter_set* get_sps(decoder_context* ctx, int id);

void process_nal_hdr(decoder_context*, nal_header*);
void process_vps(decoder_context*, video_parameter_set*);
void process_sps(decoder_context*, seq_parameter_set*);
void process_pps(decoder_context*, pic_parameter_set*);
void process_slice_segment_header(decoder_context*, slice_segment_header*);

int get_next_slice_index(decoder_context* ctx);


// --- decoder 2D data arrays ---
// All coordinates are in pixels if not stated otherwise.

int get_ctDepth(const decoder_context* ctx, int x,int y);
void set_ctDepth(decoder_context* ctx, int x,int y, int log2BlkWidth, int depth);
void debug_dump_cb_info(const decoder_context*);

//void set_intra_chroma_pred_mode(decoder_context* ctx, int x,int y, int log2BlkWidth, int mode);
//int  get_intra_chroma_pred_mode(decoder_context* ctx, int x,int y);

void set_cbf_cb(decoder_context*, int x0,int y0, int depth);
void set_cbf_cr(decoder_context*, int x0,int y0, int depth);
int  get_cbf_cb(const decoder_context*, int x0,int y0, int depth);
int  get_cbf_cr(const decoder_context*, int x0,int y0, int depth);

void    set_cu_skip_flag(      decoder_context*, int x,int y, uint8_t flag);
uint8_t get_cu_skip_flag(const decoder_context*, int x,int y);

void          set_PartMode(      decoder_context*, int x,int y, enum PartMode);
enum PartMode get_PartMode(const decoder_context*, int x,int y);

void          set_pred_mode(      decoder_context*, int x,int y, int log2BlkWidth, enum PredMode mode);
enum PredMode get_pred_mode(const decoder_context*, int x,int y);

void set_IntraPredMode(decoder_context*, int x,int y, int log2BlkWidth, enum IntraPredMode mode);
enum IntraPredMode get_IntraPredMode(const decoder_context*, int x,int y);

void set_IntraPredModeC(decoder_context*, int x,int y, int log2BlkWidth, enum IntraPredMode mode);
enum IntraPredMode get_IntraPredModeC(const decoder_context*, int x,int y);

void set_SliceAddrRS(      decoder_context*, int ctbX, int ctbY, int SliceAddrRS);
int  get_SliceAddrRS(const decoder_context*, int ctbX, int ctbY);

void set_SliceHeaderIndex(      decoder_context*, int x, int y, int SliceHeaderIndex);
int  get_SliceHeaderIndex(const decoder_context*, int x, int y);
slice_segment_header* get_SliceHeader(decoder_context*, int x, int y);
slice_segment_header* get_SliceHeaderCtb(decoder_context* ctx, int ctbX, int ctbY);

void set_split_transform_flag(decoder_context* ctx,int x0,int y0,int trafoDepth);
int  get_split_transform_flag(const decoder_context* ctx,int x0,int y0,int trafoDepth);

void set_transform_skip_flag(decoder_context* ctx,int x0,int y0,int cIdx);
int  get_transform_skip_flag(const decoder_context* ctx,int x0,int y0,int cIdx);

void set_QPY(decoder_context* ctx,int x0,int y0, int QP_Y);
int  get_QPY(const decoder_context* ctx,int x0,int y0);

void get_image_plane(const decoder_context*, int cIdx, uint8_t** image, int* stride);
void get_coeff_plane(const decoder_context*, int cIdx, int16_t** image, int* stride);

void set_CB_size(decoder_context*, int x0, int y0, int log2CbSize);
int  get_log2CbSize(const decoder_context* ctx, int x0, int y0);

// coordinates in CB units
int  get_log2CbSize_cbUnits(const decoder_context* ctx, int x0, int y0);

void    set_deblk_flags(decoder_context*, int x0,int y0, uint8_t flags);
uint8_t get_deblk_flags(const decoder_context*, int x0,int y0);

void    set_deblk_bS(decoder_context*, int x0,int y0, uint8_t bS);
uint8_t get_deblk_bS(const decoder_context*, int x0,int y0);

void            set_sao_info(decoder_context*, int ctbX,int ctbY,const sao_info* sao_info);
const sao_info* get_sao_info(const decoder_context*, int ctbX,int ctbY);


bool available_zscan(const decoder_context* ctx,
                     int xCurr,int yCurr, int xN,int yN);

bool available_pred_blk(const decoder_context* ctx,
                        int xC,int yC, int nCbS, int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN);

// --- debug ---

void write_picture(const decoder_context* ctx);
void draw_CB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_TB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_PB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_intra_pred_modes(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);

#endif
