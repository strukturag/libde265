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
#include "libde265/motion.h"
#include "libde265/de265.h"
#include "libde265/threads.h"
#include "libde265/lowlevel.h"

#define DE265_MAX_VPS_SETS 16
#define DE265_MAX_SPS_SETS 16
#define DE265_MAX_PPS_SETS 64
#define DE265_MAX_SLICES   8
#define DE265_IMAGE_OUTPUT_QUEUE_LEN 2

// TODO: check required value
#define DE265_DPB_SIZE  20

#define MAX_WARNINGS 20


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

  uint8_t  task_blocking_cnt; // for parallelization
} CTB_info;


typedef struct {
  uint8_t       depth;
  uint8_t       cu_skip_flag;

  uint8_t split_cu_flag;
  uint8_t rqt_root_cbf;
  // uint8_t pcm_flag;  // TODO
  uint8_t PartMode; // (enum PartMode)  set only in top-left of CB
  uint8_t intra_split_flag;

  int8_t  QP_Y;

  uint8_t CB_size; // log2CbSize at top-left of CB, zero otherwise

  uint8_t PredMode; // (enum PredMode)
} CB_info;


#define TU_FLAG_NONZERO_COEFF  (1<<0)
//#define TU_FLAG_AVAILABLE_LEFT    (1<<1)
//#define TU_FLAG_AVAILABLE_TOP     (1<<2)
//#define TU_FLAG_AVAILABLE_TOPLEFT (1<<3)

typedef struct {
  uint16_t cbf_cb;   // NOTE: can be thread-local // bitfield (1<<depth)
  uint16_t cbf_cr;   // NOTE: can be thread-local // bitfield (1<<depth)
  uint16_t cbf_luma; // NOTE: can be thread-local // bitfield (1<<depth)

  uint8_t split_transform_flag;  // NOTE: can be local if deblocking flags set during decoding
  uint8_t transform_skip_flag;   // NOTE: can be in local context    // read bit (1<<cIdx)
  uint8_t flags;                 // NOTE: can be removed if deblocking flags set during decoding (nonzero coefficients)
} TU_info;



#define DEBLOCK_FLAG_VERTI (1<<4)
#define DEBLOCK_FLAG_HORIZ (1<<5)
#define DEBLOCK_PB_EDGE_VERTI (1<<6)
#define DEBLOCK_PB_EDGE_HORIZ (1<<7)
#define DEBLOCK_BS_MASK     0x03

typedef struct {
  uint8_t deblock_flags;
} deblock_info;


enum LowLevelImplementation {
  LOWLEVEL_AUTO,
  LOWLEVEL_SSE,
  LOWLEVEL_AVX
};


typedef struct decoder_context {

  // --- parameters ---

  bool param_sei_check_hash;
  int  param_HighestTid;


  // --- decoder administration ---

  struct lowlevel_functions lowlevel; // CPU optimized functions

  de265_error warnings[MAX_WARNINGS];
  int nWarnings;
  de265_error warnings_shown[MAX_WARNINGS]; // warnings that have already occurred
  int nWarningsShown;

  // --- internal data ---

  rbsp_buffer pending_input_data;
  bool end_of_stream; // data in pending_input_data is end of stream

  rbsp_buffer nal_data;
  int         input_push_state;

  int*  skipped_bytes;  // up to position[x], there were 'x' skipped bytes
  int   num_skipped_bytes;
  int   max_skipped_bytes;

  video_parameter_set  vps[ DE265_MAX_VPS_SETS ];
  seq_parameter_set    sps[ DE265_MAX_SPS_SETS ];
  pic_parameter_set    pps[ DE265_MAX_PPS_SETS ];
  slice_segment_header slice[ DE265_MAX_SLICES ];
  int next_free_slice_index;

  ref_pic_set* ref_pic_sets;

  video_parameter_set* current_vps;
  seq_parameter_set*   current_sps;
  pic_parameter_set*   current_pps;

  struct thread_pool thread_pool;
  int num_worker_threads;


  // --- sequence level ---

  int HighestTid;


  // --- decoded picture buffer ---

  de265_image dpb[DE265_DPB_SIZE]; // decoded picture buffer

  de265_image* reorder_output_queue[DE265_DPB_SIZE];
  int          reorder_output_queue_length;

  de265_image* image_output_queue[DE265_DPB_SIZE];
  int          image_output_queue_length;

  de265_image* last_decoded_image;

  int current_image_poc_lsb;
  bool first_decoded_picture;
  bool NoRaslOutputFlag;
  bool HandleCraAsBlaFlag;

  int  PicOrderCntMsb;
  int prevPicOrderCntLsb;  // at precTid0Pic
  int prevPicOrderCntMsb;  // at precTid0Pic

  de265_image* img;


  // --- motion compensation ---

  int NumPocStCurrBefore;
  int NumPocStCurrAfter;
  int NumPocStFoll;
  int NumPocLtCurr;
  int NumPocLtFoll;

  // TODO: what are the actual maximum array sizes? This is just a first upper bound.
  int PocStCurrBefore[DE265_DPB_SIZE];
  int PocStCurrAfter[DE265_DPB_SIZE];
  int PocStFoll[DE265_DPB_SIZE];
  int PocLtCutt[DE265_DPB_SIZE];
  int PocLtFoll[DE265_DPB_SIZE];

  int RefPicSetStCurrBefore[DE265_DPB_SIZE];
  int RefPicSetStCurrAfter[DE265_DPB_SIZE];
  int RefPicSetStFoll[DE265_DPB_SIZE];
  int RefPicSetLtCurr[DE265_DPB_SIZE];
  int RefPicSetLtFoll[DE265_DPB_SIZE];


  // --- decoded image data --- TODO: all this should move into de265_image

  // de265_image coeff; // transform coefficients / TODO: don't use de265_image for this

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
void set_lowlevel_functions(decoder_context* ctx, enum LowLevelImplementation);
void reset_decoder_context_for_new_picture(decoder_context* ctx);
void free_decoder_context(decoder_context*);

void flush_next_picture_from_reorder_buffer(decoder_context* ctx);

seq_parameter_set* get_sps(decoder_context* ctx, int id);

void process_nal_hdr(decoder_context*, nal_header*);
void process_vps(decoder_context*, video_parameter_set*);
void process_sps(decoder_context*, seq_parameter_set*);
void process_pps(decoder_context*, pic_parameter_set*);
de265_error process_slice_segment_header(decoder_context*, slice_segment_header*);

int get_next_slice_index(decoder_context* ctx);

void add_warning(decoder_context* ctx, de265_error warning, bool once);
de265_error get_warning(decoder_context* ctx);

// TODO void free_currently_unused_memory(decoder_context* ctx); // system is low on memory, free some (e.g. unused images in the DPB)


// --- decoder 2D data arrays ---
// All coordinates are in pixels if not stated otherwise.

int get_ctDepth(const decoder_context* ctx, int x,int y);
void set_ctDepth(decoder_context* ctx, int x,int y, int log2BlkWidth, int depth);
void debug_dump_cb_info(const decoder_context*);

void set_cbf_cb(decoder_context*, int x0,int y0, int depth);
void set_cbf_cr(decoder_context*, int x0,int y0, int depth);
int  get_cbf_cb(const decoder_context*, int x0,int y0, int depth);
int  get_cbf_cr(const decoder_context*, int x0,int y0, int depth);

void    set_cu_skip_flag(      decoder_context*, int x,int y, int log2BlkWidth, uint8_t flag);
uint8_t get_cu_skip_flag(const decoder_context*, int x,int y);

void          set_PartMode(      decoder_context*, int x,int y, enum PartMode);
enum PartMode get_PartMode(const decoder_context*, int x,int y);

void    set_intra_split_flag(decoder_context*, int x,int y, uint8_t flag);
uint8_t get_intra_split_flag(decoder_context*, int x,int y);

// indicate that CB with log2CbSize is split
void    set_cu_split_flag(decoder_context*, int x,int y,int log2CbSize);
uint8_t get_cu_split_flag(decoder_context*, int x,int y,int log2CbSize);

void          set_pred_mode(      decoder_context*, int x,int y, int log2BlkWidth, enum PredMode mode);
enum PredMode get_pred_mode(const decoder_context*, int x,int y);
enum PredMode get_img_pred_mode(const decoder_context* ctx,
                                const de265_image* img, int x,int y);

enum IntraPredMode get_IntraPredMode(const decoder_context*, const de265_image*, int x,int y);

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

void set_nonzero_coefficient(decoder_context* ctx,int x0,int y0, int log2TrafoSize);
int  get_nonzero_coefficient(const decoder_context* ctx,int x0,int y0);

// TODO CHECK: should be sufficient to set value only in top left of CB
void set_rqt_root_cbf(decoder_context* ctx,int x0,int y0, int log2CbSize, int rqt_root_cbf);
int  get_rqt_root_cbf(const decoder_context* ctx,int x0,int y0);

void set_QPY(decoder_context* ctx,int x0,int y0, int QP_Y);
int  get_QPY(const decoder_context* ctx,int x0,int y0);

void get_image_plane(const decoder_context*, int cIdx, uint8_t** image, int* stride);

void set_CB_size(decoder_context*, int x0, int y0, int log2CbSize);
int  get_log2CbSize(const decoder_context* ctx, int x0, int y0);
void set_log2CbSize(decoder_context* ctx, int x0, int y0, int log2CbSize);

// coordinates in CB units
int  get_log2CbSize_cbUnits(const decoder_context* ctx, int x0, int y0);

void    set_deblk_flags(decoder_context*, int x0,int y0, uint8_t flags);
uint8_t get_deblk_flags(const decoder_context*, int x0,int y0);

void    set_deblk_bS(decoder_context*, int x0,int y0, uint8_t bS);
uint8_t get_deblk_bS(const decoder_context*, int x0,int y0);

void            set_sao_info(decoder_context*, int ctbX,int ctbY,const sao_info* sao_info);
const sao_info* get_sao_info(const decoder_context*, int ctbX,int ctbY);


const PredVectorInfo* get_mv_info(const decoder_context* ctx,int x,int y);
const PredVectorInfo* get_img_mv_info(const decoder_context* ctx,
                                      const de265_image* img, int x,int y);
void set_mv_info(decoder_context* ctx,int x,int y, int nPbW,int nPbH, const PredVectorInfo* mv);

void set_CTB_deblocking_cnt(decoder_context* ctx,int ctbX,int ctbY, int cnt);
uint8_t decrease_CTB_deblocking_cnt(decoder_context* ctx,int ctbX,int ctbY);

bool available_zscan(const decoder_context* ctx,
                     int xCurr,int yCurr, int xN,int yN);

bool available_pred_blk(const decoder_context* ctx,
                        int xC,int yC, int nCbS, int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN);

bool has_free_dpb_picture(const decoder_context* ctx);
void push_current_picture_to_output_queue(decoder_context* ctx);

// --- debug ---

LIBDE265_API void write_picture(const de265_image* img);
void draw_CB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_TB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_PB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_PB_pred_modes(const decoder_context* ctx, uint8_t* r, uint8_t* g, uint8_t* b, int stride);
void draw_intra_pred_modes(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);

#endif
