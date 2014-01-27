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
#define DE265_MAX_SLICES   1
#define DE265_IMAGE_OUTPUT_QUEUE_LEN 2

// TODO: check required value
#define DE265_DPB_OUTPUT_IMAGES  20
#define DE265_DPB_RESILIENCE_IMAGES 5
#define DE265_DPB_SIZE  (DE265_DPB_OUTPUT_IMAGES + DE265_DPB_RESILIENCE_IMAGES)

#define MAX_WARNINGS 20


// split_cu_flag             CB (MinCbSizeY)
// skip_flag                 CB
// pcm_flag                  CB
// prev_intra_luma_pred_flag CB
// rem_intra_luma_pred_mode  CB
// mpm_idx                   CB
// intra_chroma_pred_mode    CB


enum LowLevelImplementation {
  LOWLEVEL_AUTO,
  LOWLEVEL_SSE,
  LOWLEVEL_AVX
};


typedef struct decoder_context {

  // --- parameters ---

  bool param_sei_check_hash;
  int  param_HighestTid;
  bool param_conceal_stream_errors;


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

  int PocStCurrBefore[MAX_NUM_REF_PICS];
  int PocStCurrAfter[MAX_NUM_REF_PICS];
  int PocStFoll[MAX_NUM_REF_PICS];
  int PocLtCutt[MAX_NUM_REF_PICS];
  int PocLtFoll[MAX_NUM_REF_PICS];

  int RefPicSetStCurrBefore[DE265_DPB_SIZE];
  int RefPicSetStCurrAfter[DE265_DPB_SIZE];
  int RefPicSetStFoll[DE265_DPB_SIZE];
  int RefPicSetLtCurr[DE265_DPB_SIZE];
  int RefPicSetLtFoll[DE265_DPB_SIZE];


  // --- decoded image data --- TODO: all this should move into de265_image

  // de265_image coeff; // transform coefficients / TODO: don't use de265_image for this

  CTB_info* ctb_info; // in raster scan
  int ctb_info_size;

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
int initialize_new_DPB_image(decoder_context* ctx,const seq_parameter_set* sps);

seq_parameter_set* get_sps(decoder_context* ctx, int id);

void process_nal_hdr(decoder_context*, nal_header*);
void process_vps(decoder_context*, video_parameter_set*);
void process_sps(decoder_context*, seq_parameter_set*);
void process_pps(decoder_context*, pic_parameter_set*);
bool process_slice_segment_header(decoder_context*, slice_segment_header*,
                                  de265_error*);

int get_next_slice_index(decoder_context* ctx);

void add_warning(decoder_context* ctx, de265_error warning, bool once);
de265_error get_warning(decoder_context* ctx);

// TODO void free_currently_unused_memory(decoder_context* ctx); // system is low on memory, free some (e.g. unused images in the DPB)


// --- decoder 2D data arrays ---
// All coordinates are in pixels if not stated otherwise.

void debug_dump_cb_info(const decoder_context*);

slice_segment_header* get_SliceHeader(decoder_context*, int x, int y);
slice_segment_header* get_SliceHeaderCtb(decoder_context* ctx, int ctbX, int ctbY);


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

bool has_free_dpb_picture(const decoder_context* ctx, bool high_priority);
void push_current_picture_to_output_queue(decoder_context* ctx);

// --- debug ---

LIBDE265_API void write_picture(const de265_image* img);
void draw_CB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_TB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_PB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);
void draw_PB_pred_modes(const decoder_context* ctx, uint8_t* r, uint8_t* g, uint8_t* b, int stride);
void draw_intra_pred_modes(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value);

#endif
