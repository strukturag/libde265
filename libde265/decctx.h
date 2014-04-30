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
#include "libde265/dpb.h"
#include "libde265/threads.h"
#include "libde265/acceleration.h"
#include "libde265/nal-parser.h"

#define DE265_MAX_VPS_SETS 16
#define DE265_MAX_SPS_SETS 16
#define DE265_MAX_PPS_SETS 64
#define DE265_MAX_SLICES   512  // TODO: make this dynamic
#define DE265_IMAGE_OUTPUT_QUEUE_LEN 2

#define MAX_WARNINGS 20


struct slice_segment_header;

typedef struct thread_context
{
  uint8_t inUse;  // thread_context is used for the current decoding process

  int CtbAddrInRS;
  int CtbAddrInTS;

  int CtbX, CtbY;


  // motion vectors

  int8_t  refIdx[2];
  int16_t mvd[2][2]; // only in top left position
  uint8_t merge_flag;
  uint8_t merge_idx;
  uint8_t mvp_lX_flag[2];
  uint8_t inter_pred_idc; // enum InterPredIdc


  // prediction

  enum IntraPredMode IntraPredModeC; // chroma intra-prediction mode for current CB


  // residual data

  uint8_t cu_transquant_bypass_flag;
  uint8_t transform_skip_flag[3];

  ALIGNED_16(int16_t) _coeffBuf[(32*32)+8]; // alignment required for SSE code !
  int16_t *coeffBuf;

  int16_t coeffList[3][32*32];
  int16_t coeffPos[3][32*32];
  int16_t nCoeff[3];


  // quantization

  int IsCuQpDeltaCoded;
  int CuQpDelta;

  int currentQPY;
  int currentQG_x, currentQG_y;
  int lastQPYinPreviousQG;

  int qPYPrime, qPCbPrime, qPCrPrime;

  CABAC_decoder cabac_decoder;

  context_model ctx_model[CONTEXT_MODEL_TABLE_LENGTH];

  struct decoder_context* decctx;
  struct slice_segment_header* shdr;
} thread_context;



class error_queue
{
 public:
  error_queue();

  void add_warning(de265_error warning, bool once);
  de265_error get_warning();

 private:
  de265_error warnings[MAX_WARNINGS];
  int nWarnings;
  de265_error warnings_shown[MAX_WARNINGS]; // warnings that have already occurred
  int nWarningsShown;
};



struct decoder_context : public error_queue {

  // --- parameters ---

  bool param_sei_check_hash;
  int  param_HighestTid;
  bool param_conceal_stream_errors;
  bool param_suppress_faulty_pictures;

  int  param_sps_headers_fd;
  int  param_vps_headers_fd;
  int  param_pps_headers_fd;
  int  param_slice_headers_fd;


  // --- decoder administration ---

  struct acceleration_functions acceleration; // CPU optimized functions


  // --- input stream data ---

  NAL_Parser nal_parser;


  // --- internal data ---

  video_parameter_set  vps[ DE265_MAX_VPS_SETS ];
  seq_parameter_set    sps[ DE265_MAX_SPS_SETS ];
  pic_parameter_set    pps[ DE265_MAX_PPS_SETS ];
  slice_segment_header slice[ DE265_MAX_SLICES ];

  video_parameter_set* current_vps;
  seq_parameter_set*   current_sps;
  pic_parameter_set*   current_pps;

  struct thread_pool thread_pool;
  int num_worker_threads;


  // --- sequence level ---

  int HighestTid;


  // --- decoded picture buffer ---

  decoded_picture_buffer dpb;

  de265_image* last_decoded_image;

  int current_image_poc_lsb;
  bool first_decoded_picture;
  bool NoRaslOutputFlag;
  bool HandleCraAsBlaFlag;
  bool FirstAfterEndOfSequenceNAL;

  int  PicOrderCntMsb;
  int prevPicOrderCntLsb;  // at precTid0Pic
  int prevPicOrderCntMsb;  // at precTid0Pic

  de265_image* img;


  // --- motion compensation ---

  int PocLsbLt[MAX_NUM_REF_PICS];
  int UsedByCurrPicLt[MAX_NUM_REF_PICS];
  int DeltaPocMsbCycleLt[MAX_NUM_REF_PICS];
  int CurrDeltaPocMsbPresentFlag[MAX_NUM_REF_PICS];
  int FollDeltaPocMsbPresentFlag[MAX_NUM_REF_PICS];

  // The number of entries in the lists below.
  int NumPocStCurrBefore;
  int NumPocStCurrAfter;
  int NumPocStFoll;
  int NumPocLtCurr;
  int NumPocLtFoll;

  // These lists contain absolute POC values.
  int PocStCurrBefore[MAX_NUM_REF_PICS]; // used for reference in current picture, smaller POC
  int PocStCurrAfter[MAX_NUM_REF_PICS];  // used for reference in current picture, larger POC
  int PocStFoll[MAX_NUM_REF_PICS]; // not used for reference in current picture, but in future picture
  int PocLtCurr[MAX_NUM_REF_PICS]; // used in current picture
  int PocLtFoll[MAX_NUM_REF_PICS]; // used in some future picture

  // These lists contain indices into the DPB.
  int RefPicSetStCurrBefore[DE265_DPB_SIZE];
  int RefPicSetStCurrAfter[DE265_DPB_SIZE];
  int RefPicSetStFoll[DE265_DPB_SIZE];
  int RefPicSetLtCurr[DE265_DPB_SIZE];
  int RefPicSetLtFoll[DE265_DPB_SIZE];


  // --- parameters derived from parameter sets ---

  // NAL

  uint8_t nal_unit_type;

  char IdrPicFlag;
  char RapPicFlag;


  // --- decoder runtime data ---

  struct thread_context thread_context[MAX_THREAD_CONTEXTS];
};


void init_decoder_context(decoder_context*);
void set_acceleration_functions(decoder_context* ctx, enum de265_acceleration);
void reset_decoder_context_for_new_picture(decoder_context* ctx);
void free_decoder_context(decoder_context*);


void cleanup_image(decoder_context* ctx, de265_image* img);


seq_parameter_set* get_sps(decoder_context* ctx, int id);

void process_nal_hdr(decoder_context*, nal_header*);
void process_vps(decoder_context*, video_parameter_set*);
void process_sps(decoder_context*, seq_parameter_set*);
void process_pps(decoder_context*, pic_parameter_set*);
bool process_slice_segment_header(decoder_context*, slice_segment_header*,
                                  de265_error*, de265_PTS pts, void* user_data);

int get_next_slice_index(decoder_context* ctx);
int get_next_thread_context_index(decoder_context* ctx);


// --- decoder 2D data arrays ---
// All coordinates are in pixels if not stated otherwise.

void debug_dump_cb_info(const decoder_context*);

LIBDE265_INLINE static slice_segment_header* get_SliceHeader(decoder_context* ctx, int x, int y)
{
  return &ctx->slice[ get_SliceHeaderIndex(ctx->img, x,y) ];
}

LIBDE265_INLINE static slice_segment_header* get_SliceHeaderCtb(decoder_context* ctx, int ctbX, int ctbY)
{
  return &ctx->slice[ ctx->img->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex ];
}


// TODO: move to some utility file
bool available_zscan(const de265_image* ctx,
                     int xCurr,int yCurr, int xN,int yN);

bool available_pred_blk(const decoder_context* ctx,
                        int xC,int yC, int nCbS, int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN);

void push_current_picture_to_output_queue(decoder_context* ctx);

// --- debug ---

extern "C" {
LIBDE265_API void set_output_filename(const char* filename);
LIBDE265_API void write_picture(const de265_image* img);
}

void write_picture_to_file(const de265_image* img, const char* filename);

void draw_CB_grid(const de265_image* img, uint8_t* dst, int stride, uint32_t value, int pixelSize);
void draw_TB_grid(const de265_image* img, uint8_t* dst, int stride, uint32_t value, int pixelSize);
void draw_PB_grid(const de265_image* img, uint8_t* dst, int stride, uint32_t value, int pixelSize);
void draw_PB_pred_modes(const de265_image* img, uint8_t* dst, int stride, int pixelSize);
void draw_intra_pred_modes(const de265_image* img, uint8_t* dst, int stride, uint32_t value, int pixelSize);
void draw_QuantPY(const de265_image* img, uint8_t* dst, int stride, int pixelSize);
void draw_Motion(const de265_image* img, uint8_t* dst, int stride, int pixelSize);

#endif
