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
#include "libde265/sei.h"
#include "libde265/threads.h"
#include "libde265/acceleration.h"
#include "libde265/nal-parser.h"

#define DE265_MAX_VPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_SPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_PPS_SETS 64   // this is the maximum as defined in the standard
#define MAX_THREAD_CONTEXTS 68  // enough for 4K @ 32 pixel CTBs, but TODO: make this dynamic

#define MAX_WARNINGS 20


struct slice_segment_header;

struct thread_context
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
  struct de265_image *img;
  struct slice_segment_header* shdr;
};



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



struct slice_unit
{
  slice_unit(decoder_context* decctx)
  : ctx(decctx), nal(NULL), shdr(NULL), decoding_finished(false), flush_reorder_buffer(false) { }
  ~slice_unit();

  NAL_unit* nal;   // we are the owner
  slice_segment_header* shdr;  // not the owner
  bitreader reader;

  bool flush_reorder_buffer;

  bool decoding_finished;  // TODO: do we actually need this ? we could just remove the unit after decoding

private:
  decoder_context* ctx;
};


struct image_unit
{
  image_unit() { img=NULL; role=Invalid; }
  ~image_unit() { } // TODO

  de265_image* img;

  std::vector<slice_unit*> slice_units;
  std::vector<sei_message> suffix_SEIs;

  enum { Invalid, // headers not read yet
         Unknown, // SPS/PPS available
         Reference, // will be used as reference
         Leaf       // not a reference picture
  } role;

  std::vector<thread_context*> thread_contexts;
};



class decoder_context : public error_queue {
 public:
  decoder_context();
  ~decoder_context();

  de265_error start_thread_pool(int nThreads);
  void        stop_thread_pool();

  void reset();

  /* */ seq_parameter_set* get_sps(int id)       { return &sps[id]; }
  const seq_parameter_set* get_sps(int id) const { return &sps[id]; }
  /* */ pic_parameter_set* get_pps(int id)       { return &pps[id]; }
  const pic_parameter_set* get_pps(int id) const { return &pps[id]; }

  /*
  const slice_segment_header* get_SliceHeader_atCtb(int ctb) {
    return img->slices[img->get_SliceHeaderIndex_atIndex(ctb)];
  }
  */

  uint8_t get_nal_unit_type() const { return nal_unit_type; }
  bool    get_RapPicFlag() const { return RapPicFlag; }

  de265_error decode_NAL(NAL_unit* nal);
  de265_error decode_NAL_OLD(NAL_unit* nal);

  de265_error decode(int* more);
  de265_error decode_some();

  de265_error decode_slice_unit_sequential(image_unit* imgunit, slice_unit* sliceunit);

  void process_nal_hdr(nal_header*);
  void process_vps(video_parameter_set*);
  void process_sps(seq_parameter_set*);
  void process_pps(pic_parameter_set*);

  bool process_slice_segment_header(decoder_context*, slice_segment_header*,
                                    de265_error*, de265_PTS pts, void* user_data);

  int get_next_thread_context_index(decoder_context* ctx);

  //void push_current_picture_to_output_queue();
  de265_error push_picture_to_output_queue(image_unit*);


  // --- parameters ---

  bool param_sei_check_hash;
  int  param_HighestTid;
  bool param_conceal_stream_errors;
  bool param_suppress_faulty_pictures;

  int  param_sps_headers_fd;
  int  param_vps_headers_fd;
  int  param_pps_headers_fd;
  int  param_slice_headers_fd;

  void set_image_allocation_functions(de265_image_allocation* allocfunc) {
    param_image_allocation_functions = *allocfunc; }

  de265_image_allocation param_image_allocation_functions;


  // --- accelerated DSP functions ---

  void set_acceleration_functions(enum de265_acceleration);

  struct acceleration_functions acceleration; // CPU optimized functions


  // --- input stream data ---

  NAL_Parser nal_parser;


  int get_num_worker_threads() const { return num_worker_threads; }

  /* */ de265_image* get_image(int dpb_index)       { return dpb.get_image(dpb_index); }
  const de265_image* get_image(int dpb_index) const { return dpb.get_image(dpb_index); }

  de265_image* get_next_picture_in_output_queue() { return dpb.get_next_picture_in_output_queue(); }
  int          num_pictures_in_output_queue() const { return dpb.num_pictures_in_output_queue(); }
  void         pop_next_picture_in_output_queue() { dpb.pop_next_picture_in_output_queue(); }

 private:
  de265_error read_vps_NAL(bitreader&);
  de265_error read_sps_NAL(bitreader&);
  de265_error read_pps_NAL(bitreader&);
  de265_error read_sei_NAL(bitreader& reader, bool suffix);
  de265_error read_eos_NAL(bitreader& reader);
  de265_error read_slice_NAL(bitreader&, NAL_unit* nal, nal_header& nal_hdr);

 private:
  // --- internal data ---

  video_parameter_set  vps[ DE265_MAX_VPS_SETS ];
  seq_parameter_set    sps[ DE265_MAX_SPS_SETS ];
  pic_parameter_set    pps[ DE265_MAX_PPS_SETS ];

  video_parameter_set* current_vps;
  seq_parameter_set*   current_sps;
  pic_parameter_set*   current_pps;

 public:
  struct thread_pool thread_pool;

 private:
  int num_worker_threads;


  // --- sequence level ---

  int HighestTid;
  int current_HighestTid;

  // --- decoded picture buffer ---

  decoded_picture_buffer dpb;

  int current_image_poc_lsb;
  bool first_decoded_picture;
  bool NoRaslOutputFlag;
  bool HandleCraAsBlaFlag;
  bool FirstAfterEndOfSequenceNAL;

  int  PicOrderCntMsb;
  int prevPicOrderCntLsb;  // at precTid0Pic
  int prevPicOrderCntMsb;  // at precTid0Pic

  de265_image* img;

 public:
  const slice_segment_header* previous_slice_header; /* Remember the last slice for a successive
                                                        dependent slice. */


  // --- motion compensation ---

 public:
  int PocLsbLt[MAX_NUM_REF_PICS];
  int UsedByCurrPicLt[MAX_NUM_REF_PICS];
  int DeltaPocMsbCycleLt[MAX_NUM_REF_PICS];
 private:
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
  int RefPicSetStCurrBefore[MAX_NUM_REF_PICS];
  int RefPicSetStCurrAfter[MAX_NUM_REF_PICS];
  int RefPicSetStFoll[MAX_NUM_REF_PICS];
  int RefPicSetLtCurr[MAX_NUM_REF_PICS];
  int RefPicSetLtFoll[MAX_NUM_REF_PICS];


  // --- parameters derived from parameter sets ---

  // NAL

  uint8_t nal_unit_type;

  char IdrPicFlag;
  char RapPicFlag;


  // --- image unit queue ---

  std::vector<image_unit*> image_units;

  bool flush_reorder_buffer_at_this_frame;


  // --- decoder runtime data ---

 public:
  struct thread_context thread_contexts[MAX_THREAD_CONTEXTS];

 private:
  void init_thread_context(class thread_context* tctx);
  void add_task_decode_CTB_row(int thread_id, bool initCABAC);
  void add_task_decode_slice_segment(int thread_id);


  void process_picture_order_count(decoder_context* ctx, slice_segment_header* hdr);
  int generate_unavailable_reference_picture(decoder_context* ctx, const seq_parameter_set* sps,
                                             int POC, bool longTerm);
  void process_reference_picture_set(decoder_context* ctx, slice_segment_header* hdr);
  bool construct_reference_picture_lists(decoder_context* ctx, slice_segment_header* hdr);


  void remove_images_from_dpb(const std::vector<int>& removeImageList);
};


#endif
