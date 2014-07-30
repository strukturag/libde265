/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
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

#ifndef ENCODE_H
#define ENCODE_H

#include "libde265/image.h"
#include "libde265/decctx.h"
#include "libde265/image-io.h"


struct encoder_context;
struct enc_cb;


enum RateControlMethod
  {
    RateControlMethod_ConstantQP
  };

enum IntraPredSearch
  {
    IntraPredSearch_Complete
  };


struct encoder_params
{
  // input

  int first_frame;
  int max_number_of_frames;

  const char* input_yuv;
  int input_width;
  int input_height;


  // output

  const char* output_filename;


  // debug

  const char* reconstruction_yuv;


  // CB quad-tree

  int min_cb_size;
  int max_cb_size;

  int min_tb_size;
  int max_tb_size;

  int max_transform_hierarchy_depth_intra;


  // intra-prediction

  enum IntraPredSearch intraPredSearch;


  // rate-control

  enum RateControlMethod rateControlMethod;

  int constant_QP;
};



struct enc_tb
{
  const enc_tb* parent;

  uint8_t split_transform_flag : 1;
  //uint8_t cbf_luma : 1;
  //uint8_t cbf_cb : 1;
  //uint8_t cbf_cr : 1;
  uint8_t log2TbSize : 3;

  uint8_t cbf[3];

  union {
    // split
    struct {
      const enc_tb* children[4];
    };

    // non-split
    struct {
      int16_t* coeff[3];
    };
  };

  float distortion;  // total distortion for this level of the TB tree (including all children)
  float rate;        // total rate for coding this TB level and all children

  void set_cbf_flags_from_children();

  void reconstruct(acceleration_functions* accel,
                   de265_image* img, int x0,int y0, int xBase,int yBase,
                   const enc_cb* cb, int qp, int blkIdx=0) const;

private:
  void reconstruct_tb(acceleration_functions* accel,
                      de265_image* img, int x0,int y0, int log2TbSize,
                      const enc_cb* cb, int qp, int cIdx) const;
};


struct enc_pb_intra
{
  // context

  //uint8_t* border_pixels;
  //enum IntraPredMode pred_mode_left_cand;
  //enum IntraPredMode pred_mode_top_cand;
  //uint8_t  has_left_cand : 1;
  //uint8_t  has_top_cand  : 1;

  // coding mode

  enum IntraPredMode pred_mode;
  enum IntraPredMode pred_mode_chroma;
};


struct enc_pb_inter
{
  enum PredMode PredMode;
};


struct enc_cb
{
  uint8_t split_cu_flag;
  uint8_t log2CbSize;
  uint8_t ctDepth;

  union {
    // split
    struct {
      const enc_cb* children[4];   // undefined when split_cu_flag==false
    };

    // non-split
    struct {
      uint8_t cu_transquant_bypass_flag; // currently unused
      uint8_t pcm_flag;
      //uint8_t root_rqt_cbf;
      int qp; // TMP

      enum PredMode PredMode;
      enum PartMode PartMode;

      union {
        struct {
          enum IntraPredMode       pred_mode[4];
          enum IntraChromaPredMode chroma_mode;
        } intra;

        enc_pb_inter* inter_pb[4];
      };

      const enc_tb* transform_tree;
    };
  };



  float distortion;
  float rate;

  void write_to_image(de265_image*, int x,int y,int log2blkSize, bool intraSlice) const;

  void reconstruct(acceleration_functions* accel,de265_image* img,
                   int x0,int y0, int qp) const;

  void do_intra_prediction(de265_image* img, int x0,int y0, int log2BlkSize) const;
};


template <class T> class alloc_pool
{
public:
  alloc_pool() {
    mBlkSize = BLKSIZE;
  }

  ~alloc_pool() {
    for (int i=0;i<mem.size();i++) {
      delete[] mem[i].data;
    }
  }


  void set_blk_size(int blkSize) { mBlkSize=blkSize; }

  void free_all() {
    for (int i=0;i<mem.size();i++) {
      mem[i].nUsed=0;
    }
  }

  T* get_new(int n=1) {
    if (mem.empty() || mem.back().nUsed + n > mem.back().size) {
      range r;
      r.data = new T[mBlkSize];
      r.size = mBlkSize;
      r.nUsed= 0;
      mem.push_back(r);
    }

    range& r = mem.back();

    assert(r.nUsed + n <= r.size);

    T* t = r.data + r.nUsed;
    r.nUsed += n;

    return t;
  }

 private:
  static const int BLKSIZE = 128;
  int mBlkSize;

  struct range {
    T* data;
    int size;
    int nUsed;
  };

  std::vector<range> mem;
};


struct encoder_context
{
  encoder_context() {
    img_source = NULL;
    reconstruction_sink = NULL;
    packet_sink = NULL;

    enc_coeff_pool.set_blk_size(64*64*20); // TODO: this a guess

    switch_to_CABAC_stream();
  }


  encoder_params params;

  ImageSource*   img_source;
  ImageSink*     reconstruction_sink;
  PacketSink*    packet_sink;

  error_queue errqueue;
  acceleration_functions accel;

  de265_image img;

  video_parameter_set  vps;
  seq_parameter_set    sps;
  pic_parameter_set    pps;
  slice_segment_header shdr;



  // --- poor man's garbage collector for CB/TB/PB/coeff data ---

  alloc_pool<enc_cb> enc_cb_pool;
  alloc_pool<enc_tb> enc_tb_pool;
  alloc_pool<enc_pb_intra> enc_pb_intra_pool;
  alloc_pool<int16_t>  enc_coeff_pool;

  void free_all_pools() {
    enc_cb_pool.free_all();
    enc_tb_pool.free_all();
    enc_pb_intra_pool.free_all();
    enc_coeff_pool.free_all();
  }



  // --- CABAC output and rate estimation ---

  CABAC_encoder*  cabac;      // currently active CABAC output (estim or bitstream)
  context_model*  ctx_model;  // currently active ctx models (estim or bitstream)

  // temporary CABAC rate estimator
  //CABAC_encoder_estim     cabac_estim;
  //context_model_table     ctx_model_estim;

  // CABAC bitstream writer
  CABAC_encoder_bitstream cabac_bitstream;
  context_model_table     ctx_model_bitstream;

  /*
  void switch_to_CABAC_estim(context_model_table model) {
    cabac_estim.reset();
    cabac      = &cabac_estim;
    ctx_model  = model;
  }
  */

  void switch_CABAC(context_model_table model, CABAC_encoder* cabac_impl) {
    cabac      = cabac_impl;
    ctx_model  = model;
  }

  void switch_to_CABAC_stream() {
    cabac     = &cabac_bitstream;
    ctx_model = ctx_model_bitstream;
  }

  void write_packet() {
    if (packet_sink) {
      packet_sink->send_packet( cabac_bitstream.data(), cabac_bitstream.size() );
      cabac->reset();
    }
  }
};


void encode_transform_tree(encoder_context* ectx, const enc_tb* tb, const enc_cb* cb,
                           int x0,int y0, int xBase,int yBase,
                           int log2TrafoSize, int trafoDepth, int blkIdx,
                           int MaxTrafoDepth, int IntraSplitFlag, bool recurse);

void encode_coding_unit(encoder_context* ectx,
                        const enc_cb* cb, int x0,int y0, int log2CbSize, bool recurse);

void encode_quadtree(encoder_context* ectx,
                     const enc_cb* cb, int x0,int y0, int log2CbSize, int ctDepth,
                     bool recurse);

void encode_ctb(encoder_context* ectx, enc_cb* cb, int ctbX,int ctbY);


class de265_encoder
{
 public:
  virtual ~de265_encoder() { }
};

#endif
