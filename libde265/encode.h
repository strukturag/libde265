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


  // intra-prediction

  enum IntraPredSearch intraPredSearch;


  // rate-control

  enum RateControlMethod rateControlMethod;

  int constant_QP;
};



struct enc_tb
{
  enc_tb* parent;

  uint8_t split_transform_flag : 1;
  uint8_t cbf_luma : 1;
  uint8_t cbf_cb : 1;
  uint8_t cbf_cr : 1;

  union {
    // split
    struct {
      enc_tb* children[4];
    };

    // non-split
    struct {
      int16_t* coeff[3];
    };
  };
};


struct enc_pb_intra
{
  // context

  uint8_t* border_pixels;
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

  union {
    // split
    struct {
      enc_cb* children[4];   // undefined when split_cu_flag==false
    };

    // non-split
    struct {
      uint8_t cu_transquant_bypass_flag; // currently unused
      uint8_t pcm_flag;
      //uint8_t root_rqt_cbf;

      enum PredMode PredMode;
      enum PartMode PartMode;

      union {
        enc_pb_intra* intra_pb[4];
        enc_pb_inter* inter_pb[4];
      };

      enc_tb* transform_tree;
    };
  };


  void write_to_image(de265_image*, int x,int y,int log2blkSize, bool intraSlice);
};


template <class T> class alloc_pool
{
public:
  ~alloc_pool() {
    for (int i=0;i<mem.size();i++) {
      delete[] mem[i].data;
    }
  }


  void free_all() {
    for (int i=0;i<mem.size();i++) {
      mem[i].nUsed=0;
    }
  }

  T* get_new(int nCBs=1) {
    if (mem.empty() || mem.back().nUsed + nCBs > mem.back().size) {
      range r;
      r.data = new T[BLKSIZE];
      r.size = BLKSIZE;
      r.nUsed= 0;
      mem.push_back(r);
    }

    range& r = mem.back();

    assert(r.nUsed + nCBs <= r.size);

    T* t = r.data + r.nUsed;
    r.nUsed += nCBs;

    return t;
  }

 private:
  static const int BLKSIZE = 128;

  struct range {
    T* data;
    int size;
    int nUsed;
  };

  std::vector<range> mem;
};


struct encoder_context
{
  de265_image* img;
  slice_segment_header* shdr;

  alloc_pool<enc_cb> enc_cb_pool;
  alloc_pool<enc_tb> enc_tb_pool;
  alloc_pool<enc_pb_intra> enc_pb_intra_pool;

  CABAC_encoder_bitstream* cabac_encoder;

  context_model_table ctx_model;
};


void encode_ctb(encoder_context* ectx, enc_cb* cb, int ctbX,int ctbY);

#endif
