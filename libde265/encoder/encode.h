/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: Dirk Farin <farin@struktur.de>
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
#include "libde265/alloc_pool.h"

struct encoder_context;
struct enc_cb;


class enc_node
{
 public:
  enc_node() { mReconstruction=NULL; }
  ~enc_node() { delete[] mReconstruction; }

  uint16_t x,y;
  uint8_t  log2Size;

  void save_reconstruction(const de265_image*);
  void restore_reconstruction(de265_image*);

 private:
  uint8_t* mReconstruction;
};


class enc_tb : public enc_node
{
 public:
  enc_tb();
  ~enc_tb();

  const enc_tb* parent;

  uint8_t split_transform_flag : 1;
  //uint8_t cbf_luma : 1;
  //uint8_t cbf_cb : 1;
  //uint8_t cbf_cr : 1;
  //uint8_t log2TbSize : 3;
  uint8_t TrafoDepth : 2;  // 2 bits enough ? (TODO)

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
                   const enc_cb* cb, int blkIdx=0) const;

  bool isZeroBlock() const { return cbf[0]==false && cbf[1]==false && cbf[2]==false; }

  void alloc_coeff_memory(int cIdx, int tbSize);

  static void* operator new(const size_t size) { return mMemPool.new_obj(size); }
  static void operator delete(void* obj) { mMemPool.delete_obj(obj); }

private:
  static alloc_pool mMemPool;

  void reconstruct_tb(acceleration_functions* accel,
                      de265_image* img, int x0,int y0, int log2TbSize,
                      const enc_cb* cb, int cIdx) const;
};


struct enc_pb_inter
{
  enum PredMode PredMode;
};


class enc_cb : public enc_node
{
public:
  enc_cb();
  ~enc_cb();

  const enc_cb* parent;

  uint8_t split_cu_flag;
  //uint8_t log2CbSize;
  uint8_t ctDepth;

  //uint16_t x,y;

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
      uint8_t qp;

      enum PredMode PredMode;
      enum PartMode PartMode;

      union {
        struct {
          enum IntraPredMode pred_mode[4];
          enum IntraPredMode chroma_mode;
        } intra;

        struct {
          bool    skip_flag;
          uint8_t merge_index;
          enc_pb_inter* inter_pb[4];
        } inter;
      };

      const enc_tb* transform_tree;
    };
  };



  float distortion;
  float rate;

  void write_to_image(de265_image*, int x,int y,bool intraSlice) const;

  void reconstruct(acceleration_functions* accel,de265_image* img,
                   int x0,int y0) const;
  void will_overwrite_reconstruction(); // inform the CB that its reconstruction will be overwritten

  static void* operator new(const size_t size) { return mMemPool.new_obj(size); }
  static void operator delete(void* obj) { mMemPool.delete_obj(obj); }

 private:
  static alloc_pool mMemPool;

  mutable bool reconstructed; // whether the content of this CB is currently reconstructed
};



inline int childX(int x0, int idx, int log2CbSize)
{
  return x0 + ((idx&1) << (log2CbSize-1));
}

inline int childY(int y0, int idx, int log2CbSize)
{
  return y0 + ((idx>>1) << (log2CbSize-1));
}



struct encoder_context;

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
