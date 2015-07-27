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

#include <memory>

class encoder_context;
class enc_cb;


class small_image_buffer
{
 public:
  explicit small_image_buffer(int log2Size,int bytes_per_pixel=1);
  ~small_image_buffer();

  uint8_t*  get_buffer_u8() const { return mBuf; }
  int16_t*  get_buffer_s16() const { return (int16_t*)mBuf; }
  uint16_t* get_buffer_u16() const { return (uint16_t*)mBuf; }
  template <class pixel_t> pixel_t* get_buffer() const { return (pixel_t*)mBuf; }

  int get_stride() const { return mStride; }

  // small_image_buffer cannot be copied

  small_image_buffer(const small_image_buffer&) = delete;
  small_image_buffer& operator=(const small_image_buffer&) = delete;

 private:
  uint8_t* mBuf;
  int mStride;
};


class enc_node
{
 public:
  enc_node() { mReconstruction=NULL; }
 enc_node(int _x,int _y, int _log2Size)
   : x(_x), y(_y), log2Size(_log2Size), mReconstruction(NULL) { }
  virtual ~enc_node() { delete[] mReconstruction; }

  uint16_t x,y;
  uint8_t  log2Size : 3;

  virtual void save(const de265_image*); // deprecated
  virtual void restore(de265_image*);    // deprecated


  // reconstruction

  static const int METADATA_RECONSTRUCTION_BORDERS = (1<<0);
  static const int METADATA_RECONSTRUCTION = (1<<1);
  static const int METADATA_INTRA_MODES    = (1<<2);
  static const int METADATA_ALL            = 0xFF;

  struct rectangle {
    rectangle() { }
    rectangle(int l,int t,int r, int b) : left(l), top(t), right(r), bottom(b) { }

    int left,top,right,bottom;
  };

  rectangle get_rectangle() {
    rectangle r;
    r.left = x;
    r.top  = y;
    r.right  = x+(1<<log2Size);
    r.bottom = y+(1<<log2Size);
    return r;
  }

  rectangle get_rectangle(int size) {
    rectangle r;
    r.left = x;
    r.top  = y;
    r.right  = x+size;
    r.bottom = y+size;
    return r;
  }

 private:
  uint8_t* mReconstruction;
};


class enc_tb : public enc_node
{
 public:
  enc_tb(int x,int y,int log2TbSize, enc_cb* _cb);
  ~enc_tb();

  enc_tb* parent;
  enc_cb* cb;
  enc_tb** downPtr;

  uint8_t split_transform_flag : 1;
  uint8_t TrafoDepth : 2;  // 2 bits enough ? (TODO)
  uint8_t blkIdx : 2;

  enum IntraPredMode intra_mode;
  enum IntraPredMode intra_mode_chroma;

  uint8_t cbf[3];

  uint8_t metadata_in_image;

  std::shared_ptr<small_image_buffer> intra_prediction[3];
  std::shared_ptr<small_image_buffer> residual[3];
  //std::shared_ptr<small_image_buffer> reconstruction[3];

  union {
    // split
    struct {
      enc_tb* children[4];
    };

    // leaf node
    struct {
      int16_t* coeff[3];

      bool    skip_transform[3][2];
      uint8_t explicit_rdpcm[3][2];
    };
  };

  float distortion;  // total distortion for this level of the TB tree (including all children)
  float rate;        // total rate for coding this TB level and all children
  float rate_withoutCbfChroma;

  void set_cbf_flags_from_children();

  void reconstruct(encoder_context* ectx,
                   de265_image* img,
                   const enc_cb* cb, int blkIdx=0) const;

  bool isZeroBlock() const { return cbf[0]==false && cbf[1]==false && cbf[2]==false; }

  void alloc_coeff_memory(int cIdx, int tbSize);



  // === metadata ===

  // externally modified metadata
  // We call this before doing the actual modification. This allows to save e.g.
  // pixel data that has been reconstructed into the image to be copied into the TB
  // as it will most probably be needed later again.
  void willOverwriteMetadata(const de265_image* img, int whatFlags = METADATA_ALL) {
    // note: theoretically, this should be propagated upwards, but the way we use it,
    // this will never be needed.

    metadata_in_image &= ~whatFlags;
  }

  // externally wrote metadata
  void setHaveMetadata(int whatFlags) { metadata_in_image |= whatFlags; }

  int  writeMetadata(encoder_context* ectx, de265_image* img, int whatFlags);

  /*
       +--------------------------------+
       |////////////////////////////////|
       |//1-----------------------------+2
       |//|                             |
       |//|                             |
       |//|         rectangle           |
       |//|                             |
       |//|                             |
       +--+-----------------------------+
       .  3                              4
   */
  void writeSurroundingMetadata(encoder_context* ectx,
                                de265_image* img, int whatFlags, const rectangle& rect);

  // internal use only
  void writeSurroundingMetadataDown(encoder_context* ectx,
                                    de265_image* img, int whatFlags, const rectangle& rect);


  /*
  static void* operator new(const size_t size) { return mMemPool.new_obj(size); }
  static void operator delete(void* obj) { mMemPool.delete_obj(obj); }
  */

private:
  static alloc_pool mMemPool;

  void reconstruct_tb(encoder_context* ectx,
                      de265_image* img, int x0,int y0, int log2TbSize,
                      const enc_cb* cb, int cIdx) const;
};


struct enc_pb_inter
{
  /* absolute motion information (for MV-prediction candidates)
   */
  MotionVectorSpec motion;

  /* specification how to code the motion vector in the bitstream
   */
  motion_spec    spec;


  // NOT TRUE: refIdx in 'spec' is not used. It is taken from 'motion'
  // Currently, information is duplicated. Same as with inter_pred_idc/predFlag[].

  /* SPEC:
  int8_t  refIdx[2]; // not used
  int16_t mvd[2][2];

  uint8_t inter_pred_idc : 2; // enum InterPredIdc
  uint8_t mvp_l0_flag : 1;
  uint8_t mvp_l1_flag : 1;
  uint8_t merge_flag : 1;
  uint8_t merge_idx  : 3;
  */
};


class enc_cb : public enc_node
{
public:
  enc_cb();
  ~enc_cb();

  enc_cb* parent;
  enc_cb** downPtr;

  uint8_t split_cu_flag : 1;
  uint8_t ctDepth : 2;

  uint8_t metadata_in_image;

  union {
    // split
    struct {
      enc_cb* children[4];   // undefined when split_cu_flag==false
    };

    // non-split
    struct {
      uint8_t qp : 6;
      uint8_t cu_transquant_bypass_flag : 1; // currently unused
      uint8_t pcm_flag : 1;

      enum PredMode PredMode; // : 6;
      enum PartMode PartMode; // : 3;

      union {
        struct {
          //enum IntraPredMode pred_mode[4];
          //enum IntraPredMode chroma_mode;
        } intra;

        struct {
          enc_pb_inter pb[4];

          uint8_t rqt_root_cbf : 1;
        } inter;
      };

      enc_tb* transform_tree;
    };
  };


  float distortion;
  float rate;


  void set_rqt_root_bf_from_children_cbf();

  /* Save CB reconstruction in the node and restore it again to the image.
     Pixel data and metadata.
   */
  virtual void save(const de265_image*);
  virtual void restore(de265_image*);


  /* Decode this CB: pixel data and write metadata to image.
   */
  void reconstruct(encoder_context* ectx,de265_image* img) const;


  // ===== METADATA =====

  void willOverwriteMetadata(const de265_image* img, int whatFlags = METADATA_ALL) {
    // note: theoretically, this should be propagated upwards, but the way we use it,
    // this will never be needed.

    metadata_in_image &= ~whatFlags;
  }

  // externally wrote metadata
  void setHaveMetadata(int whatFlags) { metadata_in_image |= whatFlags; }

  /** Write CB-data and TB-data into the image metadata.
   */
  int writeMetadata(encoder_context* ectx, de265_image* img, int whatFlags);

  /** Write the CB-data into the image metadata. Do not write TB data.
      @return the metadata that has been written
  */
  int writeMetadata_CBOnly(encoder_context* ectx, de265_image* img, int whatFlags);

  void writeSurroundingMetadata(encoder_context* ectx,
                                de265_image* img, int whatFlags, const rectangle& rect);

  // internal use only
  void writeSurroundingMetadataDown(encoder_context* ectx,
                                    de265_image* img, int whatFlags, const rectangle& rect);


  // memory management

  static void* operator new(const size_t size) { return mMemPool.new_obj(size); }
  static void operator delete(void* obj) { mMemPool.delete_obj(obj); }

 private:
  void write_to_image(de265_image*) const;

  static alloc_pool mMemPool;
};



inline int childX(int x0, int idx, int log2CbSize)
{
  return x0 + ((idx&1) << (log2CbSize-1));
}

inline int childY(int y0, int idx, int log2CbSize)
{
  return y0 + ((idx>>1) << (log2CbSize-1));
}


#endif
