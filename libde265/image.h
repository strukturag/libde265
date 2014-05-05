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

#ifndef DE265_IMAGE_H
#define DE265_IMAGE_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#endif
#include "libde265/de265.h"
#include "libde265/sps.h"
#include "libde265/pps.h"
#include "libde265/motion.h"
#include "libde265/threads.h"
#include "libde265/slice.h"
#include "libde265/nal.h"


enum PictureState {
  UnusedForReference,
  UsedForShortTermReference,
  UsedForLongTermReference
};


/* TODO:
   At INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE images, we can check the SEI hash, whether
   the output image is correct despite the faulty reference, and set the state back to correct.
*/
#define INTEGRITY_CORRECT 0
#define INTEGRITY_UNAVAILABLE_REFERENCE 1
#define INTEGRITY_NOT_DECODED 2
#define INTEGRITY_DECODING_ERRORS 3
#define INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE 4

#define SEI_HASH_UNCHECKED 0
#define SEI_HASH_CORRECT   1
#define SEI_HASH_INCORRECT 2

#define TU_FLAG_NONZERO_COEFF  (1<<7)
#define TU_FLAG_SPLIT_TRANSFORM_MASK  0x1F

#define DEBLOCK_FLAG_VERTI (1<<4)
#define DEBLOCK_FLAG_HORIZ (1<<5)
#define DEBLOCK_PB_EDGE_VERTI (1<<6)
#define DEBLOCK_PB_EDGE_HORIZ (1<<7)
#define DEBLOCK_BS_MASK     0x03


#define CTB_PROGRESS_NONE      0
#define CTB_PROGRESS_PREFILTER 1
#define CTB_PROGRESS_FILTERED  2

template <class DataUnit> class MetaDataArray
{
 public:
  MetaDataArray() { data=NULL; data_size=0; log2unitSize=0; width_in_units=0; height_in_units=0; }
  ~MetaDataArray() { free(data); }

  bool alloc(int w,int h, int _log2unitSize) {
    int size = w*h;

    if (size != data_size) {
      free(data);
      data = (DataUnit*)malloc(size * sizeof(DataUnit));
      data_size = size;
      width_in_units = w;
      height_in_units = h;
    }

    log2unitSize = _log2unitSize;

    return data != NULL;
  }

  void clear() {
    if (data) memset(data, 0, sizeof(DataUnit) * data_size);
  }

  const DataUnit& get(int x,int y) const {
    int unitX = x>>log2unitSize;
    int unitY = y>>log2unitSize;

    return data[ unitX + unitY*width_in_units ];
  }

  DataUnit& get(int x,int y) {
    int unitX = x>>log2unitSize;
    int unitY = y>>log2unitSize;

    return data[ unitX + unitY*width_in_units ];
  }

  void set(int x,int y, const DataUnit& d) {
    int unitX = x>>log2unitSize;
    int unitY = y>>log2unitSize;

    data[ unitX + unitY*width_in_units ] = d;
  }

  DataUnit& operator[](int idx) { return data[idx]; }
  const DataUnit& operator[](int idx) const { return data[idx]; }

  // private:
  DataUnit* data;
  int data_size;
  int log2unitSize;
  int width_in_units;
  int height_in_units;
};

#define SET_CB_BLK(x,y,log2BlkWidth,  Field,value)              \
  int cbX = x >> cb_info.log2unitSize; \
  int cbY = y >> cb_info.log2unitSize; \
  int width = 1 << (log2BlkWidth - cb_info.log2unitSize);           \
  for (int cby=cbY;cby<cbY+width;cby++)                             \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                           \
      {                                                             \
        cb_info[ cbx + cby*cb_info.width_in_units ].Field = value;  \
      }


typedef struct {
  uint16_t SliceAddrRS;
  uint16_t SliceHeaderIndex; // index into array to slice header for this CTB

  sao_info saoInfo;

  uint16_t thread_context_id; // which thread-context is used to decode this CTB
} CTB_info;


typedef struct {
  uint8_t log2CbSize : 3;   // [0;6] (1<<log2CbSize) = 64
  uint8_t PartMode : 3;     // (enum PartMode)  [0;7] set only in top-left of CB
                            // TODO: could be removed if prediction-block-boundaries would be
                            // set during decoding
  uint8_t ctDepth : 2;      // [0:3]? (0:64, 1:32, 2:16, 3:8)
  uint8_t PredMode : 2;     // (enum PredMode)  [0;2] must be saved for past images
  uint8_t pcm_flag : 1;     //
  uint8_t cu_transquant_bypass : 1;

  int8_t  QP_Y;

  // uint8_t pcm_flag;  // TODO
} CB_ref_info;


typedef struct {
  PredVectorInfo mvi; // TODO: this can be done in 16x16 grid
} PB_ref_info;


/*
typedef struct {
  //uint16_t cbf_cb;   // bitfield (1<<depth)
  //uint16_t cbf_cr;   // bitfield (1<<depth)
  //uint16_t cbf_luma; // bitfield (1<<depth)

  //uint8_t IntraPredMode;  // NOTE: can be thread-local // (enum IntraPredMode)
  //uint8_t IntraPredModeC; // NOTE: can be thread-local // (enum IntraPredMode)

  //uint8_t split_transform_flag;  // NOTE: can be local if deblocking flags set during decoding
  //uint8_t transform_skip_flag;   // NOTE: can be in local context    // read bit (1<<cIdx)
  //uint8_t flags;                 // NOTE: can be removed if deblocking flags set during decoding (nonzero coefficients)
} TU_log_info;
*/


struct de265_image {
  de265_image();
  ~de265_image();


  de265_error alloc_image(int w,int h, enum de265_chroma c, const seq_parameter_set* sps);
  void fill_image(int y,int u,int v);


  uint8_t* y;   // pointer to pixels in the conformance window
  uint8_t* cb;
  uint8_t* cr;

private:
  uint8_t* y_mem;  // usually, you don't use these, but the pointers above
  uint8_t* cb_mem;
  uint8_t* cr_mem;

public:
  enum de265_chroma chroma_format;

  int width, height;  // size in luma pixels
  int chroma_width, chroma_height;
  int stride, chroma_stride;

  int border;


  // --- conformance cropping window ---

  uint8_t* y_confwin;
  uint8_t* cb_confwin;
  uint8_t* cr_confwin;

  int width_confwin, height_confwin;
  int chroma_width_confwin, chroma_height_confwin;


  // --- decoding info ---

  // If PicOutputFlag==false && PicState==UnusedForReference, image buffer is free.

  int  picture_order_cnt_lsb;
  int  PicOrderCntVal;
  bool PicOutputFlag;
  enum PictureState PicState;


  seq_parameter_set* sps;  // the SPS used for decoding this image
  pic_parameter_set* pps;  // the PPS used for decoding this image


  MetaDataArray<CTB_info>    ctb_info;
  MetaDataArray<CB_ref_info> cb_info;
  MetaDataArray<PB_ref_info> pb_info;
  MetaDataArray<uint8_t>     intraPredMode;
  MetaDataArray<uint8_t>     tu_info;
  MetaDataArray<uint8_t>     deblk_info;

  // TODO CHECK: should this move to slice header? Can this be different for each slice in the image?

  // --- meta information ---

  de265_PTS pts;
  void*     user_data;

  uint8_t integrity; /* Whether an error occured while the image was decoded.
                        When generated, this is initialized to INTEGRITY_CORRECT,
                        and changed on decoding errors.
                      */
  uint8_t sei_hash_check_result;

  nal_header nal_hdr;

  // --- multi core ---

  de265_progress_lock* ctb_progress; // ctb_info_size

  ALIGNED_8(de265_sync_int tasks_pending); // number of tasks pending to complete decoding
  de265_mutex mutex;
  de265_cond  finished_cond;

  // --- CB metadata access ---

  void set_pred_mode(int x,int y, int log2BlkWidth, enum PredMode mode)
  {
    SET_CB_BLK(x,y,log2BlkWidth, PredMode, mode);
  }

  enum PredMode get_pred_mode(int x,int y) const
  {
    return (enum PredMode)cb_info.get(x,y).PredMode;
  }

  uint8_t get_cu_skip_flag(int x,int y) const
  {
    return get_pred_mode(x,y)==MODE_SKIP;
  }

  void set_pcm_flag(int x,int y, int log2BlkWidth)
  {
    SET_CB_BLK(x,y,log2BlkWidth, pcm_flag, 1);
  }

  int  get_pcm_flag(int x,int y) const
  {
    return cb_info.get(x,y).pcm_flag;
  }

  void set_cu_transquant_bypass(int x,int y, int log2BlkWidth)
  {
    SET_CB_BLK(x,y,log2BlkWidth, cu_transquant_bypass, 1);
  }

  int  get_cu_transquant_bypass(int x,int y) const
  {
    return cb_info.get(x,y).cu_transquant_bypass;
  }

  void set_log2CbSize(int x0, int y0, int log2CbSize)
  {
    cb_info.get(x0,y0).log2CbSize = log2CbSize;

    // assume that remaining cb_info blocks are initialized to zero
  }

  int  get_log2CbSize(int x0, int y0) const
  {
    return (enum PredMode)cb_info.get(x0,y0).log2CbSize;
  }

  // coordinates in CB units
  int  get_log2CbSize_cbUnits(int xCb, int yCb) const
  {
    return (enum PredMode)cb_info[ xCb + yCb*cb_info.width_in_units ].log2CbSize;
  }

  void set_PartMode(int x,int y, enum PartMode mode)
  {
    cb_info.get(x,y).PartMode = mode;
  }

  enum PartMode get_PartMode(int x,int y) const
  {
    return (enum PartMode)cb_info.get(x,y).PartMode;
  }

  void set_ctDepth(int x,int y, int log2BlkWidth, int depth)
  {
    SET_CB_BLK(x,y,log2BlkWidth, ctDepth, depth);
  }

  int get_ctDepth(int x,int y) const
  {
    return cb_info.get(x,y).ctDepth;
  }

  void set_QPY(int x,int y, int log2BlkWidth, int QP_Y)
  {
    SET_CB_BLK (x, y, log2BlkWidth, QP_Y, QP_Y);
  }

  int  get_QPY(int x0,int y0) const
  {
    return cb_info.get(x0,y0).QP_Y;
  }

  // --- TU metadata access ---

  void set_split_transform_flag(int x0,int y0,int trafoDepth)
  {
    tu_info.get(x0,y0) |= (1<<trafoDepth);
  }

  int  get_split_transform_flag(int x0,int y0,int trafoDepth) const
  {
    return (tu_info.get(x0,y0) & (1<<trafoDepth));
  }

  void set_nonzero_coefficient(int x,int y, int log2TrafoSize)
  {
    const int tuX = x >> tu_info.log2unitSize;
    const int tuY = y >> tu_info.log2unitSize;
    const int width = 1 << (log2TrafoSize - tu_info.log2unitSize);

    for (int tuy=tuY;tuy<tuY+width;tuy++)
      for (int tux=tuX;tux<tuX+width;tux++)
        {
          tu_info[ tux + tuy*tu_info.width_in_units ] |= TU_FLAG_NONZERO_COEFF;
        }
  }

  int  get_nonzero_coefficient(int x,int y) const
  {
    return tu_info.get(x,y) & TU_FLAG_NONZERO_COEFF;
  }


  // --- intraPredMode metadata access ---

  enum IntraPredMode get_IntraPredMode(int x,int y) const
  {
    return (enum IntraPredMode)intraPredMode.get(x,y);
  }

  void set_IntraPredMode(int PUidx,int log2blkSize, enum IntraPredMode mode)
  {
    int pbSize = 1<<(log2blkSize - intraPredMode.log2unitSize);
    
    for (int y=0;y<pbSize;y++)
      for (int x=0;x<pbSize;x++)
        intraPredMode[PUidx + x + y*intraPredMode.width_in_units] = mode;
  }


  // --- CTB metadata access ---

  // address of first CTB in slice
  void set_SliceAddrRS(int ctbX, int ctbY, int SliceAddrRS)
  {
    int idx = ctbX + ctbY*ctb_info.width_in_units;
    ctb_info[idx].SliceAddrRS = SliceAddrRS;
  }

  int  get_SliceAddrRS(int ctbX, int ctbY) const
  {
    return ctb_info[ctbX + ctbY*ctb_info.width_in_units].SliceAddrRS;
  }

  int  get_SliceAddrRS_atCtbRS(int ctbRS) const
  {
    return ctb_info[ctbRS].SliceAddrRS;
  }


  void set_SliceHeaderIndex(int x, int y, int SliceHeaderIndex)
  {
    ctb_info.get(x,y).SliceHeaderIndex = SliceHeaderIndex;
  }

  int  get_SliceHeaderIndex(int x, int y) const
  {
    return ctb_info.get(x,y).SliceHeaderIndex;
  }

  void set_sao_info(int ctbX,int ctbY,const sao_info* saoinfo)
  {
    sao_info* sao = &ctb_info[ctbX + ctbY*ctb_info.width_in_units].saoInfo;
    
    memcpy(sao,
           saoinfo,
           sizeof(sao_info));
  }
  
  const sao_info* get_sao_info(int ctbX,int ctbY) const
  {
    return &ctb_info[ctbX + ctbY*ctb_info.width_in_units].saoInfo;
  }



  // --- DEBLK metadata access ---

  void    set_deblk_flags(int x0,int y0, uint8_t flags)
  {
    const int xd = x0/4;
    const int yd = y0/4;
    
    if (xd<deblk_info.width_in_units &&
        yd<deblk_info.height_in_units) {
      deblk_info[xd + yd*deblk_info.width_in_units] |= flags;
    }
  }

  uint8_t get_deblk_flags(int x0,int y0) const
  {
    const int xd = x0/4;
    const int yd = y0/4;

    return deblk_info[xd + yd*deblk_info.width_in_units];
  }

  void    set_deblk_bS(int x0,int y0, uint8_t bS)
  {
    uint8_t* data = &deblk_info[x0/4 + y0/4*deblk_info.width_in_units];
    *data &= ~DEBLOCK_BS_MASK;
    *data |= bS;
  }

  uint8_t get_deblk_bS(int x0,int y0) const
  {
    return deblk_info[x0/4 + y0/4*deblk_info.width_in_units] & DEBLOCK_BS_MASK;
  }


  // --- PB metadata access ---

  const PredVectorInfo* get_mv_info(int x,int y) const
  {
    return &pb_info.get(x,y).mvi;
  }

  void set_mv_info(int x,int y, int nPbW,int nPbH, const PredVectorInfo* mv);

// --- value logging ---

};


void de265_copy_image(de265_image* dest, const de265_image* src);

LIBDE265_INLINE static void get_image_plane(const de265_image* img, int cIdx, uint8_t** image, int* stride)
{
  switch (cIdx) {
  case 0: *image = img->y;  if (stride) *stride = img->stride; break;
  case 1: *image = img->cb; if (stride) *stride = img->chroma_stride; break;
  case 2: *image = img->cr; if (stride) *stride = img->chroma_stride; break;
  default: *image = NULL; if (stride) *stride = 0; break;
  }
}
void set_conformance_window(de265_image* img, int left,int right,int top,int bottom);


void increase_pending_tasks(de265_image* img, int n);
void decrease_pending_tasks(de265_image* img, int n);
void wait_for_completion(de265_image* img);  // block until image is decoded by background threads


/* Clear all CTB/CB/PB decoding data of this image.
   All CTB's processing states are set to 'unprocessed'.
 */
void img_clear_decoding_data(de265_image*);


#endif
