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

#ifndef DE265_IMAGE_H
#define DE265_IMAGE_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdint.h>
#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#endif
#include "libde265/de265.h"
#include "libde265/sps.h"
#include "libde265/pps.h"
#include "libde265/motion.h"
#include "libde265/threads.h"
#include "libde265/slice.h"


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


typedef struct {
  uint16_t SliceAddrRS;
  uint16_t SliceHeaderIndex; // index into array to slice header for this CTB

  sao_info saoInfo;

  de265_sync_int task_blocking_cnt; // for parallelization
} CTB_info;


typedef struct {
  uint8_t cu_skip_flag : 1; // only for decoding of current image
  uint8_t log2CbSize : 3;   // [0;6] (1<<log2CbSize) = 64
  uint8_t PartMode : 3;     // (enum PartMode)  [0;7] set only in top-left of CB
                            // TODO: could be removed if prediction-block-boundaries would be
                            // set during decoding
  uint8_t PredMode : 2;     // (enum PredMode)  [0;2] must be safed for past images
  uint8_t ctDepth : 2;      // [0:3]? (0:64, 1:32, 2:16, 3:8)

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


typedef struct de265_image {
  uint8_t* y;   // pointer to pixel at (0,0), which is inside the optional image borders
  uint8_t* cb;
  uint8_t* cr;

  uint8_t* y_mem;  // usually, you don't use these, but the pointers above
  uint8_t* cb_mem;
  uint8_t* cr_mem;

  enum de265_chroma chroma_format;

  int width, height;  // size in luma pixels
  int chroma_width, chroma_height;
  int stride, chroma_stride;

  int border;


  // --- decoding info ---

  // If PicOutputFlag==false && PicState==UnusedForReference, image buffer is free.

  int  picture_order_cnt_lsb;
  int  PicOrderCntVal;
  bool PicOutputFlag;
  enum PictureState PicState;

  CTB_info* ctb_info; // in raster scan
  int ctb_info_size;

  CB_ref_info* cb_info;
  int cb_info_size;

  PB_ref_info* pb_info;
  int pb_info_size;
  int pb_info_stride;

  int* pb_rootIdx;
  //int  pb_info_nextRootIdx;

  uint8_t* intraPredMode; // sps->PicWidthInMinPUs * sps->PicHeightInMinPUs
  int intraPredModeSize;

  uint8_t* tu_info;
  int tu_info_size;

  uint8_t* deblk_info;
  int deblk_info_size;
  int deblk_width;
  int deblk_height;

  int RefPicList_POC[2][14+1];

  // --- meta information ---

  uint8_t integrity; /* Whether an error occured while the image was decoded.
                        When generated, this is initialized to INTEGRITY_CORRECT,
                        and changed on decoding errors.
                      */
  uint8_t sei_hash_check_result;

  // --- multi core ---

  de265_sync_int tasks_pending; // number of tasks pending to complete decoding
  de265_mutex mutex;
  de265_cond  finished_cond;

} de265_image;


void de265_init_image (de265_image* img); // (optional) init variables, do not alloc image
de265_error de265_alloc_image(de265_image* img, int w,int h, enum de265_chroma c,
                              const seq_parameter_set* sps);
void de265_free_image (de265_image* img);

void de265_fill_image(de265_image* img, int y,int u,int v);
void de265_copy_image(de265_image* dest, const de265_image* src);

void get_image_plane(const de265_image*, int cIdx, uint8_t** image, int* stride);


void increase_pending_tasks(de265_image* img, int n);
void decrease_pending_tasks(de265_image* img, int n);
void wait_for_completion(de265_image* img);  // block until image is decoded by background threads


void prepare_image_for_decoding(de265_image*);

void    set_cu_skip_flag(const seq_parameter_set* sps, de265_image* img,
                         int x,int y, int log2BlkWidth, uint8_t flag);
uint8_t get_cu_skip_flag(const seq_parameter_set* sps, const de265_image* img, int x,int y);

void set_pred_mode(de265_image* img, const seq_parameter_set* sps,
                   int x,int y, int log2BlkWidth, enum PredMode mode);
enum PredMode get_pred_mode(const de265_image* img, const seq_parameter_set* sps, int x,int y);


void set_log2CbSize(de265_image* img, const seq_parameter_set* sps, int x0, int y0, int log2CbSize);
int  get_log2CbSize(const de265_image* img, const seq_parameter_set* sps, int x0, int y0);
int  get_log2CbSize_cbUnits(de265_image* img, const seq_parameter_set* sps, int xCb, int yCb);


void          set_PartMode(      de265_image*, const seq_parameter_set*, int x,int y, enum PartMode);
enum PartMode get_PartMode(const de265_image*, const seq_parameter_set*, int x,int y);


void set_ctDepth(de265_image*, const seq_parameter_set*, int x,int y, int log2BlkWidth, int depth);
int get_ctDepth(const de265_image*, const seq_parameter_set*, int x,int y);

void set_QPY(de265_image*, const seq_parameter_set*,
             const pic_parameter_set* pps, int x,int y, int QP_Y);
int  get_QPY(const de265_image*, const seq_parameter_set*,int x0,int y0);

void set_split_transform_flag(de265_image* img,const seq_parameter_set* sps,
                              int x0,int y0,int trafoDepth);
int  get_split_transform_flag(const de265_image* img, const seq_parameter_set* sps,
                              int x0,int y0,int trafoDepth);

void set_nonzero_coefficient(de265_image* img,const seq_parameter_set* sps,
                             int x,int y, int log2TrafoSize);

int  get_nonzero_coefficient(const de265_image* img,const seq_parameter_set* sps,
                             int x,int y);

enum IntraPredMode get_IntraPredMode(const de265_image* img, const seq_parameter_set* sps, int x,int y);


void    set_deblk_flags(de265_image* img, int x0,int y0, uint8_t flags);
uint8_t get_deblk_flags(const de265_image* img, int x0,int y0);

void    set_deblk_bS(de265_image* img, int x0,int y0, uint8_t bS);
uint8_t get_deblk_bS(const de265_image* img, int x0,int y0);


void set_SliceAddrRS(de265_image* img, const seq_parameter_set* sps,
                     int ctbX, int ctbY, int SliceAddrRS);
int  get_SliceAddrRS(const de265_image* img, const seq_parameter_set* sps, int ctbX, int ctbY);


void set_SliceHeaderIndex(de265_image* img, const seq_parameter_set* sps,
                          int x, int y, int SliceHeaderIndex);
int  get_SliceHeaderIndex(const de265_image* img, const seq_parameter_set* sps, int x, int y);

void set_sao_info(de265_image* img,const seq_parameter_set* sps,
                  int ctbX,int ctbY,const sao_info* saoinfo);
const sao_info* get_sao_info(const de265_image* img,const seq_parameter_set* sps, int ctbX,int ctbY);


void set_CTB_deblocking_cnt_new(de265_image* img,const seq_parameter_set* sps,int ctbX,int ctbY, int cnt);
uint8_t decrease_CTB_deblocking_cnt_new(de265_image* img,const seq_parameter_set* sps,int ctbX,int ctbY);


// --- value logging ---

#endif
