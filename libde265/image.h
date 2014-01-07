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
#include "libde265/motion.h"


enum PictureState {
  UnusedForReference,
  UsedForShortTermReference,
  UsedForLongTermReference
};


typedef struct {
  uint8_t PredMode; // (enum PredMode)
} CB_ref_info;

typedef struct {
  PredVectorInfo mvi; // TODO: this can be done in 16x16 grid
} PB_ref_info;


typedef struct {
  //uint16_t cbf_cb;   // bitfield (1<<depth)
  //uint16_t cbf_cr;   // bitfield (1<<depth)
  //uint16_t cbf_luma; // bitfield (1<<depth)

  uint8_t IntraPredMode;  // NOTE: can be thread-local // (enum IntraPredMode)
  uint8_t IntraPredModeC; // NOTE: can be thread-local // (enum IntraPredMode)

  uint8_t split_transform_flag;  // NOTE: can be local if deblocking flags set during decoding
  uint8_t transform_skip_flag;   // NOTE: can be in local context    // read bit (1<<cIdx)
  uint8_t flags;                 // NOTE: can be removed if deblocking flags set during decoding (nonzero coefficients)
} TU_log_info;


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

  CB_ref_info* cb_info;
  int cb_info_size;

  PB_ref_info* pb_info;
  int pb_info_size;
  int pb_info_stride;

  int* pb_rootIdx;
  int  pb_info_nextRootIdx;

  uint8_t* intraPredMode; // sps->PicWidthInMinPUs * sps->PicHeightInMinPUs
  int intraPredModeSize;


  int RefPicList_POC[2][14+1];

} de265_image;


void de265_init_image (de265_image* img); // (optional) init variables, do not alloc image
void de265_alloc_image(de265_image* img, int w,int h, enum de265_chroma c,
                       const seq_parameter_set* sps);
void de265_free_image (de265_image* img);

void de265_fill_image(de265_image* img, int y,int u,int v);
void de265_copy_image(de265_image* dest, const de265_image* src);


// --- value logging ---

#endif
