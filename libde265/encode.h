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

struct enc_tb
{
  uint8_t split_transform_flag;
  uint8_t cbf[3];

  union {
    struct {
      enc_tb* children[4];
    } split;

    struct {
      int16_t* coeff[3];
    } leaf;
  };
};

struct enc_cb
{
  uint8_t split_cu_flag;

  union {
    struct {
      enc_cb* children[4];   // undefined when split_cu_flag==false
    } split;

    struct {
      uint8_t cu_transquant_bypass_flag; // currently unused
      uint8_t root_rqt_cbf;
      uint8_t pcm_flag;

      enum PredMode PredMode;
      enum PartMode PartMode;

      enum IntraPredMode       intra_luma_pred_mode;
      enum IntraChromaPredMode intra_chroma_pred_mode;

      enc_tb* transform_tree;
    } leaf;
  };
};


struct encoder_context
{
  de265_image* img;
  slice_segment_header* shdr;

  CABAC_encoder* cabac_encoder;

  context_model ctx_model[CONTEXT_MODEL_TABLE_LENGTH];
};


/* Image contains the input image with all its metadata and will be
   overwritten by the reconstructed image.
*/
void encode_image(encoder_context*);

#endif
