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
