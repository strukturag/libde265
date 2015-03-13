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


#include "libde265/encoder/algo/cb-interpartmode.h"
#include "libde265/encoder/algo/coding-options.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>




enc_cb* Algo_CB_InterPartMode_Fixed::analyze(encoder_context* ectx,
                                             context_model_table& ctxModel,
                                             enc_cb* cb)
{
  const int x = cb->x;
  const int y = cb->y;

  enum PartMode partMode = mParams.partMode();

  cb->PartMode = partMode;
  ectx->img->set_PartMode(x,y, partMode);

  const int log2CbSize = cb->log2Size;

  cb = mChildAlgo->analyze(ectx, ctxModel, cb);

  return cb;
}
