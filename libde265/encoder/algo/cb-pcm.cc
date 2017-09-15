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


#include "libde265/encoder/algo/cb-pcm.h"
#include "libde265/encoder/algo/coding-options.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>
#include <iostream>


#define ENCODER_DEVELOPMENT 1



enc_cb* Algo_CB_PCM::analyze(encoder_context* ectx,
                             context_model_table& ctxModel,
                             enc_cb* cb_in)
{
  enum ALGO_CB_PCM pcmMode = mParams.pcmMode();

  if (pcmMode == ALGO_CB_PCM_Never) {
    return mChildAlgo->analyze(ectx, ctxModel, cb_in);
  }


  const int log2CbSize = cb_in->log2Size;

  bool pcmPossible = ((1<<log2CbSize) >= mParams.minBlockSize() &&
                      (1<<log2CbSize) <= mParams.maxBlockSize());

  if (!pcmPossible) {
    return mChildAlgo->analyze(ectx, ctxModel, cb_in);
  }

  if (pcmMode == ALGO_CB_PCM_Always) {
    cb_in->pcm_flag = true;
    cb_in->PartMode = PART_2Nx2N;

    std::shared_ptr<const image> img = ectx->imgdata->input;

    const int x = cb_in->x;
    const int y = cb_in->y;

    cb_in->intra.pcm_data_ptr[0] = img->get_image_plane_at_pos(0, x,y);
    cb_in->intra.pcm_data_ptr[1] = img->get_image_plane_at_pos(1, x,y);
    cb_in->intra.pcm_data_ptr[2] = img->get_image_plane_at_pos(2, x,y);
  }

  return cb_in;
}
