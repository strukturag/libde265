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


#include "libde265/encoder/algo/cb-predmode.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>




enc_cb* Algo_CB_PredMode_BruteForce::analyze(encoder_context* ectx,
                                             context_model_table ctxModel,
                                             enc_cb* cb)
{
  assert(cb->pcm_flag==0);

  // if we try both variants, make a copy of the ctxModel and use the copy for splitting

  bool try_intra = true;
  bool try_inter = (ectx->shdr->slice_type != SLICE_TYPE_I);
  //try_intra = !try_inter;

  // 0: intra
  // 1: inter

  CodingOptions options(ectx);
  options.set_input(cb,ctxModel);
  options.activate_option(0, try_intra);
  options.activate_option(1, try_inter);


  enc_cb* cb_inter = NULL;
  enc_cb* cb_intra = NULL;

  const int log2CbSize = cb->log2Size;
  const int x = cb->x;
  const int y = cb->y;

  // try encoding with inter

  if (try_inter) {
    options.begin_reconstruction(1);
    cb_inter = options.get_cb(1);

    cb_inter->PredMode = MODE_INTER;
    ectx->img->set_pred_mode(x,y, log2CbSize, MODE_INTER);

    options.set_cb(1, mInterAlgo->analyze(ectx, options.get_context(1), cb_inter));

    options.end_reconstruction(1);
  }


  // try intra

  if (try_intra) {
    options.begin_reconstruction(0);
    cb_intra = options.get_cb(0);

    cb_intra->PredMode = MODE_INTRA;
    ectx->img->set_pred_mode(x,y, log2CbSize, MODE_INTRA);

    options.set_cb(0, mIntraAlgo->analyze(ectx, options.get_context(0), cb_intra));

    options.end_reconstruction(0);
  }


  options.compute_rdo_costs();
  return options.return_best_rdo();
}

