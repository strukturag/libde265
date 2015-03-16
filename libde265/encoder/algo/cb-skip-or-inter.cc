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


#include "libde265/encoder/algo/cb-skip-or-inter.h"
#include "libde265/encoder/algo/coding-options.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>




enc_cb* Algo_CB_SkipOrInter_BruteForce::analyze(encoder_context* ectx,
                                                context_model_table& ctxModel,
                                                enc_cb* cb)
{
  bool try_skip  = false;
  bool try_inter = true;

  CodingOptions options(ectx,cb,ctxModel);
  CodingOption option_skip  = options.new_option(try_skip);
  CodingOption option_inter = options.new_option(try_inter);
  options.start();

  if (option_skip) {
    CodingOption& opt = option_skip;
    opt.begin();

    enc_cb* cb = opt.get_cb();

    cb->PredMode = MODE_SKIP;
    ectx->img->set_pred_mode(cb->x,cb->y, cb->log2Size, cb->PredMode);

    opt.set_cb( mSkipAlgo->analyze(ectx, opt.get_context(), cb) );
    opt.end();
  }

  if (option_inter) {
    CodingOption& opt = option_inter;
    enc_cb* cb = opt.get_cb();

    opt.begin();
    opt.set_cb( mInterAlgo->analyze(ectx, opt.get_context(), cb) );
    opt.end();
  }

  options.compute_rdo_costs();
  return options.return_best_rdo();
}
