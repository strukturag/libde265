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

#ifndef CB_PREDMODE_H
#define CB_PREDMODE_H

#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/encoder/encode.h"
#include "libde265/slice.h"
#include "libde265/scan.h"
#include "libde265/intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include "libde265/quality.h"
#include "libde265/fallback.h"
#include "libde265/configparam.h"

#include "libde265/encoder/algo/tb-intrapredmode.h"
#include "libde265/encoder/algo/tb-split.h"
#include "libde265/encoder/algo/cb-intrapartmode.h"


// ========== CB Intra/Inter decision ==========

class Algo_CB_PredMode
{
 public:
  virtual ~Algo_CB_PredMode() { }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CbSize, int ctDepth, int qp) = 0;

  void setIntraChildAlgo(Algo_CB_IntraPartMode* algo) { mIntraPartModeAlgo = algo; }
  // TODO void setInterChildAlgo(Algo_CB_IntraPartMode* algo) { mInterPartModeAlgo = algo; }

 protected:
  Algo_CB_IntraPartMode* mIntraPartModeAlgo;

  enc_cb* encode_cb_split(encoder_context* ectx,
                          context_model_table ctxModel,
                          const de265_image* input,
                          int x0,int y0, int Log2CbSize, int ctDepth, int qp);
};

class Algo_CB_PredMode_BruteForce : public Algo_CB_PredMode
{
 public:
  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CtbSize, int ctDepth, int qp);
};

#endif
