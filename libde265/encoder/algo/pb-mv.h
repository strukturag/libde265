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

#ifndef PB_MV_H
#define PB_MV_H

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

#include "libde265/encoder/algo/algo.h"


// ========== CB Intra/Inter decision ==========

class Algo_PB_MV : public Algo_PB
{
 public:
  virtual ~Algo_PB_MV() { }

 protected:
};




enum MVTestMode
  {
    MVTestMode_Zero,
    MVTestMode_Random,
    MVTestMode_Horizontal,
    MVTestMode_Vertical
  };

class option_MVTestMode : public choice_option<enum MVTestMode>
{
 public:
  option_MVTestMode() {
    add_choice("zero",   MVTestMode_Zero, true);
    add_choice("random", MVTestMode_Random);
    add_choice("horiz",  MVTestMode_Horizontal);
    add_choice("verti",  MVTestMode_Vertical);
  }
};

class Algo_PB_MV_Test : public Algo_PB_MV
{
 public:
 Algo_PB_MV_Test() : mCodeResidual(false) { }

  struct params
  {
    params() {
      testMode.set_ID("PB-MVTestMode");
    }

    option_MVTestMode testMode;
  };

  void registerParams(config_parameters& config) {
    config.add_option(&mParams.testMode);
  }

  void setParams(const params& p) { mParams=p; }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table&,
                          enc_cb* cb,
                          int PBidx, int x,int y,int w,int h);

 private:
  params mParams;

  bool mCodeResidual;
};

#endif
