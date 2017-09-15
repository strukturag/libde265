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

#ifndef CB_PCM_H
#define CB_PCM_H

#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/encoder/encoder-types.h"
#include "libde265/slice.h"
#include "libde265/scan.h"
#include "libde265/intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include "libde265/quality.h"
#include "libde265/fallback.h"
#include "libde265/configparam.h"

#include "libde265/encoder/algo/algo.h"


// ========== CB PCM mode ==========

enum ALGO_CB_PCM {
  ALGO_CB_PCM_Never,
  ALGO_CB_PCM_Always,
  ALGO_CB_PCM_RDOpt
};

class option_ALGO_CB_PCM : public choice_option<enum ALGO_CB_PCM>
{
 public:
  option_ALGO_CB_PCM() {
    add_choice("never",  ALGO_CB_PCM_Never, true);
    add_choice("always", ALGO_CB_PCM_Always);
    add_choice("rdopt",  ALGO_CB_PCM_RDOpt);
  }
};


class Algo_CB_PCM : public Algo_CB
{
 public:
  Algo_CB_PCM() : mChildAlgo(nullptr) { }
  virtual ~Algo_CB_PCM() { }

  struct params
  {
    params() {
      pcmMode.set_ID("CB-PCM-Decision");
      minBlockSize.set_ID("CB-PCM-minBlockSize");
      maxBlockSize.set_ID("CB-PCM-maxBlockSize");

      minBlockSize.set_default(4);
      maxBlockSize.set_default(16);
    }

    option_ALGO_CB_PCM pcmMode;
    option_int         minBlockSize;
    option_int         maxBlockSize;
  };

  void registerParams(config_parameters& config) {
    config.add_option(&mParams.pcmMode);
    config.add_option(&mParams.minBlockSize);
    config.add_option(&mParams.maxBlockSize);
  }

  void setParams(const params& p) { mParams=p; }

  enc_cb* analyze(encoder_context*,
                  context_model_table&,
                  enc_cb* cb) override;

  void setChildAlgo(Algo_CB* algo) { mChildAlgo = algo; }

  virtual const char* name() const { return "cb-pcm"; }

 protected:
  Algo_CB* mChildAlgo;

 private:
  params mParams;
};

#endif
