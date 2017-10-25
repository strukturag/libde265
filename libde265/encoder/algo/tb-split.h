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

#ifndef TB_SPLIT_H
#define TB_SPLIT_H

#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/encoder/encoder-types.h"
#include "libde265/encoder/algo/algo.h"
#include "libde265/slice.h"
#include "libde265/scan.h"
#include "libde265/intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include "libde265/quality.h"
#include "libde265/fallback.h"
#include "libde265/configparam.h"

#include "libde265/encoder/algo/tb-intrapredmode.h"
#include "libde265/encoder/algo/tb-rateestim.h"
#include "libde265/encoder/algo/tb-transform.h"


// ========== TB split decision ==========

class Algo_TB_Split : public Algo_TB
{
 public:
 Algo_TB_Split() { }
  virtual ~Algo_TB_Split() { }

  void setAlgo_TB_Split(Algo_TB* algo) { mAlgo_TB_Split=algo; }
  void setAlgo_TB_NoSplit(Algo_TB* algo) { mAlgo_TB_NoSplit=algo; }

 protected:
  enc_tb* encode_transform_tree_split(encoder_context* ectx,
                                      context_model_table& ctxModel,
                                      std::shared_ptr<const image> input,
                                      enc_tb* tb,
                                      enc_cb* cb,
                                      int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag);

  Algo_TB* mAlgo_TB_Split = nullptr;
  Algo_TB* mAlgo_TB_NoSplit = nullptr;
};



enum ALGO_TB_Split_BruteForce_ZeroBlockPrune {
  // numeric value specifies the maximum size for log2Tb for which the pruning is applied
  ALGO_TB_BruteForce_ZeroBlockPrune_off = 0,
  ALGO_TB_BruteForce_ZeroBlockPrune_8x8 = 3,
  ALGO_TB_BruteForce_ZeroBlockPrune_8x8_16x16 = 4,
  ALGO_TB_BruteForce_ZeroBlockPrune_all = 5
};

class option_ALGO_TB_Split_BruteForce_ZeroBlockPrune
: public choice_option<enum ALGO_TB_Split_BruteForce_ZeroBlockPrune>
{
 public:
  option_ALGO_TB_Split_BruteForce_ZeroBlockPrune() {
    add_choice("off"     ,ALGO_TB_BruteForce_ZeroBlockPrune_off);
    add_choice("8x8"     ,ALGO_TB_BruteForce_ZeroBlockPrune_8x8);
    add_choice("8-16"    ,ALGO_TB_BruteForce_ZeroBlockPrune_8x8_16x16);
    add_choice("all"     ,ALGO_TB_BruteForce_ZeroBlockPrune_all, true);
  }
};


class Algo_TB_Split_FixedSize : public Algo_TB_Split
{
 public:
  struct params
  {
    params() {
      targetTBSize.set_ID("TB-Split-FixedSize-TBSize");
      targetTBSize.set_valid_values({ 4,8,16,32 });
      targetTBSize.set_default(8);
    }

    option_int targetTBSize;
  };

  void setParams(const params& p) { mParams=p; }

  void registerParams(config_parameters& config) {
    config.add_option(&mParams.targetTBSize);
  }

  void setTargetTBSize(int size) { mParams.targetTBSize.set(size); }

  enc_tb* analyze(encoder_context*,
                  context_model_table&,
                  std::shared_ptr<const image> input,
                  enc_tb* tb,
                  int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag) override;

  const char* name() const { return "tb-split-fixed-size"; }

 private:
  params mParams;
};



class Algo_TB_Split_BruteForce : public Algo_TB_Split
{
 public:
  struct params
  {
    params() {
      zeroBlockPrune.set_ID("TB-Split-BruteForce-ZeroBlockPrune");
    }

    option_ALGO_TB_Split_BruteForce_ZeroBlockPrune zeroBlockPrune;
  };

  void setParams(const params& p) { mParams=p; }

  void registerParams(config_parameters& config) {
    config.add_option(&mParams.zeroBlockPrune);
  }

  enc_tb* analyze(encoder_context*,
                  context_model_table&,
                  std::shared_ptr<const image> input,
                  enc_tb* tb,
                  int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag) override;

  const char* name() const { return "tb-split-bruteforce"; }

 private:
  params mParams;
};

#endif
