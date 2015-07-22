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

#ifndef CODING_OPTIONS_H
#define CODING_OPTIONS_H

#include "libde265/encoder/encoder-types.h"


class CodingOption;


class CodingOptions
{
 public:
  CodingOptions(encoder_context*, enc_cb*, context_model_table& tab);
  CodingOptions(encoder_context*, enc_tb*, context_model_table& tab);
  ~CodingOptions();

  // --- init --- call before object use

  CodingOption new_option(bool active=true);

  enum RateEstimationMethod
  {
    Rate_Default,  // take default value from encoder_context
    Rate_AdaptiveContext,
    Rate_FixedContext
  };

  void start(enum RateEstimationMethod = Rate_Default);


  // --- processing ---

  // compute RDO cost (D + lambda*R) for all options
  void compute_rdo_costs();


  // --- end processing --- do not call any function after this one

  /* Return the CB with the lowest RDO cost. All other CBs are destroyed.
     If the current metadata stored in the image are not from the returned block,
     its metadata flags are set to zero.
   */
  enc_cb* return_best_rdo_cb();
  enc_tb* return_best_rdo_tb();

 private:
  struct CodingOptionData
  {
    union {
      enc_cb* cb;
      enc_tb* tb;
    };

    context_model_table context;
    bool  mOptionActive;
    bool  computed;
    float rdoCost;
  };


  encoder_context* mECtx;

  bool mCBMode;
  enc_cb* mCBInput;
  enc_tb* mTBInput;

  context_model_table* mContextModelInput;

  int mCurrentlyReconstructedOption;
  int mBestRDO;

  std::vector<CodingOptionData> mOptions;

  CABAC_encoder_estim           cabac_adaptive;
  CABAC_encoder_estim_constant  cabac_constant;
  CABAC_encoder_estim*          cabac;

  friend class CodingOption;

  int find_best_rdo_index();
};


class CodingOption
{
 public:
  CodingOption() {
    mParent = nullptr;
    mOptionIdx = 0;
  }

  enc_cb* get_cb() { return mParent->mOptions[mOptionIdx].cb; }
  void set_cb(enc_cb* cb) { mParent->mOptions[mOptionIdx].cb = cb; }

  enc_tb* get_tb() { return mParent->mOptions[mOptionIdx].tb; }
  void set_tb(enc_tb* tb) {
    if (tb != mParent->mOptions[mOptionIdx].tb) {
      printf("delete TB %p\n", mParent->mOptions[mOptionIdx].tb);
      delete mParent->mOptions[mOptionIdx].tb;
    }
    mParent->mOptions[mOptionIdx].tb = tb;
  }

  context_model_table& get_context() { return mParent->mOptions[mOptionIdx].context; }

  /** @return True if the option is active.
   */
  operator bool() const { return mParent; }

  /* When modifying the metadata stored in the image, you have to
     encapsulate the modification between these two functions to ensure
     that the correct reconstruction will be active after return_best_rdo().
   */
  void begin();
  void end();

  // Manually set RDO costs instead of computing them with compute_rdo_costs.
  // Only required when using custom costs.
  void set_rdo_cost(float rdo) { mParent->mOptions[mOptionIdx].rdoCost=rdo; }

  CABAC_encoder_estim* get_cabac() { return mParent->cabac; }
  float get_cabac_rate() const { return mParent->cabac->getRDBits(); }

private:
  CodingOption(class CodingOptions* parent, int idx)
    : mParent(parent), mOptionIdx(idx) { }

  class CodingOptions* mParent;
  int   mOptionIdx;

  friend class CodingOptions;
};


#endif
