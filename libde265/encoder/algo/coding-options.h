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

#include "libde265/encoder/encode.h"


class CodingOption;


class CodingOptions
{
 public:
  CodingOptions(encoder_context*, int nOptions=2);
  ~CodingOptions();

  // --- init --- call before object use

  void set_input(enc_cb*, context_model_table& tab);

  CodingOption new_option(bool active=true);

  void start(bool will_modify_context_model);


  // --- processing ---

  // compute RDO cost (D + lambda*R) for all options
  void compute_rdo_costs();


  // --- end processing --- do not call any function after this one

  /* Return the CB with the lowest RDO cost. All other CBs are destroyed.
     If the current reconstruction and metadata are not from the returned CB,
     the data from the returned CB is reconstructed.
   */
  enc_cb* return_best_rdo();

 private:
  struct CodingOptionData
  {
    enc_cb* cb;
    context_model_table context;
    bool  mOptionActive;
    float rdoCost;
  };


  encoder_context* mECtx;

  enc_cb* mCBInput;
  context_model_table* mContextModelInput;

  int mCurrentlyReconstructedOption;
  int mBestRDO;

  std::vector<CodingOptionData> mOptions;

  friend class CodingOption;
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

  context_model_table& get_context() { return mParent->mOptions[mOptionIdx].context; }

  operator bool() const { return mParent; }

  /* When modifying the reconstruction image or metadata, you have to
     encapsulate the modification between these two functions to ensure
     that the correct reconstruction will be active after return_best_rdo().
   */
  void begin_reconstruction();
  void end_reconstruction();

  // Manually set RDO costs instead of computing them with compute_rdo_costs.
  // Only required when using custom costs.
  void set_rdo_cost(float rdo) { mParent->mOptions[mOptionIdx].rdoCost=rdo; }

private:
  CodingOption(class CodingOptions* parent, int idx)
    : mParent(parent), mOptionIdx(idx) { }

  class CodingOptions* mParent;
  int   mOptionIdx;

  friend class CodingOptions;
};


#endif
