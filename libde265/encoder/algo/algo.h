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

#ifndef ALGO_H
#define ALGO_H

#include "libde265/encoder/encode.h"


/* When entering the next recursion level, it is assumed that
   a valid CB structure is passed down. If needed, the algorithm
   will do a copy of this structure and return the chosen variant.

   The context_model_table passed down is at the current state.
   When the algorithm returns, the state should represent the state
   after running this algorithm.

   When returning from the algorithm, it is also assumed that the
   ectx->img content (reconstruction and metadata) represent the
   current state. When the algorithm tries several variants, it
   has to restore the state to the selected variant.
 */

class Algo_CB
{
 public:
  virtual ~Algo_CB() { }

  /* The context_model_table that is provided can be modified and
     even released in the function. On exit, it should be filled with
     a (optionally new) context_model_table that represents the state
     after encoding the syntax element. However, to speed up computation,
     it is also allowed to not modify the context_model_table at all.
   */
  virtual enc_cb* analyze(encoder_context*,
                          context_model_table2&,
                          enc_cb* cb) = 0;
};



class CodingOption;


class CodingOptions
{
 public:
  CodingOptions(encoder_context*, int nOptions=2);
  ~CodingOptions();

  // --- init --- call before object use

  void set_input(enc_cb*, context_model_table2& tab);
  //void activate_option(int idx, bool flag=true);

  CodingOption new_option(bool active=true);

  void start(bool will_modify_context_model);


  // --- processing ---

  // compute RDO cost (D + lambda*R)
  void compute_rdo_costs();


  // --- end processing --- do not call any function after this one

  /* Return the CB with the lowest RDO cost. All other CBs are destroyed.
     If the current reconstruction and metadata are not from the returned CB,
     the data from the returned CB is reconstructed.
   */
  enc_cb* return_best_rdo();

#if 0
  enc_cb* get_cb(int idx) { return mOptions[idx].cb; }
  context_model_table2& get_context(int idx) {
    mOptions[idx].context_table_memory.decouple();
    return mOptions[idx].context_table_memory;
  }
  bool is_active(int idx) const { return mOptions[idx].optionActive; }

  void set_cb(int idx,enc_cb* cb) { mOptions[idx].cb = cb; }
  enc_cb*& operator[](int idx) { return mOptions[idx].cb; }
  enc_cb*const& operator[](int idx) const { return mOptions[idx].cb; }

  // Manually set RDO costs instead of computing them with compute_rdo_costs.
  // Only required when using custom costs.
  void set_rdo_cost(int idx, float rdo) { mOptions[idx].rdoCost=rdo; }
#endif

 private:
  struct CodingOptionData
  {
    enc_cb* cb;
    context_model_table2 context;
    //bool isOriginalCBStruct;
    bool mOptionActive;
    float rdoCost;
  };


  encoder_context* mECtx;

  enc_cb* mCBInput;
  context_model_table2* mContextModelInput;
  //bool mOriginalCBStructsAssigned;

  int mCurrentlyReconstructedOption;
  int mBestRDO;

  std::vector<CodingOptionData> mOptions;

  friend class CodingOption;
};


class CodingOption
{
 public:
  CodingOption() {
    mActive = false;
    mParent = nullptr;
    mOptionIdx = 0;
  }

  enc_cb* get_cb() { return mParent->mOptions[mOptionIdx].cb; }
  void set_cb(enc_cb* cb) { mParent->mOptions[mOptionIdx].cb = cb; }

  context_model_table2& get_context() {
    return mParent->mOptions[mOptionIdx].context;
  }

  operator bool() const { return mActive; }

  /* When modifying the reconstruction image or metadata, you have to
     encapsulate the modification between these two functions to ensure
     that the correct reconstruction will be active after return_best_rdo().
   */
  void begin_reconstruction();
  void end_reconstruction();

 private:
 CodingOption(class CodingOptions* parent, int idx)
   : mParent(parent), mOptionIdx(idx), mActive(true) { }

  class CodingOptions* mParent;
  int   mOptionIdx;
  bool  mActive;

  friend class CodingOptions;
};


#endif
