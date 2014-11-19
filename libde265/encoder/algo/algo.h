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

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          enc_cb* cb) = 0;
};



class CodingOptions
{
 public:
  CodingOptions(encoder_context*, int nOptions=2);
  ~CodingOptions();

  void set_input(enc_cb*, context_model_table tab);
  void activate_option(int idx, bool flag=true);

  enc_cb* get_cb(int idx) { return mOptions[idx].cb; }
  context_model* get_context(int idx) { return mOptions[idx].context; }

  void begin_reconstruction(int idx);
  void end_reconstruction(int idx);

  void set_rdo_cost(int idx, float rdo) { mOptions[idx].rdoCost=rdo; }
  void compute_rdo_costs();
  enc_cb* return_best_rdo();

 private:
  struct CodingOption {
    CodingOption() : optionActive(false) { }

    enc_cb* cb;
    context_model* context;
    context_model_table context_table_memory;
    bool isOriginalCBStruct;
    bool optionActive;
    float rdoCost;
  };

  encoder_context* mECtx;

  enc_cb* mCBInput;
  context_model* mContextModelInput;
  bool mOriginalCBStructsAssigned;

  int  mCurrentlyReconstructedOption;
  int mBestRDO;

  std::vector<CodingOption> mOptions;
};

#endif
