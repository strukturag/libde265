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

#include "libde265/encoder/algo/algo.h"
#include "libde265/encoder/encoder-context.h"


CodingOptions::CodingOptions(encoder_context* ectx, int nOptions)
  : mOptions(nOptions)
{
  mCBInput = NULL;
  mContextModelInput = NULL;
  mOriginalCBStructsAssigned=false;
  mCurrentlyReconstructedOption=-1;
  mBestRDO=-1;

  mECtx = ectx;
}

CodingOptions::~CodingOptions()
{
}

void CodingOptions::set_input(enc_cb* cb, context_model_table tab)
{
  mCBInput = cb;
  mContextModelInput = tab;
}

void CodingOptions::activate_option(int idx, bool active)
{
  if (mOptions.size() < idx+1) {
    mOptions.resize(idx+1);
  }

  mOptions[idx].optionActive = active;

  if (active) {
    mOptions[idx].rdoCost = -1;

    if (!mOriginalCBStructsAssigned) {
      mOptions[idx].isOriginalCBStruct = true;
      mOptions[idx].cb = mCBInput;
      mOptions[idx].context = mContextModelInput;

      mOriginalCBStructsAssigned=true;
    }
    else {
      mOptions[idx].isOriginalCBStruct = false;
      mOptions[idx].cb = new enc_cb;
      *mOptions[idx].cb = *mCBInput;

      copy_context_model_table(mOptions[idx].context_table_memory, mContextModelInput);
      mOptions[idx].context = mOptions[idx].context_table_memory;
    }
  }
}


void CodingOptions::begin_reconstruction(int idx)
{
  if (mCurrentlyReconstructedOption >= 0) {
    mOptions[mCurrentlyReconstructedOption].cb->save(mECtx->img);
  }

  mCurrentlyReconstructedOption = idx;
}

void CodingOptions::end_reconstruction(int idx)
{
  assert(mCurrentlyReconstructedOption == idx);
}


void CodingOptions::compute_rdo_costs()
{
  for (int i=0;i<mOptions.size();i++)
    if (mOptions[i].optionActive) {
      mOptions[i].rdoCost = mOptions[i].cb->distortion + mECtx->lambda * mOptions[i].cb->rate;
    }
}


enc_cb* CodingOptions::return_best_rdo()
{
  float bestRDOCost = 0;
  bool  first=true;
  int   bestRDO=-1;

  for (int i=0;i<mOptions.size();i++)
    if (mOptions[i].optionActive) {
      float cost = mOptions[i].rdoCost;
      if (first || cost < bestRDOCost) {
        bestRDOCost = cost;
        first = false;
        bestRDO = i;
      }
    }

  
  assert(bestRDO>=0);

  if (bestRDO != mCurrentlyReconstructedOption) {
    mOptions[bestRDO].cb->restore(mECtx->img);
  }

  if ( ! mOptions[bestRDO].isOriginalCBStruct ) {
    copy_context_model_table(mContextModelInput, mOptions[bestRDO].context_table_memory);
  }


  // delete all CBs except the best one

  for (int i=0;i<mOptions.size();i++) {
    if (i != bestRDO &&
        mOptions[i].optionActive)
      {
        delete mOptions[i].cb;
        mOptions[i].cb = NULL;
      }
  }

  return mOptions[bestRDO].cb;
}
