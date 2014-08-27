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

#ifndef ANALYZE_H
#define ANALYZE_H

#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/encode.h"
#include "libde265/slice.h"
#include "libde265/scan.h"
#include "libde265/intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include "libde265/quality.h"
#include "libde265/fallback.h"
#include "libde265/configparam.h"


// --- CB split decision ---

class Algo_CB_Split
{
 public:
  virtual ~Algo_CB_Split() { }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CbSize, int ctDepth, int qp) = 0;
};

class Algo_CB_Split_BruteForce : public Algo_CB_Split
{
 public:
  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CtbSize, int ctDepth, int qp);
};



// --- choose a qscale at CTB level ---

class Algo_CTB_QScale
{
 public:
 Algo_CTB_QScale() : mChildAlgo(NULL) { }
  virtual ~Algo_CTB_QScale() { }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CtbSize, int ctDepth) = 0;

  void setChildAlgo(Algo_CB_Split* algo) { mChildAlgo = algo; }

 protected:
  Algo_CB_Split* mChildAlgo;
};

class Algo_CTB_QScale_Constant : public Algo_CTB_QScale
{
 public:
  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CtbSize, int ctDepth);

  int getQP() const { return mQP; }

 private:
  int mQP;
};



// --- an encoding algorithm combines a set of algorithm modules ---

class EncodingAlgorithm
{
 public:
  virtual ~EncodingAlgorithm() { }

  virtual void prepare() = 0;

  virtual Algo_CTB_QScale* getAlgoCTBQScale() = 0;

  virtual int getPPS_QP() const = 0;
  virtual int getSlice_QPDelta() const { return 0; }
};


class EncodingAlgorithm_Custom : public EncodingAlgorithm
{
 public:

  virtual void prepare() {
    mAlgo_CTB_QScale_Constant.setChildAlgo(&mAlgo_CB_Split_BruteForce);
  }

  virtual Algo_CTB_QScale* getAlgoCTBQScale() { return &mAlgo_CTB_QScale_Constant; }

  virtual int getPPS_QP() const { return mAlgo_CTB_QScale_Constant.getQP(); }

 private:
  Algo_CTB_QScale_Constant mAlgo_CTB_QScale_Constant;
  Algo_CB_Split_BruteForce mAlgo_CB_Split_BruteForce;
};



enum IntraPredMode find_best_intra_mode(de265_image& img,int x0,int y0, int log2BlkSize, int cIdx,
                                        const uint8_t* ref, int stride);

enc_cb* encode_cb_no_split(encoder_context*, context_model_table ctxModel, const de265_image* input,
                           int x0,int y0, int log2CbSize, int ctDepth, int qp);

enc_cb* encode_cb_split(encoder_context*, context_model_table ctxModel, const de265_image* input,
                        int x0,int y0, int Log2CbSize, int ctDepth, int qp);

enc_cb* encode_cb_may_split(encoder_context*, context_model_table ctxModel,
                            const de265_image* input,
                            int x0,int y0, int Log2CtbSize, int ctDepth, int qp);

double encode_image(encoder_context*, const de265_image* input, int qp);

void encode_sequence(encoder_context*);

#endif
