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


/*  Encoder search tree, bottom up:

    - Algo_TB_Split - whether TB is split or not

    - Algo_TB_IntraPredMode - choose the intra prediction mode (or NOP, if at the wrong tree level)

    - Algo_CB_IntraPartMode - choose between NxN and 2Nx2N intra parts

    - Algo_CB_Split - whether CB is split or not

    - Algo_CTB_QScale - select QScale on CTB granularity
 */


// ========== TB split decision ==========

class Algo_TB_Split
{
 public:
  Algo_TB_Split() : mAlgo_TB_IntraPredMode(NULL) { }
  virtual ~Algo_TB_Split() { }

  virtual const enc_tb* analyze(encoder_context*,
                                context_model_table,
                                const de265_image* input,
                                const enc_tb* parent,
                                enc_cb* cb,
                                int x0,int y0, int xBase,int yBase, int log2TbSize,
                                int blkIdx,
                                int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                int qp) = 0;

  void setAlgo_TB_IntraPredMode(Algo_TB_IntraPredMode* algo) { mAlgo_TB_IntraPredMode=algo; }

 protected:
  const enc_tb* encode_transform_tree_split(encoder_context* ectx,
                                            context_model_table ctxModel,
                                            const de265_image* input,
                                            const enc_tb* parent,
                                            enc_cb* cb,
                                            int x0,int y0, int log2TbSize,
                                            int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                            int qp);

  Algo_TB_IntraPredMode* mAlgo_TB_IntraPredMode;
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

  virtual const enc_tb* analyze(encoder_context*,
                                context_model_table,
                                const de265_image* input,
                                const enc_tb* parent,
                                enc_cb* cb,
                                int x0,int y0, int xBase,int yBase, int log2TbSize,
                                int blkIdx,
                                int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                int qp);

 private:
  params mParams;
};


// ========== CB intra NxN vs. 2Nx2N decision ==========

enum ALGO_CB_IntraPartMode {
  ALGO_CB_IntraPartMode_BruteForce,
  ALGO_CB_IntraPartMode_Fixed
};

class option_ALGO_CB_IntraPartMode : public choice_option<enum ALGO_CB_IntraPartMode>
{
 public:
  option_ALGO_CB_IntraPartMode() {
    add_choice("fixed",      ALGO_CB_IntraPartMode_Fixed);
    add_choice("brute-force",ALGO_CB_IntraPartMode_BruteForce, true);
  }
};


class Algo_CB_IntraPartMode
{
 public:
  Algo_CB_IntraPartMode() : mTBIntraPredModeAlgo(NULL) { }
  virtual ~Algo_CB_IntraPartMode() { }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CbSize, int ctDepth, int qp) = 0;

  void setChildAlgo(Algo_TB_IntraPredMode* algo) { mTBIntraPredModeAlgo = algo; }

 protected:
  Algo_TB_IntraPredMode* mTBIntraPredModeAlgo;
};

/* Try both NxN, 2Nx2N and choose better one.
 */
class Algo_CB_IntraPartMode_BruteForce : public Algo_CB_IntraPartMode
{
 public:
  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CbSize, int ctDepth, int qp);
};


class option_PartMode : public choice_option<enum PartMode> // choice_option
{
 public:
  option_PartMode() {
    add_choice("NxN",   PART_NxN);
    add_choice("2Nx2N", PART_2Nx2N, true);
  }
};


/* Always use choose selected part mode.
   If NxN is chosen but cannot be applied (CB tree not at maximum depth), 2Nx2N is used instead.
 */
class Algo_CB_IntraPartMode_Fixed : public Algo_CB_IntraPartMode
{
 public:
 Algo_CB_IntraPartMode_Fixed() { }

  struct params
  {
    params() {
      partMode.set_ID("CB-IntraPartMode-Fixed-partMode");
    }

    option_PartMode partMode;
  };

  void registerParams(config_parameters& config) {
    config.add_option(&mParams.partMode);
  }

  void setParams(const params& p) { mParams=p; }

  virtual enc_cb* analyze(encoder_context* ectx,
                          context_model_table ctxModel,
                          const de265_image* input,
                          int x0,int y0, int log2CbSize, int ctDepth,
                          int qp);

 private:
  params mParams;
};


// ========== CB split decision ==========

class Algo_CB_Split
{
 public:
  virtual ~Algo_CB_Split() { }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CbSize, int ctDepth, int qp) = 0;

  // TODO: probably, this will later be a intra/inter decision which again
  // has two child algorithms, depending on the coding mode.
  void setChildAlgo(Algo_CB_IntraPartMode* algo) { mIntraPartModeAlgo = algo; }

 protected:
  Algo_CB_IntraPartMode* mIntraPartModeAlgo;

  bool forcedSplit(const de265_image* input, int x0,int y0, int Log2CbSize) const;

  enc_cb* encode_cb_split(encoder_context* ectx,
                          context_model_table ctxModel,
                          const de265_image* input,
                          int x0,int y0, int Log2CbSize, int ctDepth, int qp);
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



// ========== choose a qscale at CTB level ==========

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
  struct params
  {
    params() {
      mQP.set_range(1,51);
      mQP.set_default(27);
      mQP.set_ID("CTB-QScale-Constant");
      mQP.set_cmd_line_options("qp",'q');
    }

    option_int mQP;
  };

  void setParams(const params& p) { mParams=p; }

  void registerParams(config_parameters& config) {
    config.add_option(&mParams.mQP);
  }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table,
                          const de265_image* input,
                          int ctb_x,int ctb_y,
                          int log2CtbSize, int ctDepth);

  int getQP() const { return mParams.mQP; }

 private:
  params mParams;
};



// ========== an encoding algorithm combines a set of algorithm modules ==========

class EncodingAlgorithm
{
 public:
  virtual ~EncodingAlgorithm() { }

  virtual Algo_CTB_QScale* getAlgoCTBQScale() = 0;

  virtual int getPPS_QP() const = 0;
  virtual int getSlice_QPDelta() const { return 0; }
};


class EncodingAlgorithm_Custom : public EncodingAlgorithm
{
 public:

  void setParams(struct encoder_params& params);

  void registerParams(config_parameters& config) {
    mAlgo_CTB_QScale_Constant.registerParams(config);
    mAlgo_CB_IntraPartMode_Fixed.registerParams(config);
    mAlgo_TB_IntraPredMode_FastBrute.registerParams(config);
    mAlgo_TB_IntraPredMode_MinResidual.registerParams(config);
    mAlgo_TB_Split_BruteForce.registerParams(config);
  }

  virtual Algo_CTB_QScale* getAlgoCTBQScale() { return &mAlgo_CTB_QScale_Constant; }

  virtual int getPPS_QP() const { return mAlgo_CTB_QScale_Constant.getQP(); }

 private:
  Algo_CTB_QScale_Constant         mAlgo_CTB_QScale_Constant;
  Algo_CB_Split_BruteForce         mAlgo_CB_Split_BruteForce;

  Algo_CB_IntraPartMode_BruteForce mAlgo_CB_IntraPartMode_BruteForce;
  Algo_CB_IntraPartMode_Fixed      mAlgo_CB_IntraPartMode_Fixed;

  Algo_TB_Split_BruteForce          mAlgo_TB_Split_BruteForce;

  Algo_TB_IntraPredMode_BruteForce  mAlgo_TB_IntraPredMode_BruteForce;
  Algo_TB_IntraPredMode_FastBrute   mAlgo_TB_IntraPredMode_FastBrute;
  Algo_TB_IntraPredMode_MinResidual mAlgo_TB_IntraPredMode_MinResidual;
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

double encode_image(encoder_context*, const de265_image* input, EncodingAlgorithm&);

void encode_sequence(encoder_context*);

void en265_print_logging(const encoder_context* ectx, const char* id, const char* filename);

#endif
