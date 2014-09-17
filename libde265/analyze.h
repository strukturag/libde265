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
#include "libde265/encode.h"


/*  Encoder search tree, bottom up:

    - Algo_TB_Split - whether TB is split or not

    - Algo_TB_IntraPredMode - choose the intra prediction mode (or NOP, if at the wrong tree level)

    - Algo_CB_IntraPartMode - choose between NxN and 2Nx2N intra parts

    - Algo_CB_Split - whether CB is split or not

    - Algo_CTB_QScale - select QScale on CTB granularity
 */


// ========== TB intra prediction mode ==========

enum ALGO_TB_IntraPredMode {
  ALGO_TB_IntraPredMode_BruteForce,
  ALGO_TB_IntraPredMode_FastBrute,
  ALGO_TB_IntraPredMode_MinDistortion
};

class option_ALGO_TB_IntraPredMode : public choice_option
{
 public:
  option_ALGO_TB_IntraPredMode() {
    addChoice("minDist"    ,ALGO_TB_IntraPredMode_MinDistortion);
    addChoice("brute-force",ALGO_TB_IntraPredMode_BruteForce);
    addChoice("fast-brute" ,ALGO_TB_IntraPredMode_FastBrute);

    setID(ALGO_TB_IntraPredMode_FastBrute);
  }

  enum ALGO_TB_IntraPredMode operator() () const { return (enum ALGO_TB_IntraPredMode)getID(); }
};


enum TBBitrateEstimMethod {
  //TBBitrateEstim_AccurateBits,
  TBBitrateEstim_SSD,
  TBBitrateEstim_SAD,
  TBBitrateEstim_SATD_DCT,
  TBBitrateEstim_SATD_Hadamard
};

class option_TBBitrateEstimMethod : public choice_option
{
 public:
  option_TBBitrateEstimMethod() {
    addChoice("ssd",TBBitrateEstim_SSD);
    addChoice("sad",TBBitrateEstim_SAD);
    addChoice("satd-dct",TBBitrateEstim_SATD_DCT);
    addChoice("satd",TBBitrateEstim_SATD_Hadamard);

    setID(TBBitrateEstim_SATD_Hadamard);
  }

  enum TBBitrateEstimMethod operator() () const { return (enum TBBitrateEstimMethod)getID(); }
};

class Algo_TB_Split;

class Algo_TB_IntraPredMode
{
 public:
  Algo_TB_IntraPredMode() : mTBSplitAlgo(NULL) { }
  virtual ~Algo_TB_IntraPredMode() { }

  virtual const enc_tb* analyze(encoder_context*,
                                context_model_table,
                                const de265_image* input,
                                const enc_tb* parent,
                                enc_cb* cb,
                                int x0,int y0, int xBase,int yBase, int log2TbSize,
                                int blkIdx,
                                int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                int qp) = 0;

  void setChildAlgo(Algo_TB_Split* algo) { mTBSplitAlgo = algo; }

 protected:
  Algo_TB_Split* mTBSplitAlgo;
};


enum ALGO_TB_IntraPredMode_Subset {
  ALGO_TB_IntraPredMode_Subset_All,
  ALGO_TB_IntraPredMode_Subset_HVPlus,
  ALGO_TB_IntraPredMode_Subset_DC,
  ALGO_TB_IntraPredMode_Subset_Planar
};

class option_ALGO_TB_IntraPredMode_Subset : public choice_option
{
 public:
  option_ALGO_TB_IntraPredMode_Subset() {
    addChoice("all"   ,ALGO_TB_IntraPredMode_Subset_All);
    addChoice("HV+"   ,ALGO_TB_IntraPredMode_Subset_HVPlus);
    addChoice("DC"    ,ALGO_TB_IntraPredMode_Subset_DC);
    addChoice("planar",ALGO_TB_IntraPredMode_Subset_Planar);

    setID(ALGO_TB_IntraPredMode_Subset_All);
  }

  enum ALGO_TB_IntraPredMode_Subset operator() () const { return (enum ALGO_TB_IntraPredMode_Subset)getID(); }
};


class Algo_TB_IntraPredMode_ModeSubset : public Algo_TB_IntraPredMode
{
 public:
  Algo_TB_IntraPredMode_ModeSubset() {
    for (int i=0;i<35;i++) {
      mPredMode_enabled[i] = true;
    }
  }

  void disableAllIntraPredModes() {
    for (int i=0;i<35;i++) {
      mPredMode_enabled[i] = false;
    }
  }

  void enableIntraPredMode(int mode, bool flag=true) {
    mPredMode_enabled[mode] = flag;
  }

 protected:
  bool mPredMode_enabled[35];
};


class Algo_TB_IntraPredMode_BruteForce : public Algo_TB_IntraPredMode_ModeSubset
{
 public:

  virtual const enc_tb* analyze(encoder_context*,
                                context_model_table,
                                const de265_image* input,
                                const enc_tb* parent,
                                enc_cb* cb,
                                int x0,int y0, int xBase,int yBase, int log2TbSize,
                                int blkIdx,
                                int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                int qp);
};


class Algo_TB_IntraPredMode_FastBrute : public Algo_TB_IntraPredMode_ModeSubset
{
 public:

  struct params
  {
    params() {
      bitrateEstimMethod.setID(TBBitrateEstim_SATD_Hadamard);
      keepNBest = 5;
    }

    option_TBBitrateEstimMethod bitrateEstimMethod;
    int keepNBest;
  };

  void setParams(const params& p) { mParams=p; }


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


class Algo_TB_IntraPredMode_MinDistortion : public Algo_TB_IntraPredMode_ModeSubset
{
 public:

  struct params
  {
    params() {
      bitrateEstimMethod.setID(TBBitrateEstim_SATD_Hadamard);
    }

    option_TBBitrateEstimMethod bitrateEstimMethod;
  };

  void setParams(const params& p) { mParams=p; }


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


class Algo_TB_Split_BruteForce : public Algo_TB_Split
{
 public:
  virtual const enc_tb* analyze(encoder_context*,
                                context_model_table,
                                const de265_image* input,
                                const enc_tb* parent,
                                enc_cb* cb,
                                int x0,int y0, int xBase,int yBase, int log2TbSize,
                                int blkIdx,
                                int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                int qp);
};


// ========== CB intra NxN vs. 2Nx2N decision ==========

enum ALGO_CB_IntraPartMode {
  ALGO_CB_IntraPartMode_BruteForce,
  ALGO_CB_IntraPartMode_Fixed
};

class option_ALGO_CB_IntraPartMode : public choice_option
{
 public:
  option_ALGO_CB_IntraPartMode() {
    addChoice("fixed",      ALGO_CB_IntraPartMode_Fixed);
    addChoice("brute-force",ALGO_CB_IntraPartMode_BruteForce);

    setID(ALGO_CB_IntraPartMode_BruteForce);
  }

  enum ALGO_CB_IntraPartMode operator() () const { return (enum ALGO_CB_IntraPartMode)getID(); }
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


class option_PartMode : public choice_option
{
 public:
  option_PartMode() {
    addChoice("NxN",   PART_NxN);
    addChoice("2Nx2N", PART_2Nx2N);

    setID(PART_2Nx2N);
  }

  enum PartMode operator() () const { return (enum PartMode)getID(); }
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
    params() { }

    option_PartMode partMode;
  };

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
  params() : mQP(27) { }

    int mQP;
  };

  void setParams(const params& p) { mParams=p; }


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

  virtual Algo_CTB_QScale* getAlgoCTBQScale() { return &mAlgo_CTB_QScale_Constant; }

  virtual int getPPS_QP() const { return mAlgo_CTB_QScale_Constant.getQP(); }

 private:
  Algo_CTB_QScale_Constant         mAlgo_CTB_QScale_Constant;
  Algo_CB_Split_BruteForce         mAlgo_CB_Split_BruteForce;

  Algo_CB_IntraPartMode_BruteForce mAlgo_CB_IntraPartMode_BruteForce;
  Algo_CB_IntraPartMode_Fixed      mAlgo_CB_IntraPartMode_Fixed;

  Algo_TB_Split_BruteForce          mAlgo_TB_Split_BruteForce;

  Algo_TB_IntraPredMode_BruteForce    mAlgo_TB_IntraPredMode_BruteForce;
  Algo_TB_IntraPredMode_FastBrute     mAlgo_TB_IntraPredMode_FastBrute;
  Algo_TB_IntraPredMode_MinDistortion mAlgo_TB_IntraPredMode_MinDistortion;
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
