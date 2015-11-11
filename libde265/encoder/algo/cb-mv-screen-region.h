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

#ifndef CB_MV_SCREEN_REGION_H
#define CB_MV_SCREEN_REGION_H

#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/slice.h"
#include "libde265/scan.h"
#include "libde265/intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include "libde265/quality.h"
#include "libde265/fallback.h"
#include "libde265/configparam.h"

#include "libde265/encoder/algo/algo.h"
#include "libde265/encoder/algo/cb-mergeindex.h"


// ========== CB Skip vs. Inter/Intra decision ==========

class Algo_CB_MV_ScreenRegion : public Algo_CB
{
 public:
  Algo_CB_MV_ScreenRegion();
  virtual ~Algo_CB_MV_ScreenRegion() { }

  void setIntraAlgo(Algo_CB* algo) { mIntraAlgo = algo; }

  virtual enc_cb* analyze(encoder_context*,
                          context_model_table&,
                          enc_cb* cb);

  const char* name() const { return "cb-mv-screen-region"; }

 protected:
  Algo_CB* mIntraAlgo;

  int mMaxPixelDifference;
  int mMaxMergePixelDifference;

  int mCurrentPicturePOC;


  struct HashInfo
  {
    uint32_t cnt;
    uint16_t x,y;
  };

  HashInfo hash[65536];
  int mProcessedHashesPOC;


  void build_feature_image(de265_image* feature_img, const de265_image* img, int blkSize);

  void process_picture(const encoder_context* ectx,
                       const enc_cb* cb);
};

#endif
