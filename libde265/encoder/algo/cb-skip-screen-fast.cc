/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
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


#include "libde265/encoder/algo/cb-skip-screen-fast.h"
#include "libde265/encoder/algo/coding-options.h"
#include "libde265/encoder/encoder-syntax.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>


Algo_CB_Skip_ScreenFast::Algo_CB_Skip_ScreenFast()
{
  mMaxPixelDifference = 32;
}


static bool compare_blocks_for_equality(const de265_image* imgA, int xA,int yA, int size,
                                        const de265_image* imgB, int xB,int yB,
                                        int maxPixelDifference)
{
  for (int c=0;c<3;c++) {
    //printf("COMPARE %d/%d\n",c+1,3);

    const uint8_t* pA = imgA->get_image_plane_at_pos(c, xA, yA);
    const uint8_t* pB = imgB->get_image_plane_at_pos(c, xB, yB);

    int strideA = imgA->get_image_stride(c);
    int strideB = imgB->get_image_stride(c);

    //printBlk(NULL,pA,size,strideA,"A ");
    //printBlk(NULL,pB,size,strideB,"B ");

    for (int y=0;y<size;y++)
      for (int x=0;x<size;x++) {
        int diff    = pA[x+strideA*y] - pB[x+strideB*y];
        int absdiff = abs_value(diff);

        //printf("diff: %d/%d %d\n",x,y,diff);

        if (absdiff > maxPixelDifference) {
          return false;
        }
      }
  }

  return true;
}


enc_cb* Algo_CB_Skip_ScreenFast::analyze(encoder_context* ectx,
                                         context_model_table& ctxModel,
                                         enc_cb* cb)
{
  bool try_skip  = (ectx->shdr->slice_type != SLICE_TYPE_I);
  bool try_nonskip = true;


  // We try to find a good merge candidate for skipping.
  // If there is a good match, do not try to code without skipping.
  if (try_skip) {

    // --- get all merge candidates ---

    int partIdx = 0;
    int cbSize = 1 << cb->log2Size;

    PBMotion mergeCandList[5];

    get_merge_candidate_list_from_tree(ectx, ectx->shdr,
                                       cb->x, cb->y, // xC/yC
                                       cb->x, cb->y, // xP/yP
                                       cbSize, // nCS
                                       cbSize,cbSize, // nPbW/nPbH
                                       partIdx, // partIdx
                                       mergeCandList);

    int num_merge_cand = 5 - ectx->shdr->five_minus_max_num_merge_cand;
    int selected_candidate = -1;


    // --- try all merge candidates until we find one with low error ---

    for (int idx=0 ; idx<num_merge_cand ; idx++) {
      const PBMotion& vec = mergeCandList[idx];

      // if we tried the same before, skip this candidate

      if (idx>0 && vec == mergeCandList[idx-1]) {
        continue;
      }


      //printf("try candidate\n");
      //logmvcand(mergeCandList[idx]);

      // generate prediction. Luma and chroma because we will check the error in all channels.

      generate_inter_prediction_samples(ectx, ectx, //&ectx->get_input_image_history(),
                                        ectx->shdr, ectx->img,
                                        cb->x,cb->y, // xP,yP
                                        1<<cb->log2Size, // int nCS,
                                        1<<cb->log2Size,
                                        1<<cb->log2Size, // int nPbW,int nPbH,
                                        &vec);

      // check error

      bool equal = compare_blocks_for_equality(ectx->img,            cb->x, cb->y, cbSize,
                                               ectx->imgdata->input, cb->x, cb->y,
                                               mMaxPixelDifference);

      // if it is similar enough, use this candidate

      if (equal) {
        selected_candidate = idx;
        break;
      }
    }


    // --- if we have found a matching candidate, use this to code the block in skip mode ---

    if (selected_candidate >= 0) {
      // do not try non-skip mode when we found a good match
      try_nonskip = false;

      // set motion parameters
      cb->PredMode = MODE_SKIP;
      cb->inter.rqt_root_cbf = 0; // no residual
      cb->inter.pb[partIdx].motion = mergeCandList[selected_candidate];

      // set motion coding parameters
      PBMotionCoding&   spec = cb->inter.pb[partIdx].spec;
      spec.merge_flag = 1;
      spec.merge_idx  = selected_candidate;


      // compute distortion

      const uint8_t* pA = ectx->img           ->get_image_plane_at_pos(0, cb->x, cb->y);
      const uint8_t* pB = ectx->imgdata->input->get_image_plane_at_pos(0, cb->x, cb->y);
      int strideA = ectx->img           ->get_image_stride(0);
      int strideB = ectx->imgdata->input->get_image_stride(0);

      cb->distortion = SSD(pA,strideA, pB,strideB, cbSize,cbSize);


      // for the moment ignore rate computation, because we will not use this

      /*
      CABAC_encoder_estim cabac;
      cabac.set_context_models(&ctxModel);
      // cabac->write_CABAC_bit(CONTEXT_MODEL_CU_SKIP_FLAG, 1); TODO (cu_skip_flag)
      */

      cb->rate = 0; // ignore merge_candidate rate for now (TODO)


      // --- build dummy TB tree and store reconstruction ---

      enc_tb* tb = new enc_tb(cb->x,cb->y,cb->log2Size,cb);
      tb->downPtr = &cb->transform_tree;
      cb->transform_tree = tb;

      tb->copy_reconstruction_from_image(ectx, ectx->img);
    }
  }


  if (try_nonskip) {
    cb = mNonSkipAlgo->analyze(ectx, ctxModel, cb);
  }

  return cb;
}
