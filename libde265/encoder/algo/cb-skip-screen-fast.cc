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



bool compare_blocks_for_equality(const de265_image* imgA, int xA,int yA, int size,
                                 const de265_image* imgB, int xB,int yB)
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
        int diff = (pA[x+strideA*y] - pB[x+strideB*y]);
        if (abs_value(diff) > 32) {
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

  //try_nonskip = !try_skip;

  if (try_skip) {
    //ectx->img->set_pred_mode(cb->x,cb->y, cb->log2Size, cb->PredMode);

    // --- try all merge candidates until we find one with zero error ---

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

    for (int idx=0 ; idx<num_merge_cand ; idx++) {
      const PBMotion& vec = mergeCandList[idx];

      if (idx>0 && vec == mergeCandList[idx-1]) {
        continue;
      }

      //printf("%d/%d  %d\n",cb->x,cb->y, idx);

      generate_inter_prediction_samples(ectx, ectx->shdr, ectx->img,
                                        cb->x,cb->y, // int xC,int yC,
                                        0,0,         // int xB,int yB,
                                        1<<cb->log2Size, // int nCS,
                                        1<<cb->log2Size,
                                        1<<cb->log2Size, // int nPbW,int nPbH,
                                        &vec);

      // check error

      bool equal = compare_blocks_for_equality(ectx->img,            cb->x, cb->y, cbSize,
                                               ectx->imgdata->input, cb->x, cb->y);
      if (equal) {
        //printf("EQUAL\n");

        selected_candidate = idx;
        break;
      }
    }


    if (selected_candidate >= 0) {
      // set skip flag
      cb->PredMode = MODE_SKIP;
      cb->inter.pb[partIdx].motion = mergeCandList[selected_candidate];

      PBMotionCoding&   spec = cb->inter.pb[partIdx].spec;

      spec.merge_flag = 1;
      spec.merge_idx  = selected_candidate;

      try_nonskip = false;


      // build dummy TB tree and store reconstruction

      cb->rate = 0; // ignore merge_candidate rate for now (TODO)
      cb->distortion = 0;

      cb->inter.rqt_root_cbf = 0;

      enc_tb* tb = new enc_tb(cb->x,cb->y,cb->log2Size,cb);
      tb->downPtr = &cb->transform_tree;
      cb->transform_tree = tb;

      tb->reconstruct(ectx, ectx->img);
    }
  }


  if (try_nonskip) {
    cb = mNonSkipAlgo->analyze(ectx, ctxModel, cb);
  }

  return cb;
}
