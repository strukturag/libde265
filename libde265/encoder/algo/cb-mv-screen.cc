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


#include "libde265/encoder/algo/cb-mv-screen.h"
#include "libde265/encoder/algo/coding-options.h"
#include "libde265/encoder/encoder-syntax.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>


Algo_CB_MV_Screen::Algo_CB_MV_Screen()
{
  mIntraAlgo = NULL;
}


static bool compare_blocks_for_equality(const image* imgA, int xA,int yA, int size,
                                        const image* imgB, int xB,int yB,
                                        int maxPixelDifference)
{
  for (int c=0;c<3;c++) {
    //printf("COMPARE %d/%d\n",c+1,3);

    const uint8_t* pA = imgA->get_image_plane_at_pos<const uint8_t>(c, xA, yA);
    const uint8_t* pB = imgB->get_image_plane_at_pos<const uint8_t>(c, xB, yB);

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


enc_cb* Algo_CB_MV_Screen::analyze(encoder_context* ectx,
                                   context_model_table& ctxModel,
                                   enc_cb* cb)
{
  bool try_inter = (ectx->shdr->slice_type != SLICE_TYPE_I);
  bool try_intra = true;

  const int cbSize = 1<<cb->log2Size;


  int x = cb->x;
  int y = cb->y;
  int w = ectx->img->get_width();
  int h = ectx->img->get_height();


  if (try_inter) {
    // get the two motion vector predictors

    MotionVector mvp[2];
    fill_luma_motion_vector_predictors_from_tree(ectx, ectx->shdr,
                                                 cb->x,cb->y,1<<cb->log2Size, x,y,cbSize,cbSize,
                                                 0, // l
                                                 0, 0, // int refIdx, int partIdx,
                                                 mvp);

    //printf("pred vectors: %d/%d  %d/%d\n",mvp[0].x,mvp[0].y,   mvp[1].x,mvp[1].y);

    PBMotion motion;
    PBMotionCoding spec;

    const int refIdx = 0; // get first reference frame
    const image* refPic = ectx->get_image(ectx->shdr->RefPicList[0][refIdx]).get();

    int range = 30;
    int dx = 0;

    int maxPixelDifference = 32;
    bool isMatch = false;

    for (int dx=-range; dx<=range && !isMatch; dx++)
      for (int dy=-range; dy<=range; dy++)
        {
          if (y+dy < 0 || y+dy+cbSize >= h) {
            continue;
          }

          if (x+dx < 0 || x+dx+cbSize >= w) {
            continue;
          }


          isMatch = compare_blocks_for_equality(refPic, x+dx,y+dy, cbSize,
                                                ectx->imgdata->input.get(), x,y,
                                                maxPixelDifference);

          //isMatch = (dx==1) && (dy==0);

          //printf("%d %d   %d %d    %s\n",x,y,dx,dy, isMatch ? "MATCH":"-----");

          if (isMatch) {
            motion.predFlag[0] = 1;
            motion.predFlag[1] = 0;

            motion.refIdx[0] = 0;
            motion.mv[0].x = dx<<2;
            motion.mv[0].y = dy<<2;

            spec.merge_flag = 0;
            spec.inter_pred_idc = PRED_L0;
            spec.refIdx[0] = 0;
            spec.mvp_l0_flag = 0; // use first predictor
            spec.mvd[0][0] = (dx<<2) - mvp[0].x;
            spec.mvd[0][1] = (dy<<2) - mvp[0].y;

            break;
          }
        }


    // --- if we have found a matching candidate, use this to code the block in skip mode ---

    if (isMatch) {
      // generate prediction. Luma and chroma because we will check the error in all channels.

      generate_inter_prediction_samples(ectx, ectx, //&ectx->get_input_image_history(),
                                        ectx->shdr, ectx->img.get(),
                                        cb->x,cb->y, // xP,yP
                                        1<<cb->log2Size, // int nCS,
                                        1<<cb->log2Size,
                                        1<<cb->log2Size, // int nPbW,int nPbH,
                                        &motion);


      // do not try intra mode when we found a good match
      try_intra = false;

      // set motion parameters
      int partIdx = 0;

      cb->PartMode = PART_2Nx2N;
      cb->PredMode = MODE_INTER;
      cb->inter.rqt_root_cbf = 0; // no residual
      cb->inter.pb[partIdx].motion = motion;
      cb->inter.pb[partIdx].spec   = spec;


      // compute distortion

      const uint8_t* pA = ectx->img           ->get_image_plane_at_pos<const uint8_t>(0, cb->x, cb->y);
      const uint8_t* pB = ectx->imgdata->input->get_image_plane_at_pos<const uint8_t>(0, cb->x, cb->y);
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

      tb->copy_reconstruction_from_image(ectx, ectx->img.get());
    }
  }

  if (try_intra) {
    cb->PredMode = MODE_INTRA;

    cb = mIntraAlgo->analyze(ectx, ctxModel, cb);
  }

  return cb;
}
