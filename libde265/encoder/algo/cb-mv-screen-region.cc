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


#define DEBUG_OUTPUT 0


Algo_CB_MV_ScreenRegion::Algo_CB_MV_ScreenRegion()
{
  mMaxPixelDifference = 0;
  mMaxMergePixelDifference = 5;
  mCurrentPicturePOC = -1;
  mProcessedHashesPOC = -1;
}


static bool compare_blocks_for_equality(const de265_image* imgA, int xA,int yA, int size,
                                        const de265_image* imgB, int xB,int yB,
                                        int maxPixelDifference, int maxSADDifference)
{
  int nChannelsToCheck = 3;
  int sad=0;

  for (int c=0;c<nChannelsToCheck;c++) {
    //printf("COMPARE %d/%d\n",c+1,3);

    const uint8_t* pA = imgA->get_image_plane_at_pos(c, xA, yA);
    const uint8_t* pB = imgB->get_image_plane_at_pos(c, xB, yB);

    int strideA = imgA->get_image_stride(c);
    int strideB = imgB->get_image_stride(c);

    //printBlk("a",pA,size,strideA,"A ");
    //printBlk("b",pB,size,strideB,"B ");

    for (int y=0;y<size;y++)
      for (int x=0;x<size;x++) {
        int diff    = pA[x+strideA*y] - pB[x+strideB*y];
        int absdiff = abs_value(diff);

        if (absdiff > maxPixelDifference) {
          //printf("diff: %d\n",diff);
          return false;
        }

        sad += absdiff;
      }
  }

  if (sad>3*maxSADDifference) { return false; }

  //printf("equal\n");
  return true;
}


const int hashBlkRadius = 7;
const int hashBlkSize = (2*hashBlkRadius + 1);


static inline uint16_t crc_process_byte_parallel(uint16_t crc, uint8_t byte)
{
  uint16_t s = byte ^ (crc >> 8);
  uint16_t t = s ^ (s >> 4);

  return  ((crc << 8) ^
	   t ^
	   (t <<  5) ^
	   (t << 12)) & 0xFFFF;
}


static uint16_t block_hash_crc(const uint8_t* p, int stride, int blkSize = hashBlkSize)
{
  uint16_t crc = 0;
  for (int yy=0;yy<blkSize;yy++)
    for (int xx=0;xx<blkSize;xx++) {
      crc = crc_process_byte_parallel(crc, *(p+xx+yy*stride));
    }

  return crc;
}


static uint16_t block_hash_DJB2a(const uint8_t* p, int stride, int blkSize = hashBlkSize)
{
  uint16_t hash = 0;

  for (int yy=0;yy<blkSize;yy++)
    for (int xx=0;xx<blkSize;xx++) {
      hash = ((hash << 5) + hash) ^ *(p+xx+yy*stride); /* (hash * 33) ^ c */
    }

  return hash;
}


#define block_hash block_hash_DJB2a


void Algo_CB_MV_ScreenRegion::build_feature_image(de265_image* feature_img,
                                                  const de265_image* img,
                                                  int blkSize)
{
  int stride = img->get_image_stride(0);

  int w = img->get_width();
  int h = img->get_height();

  int cnt=0;

  std::vector<int16_t> rightPos(h, -2);
  std::vector<int16_t> bottomPos(w, -2);

  for (int i=0;i<65536;i++) { hash[i].cnt=0; }

  for (int y=1;y<h-1;y++)
    for (int x=1;x<w-1;x++)
      {
        const uint8_t* p = img->get_image_plane_at_pos(0, x, y);

        if (*p > *(p-1) &&
            *p > *(p-stride) &&
            *p > *(p-stride-1) &&
            *p > *(p-stride+1) &&
            *p > *(p+stride-1)) {

          if (rightPos[y] != x-1 && bottomPos[x] != y-1) {
#if DEBUG_OUTPUT
            *feature_img->get_image_plane_at_pos(0, x, y) = 255;
            *feature_img->get_image_plane_at_pos(1, x, y) = 0;
            *feature_img->get_image_plane_at_pos(2, x, y) = 0;
#endif
            cnt++;

              if (x>=hashBlkRadius &&
                  y>=hashBlkRadius &&
                  x< w-hashBlkRadius &&
                  y< h-hashBlkRadius) {
                int crc = block_hash(p-hashBlkRadius*(1+stride), stride);

                hash[crc].cnt++;
                hash[crc].x = x;
                hash[crc].y = y;
              }
          }

          rightPos[y] = x;
          bottomPos[x] = y;
        }
      }

  //printf("features: %d / %d\n",cnt,w*h);
}


struct BlkInfo
{
  bool haveMV;
  MotionVector mv;
};

std::vector<BlkInfo> blkInfo;



void Algo_CB_MV_ScreenRegion::process_picture(const encoder_context* ectx,
                                              const enc_cb* cb)
{
  const int blkSize = 1 << cb->log2Size;
  const de265_image* inputPic = ectx->imgdata->input;

  const int refIdx = 0; // get first reference frame
  const int refPicId = ectx->shdr->RefPicList[0][refIdx];
  const de265_image* refPic = ectx->get_input_image_history().get_image(refPicId);


  int stride = inputPic->get_image_stride(0);

  int w = inputPic->get_width();
  int h = inputPic->get_height();

  int bw = (w+blkSize-1)/blkSize;
  int bh = (h+blkSize-1)/blkSize;

  blkInfo.resize(bw*bh);


#if DEBUG_OUTPUT
  de265_image feature_img;
  feature_img.copy_image(inputPic);
#endif


  for (int by=0;by<bh;by++)
    for (int bx=0;bx<bw;bx++) {
      int x0 = bx<<cb->log2Size;
      int y0 = by<<cb->log2Size;

      BlkInfo& blk = blkInfo[bx+by*bw];
      blk.haveMV = false;

      // check whether we can use skip mode with zero MV

      //printf("test for skip: %d;%d\n",bx,by);

      bool equal = compare_blocks_for_equality(refPic,   x0,y0, blkSize,
                                               inputPic, x0,y0,
                                               mMaxPixelDifference,
                                               mMaxPixelDifference *blkSize*blkSize);

      if (equal) {
        blk.haveMV = true;
        blk.mv.x = 0;
        blk.mv.y = 0;
      }

      // find feature points and try to find matching hash in previous frame

      if (!equal) {
        //printf("-------------------------------------------------------- match hashes\n");

        for (int dy=1;dy<blkSize-1;dy++)
          for (int dx=1;dx<blkSize-1;dx++) {
            int x = x0+dx;
            int y = y0+dy;

            const uint8_t* p = inputPic->get_image_plane_at_pos(0, x, y);

            if (*p > *(p-1) &&
                *p > *(p-stride) &&
                *p > *(p-stride-1) &&
                *p > *(p-stride+1) &&
                *p > *(p+stride-1)) {

              if (x>=hashBlkRadius &&
                  y>=hashBlkRadius &&
                  x< w-hashBlkRadius &&
                  y< h-hashBlkRadius) {
                int crc = block_hash(p-hashBlkRadius*(1+stride), stride);

                //printf("%d %d, CRC: %d  [%d]\n",x,y,crc,hash[crc].cnt);

#if DEBUG_OUTPUT
                if (hash[crc].cnt==1) {
                  *feature_img.get_image_plane_at_pos(0, x, y) = 255; // green
                  *feature_img.get_image_plane_at_pos(1, x, y) = 0;
                  *feature_img.get_image_plane_at_pos(2, x, y) = 0;
                }
                else if (hash[crc].cnt>1) {
                  *feature_img.get_image_plane_at_pos(0, x, y) = 0;
                  *feature_img.get_image_plane_at_pos(1, x, y) = 255; // blue
                  *feature_img.get_image_plane_at_pos(2, x, y) = 0;
                }
                else {
                  *feature_img.get_image_plane_at_pos(0, x, y) = 0;
                  *feature_img.get_image_plane_at_pos(1, x, y) = 0;
                  *feature_img.get_image_plane_at_pos(2, x, y) = 255; // red
                }
#endif

                if (hash[crc].cnt==1) {

                  int dx = hash[crc].x - x;
                  int dy = hash[crc].y - y;

                  if (x0+dx>=0 && y0+dy>=0 &&
                      x0+dx+blkSize<w &&
                      y0+dy+blkSize<h) {
                    if (compare_blocks_for_equality(inputPic, x0,   y0,    blkSize,
                                                    refPic,   x0+dx,y0+dy,
                                                    mMaxPixelDifference,
                                                    mMaxPixelDifference*blkSize*blkSize))
                      {
                        blk.haveMV = true;
                        blk.mv.x = dx;
                        blk.mv.y = dy;

                        //printf("exact match\n");
                      }
                  }
                  else {
                    //printf("hash collision\n");
                  }
                }

                if (hash[crc].cnt<0) {
                  //printf("no match\n");
                }

                if (hash[crc].cnt>1) {
                  //printf("multiple matches\n");
                }
              }
            }
          }
      }
    }


#if DEBUG_OUTPUT
  char buf[100];
  static int cnt=0;
  sprintf(buf,"img%03d.yuv",cnt++);
  feature_img.write_image(buf);
#endif
}


enc_cb* Algo_CB_MV_ScreenRegion::analyze(encoder_context* ectx,
                                         context_model_table& ctxModel,
                                         enc_cb* cb)
{
  bool try_skip  = (ectx->shdr->slice_type != SLICE_TYPE_I);
  bool try_nonskip = true;


  const de265_image* inputPic = ectx->imgdata->input;


  // We try to find a good merge candidate for skipping.
  // If there is a good match, do not try to code without skipping.
  if (try_skip) {

    const int refIdx = 0; // get first reference frame
    const int refPicId = ectx->shdr->RefPicList[0][refIdx];
    const de265_image* refPic = ectx->get_input_image_history().get_image(refPicId);

#if 0
    bool isNextImage = (inputPic->PicOrderCntVal != mCurrentPicturePOC);
    if (isNextImage) {
      mCurrentPicturePOC = inputPic->PicOrderCntVal;

      //assert(refPic->PicOrderCntVal != feature_poc);

      de265_image feature_img;
#if DEBUG_OUTPUT
      feature_img.copy_image(refPic);
#endif

      printf("building feature image\n");
      build_feature_image(&feature_img, refPic, 0); //cbSize);
      mProcessedHashesPOC = refPic->PicOrderCntVal;
      printf("...done\n");

      char buf[100];
      sprintf(buf,"img%03d.yuv",refPic->PicOrderCntVal);
      //feature_img.write_image(buf);


      process_picture(ectx, cb);
    }
#endif

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


      //printf("try candidate %d\n",idx);
      //logmvcand(mergeCandList[idx]);

      // generate prediction. Luma and chroma because we will check the error in all channels.

      generate_inter_prediction_samples(ectx, &ectx->get_input_image_history(),
                                        ectx->shdr, ectx->img,
                                        cb->x,cb->y, // xP,yP
                                        1<<cb->log2Size, // int nCS,
                                        1<<cb->log2Size,
                                        1<<cb->log2Size, // int nPbW,int nPbH,
                                        &vec);

      // check error

      //printf("check merge: %d %d\n",cb->x,cb->y);
      bool equal = compare_blocks_for_equality(ectx->img,            cb->x, cb->y, cbSize,
                                               ectx->imgdata->input, cb->x, cb->y,
                                               0, //mMaxMergePixelDifference,
                                               mMaxMergePixelDifference*cbSize*cbSize);

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
    else {

      if (mProcessedHashesPOC != refPic->PicOrderCntVal) {
        //mCurrentPicturePOC = inputPic->PicOrderCntVal;

        //assert(refPic->PicOrderCntVal != feature_poc);

        de265_image feature_img;
#if DEBUG_OUTPUT
        feature_img.copy_image(refPic);
#endif

        printf("building feature image\n");
        build_feature_image(&feature_img, refPic, 0); //cbSize);
        mProcessedHashesPOC = refPic->PicOrderCntVal;
        printf("...done\n");

        char buf[100];
        sprintf(buf,"img%03d.yuv",refPic->PicOrderCntVal);
        //feature_img.write_image(buf);


        process_picture(ectx, cb);
      }


      int w = inputPic->get_width();
      int h = inputPic->get_height();

      int bx = cb->x >> cb->log2Size;
      int by = cb->y >> cb->log2Size;
      int bw = (w+(1<<cb->log2Size)-1) >> cb->log2Size;

      const BlkInfo& blk = blkInfo[bx+by*bw];
      if (blk.haveMV) {

        // get the two motion vector predictors

        MotionVector mvp[2];
        int x = cb->x;
        int y = cb->y;
        int cbSize = 1<<cb->log2Size;
        fill_luma_motion_vector_predictors_from_tree(ectx, ectx->shdr,
                                                     cb->x,cb->y,1<<cb->log2Size, x,y,cbSize,cbSize,
                                                     0, // l
                                                     0, 0, // int refIdx, int partIdx,
                                                     mvp);

        int dx = blk.mv.x;
        int dy = blk.mv.y;

        PBMotion motion;
        PBMotionCoding spec;

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


        // set motion parameters
        int partIdx = 0;

        cb->PartMode = PART_2Nx2N;
        cb->PredMode = MODE_INTER;
        cb->inter.rqt_root_cbf = 0; // no residual
        cb->inter.pb[partIdx].motion = motion;
        cb->inter.pb[partIdx].spec   = spec;

        try_nonskip = false;

        // --- build dummy TB tree and store reconstruction ---

        enc_tb* tb = new enc_tb(cb->x,cb->y,cb->log2Size,cb);
        tb->downPtr = &cb->transform_tree;
        cb->transform_tree = tb;

        generate_inter_prediction_samples(ectx, ectx, //&ectx->get_input_image_history(),
                                          ectx->shdr, ectx->img,
                                          cb->x,cb->y, // xP,yP
                                          1<<cb->log2Size, // int nCS,
                                          1<<cb->log2Size,
                                          1<<cb->log2Size, // int nPbW,int nPbH,
                                          &motion);

        tb->copy_reconstruction_from_image(ectx, ectx->img);
      }
    }
  }


  if (try_nonskip) {
    cb->PredMode = MODE_INTRA;

    cb = mIntraAlgo->analyze(ectx, ctxModel, cb);
  }

  return cb;
}
