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

#include "encode.h"
#include "slice.h"
#include "intrapred.h"


static void encode_split_cu_flag(encoder_context* ectx,
                                 int x0, int y0, int ctDepth, int split_flag)
{
  // check if neighbors are available

  int availableL = check_CTB_available(ectx->img,ectx->shdr, x0,y0, x0-1,y0);
  int availableA = check_CTB_available(ectx->img,ectx->shdr, x0,y0, x0,y0-1);

  int condL = 0;
  int condA = 0;

  if (availableL && ectx->img->get_ctDepth(x0-1,y0) > ctDepth) condL=1;
  if (availableA && ectx->img->get_ctDepth(x0,y0-1) > ctDepth) condA=1;

  int contextOffset = condL + condA;
  int context = contextOffset;

  // decode bit

  logtrace(LogSlice,"> split_cu_flag = %d\n",split_flag);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_SPLIT_CU_FLAG + context], split_flag);
}


static void encode_part_mode(encoder_context* ectx,
                             enum PredMode PredMode, enum PartMode PartMode)
{
  logtrace(LogSlice,"> part_mode = %d\n",PartMode);

  if (PredMode == MODE_INTRA) {
    int bin = (PartMode==PART_2Nx2N);
    ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_PART_MODE], bin);
  }
  else {
    assert(0); // TODO
  }
}


static void encode_prev_intra_luma_pred_flag(encoder_context* ectx, int intraPred)
{
  int bin = (intraPred>=0);

  logtrace(LogSlice,"> prev_intra_luma_pred_flag = %d\n",bin);

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_PREV_INTRA_LUMA_PRED_FLAG], bin);
}

static void encode_intra_mpm_or_rem(encoder_context* ectx, int intraPred)
{
  if (intraPred>=0) {
    logtrace(LogSlice,"> mpm_idx = %d\n",intraPred);
    ectx->cabac_encoder->write_CABAC_TU_bypass(intraPred, 2);
  }
  else {
    logtrace(LogSlice,"> rem_intra_luma_pred_mode = %d\n",-intraPred-1);
    ectx->cabac_encoder->write_CABAC_FL_bypass(-intraPred-1, 5);
  }
}


static void encode_intra_chroma_pred_mode(encoder_context* ectx, int mode)
{
  logtrace(LogSlice,"> intra_chroma_pred_mode = %d\n",mode);

  if (mode==4) {
    ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE],0);
  }
  else {
    ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_INTRA_CHROMA_PRED_MODE],1);
    ectx->cabac_encoder->write_CABAC_FL_bypass(mode, 2);
  }
}


void encode_coding_unit(encoder_context* ectx,
                        int x0,int y0, int log2CbSize)
{
  de265_image* img = ectx->img;
  const slice_segment_header* shdr = ectx->shdr;
  const seq_parameter_set* sps = &img->sps;


  int nCbS = 1<<log2CbSize;

  enum PredMode PredMode = img->get_pred_mode(x0,y0);
  enum PartMode PartMode = PART_2Nx2N;

  if (PredMode != MODE_INTRA ||
      log2CbSize == sps->Log2MinCbSizeY) {
    PartMode = img->get_PartMode(x0,y0);
    encode_part_mode(ectx, PredMode, PartMode);
  }

  if (PredMode == MODE_INTRA) {

    int availableA0 = check_CTB_available(img, shdr, x0,y0, x0-1,y0);
    int availableB0 = check_CTB_available(img, shdr, x0,y0, x0,y0-1);

    if (PartMode==PART_2Nx2N) {
      int PUidx = (x0>>sps->Log2MinPUSize) + (y0>>sps->Log2MinPUSize)*sps->PicWidthInMinPUs;

      int candModeList[3];
      fillIntraPredModeCandidates(candModeList,x0,y0,PUidx,
                                  availableA0,availableB0, img);

      enum IntraPredMode mode = img->get_IntraPredMode(x0,y0);
      int intraPred = find_intra_pred_mode(mode, candModeList);
      encode_prev_intra_luma_pred_flag(ectx, intraPred);
      encode_intra_mpm_or_rem(ectx, intraPred);
    }
    else {
      int pbOffset = nCbS/2;
      int PUidx;

      int intraPred[4];

      for (int j=0;j<nCbS;j+=pbOffset)
        for (int i=0;i<nCbS;i+=pbOffset)
          {
            int x=x0+i, y=y0+j;

            int availableA = availableA0 || (i>0); // left candidate always available for right blk
            int availableB = availableB0 || (j>0); // top candidate always available for bottom blk

            PUidx = (x>>sps->Log2MinPUSize) + (y>>sps->Log2MinPUSize)*sps->PicWidthInMinPUs;

            int candModeList[3];
            fillIntraPredModeCandidates(candModeList,x,y,PUidx,
                                        availableA,availableB, img);

            enum IntraPredMode mode = img->get_IntraPredMode(x,y);
            intraPred[2*j+i] = find_intra_pred_mode(mode, candModeList);
          }

      for (int i=0;i<4;i++)
        encode_prev_intra_luma_pred_flag(ectx, intraPred[i]);

      for (int i=0;i<4;i++)
        encode_intra_mpm_or_rem(ectx, intraPred[i]);
    }
    
    
  }
}


void encode_quadtree(encoder_context* ectx,
                     int x0,int y0, int log2CbSize, int ctDepth)
{
  de265_image* img = ectx->img;
  const seq_parameter_set* sps = &img->sps;

  int split_flag;

  /*
     CU split flag:

          | overlaps | minimum ||
     case | border   | size    ||  split
     -----+----------+---------++----------
       A  |    0     |     0   || optional
       B  |    0     |     1   ||    0
       C  |    1     |     0   ||    1
       D  |    1     |     1   ||    0
   */
  if (x0+(1<<log2CbSize) <= sps->pic_width_in_luma_samples &&
      y0+(1<<log2CbSize) <= sps->pic_height_in_luma_samples &&
      log2CbSize > sps->Log2MinCbSizeY) {

    // case A

    split_flag = (img->get_ctDepth(x0,y0) != ctDepth);

    encode_split_cu_flag(ectx, x0,y0, ctDepth, split_flag);
  } else {
    // case B/C/D

    if (log2CbSize > sps->Log2MinCbSizeY) { split_flag=1; }
    else                                  { split_flag=0; }
  }



  if (split_flag) {
    int x1 = x0 + (1<<(log2CbSize-1));
    int y1 = y0 + (1<<(log2CbSize-1));

    encode_quadtree(ectx,x0,y0, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples)
      encode_quadtree(ectx,x1,y0, log2CbSize-1, ctDepth+1);

    if (y1<sps->pic_height_in_luma_samples)
      encode_quadtree(ectx,x0,y1, log2CbSize-1, ctDepth+1);

    if (x1<sps->pic_width_in_luma_samples &&
        y1<sps->pic_height_in_luma_samples)
      encode_quadtree(ectx,x1,y1, log2CbSize-1, ctDepth+1);
  }
  else {
    encode_coding_unit(ectx,x0,y0, log2CbSize);
  }
}


void encode_image(encoder_context* ectx)
{
  de265_image* img = ectx->img;
  int log2ctbSize = img->sps.Log2CtbSizeY;

  for (int ctbY=0;ctbY<img->sps.PicHeightInCtbsY;ctbY++)
    for (int ctbX=0;ctbX<img->sps.PicWidthInCtbsY;ctbX++)
      {
        encode_quadtree(ectx, ctbX,ctbY, log2ctbSize, 0);
      }
}
