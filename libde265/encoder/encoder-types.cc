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

#include "encoder-types.h"
#include "encoder-context.h"
#include "slice.h"
#include "scan.h"
#include "intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include <iostream>


int allocTB = 0;
int allocCB = 0;

#define DEBUG_ALLOCS 0



void enc_node::save(const de265_image* img)
{
  delete[] mReconstruction;

  int blkSize = Log2SizeToArea(log2Size);
  mReconstruction = new uint8_t[blkSize * 3/2];

  int w = 1<<log2Size;

  copy_subimage(mReconstruction, w,
                img->get_image_plane_at_pos(0, x,y),
                img->get_image_stride(0),
                w,w);

  copy_subimage(mReconstruction + blkSize, w>>1,
                img->get_image_plane_at_pos(1, x>>1,y>>1),
                img->get_image_stride(1),
                w>>1,w>>1);

  copy_subimage(mReconstruction + blkSize*5/4, w>>1,
                img->get_image_plane_at_pos(2, x>>1,y>>1),
                img->get_image_stride(2),
                w>>1,w>>1);
}


void enc_node::restore(de265_image* img)
{
  assert(mReconstruction);

  int blkSize = Log2SizeToArea(log2Size);
  int w = 1<<log2Size;

  copy_subimage(img->get_image_plane_at_pos(0, x,y),
                img->get_image_stride(0),
                mReconstruction, w,
                w,w);

  copy_subimage(img->get_image_plane_at_pos(1, x>>1,y>>1),
                img->get_image_stride(1),
                mReconstruction + blkSize, w>>1,
                w>>1,w>>1);

  copy_subimage(img->get_image_plane_at_pos(2, x>>1,y>>1),
                img->get_image_stride(2),
                mReconstruction + blkSize*5/4, w>>1,
                w>>1,w>>1);
}


void enc_cb::save(const de265_image* img)
{
  enc_node::save(img);

  // TODO: save metadata in node buffer memory
}


void enc_cb::restore(de265_image* img)
{
  enc_node::restore(img);

  // write back all the metadata

  write_to_image(img);
}


void enc_cb::set_rqt_root_bf_from_children_cbf()
{
  assert(transform_tree);
  inter.rqt_root_cbf = (transform_tree->cbf[0] |
                        transform_tree->cbf[1] |
                        transform_tree->cbf[2]);
}




alloc_pool enc_tb::mMemPool(sizeof(enc_tb));

enc_tb::enc_tb()
  : split_transform_flag(false)
{
  coeff[0]=coeff[1]=coeff[2]=NULL;

  if (DEBUG_ALLOCS) { allocTB++; printf("TB  : %d\n",allocTB); }
}


enc_tb::~enc_tb()
{
  if (split_transform_flag) {
    for (int i=0;i<4;i++) {
      delete children[i];
    }
  }
  else {
    for (int i=0;i<3;i++) {
      delete[] coeff[i];
    }
  }

  if (DEBUG_ALLOCS) { allocTB--; printf("TB ~: %d\n",allocTB); }
}


void enc_tb::alloc_coeff_memory(int cIdx, int tbSize)
{
  assert(coeff[cIdx]==NULL);
  coeff[cIdx] = new int16_t[tbSize*tbSize];
}


void enc_tb::reconstruct_tb(encoder_context* ectx,
                            de265_image* img,
                            int x0,int y0,  // luma
                            int log2TbSize, // chroma adapted
                            const enc_cb* cb, int cIdx) const
{
  // chroma adapted position
  int xC=x0;
  int yC=y0;

  if (cIdx>0) {
    xC>>=1;
    yC>>=1;
  }

  if (cb->PredMode == MODE_INTRA) {

    enum IntraPredMode intraPredMode  = img->get_IntraPredMode(x0,y0);

    if (cIdx>0) {
      intraPredMode = cb->intra.chroma_mode;
      //intraPredMode = lumaPredMode_to_chromaPredMode(intraPredMode, cb->intra.chroma_mode);
    }

    decode_intra_prediction(img, xC,yC,  intraPredMode, 1<< log2TbSize   , cIdx);
  }
  else {
    int size = 1<<log2TbSize;

    uint8_t* dst_ptr  = img->get_image_plane_at_pos(cIdx, xC,  yC  );
    int dst_stride  = img->get_image_stride(cIdx);

    uint8_t* src_ptr  = ectx->prediction->get_image_plane_at_pos(cIdx, xC,  yC  );
    int src_stride  = ectx->prediction->get_image_stride(cIdx);

    for (int y=0;y<size;y++) {
      for (int x=0;x<size;x++) {
        dst_ptr[y*dst_stride+x] = src_ptr[y*src_stride+x];
      }
    }
  }

  ALIGNED_16(int16_t) dequant_coeff[32*32];

  if (cbf[cIdx]) dequant_coefficients(dequant_coeff, coeff[cIdx], log2TbSize, cb->qp);

  //printf("--- quantized coeffs ---\n");
  //printBlk(coeff[0],1<<log2BlkSize,1<<log2BlkSize);

  //printf("--- dequantized coeffs ---\n");
  //printBlk(dequant_coeff[0],1<<log2BlkSize,1<<log2BlkSize);

  //printf("--- plane at %d %d / %d ---\n",x0,y0,cIdx);

  uint8_t* ptr  = img->get_image_plane_at_pos(cIdx, xC,  yC  );
  int stride  = img->get_image_stride(cIdx);

  int trType = (cIdx==0 && log2TbSize==2); // TODO: inter

  //printf("--- prediction %d %d / %d ---\n",x0,y0,cIdx);
  //printBlk("prediction",ptr,1<<log2TbSize,stride);

  if (cbf[cIdx]) inv_transform(&ectx->acceleration,
                               ptr,stride,   dequant_coeff, log2TbSize,   trType);


  //printf("--- RECO intra prediction %d %d ---\n",x0,y0);
  //printBlk("prediction",ptr,1<<log2TbSize,stride);

  //dequant_and_add_transform(accel, img, x0,y0, qp);

  //printf("--- RECO add residual %d %d ---\n",x0,y0);
  //img->printBlk(x0,y0,0,log2CbSize);
}


void enc_tb::reconstruct(encoder_context* ectx,
                         de265_image* img,
                         const enc_cb* cb,
                         int blkIdx) const
{
  if (split_transform_flag) {
    for (int i=0;i<4;i++) {
      children[i]->reconstruct(ectx,img,
                               cb, i);
    }
  }
  else {
    reconstruct_tb(ectx, img, x,y, log2Size, cb, 0);

    if (log2Size>2) {
      reconstruct_tb(ectx, img, x,y, log2Size-1, cb, 1);
      reconstruct_tb(ectx, img, x,y, log2Size-1, cb, 2);
    }
    else if (blkIdx==3) {
      int xBase = x - (1<<log2Size);
      int yBase = y - (1<<log2Size);

      reconstruct_tb(ectx, img, xBase,yBase, log2Size, cb, 1);
      reconstruct_tb(ectx, img, xBase,yBase, log2Size, cb, 2);
    }
  }
}


void enc_tb::set_cbf_flags_from_children()
{
  assert(split_transform_flag);

  cbf[0] = 0;
  cbf[1] = 0;
  cbf[2] = 0;

  for (int i=0;i<4;i++) {
    cbf[0] |= children[i]->cbf[0];
    cbf[1] |= children[i]->cbf[1];
    cbf[2] |= children[i]->cbf[2];
  }
}




alloc_pool enc_cb::mMemPool(sizeof(enc_cb), 200);


enc_cb::enc_cb()
  : split_cu_flag(false),
    cu_transquant_bypass_flag(false),
    pcm_flag(false),
    transform_tree(NULL),
    distortion(0),
    rate(0)
{
  if (DEBUG_ALLOCS) { allocCB++; printf("CB  : %d\n",allocCB); }
}

enc_cb::~enc_cb()
{
  if (split_cu_flag) {
    for (int i=0;i<4;i++) {
      delete children[i];
    }
  }
  else {
    delete transform_tree;
  }

  if (DEBUG_ALLOCS) { allocCB--; printf("CB ~: %d\n",allocCB); }
}


void enc_cb::write_to_image(de265_image* img) const
{
  //printf("write_to_image %d %d size:%d\n",x,y,1<<log2Size);


  if (!split_cu_flag) {
    img->set_log2CbSize(x,y,log2Size, true);
    img->set_ctDepth(x,y,log2Size, ctDepth);
    assert(pcm_flag==0);
    img->set_pcm_flag(x,y,log2Size, pcm_flag);
    img->set_cu_transquant_bypass(x,y,log2Size, cu_transquant_bypass_flag);
    img->set_QPY(x,y,log2Size, qp);
    img->set_pred_mode(x,y, log2Size, PredMode);
    img->set_PartMode(x,y, PartMode);

    if (PredMode == MODE_INTRA) {
      //img->set_ChromaIntraPredMode(x,y,log2Size, intra.chroma_mode);

      if (PartMode == PART_NxN) {
        int h = 1<<(log2Size-1);
        img->set_IntraPredMode(x  ,y  ,log2Size-1, intra.pred_mode[0]);
        img->set_IntraPredMode(x+h,y  ,log2Size-1, intra.pred_mode[1]);
        img->set_IntraPredMode(x  ,y+h,log2Size-1, intra.pred_mode[2]);
        img->set_IntraPredMode(x+h,y+h,log2Size-1, intra.pred_mode[3]);
      }
      else {
        img->set_IntraPredMode(x,y,log2Size, intra.pred_mode[0]);
      }
    }
    else {
      int nC = 1<<log2Size;
      int nC2 = nC>>1;
      int nC4 = nC>>2;
      int nC3 = nC-nC4;
      switch (PartMode) {
      case PART_2Nx2N:
        img->set_mv_info(x,y,nC,nC, inter.pb[0].motion);
        break;
      case PART_NxN:
        img->set_mv_info(x    ,y    ,nC2,nC2, inter.pb[0].motion);
        img->set_mv_info(x+nC2,y    ,nC2,nC2, inter.pb[1].motion);
        img->set_mv_info(x    ,y+nC2,nC2,nC2, inter.pb[2].motion);
        img->set_mv_info(x+nC2,y+nC2,nC2,nC2, inter.pb[3].motion);
        break;
      case PART_2NxN:
        img->set_mv_info(x,y    ,nC,nC2, inter.pb[0].motion);
        img->set_mv_info(x,y+nC2,nC,nC2, inter.pb[1].motion);
        break;
      case PART_Nx2N:
        img->set_mv_info(x    ,y,nC2,nC, inter.pb[0].motion);
        img->set_mv_info(x+nC2,y,nC2,nC, inter.pb[1].motion);
        break;
      case PART_2NxnU:
        img->set_mv_info(x,y    ,nC,nC4, inter.pb[0].motion);
        img->set_mv_info(x,y+nC4,nC,nC3, inter.pb[1].motion);
        break;
      case PART_2NxnD:
        img->set_mv_info(x,y    ,nC,nC3, inter.pb[0].motion);
        img->set_mv_info(x,y+nC3,nC,nC4, inter.pb[1].motion);
        break;
      case PART_nLx2N:
        img->set_mv_info(x    ,y,nC4,nC, inter.pb[0].motion);
        img->set_mv_info(x+nC4,y,nC3,nC, inter.pb[1].motion);
        break;
      case PART_nRx2N:
        img->set_mv_info(x    ,y,nC3,nC, inter.pb[0].motion);
        img->set_mv_info(x+nC3,y,nC4,nC, inter.pb[1].motion);
        break;
      }
    }
  }
  else {
    for (int i=0;i<4;i++) {
      if (children[i]) {
        children[i]->write_to_image(img);
      }
    }
  }
}


void enc_cb::reconstruct(encoder_context* ectx, de265_image* img) const
{
  if (split_cu_flag) {
    for (int i=0;i<4;i++) {
      children[i]->reconstruct(ectx, img);
    }
  }
  else {
    write_to_image(img);
    transform_tree->reconstruct(ectx,img,this,0);
  }
}
