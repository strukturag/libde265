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


small_image_buffer::small_image_buffer(int log2Size,int bytes_per_pixel)
{
  int bytes = (1<<(log2Size<<1))*bytes_per_pixel;
  mBuf = new uint8_t[bytes];
  mStride = 1<<log2Size;
}


small_image_buffer::~small_image_buffer()
{
  delete[] mBuf;
}


void enc_node::save(const de265_image* img)
{
  logtrace(LogEncoder,"PERF-WARNING: enc_node save %d;%d size:%d\n",x,y,1<<log2Size);

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
  logtrace(LogEncoder,"PERF-WARNING: enc_node restore %d;%d size:%d\n",x,y,1<<log2Size);

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

enc_tb::enc_tb(int x,int y,int log2TbSize, enc_cb* _cb)
  : enc_node(x,y,log2TbSize)
{
  parent = NULL;
  cb = _cb;
  downPtr = NULL;

  split_transform_flag = false;
  coeff[0]=coeff[1]=coeff[2]=NULL;

  TrafoDepth = 0;
  cbf[0] = cbf[1] = cbf[2] = 0;

  metadata_in_image = 0;

  distortion = 0;
  rate = 0;
  rate_withoutCbfChroma = 0;

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

    //enum IntraPredMode intraPredMode  = img->get_IntraPredMode(x0,y0);
    enum IntraPredMode intraPredMode  = intra_mode;

    if (cIdx>0) {
      intraPredMode = intra_mode_chroma;
    }

    //printf("reconstruct TB (%d;%d): intra mode (cIdx=%d) = %d\n",xC,yC,cIdx,intraPredMode);

    //decode_intra_prediction(img, xC,yC,  intraPredMode, 1<< log2TbSize   , cIdx);

    copy_subimage(img->get_image_plane_at_pos(cIdx,xC,yC),
                  img->get_image_stride(cIdx),
                  intra_prediction[cIdx]->get_buffer<uint8_t>(), 1<<log2TbSize,
                  1<<log2TbSize, 1<<log2TbSize);
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
  //printBlk("qcoeffs",coeff[0],1<<log2TbSize,1<<log2TbSize);

  //printf("--- dequantized coeffs ---\n");
  //printBlk("dequant",dequant_coeff,1<<log2TbSize,1<<log2TbSize);

  uint8_t* ptr  = img->get_image_plane_at_pos(cIdx, xC,  yC  );
  int stride  = img->get_image_stride(cIdx);

  int trType = (cIdx==0 && log2TbSize==2); // TODO: inter

  //printf("--- prediction %d %d / %d ---\n",x0,y0,cIdx);
  //printBlk("prediction",ptr,1<<log2TbSize,stride);

  if (cbf[cIdx]) inv_transform(&ectx->acceleration,
                               ptr,stride,   dequant_coeff, log2TbSize,   trType);

  //printf("--- RECO intra prediction %d %d ---\n",x0,y0);
  //printBlk("RECO",ptr,1<<log2TbSize,stride);
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


int enc_cb::writeMetadata_CBOnly(encoder_context* ectx, de265_image* img, int whatFlags)
{
  //printf("enc_cb::writeMetadata_CBOnly (%d;%d x%d)\n",x,y,1<<log2Size);

  int missing = whatFlags & ~metadata_in_image;
  if (!missing) {
    return 0; // leave early if we have everything we need
  }

  int written = 0;

  if (split_cu_flag) {
    for (int i=0;i<4;i++)
      written = children[i]->writeMetadata_CBOnly(ectx, img,whatFlags);
  }
  else {
    // write CB data

    // TODO

    // metadata_in_image |= whatFlags;
  }

  metadata_in_image |= written;

  return written;
}


int enc_cb::writeMetadata(encoder_context* ectx, de265_image* img, int whatFlags)
{
  //printf("enc_cb::writeMetadata (%d;%d x%d)\n",x,y,1<<log2Size);

  int missing = whatFlags & ~metadata_in_image;
  if (!missing) {
    return 0; // leave early if we have everything we need
  }

  int written = 0;

  if (split_cu_flag) {
    for (int i=0;i<4;i++)
      written = children[i]->writeMetadata(ectx, img,whatFlags);
  }
  else {
    written |= writeMetadata_CBOnly(ectx,img,whatFlags);
    written |= transform_tree->writeMetadata(ectx, img,whatFlags);
  }

  metadata_in_image |= written;

  return written;
}


int enc_tb::writeMetadata(encoder_context* ectx, de265_image* img, int whatFlags)
{
  //printf("enc_tb::writeMetadata (%d;%d x%d)\n",x,y,1<<log2Size);

  int missing = whatFlags & ~metadata_in_image;
  if (!missing) {
    return 0; // leave early if we have everything we need
  }

  int written = 0;

  if (split_transform_flag) {
    for (int i=0;i<4;i++)
      written = children[i]->writeMetadata(ectx, img,whatFlags);
  }
  else {
    //printf("write intra pred mode (%d;%d) = %d\n",x,y,intra_mode);

    if (missing & METADATA_INTRA_MODES) {
      img->set_IntraPredMode(x,y,log2Size, intra_mode);
      //img->set_IntraPredModeC(int x,int y) const

      written |= METADATA_INTRA_MODES;
    }

    if (missing & METADATA_RECONSTRUCTION_BORDERS ||
        missing & METADATA_RECONSTRUCTION) {
      reconstruct(ectx, img, cb, blkIdx);

      written |= (METADATA_RECONSTRUCTION_BORDERS |
                  METADATA_RECONSTRUCTION);
    }
  }

  metadata_in_image |= written;

  return written;
}


void enc_tb::writeSurroundingMetadata(encoder_context* ectx,
                                      de265_image* img, int whatFlags, const rectangle& rect)
{
  //printf("enc_tb::writeSurroundingMetadata (%d;%d x%d) (%d;%d;%d;%d) flags=%d\n",x,y,1<<log2Size,
  //       rect.left,rect.right, rect.top,rect.bottom, whatFlags);

  // top and left border of block surrounding must always be within TB, as we call
  // this only for sub-blocks within this TB
  assert(rect.left >= x && rect.top >= y);

  if (rect.left == x || rect.top == y) {
    if (parent) {
      parent->writeSurroundingMetadata(ectx, img, whatFlags, rect);
    }
    else {
      assert(cb);
      // TODO cb->nodeNeedsReconstruction(whatFlags, rect);

      // TODO: remove this later when we go up through CB [does not work, because we cannot
      // reconstruct the currently coded block]
      if (rect.left <= x && rect.top <= y) {
        // NOP
      }
      else {
        //writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
      }

      cb->writeSurroundingMetadata(ectx, img, whatFlags, rect);
    }
  }
  else {
    writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
  }
}


void enc_cb::writeSurroundingMetadata(encoder_context* ectx,
                                      de265_image* img, int whatFlags, const rectangle& rect)
{
  //printf("enc_cb::writeSurroundingMetadata (%d;%d x%d) (%d;%d;%d;%d) flags=%d\n",x,y,1<<log2Size,
  //       rect.left,rect.right, rect.top,rect.bottom, whatFlags);

  // top and left border of block surrounding must always be within CB, as we call
  // this only for sub-blocks within this CB
  assert(rect.left >= x && rect.top >= y);

  if (parent && (rect.left == x || rect.top == y)) {
    parent->writeSurroundingMetadata(ectx, img, whatFlags, rect);
  }
  else {
    writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
  }
}


bool overlaps(const enc_node::rectangle& border, int x0,int y0,int x1,int y1)
{
  // left edge (full height)

  if (y1 <= border.top-1 || y0 >= border.bottom) {
  }
  else if (x0 < border.left && x1 >= border.left) {
    return true;
  }

  // top edge (excluding left pixel)

  if (x1 <= border.left || x0 >= border.right) {
  }
  else if (y0 < border.top && y1 >= border.top) {
    return true;
  }

  return false;
}


void enc_tb::writeSurroundingMetadataDown(encoder_context* ectx,
                                          de265_image* img, int whatFlags, const rectangle& rect)
{
  //printf("enc_tb::writeSurroundingMetadataDown (%d;%d x%d) (%d;%d;%d;%d)\n",x,y,1<<log2Size,
  //       rect.left,rect.right, rect.top,rect.bottom);

  if ((metadata_in_image & whatFlags) == whatFlags) {
    // nothing to do, data already exists
  }
  else if (!split_transform_flag) {
    if (rect.left <= x && rect.top <= y) {
      // NOP
    }
    else {
      writeMetadata(ectx, img, whatFlags);
    }
  }
  else {
    int xhalf = x+(1<<(log2Size-1));
    int yhalf = y+(1<<(log2Size-1));
    int xend  = x+(1<<log2Size);
    int yend  = y+(1<<log2Size);

    if (overlaps(rect, x,y, xhalf,yhalf)) {
      children[0]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }

    if (overlaps(rect, xhalf,y, xend,yhalf) && children[1]) {
      children[1]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }

    if (overlaps(rect, x,yhalf, xhalf,yend) && children[2]) {
      children[2]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }

    if (overlaps(rect, xhalf,yhalf, xend,yend)) {
      children[3]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }
  }
}


void enc_cb::writeSurroundingMetadataDown(encoder_context* ectx,
                                          de265_image* img, int whatFlags, const rectangle& rect)
{
  //printf("enc_cb::writeSurroundingMetadataDown (%d;%d x%d) (%d;%d;%d;%d)\n",x,y,1<<log2Size,
  //       rect.left,rect.right, rect.top,rect.bottom);

  if ((metadata_in_image & whatFlags) == whatFlags) {
    // nothing to do, data already exists
  }
  else if (!split_cu_flag) {
    if (rect.left <= x && rect.top <= y) {
      // NOP
    }
    else {
      writeMetadata_CBOnly(ectx, img, whatFlags);

      transform_tree->writeSurroundingMetadataDown(ectx,img,whatFlags,rect);
    }
  }
  else {
    int xhalf = x+(1<<(log2Size-1));
    int yhalf = y+(1<<(log2Size-1));
    int xend  = x+(1<<log2Size);
    int yend  = y+(1<<log2Size);

    if (overlaps(rect, x,y, xhalf,yhalf)) {
      children[0]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }

    if (overlaps(rect, xhalf,y, xend,yhalf) && children[1]) {
      children[1]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }

    if (overlaps(rect, x,yhalf, xhalf,yend) && children[2]) {
      children[2]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }

    if (overlaps(rect, xhalf,yhalf, xend,yend)) {
      children[3]->writeSurroundingMetadataDown(ectx, img, whatFlags, rect);
    }
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
  downPtr = NULL;
  metadata_in_image = 0;

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

      if (PartMode == PART_NxN) {
        int h = 1<<(log2Size-1);
        img->set_IntraPredMode(x  ,y  ,log2Size-1, transform_tree->children[0]->intra_mode);
        img->set_IntraPredMode(x+h,y  ,log2Size-1, transform_tree->children[1]->intra_mode);
        img->set_IntraPredMode(x  ,y+h,log2Size-1, transform_tree->children[2]->intra_mode);
        img->set_IntraPredMode(x+h,y+h,log2Size-1, transform_tree->children[3]->intra_mode);
      }
      else {
        img->set_IntraPredMode(x,y,log2Size, transform_tree->intra_mode);
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
  assert(0);
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
