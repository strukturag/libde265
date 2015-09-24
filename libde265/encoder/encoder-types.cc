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

  mBytesPerRow = bytes_per_pixel * (1<<log2Size);
  mHeight = 1<<log2Size;
}


small_image_buffer::~small_image_buffer()
{
  delete[] mBuf;
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
                            int cIdx) const
{
  // chroma adapted position
  int xC=x0;
  int yC=y0;

  if (cIdx>0) {
    xC>>=1;
    yC>>=1;
  }


  if (!reconstruction[cIdx]) {

    reconstruction[cIdx] = std::make_shared<small_image_buffer>(log2TbSize, sizeof(uint8_t));

    if (cb->PredMode == MODE_INTRA) {

      //enum IntraPredMode intraPredMode  = img->get_IntraPredMode(x0,y0);
      enum IntraPredMode intraPredMode  = intra_mode;

      if (cIdx>0) {
        intraPredMode = intra_mode_chroma;
      }

      //printf("reconstruct TB (%d;%d): intra mode (cIdx=%d) = %d\n",xC,yC,cIdx,intraPredMode);

      //decode_intra_prediction(img, xC,yC,  intraPredMode, 1<< log2TbSize   , cIdx);

      //printf("access intra-prediction of TB %p\n",this);

      intra_prediction[cIdx]->copy_to(*reconstruction[cIdx]);
      /*
        copy_subimage(img->get_image_plane_at_pos(cIdx,xC,yC),
        img->get_image_stride(cIdx),
        intra_prediction[cIdx]->get_buffer<uint8_t>(), 1<<log2TbSize,
        1<<log2TbSize, 1<<log2TbSize);
      */
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
                                 reconstruction[cIdx]->get_buffer<uint8_t>(), 1<<log2TbSize,
                                 dequant_coeff, log2TbSize,   trType);
  }


  // copy reconstruction into image

  copy_subimage(img->get_image_plane_at_pos(cIdx,xC,yC),
                img->get_image_stride(cIdx),
                reconstruction[cIdx]->get_buffer<uint8_t>(), 1<<log2TbSize,
                1<<log2TbSize, 1<<log2TbSize);

  //printf("--- RECO intra prediction %d %d ---\n",x0,y0);
  //printBlk("RECO",ptr,1<<log2TbSize,stride);
}


void enc_tb::debug_writeBlack(encoder_context* ectx, de265_image* img) const
{
  if (split_transform_flag) {
    for (int i=0;i<4;i++) {
      children[i]->debug_writeBlack(ectx,img);
    }
  }
  else {
    //reconstruct_tb(ectx, img, x,y, log2Size, 0);

    int size = 1<<(log2Size<<1);
    std::vector<uint8_t> buf(size);
    memset(&buf[0],0x12,size);

    int cIdx=0;
    int xC=x,yC=y;

    copy_subimage(img->get_image_plane_at_pos(cIdx,xC,yC),
                  img->get_image_stride(cIdx),
                  &buf[0], 1<<log2Size,
                  1<<log2Size, 1<<log2Size);
  }
}


void enc_tb::reconstruct(encoder_context* ectx, de265_image* img) const
{
  if (split_transform_flag) {
    for (int i=0;i<4;i++) {
      children[i]->reconstruct(ectx,img);
    }
  }
  else {
    reconstruct_tb(ectx, img, x,y, log2Size, 0);

    if (log2Size>2) {
      reconstruct_tb(ectx, img, x,y, log2Size-1, 1);
      reconstruct_tb(ectx, img, x,y, log2Size-1, 2);
    }
    else if (blkIdx==3) {
      int xBase = x - (1<<log2Size);
      int yBase = y - (1<<log2Size);

      reconstruct_tb(ectx, img, xBase,yBase, log2Size, 1);
      reconstruct_tb(ectx, img, xBase,yBase, log2Size, 2);
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
  logdebug(LogEncoderMetadata,"enc_cb::writeMetadata_CBOnly (%d;%d x%d)\n",x,y,1<<log2Size);

  int missing = whatFlags & ~metadata_in_image;
  if (!missing) {
    return 0; // leave early if we have everything we need
  }

  int written = 0;

  if (missing & METADATA_CT_DEPTH) {
    img->set_ctDepth(x,y,log2Size, ctDepth);
    written |= METADATA_CT_DEPTH;
  }

  metadata_in_image |= written;

  return written;
}


int enc_cb::writeMetadata(encoder_context* ectx, de265_image* img, int whatFlags)
{
  logdebug(LogEncoderMetadata,"enc_cb::writeMetadata (%d;%d x%d)\n",x,y,1<<log2Size);

  int missing = whatFlags & ~metadata_in_image;
  if (!missing) {
    return 0; // leave early if we have everything we need
  }

  int written = 0;

  if (split_cu_flag) {
    int written = whatFlags;

    for (int i=0;i<4;i++)
      written &= children[i]->writeMetadata(ectx, img,whatFlags);
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
  logdebug(LogEncoderMetadata,"enc_tb::writeMetadata (%d;%d x%d)\n",x,y,1<<log2Size);

  int missing = whatFlags & ~metadata_in_image;
  if (!missing) {
    return 0; // leave early if we have everything we need
  }

  int written = 0;


  // write CB data

  // We have to write intra modes at the highest level possible, because the PB-size is
  // half of the CB size and TB sizes may be smaller than the min PB size.
  // If we would write in a lower TB, it could be smaller than PB-min and no metadata
  // would be written at all.

  if (missing & METADATA_INTRA_MODES) {
    if (cb->PartMode == PART_2Nx2N && TrafoDepth==0) {
      img->set_IntraPredMode(x,y,log2Size, intra_mode);
    }
    else if (cb->PartMode == PART_NxN && TrafoDepth==1) {
      img->set_IntraPredMode(x,y,log2Size, intra_mode);
    }

    //img->set_IntraPredModeC(int x,int y) const

    logdebug(LogEncoderMetadata,"  writeIntraPredMode=%d (log2size=%d)\n",log2Size);

    written |= METADATA_INTRA_MODES;
  }


  if (split_transform_flag) {
    int written = whatFlags;

    for (int i=0;i<4;i++)
      written &= children[i]->writeMetadata(ectx, img,whatFlags);
  }
  else {
    //printf("write intra pred mode (%d;%d) = %d\n",x,y,intra_mode);

    if (missing & METADATA_RECONSTRUCTION_BORDERS ||
        missing & METADATA_RECONSTRUCTION) {
      reconstruct(ectx, img);

      written |= (METADATA_RECONSTRUCTION_BORDERS |
                  METADATA_RECONSTRUCTION);
    }
  }

  metadata_in_image |= written;

  return written;
}


void enc_tb::writeSurroundingMetadata(encoder_context* ectx,
                                      de265_image* img, int whatFlags,
                                      const rectangle& borderRect)
{
  logdebug(LogEncoderMetadata,
           "enc_tb::writeSurroundingMetadata (%d;%d x%d) (%d;%d;%d;%d) flags=%d\n",x,y,1<<log2Size,
           borderRect.left,borderRect.right,
           borderRect.top, borderRect.bottom,
           whatFlags);

  // --- first check whether we have to go up in the tree ---

  const bool partOfBorderIsLeftOrTopOfTB = (borderRect.left <= x || borderRect.top <= y);

  if (partOfBorderIsLeftOrTopOfTB) {
    if (parent) {
      // there is a parent TB

      parent->writeSurroundingMetadata(ectx, img, whatFlags, borderRect);
    }
    else {
      // go through parent CB

      assert(cb);
      cb->writeSurroundingMetadata(ectx, img, whatFlags, borderRect);
    }
  }
  else {
    // We do not have to go further up. Process tree down.

    writeSurroundingMetadataDown(ectx, img, whatFlags, borderRect);
  }
}


void enc_cb::writeSurroundingMetadata(encoder_context* ectx,
                                      de265_image* img, int whatFlags,
                                      const rectangle& borderRect)
{
  logdebug(LogEncoderMetadata,
           "enc_cb::writeSurroundingMetadata (%d;%d x%d) (%d;%d;%d;%d) flags=%d\n",
           x,y,1<<log2Size,
           borderRect.left,borderRect.right,
           borderRect.top,borderRect.bottom,
           whatFlags);

  const bool partOfBorderIsLeftOrTopOfTB = (borderRect.left <= x || borderRect.top <= y);

  if (parent && partOfBorderIsLeftOrTopOfTB) {
    // go further up if we have to and this is not the root

    parent->writeSurroundingMetadata(ectx, img, whatFlags, borderRect);
  }
  else {
    // if we do not need to go up, go down.

    writeSurroundingMetadataDown(ectx, img, whatFlags, borderRect);
  }
}


void enc_tb::writeSurroundingMetadataDown(encoder_context* ectx,
                                          de265_image* img, int whatFlags,
                                          const rectangle& borderRect)
{
  logdebug(LogEncoderMetadata,
           "enc_tb::writeSurroundingMetadataDown (%d;%d x%d) (%d;%d;%d;%d)\n",x,y,1<<log2Size,
           borderRect.left,borderRect.right, borderRect.top,borderRect.bottom);


  // If we do not overlap with the border, we can stop here to process the tree.

  if (borderRect.left <= x && borderRect.top <= y) {
    // case A: we do not overlap with the border (right side or below border) -> NOP
    return;
  }
  if (x+(1<<log2Size) < borderRect.left ||
      y+(1<<log2Size) < borderRect.top) {
    // case B: we do not overlap with the border (left side of or above borderRect) -> NOP

    assert(0); // actually, in our implementation, this case never occurs (true ?)
    return;
  }
  else if (x >= borderRect.right ||
           y >= borderRect.bottom) {
    // case C: we do not overlap with the border (right side or below border) -> NOP
    return;
  }



  // --- write metadata or recurse to deeper TB level ---

  if ((metadata_in_image & whatFlags) == whatFlags) {
    // nothing to do, data already exists
  }
  else if (!split_transform_flag) {
    writeMetadata(ectx, img, whatFlags);
  }
  else {
    for (int i=0;i<4;i++)
      if (children[i] != NULL) {
        children[i]->writeSurroundingMetadataDown(ectx, img, whatFlags, borderRect);
      }
  }
}


void enc_cb::writeSurroundingMetadataDown(encoder_context* ectx,
                                          de265_image* img, int whatFlags,
                                          const rectangle& borderRect)
{
  logdebug(LogEncoderMetadata,
           "enc_cb::writeSurroundingMetadataDown (%d;%d x%d) (%d;%d;%d;%d)\n",x,y,1<<log2Size,
           borderRect.left,borderRect.right, borderRect.top,borderRect.bottom);

  if ((metadata_in_image & whatFlags) == whatFlags) {
    // nothing to do, data already exists

    return;
  }


  // If we do not overlap with the border, we can stop here to process the tree.

  if (borderRect.left <= x && borderRect.top <= y) {
    // case A: we do not overlap with the border (right side or below border) -> NOP
    return;
  }
  if (x+(1<<log2Size) < borderRect.left ||
      y+(1<<log2Size) < borderRect.top) {
    // case B: we do not overlap with the border (left side of or above borderRect) -> NOP

    assert(0); // actually, in our implementation, this case never occurs (true ?)
    return;
  }
  else if (x >= borderRect.right ||
           y >= borderRect.bottom) {
    // case C: we do not overlap with the border (right side or below border) -> NOP
    return;
  }


  if (!split_cu_flag) {
    writeMetadata_CBOnly(ectx, img, whatFlags);

    transform_tree->writeSurroundingMetadataDown(ectx,img,whatFlags,borderRect);
  }
  else {
    for (int i=0;i<4;i++)
      if (children[i] != NULL) {
        children[i]->writeSurroundingMetadataDown(ectx, img, whatFlags, borderRect);
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
  parent = NULL;
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


/*
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
*/

void enc_cb::reconstruct(encoder_context* ectx, de265_image* img) const
{
  assert(0);
  if (split_cu_flag) {
    for (int i=0;i<4;i++) {
      children[i]->reconstruct(ectx, img);
    }
  }
  else {
    //write_to_image(img);
    transform_tree->reconstruct(ectx,img);
  }
}
