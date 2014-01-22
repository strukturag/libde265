/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "transform.h"
#include "util.h"
#include <assert.h>


static int nDCT_4x4, nDCT_8x8, nDCT_16x16, nDCT_32x32, nDST_4x4;
static int nSkip_4x4;
static int nCoeff4x4[16+1], nCoeff8x8[64+1], nCoeff16x16[16*16+1], nCoeff32x32[32*32+1];

LIBDE265_API void showTransformProfile()
{
  fprintf(stderr,"transform usage:\n");
  fprintf(stderr,"  DST 4x4:   %d\n",nDST_4x4);
  fprintf(stderr,"  DCT 4x4:   %d\n",nDCT_4x4);
  fprintf(stderr,"  DCT 8x8:   %d\n",nDCT_8x8);
  fprintf(stderr,"  DCT 16x16: %d\n",nDCT_16x16);
  fprintf(stderr,"  DCT 32x32: %d\n",nDCT_32x32);
  fprintf(stderr,"  Skip 4x4:   %d\n",nSkip_4x4);

  fprintf(stderr,"nCoeff DCT 4x4: ");
  for (int i=1;i<=16;i++)
    fprintf(stderr,"%d ",nCoeff4x4[i]);
  fprintf(stderr,"\n");

  fprintf(stderr,"nCoeff DCT 8x8: ");
  for (int i=1;i<=8*8;i++)
    fprintf(stderr,"%d ",nCoeff8x8[i]);
  fprintf(stderr,"\n");

  fprintf(stderr,"nCoeff DCT 16x16: ");
  for (int i=1;i<=16*16;i++)
    fprintf(stderr,"%d ",nCoeff16x16[i]);
  fprintf(stderr,"\n");

  fprintf(stderr,"nCoeff DCT 32x32: ");
  for (int i=1;i<=32*32;i++)
    fprintf(stderr,"%d ",nCoeff32x32[i]);
  fprintf(stderr,"\n");
}


static const int tab8_22[] = { 29,30,31,32,33,33,34,34,35,35,36,36,37 /*,37*/ };

int table8_22(int qPi)
{
  if (qPi<30) return qPi;
  if (qPi>=43) return qPi-6;
  return tab8_22[qPi-30];
}


// (8.6.1)
void decode_quantization_parameters(decoder_context* ctx,
                                    thread_context* tctx, int xC,int yC)
{
  pic_parameter_set* pps = ctx->current_pps;
  seq_parameter_set* sps = ctx->current_sps;
  slice_segment_header* shdr = tctx->shdr;

  // top left pixel position of current quantization group
  int xQG = xC - (xC & ((1<<pps->Log2MinCuQpDeltaSize)-1));
  int yQG = yC - (yC & ((1<<pps->Log2MinCuQpDeltaSize)-1));

  // if first QG in CU, remember last QPY of last CU previous QG

  if ((xQG & ((1<<sps->Log2CtbSizeY)-1)) == 0 &&
      (yQG & ((1<<sps->Log2CtbSizeY)-1)) == 0) {
    tctx->lastQPYinPreviousQG = tctx->currentQPY;
  }

  int qPY_PRED;
  bool firstQGInSlice;
  bool firstQGInTile = false; // TODO
  bool firstInCTBRow = (xC==0); // TODO
  
  int first_ctb_in_slice_RS = tctx->shdr->slice_segment_address;

  int SliceStartX = (first_ctb_in_slice_RS % sps->PicWidthInCtbsY) * sps->CtbSizeY;
  int SliceStartY = (first_ctb_in_slice_RS / sps->PicWidthInCtbsY) * sps->CtbSizeY;

  firstQGInSlice = (SliceStartX == xQG && SliceStartY == yQG);

  if (firstQGInSlice || firstQGInTile ||
      (firstInCTBRow && pps->entropy_coding_sync_enabled_flag)) {
    qPY_PRED = tctx->shdr->SliceQPY;
  }
  else {
    qPY_PRED = tctx->lastQPYinPreviousQG;
  }


  int qPYA,qPYB;

  if (available_zscan(ctx,xQG,yQG, xQG-1,yQG)) {
    // unused: int xTmp = (xQG-1) >> sps->Log2MinTrafoSize;
    // unused: int yTmp = (yQG  ) >> sps->Log2MinTrafoSize;
    // unused: int minTbAddrA = pps->MinTbAddrZS[xTmp + yTmp*sps->PicWidthInTbsY];
    // unused: int ctbAddrA = (minTbAddrA>>2)*(sps->Log2CtbSizeY-sps->Log2MinTrafoSize);

    qPYA = get_QPY(ctx,xQG-1,yQG);
  }
  else {
    qPYA = qPY_PRED;
  }

  if (available_zscan(ctx,xQG,yQG, xQG,yQG-1)) {
    // unused: int xTmp = (xQG  ) >> sps->Log2MinTrafoSize;
    // unused: int yTmp = (yQG-1) >> sps->Log2MinTrafoSize;
    // unused: int minTbAddrA = pps->MinTbAddrZS[xTmp + yTmp*sps->PicWidthInTbsY];
    // unused: int ctbAddrA = (minTbAddrA>>2)*(sps->Log2CtbSizeY-sps->Log2MinTrafoSize);

    qPYB = get_QPY(ctx,xQG,yQG-1);
  }
  else {
    qPYB = qPY_PRED;
  }

  qPY_PRED = (qPYA + qPYB + 1)>>1;

  int QPY = ((qPY_PRED + shdr->CuQpDelta + 52+2*sps->QpBdOffset_Y) %
             (52 + sps->QpBdOffset_Y)) - sps->QpBdOffset_Y;

  tctx->qPYPrime = QPY + sps->QpBdOffset_Y;

  int qPiCb = Clip3(-sps->QpBdOffset_C,57, QPY+pps->pic_cb_qp_offset + shdr->slice_cb_qp_offset);
  int qPiCr = Clip3(-sps->QpBdOffset_C,57, QPY+pps->pic_cr_qp_offset + shdr->slice_cr_qp_offset);

  logtrace(LogTransform,"qPiCb:%d (%d %d), qPiCr:%d (%d %d)\n",
           qPiCb, pps->pic_cb_qp_offset, shdr->slice_cb_qp_offset,
           qPiCr, pps->pic_cr_qp_offset, shdr->slice_cr_qp_offset);

  int qPCb = table8_22(qPiCb);
  int qPCr = table8_22(qPiCr);

  tctx->qPCbPrime = qPCb + sps->QpBdOffset_C;
  tctx->qPCrPrime = qPCr + sps->QpBdOffset_C;

  set_QPY(ctx,xQG,yQG, QPY);
  tctx->currentQPY = QPY;

  logtrace(LogTransform,"qPY(%d,%d)= %d\n",xC,yC,QPY);
}



void transform_coefficients(decoder_context* ctx, slice_segment_header* shdr,
                            int16_t* coeff, int coeffStride, int nT, int trType, int postShift,
                            uint8_t* dst, int dstStride)
{
  logtrace(LogTransform,"transform --- trType: %d nT: %d\n",trType,nT);

  if (trType==1) {

    ctx->lowlevel.transform_4x4_luma_add_8(dst, coeff, dstStride);
    nDST_4x4++;

  } else {

    /**/ if (nT==4)  { ctx->lowlevel.transform_4x4_add_8(dst,coeff,dstStride); nDCT_4x4++; }
    else if (nT==8)  { ctx->lowlevel.transform_8x8_add_8(dst,coeff,dstStride); nDCT_8x8++; }
    else if (nT==16) { ctx->lowlevel.transform_16x16_add_8(dst,coeff,dstStride); nDCT_16x16++; }
    else             { ctx->lowlevel.transform_32x32_add_8(dst,coeff,dstStride); nDCT_32x32++; }
  }
}


static const int levelScale[] = { 40,45,51,57,64,72 };

// (8.6.2) and (8.6.3)
void scale_coefficients(decoder_context* ctx, thread_context* tctx,
                        int xT,int yT, // position of TU in frame (chroma adapted)
                        int x0,int y0, // position of CU in frame (chroma adapted)
                        int nT, int cIdx)
{
  seq_parameter_set* sps = ctx->current_sps;
  slice_segment_header* shdr = tctx->shdr;

  int qP;
  switch (cIdx) {
  case 0: qP = tctx->qPYPrime;  break;
  case 1: qP = tctx->qPCbPrime; break;
  case 2: qP = tctx->qPCrPrime; break;
  default: qP = 0; assert(0); break; // should never happen
  }

  logtrace(LogTransform,"qP: %d\n",qP);

  //printf("residual %d;%d cIdx=%d qp=%d\n",xT * (cIdx?2:1),yT * (cIdx?2:1),cIdx,qP);


  int16_t* coeff;
  int      coeffStride;

  coeff = tctx->coeffBuf;
  coeffStride = nT;





  uint8_t* pred;
  int      stride;
  get_image_plane(ctx->img,cIdx,&pred,&stride);
  pred += xT + yT*stride;

  /*
  int x,y;
  for (y=0;y<nT;y++)
    {
      printf("P: ");

      for (x=0;x<nT;x++)
        {
          printf("%02x ",pred[x+y*stride]);
        }

      printf("\n");
    }
  */

  if (shdr->cu_transquant_bypass_flag) {
    assert(false); // TODO
  }
  else {
    // (8.6.3)

    int bdShift = (cIdx==0 ? sps->BitDepth_Y : sps->BitDepth_C) + Log2(nT) - 5;

    logtrace(LogTransform,"bdShift=%d\n",bdShift);

    if (sps->scaling_list_enable_flag==0) {
      for (int i=0;i<tctx->nCoeff[cIdx];i++) {

        int currCoeff  = tctx->coeffList[cIdx][i];

        const int m_x_y = 16;
        currCoeff = Clip3(-32768,32767,
                          ( (currCoeff * m_x_y * levelScale[qP%6] << (qP/6))
                            + (1<<(bdShift-1)) ) >> bdShift);

        tctx->coeffBuf[ tctx->coeffPos[cIdx][i] ] = currCoeff;
      }
    }
    else {
      assert(false); // TODO
    }

    logtrace(LogTransform,"coefficients OUT:\n");
    for (int y=0;y<nT;y++) {
      logtrace(LogTransform,"  ");
      for (int x=0;x<nT;x++) {
        logtrace(LogTransform,"*%3d ", coeff[x+y*coeffStride]);
      }
      logtrace(LogTransform,"*\n");
    }

    int bdShift2 = (cIdx==0) ? 20-sps->BitDepth_Y : 20-sps->BitDepth_C;

    logtrace(LogTransform,"bdShift2=%d\n",bdShift2);

    logtrace(LogSlice,"get_transform_skip_flag(%d,%d, cIdx=%d)=%d\n",xT,yT,cIdx,
             get_transform_skip_flag(ctx,xT,yT,cIdx));

    if (get_transform_skip_flag(ctx,xT,yT,cIdx)) { // NOTE: could add shortcut nT==4 && ...

      ctx->lowlevel.transform_skip_8(pred, coeff, stride);

      nSkip_4x4++;
    }
    else {
      int trType;

      if (nT==4 && cIdx==0 && get_pred_mode(ctx->img,ctx->current_sps,xT,yT)==MODE_INTRA) {
        trType=1;
      }
      else {
        trType=0;
      }

      transform_coefficients(ctx,shdr, coeff, coeffStride, nT, trType, bdShift2,
                             pred, stride);
    }
  }


  logtrace(LogTransform,"pixels (cIdx:%d), position %d %d:\n",cIdx, xT,yT);

  for (int y=0;y<nT;y++) {
    logtrace(LogTransform,"RECO-%d-%d-%d ",xT,yT+y,cIdx);

    for (int x=0;x<nT;x++) {
      logtrace(LogTransform,"*%02x ", pred[x+y*stride]);
    }

    logtrace(LogTransform,"*\n");
  }  

  /*
  for (y=0;y<nT;y++)
    {
      printf("C: ");

      for (x=0;x<nT;x++)
        {
          printf("%4d ",coeff[x+y*nT]);
        }

      printf("\n");
    }

  for (y=0;y<nT;y++)
    {
      for (x=0;x<nT;x++)
        {
          printf("%02x ",pred[x+y*stride]);
        }

      printf("\n");
    }
  */

  // zero out scrap coefficient buffer again

  for (int i=0;i<tctx->nCoeff[cIdx];i++) {
    tctx->coeffBuf[ tctx->coeffPos[cIdx][i] ] = 0;
  }
}
