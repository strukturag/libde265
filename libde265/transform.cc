/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
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


const int tab8_22[] = { 29,30,31,32,33,33,34,34,35,35,36,36,37 /*,37*/ };


static int nDCT_4x4, nDCT_8x8, nDCT_16x16, nDCT_32x32, nDST_4x4;
static int nSkip_4x4;
static int nCoeff4x4[16+1], nCoeff8x8[64+1], nCoeff16x16[16*16+1], nCoeff32x32[32*32+1];

extern "C" {
LIBDE265_API void showTransformProfile()
{
  const int nDCT_sum = nDST_4x4 + nDCT_4x4 + nDCT_8x8 + nDCT_16x16 + nDCT_32x32 + nSkip_4x4;
  fprintf(stderr,"transform usage:\n");
  fprintf(stderr,"  IDST   4x4:  %8d  %4.1f%%\n",nDST_4x4,(float)(nDST_4x4 * 100) / nDCT_sum);
  fprintf(stderr,"  IDCT   4x4:  %8d  %4.1f%%\n",nDCT_4x4,(float)(nDCT_4x4 * 100) / nDCT_sum);
  fprintf(stderr,"  IDCT   8x8:  %8d  %4.1f%%\n",nDCT_8x8,(float)(nDCT_8x8 * 100) / nDCT_sum);
  fprintf(stderr,"  IDCT 16x16:  %8d  %4.1f%%\n",nDCT_16x16,(float)(nDCT_16x16 * 100) / nDCT_sum);
  fprintf(stderr,"  IDCT 32x32:  %8d  %4.1f%%\n",nDCT_32x32,(float)(nDCT_32x32 * 100) / nDCT_sum);
  fprintf(stderr,"  Skip   4x4:  %8d  %4.1f%%\n",nSkip_4x4,(float)(nSkip_4x4 * 100) / nDCT_sum);

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
}


// (8.6.1)
void decode_quantization_parameters(thread_context* tctx, int xC,int yC,
                                    int xCUBase, int yCUBase)
{
  logtrace(LogTransform,">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> decode_quantization_parameters(int xC,int yC)=(%d,%d)\n", xC,yC);

  pic_parameter_set* pps = &tctx->img->pps;
  seq_parameter_set* sps = &tctx->img->sps;
  slice_segment_header* shdr = tctx->shdr;

  // top left pixel position of current quantization group
  int xQG = xCUBase - (xCUBase & ((1<<pps->Log2MinCuQpDeltaSize)-1));
  int yQG = yCUBase - (yCUBase & ((1<<pps->Log2MinCuQpDeltaSize)-1));

  logtrace(LogTransform,"QG: %d,%d\n",xQG,yQG);


  // we only have to set QP in the first call in a quantization-group

  /* TODO: check why this does not work with HoneyBee stream

  if (xQG == tctx->currentQG_x &&
      yQG == tctx->currentQG_y)
    {
      return;
    }
  */

  // if first QG in CU, remember last QPY of last CU previous QG

  if (xQG != tctx->currentQG_x ||
      yQG != tctx->currentQG_y)
    {
      tctx->lastQPYinPreviousQG = tctx->currentQPY;
      tctx->currentQG_x = xQG;
      tctx->currentQG_y = yQG;
    }

  int qPY_PRED;

  // first QG in CTB row ?
  
  int ctbLSBMask = ((1<<sps->Log2CtbSizeY)-1);
  bool firstInCTBRow = (xQG == 0 && ((yQG & ctbLSBMask)==0));

  // first QG in slice ?    TODO: a "firstQG" flag in the thread context would be faster

  int first_ctb_in_slice_RS = tctx->shdr->SliceAddrRS;

  int SliceStartX = (first_ctb_in_slice_RS % sps->PicWidthInCtbsY) * sps->CtbSizeY;
  int SliceStartY = (first_ctb_in_slice_RS / sps->PicWidthInCtbsY) * sps->CtbSizeY;

  bool firstQGInSlice = (SliceStartX == xQG && SliceStartY == yQG);

  // first QG in tile ?

  bool firstQGInTile = false;
  if (pps->tiles_enabled_flag) {
    if ((xQG & ((1 << sps->Log2CtbSizeY)-1)) == 0 &&
        (yQG & ((1 << sps->Log2CtbSizeY)-1)) == 0)
      {
        int ctbX = xQG >> sps->Log2CtbSizeY;
        int ctbY = yQG >> sps->Log2CtbSizeY;

        firstQGInTile = pps->is_tile_start_CTB(ctbX,ctbY); // TODO: this is slow
      }
  }


  if (firstQGInSlice || firstQGInTile ||
      (firstInCTBRow && pps->entropy_coding_sync_enabled_flag)) {
    qPY_PRED = tctx->shdr->SliceQPY;
  }
  else {
    qPY_PRED = tctx->lastQPYinPreviousQG;
  }


  int qPYA,qPYB;

  if (tctx->img->available_zscan(xQG,yQG, xQG-1,yQG)) {
    int xTmp = (xQG-1) >> sps->Log2MinTrafoSize;
    int yTmp = (yQG  ) >> sps->Log2MinTrafoSize;
    int minTbAddrA = pps->MinTbAddrZS[xTmp + yTmp*sps->PicWidthInTbsY];
    int ctbAddrA = minTbAddrA >> (2 * (sps->Log2CtbSizeY-sps->Log2MinTrafoSize));
    if (ctbAddrA == tctx->CtbAddrInTS) {
      qPYA = tctx->img->get_QPY(xQG-1,yQG);
    }
    else {
      qPYA = qPY_PRED;
    }
  }
  else {
    qPYA = qPY_PRED;
  }

  if (tctx->img->available_zscan(xQG,yQG, xQG,yQG-1)) {
    int xTmp = (xQG  ) >> sps->Log2MinTrafoSize;
    int yTmp = (yQG-1) >> sps->Log2MinTrafoSize;
    int minTbAddrB = pps->MinTbAddrZS[xTmp + yTmp*sps->PicWidthInTbsY];
    int ctbAddrB = minTbAddrB >> (2 * (sps->Log2CtbSizeY-sps->Log2MinTrafoSize));
    if (ctbAddrB == tctx->CtbAddrInTS) {
      qPYB = tctx->img->get_QPY(xQG,yQG-1);
    }
    else {
      qPYB = qPY_PRED;
    }
  }
  else {
    qPYB = qPY_PRED;
  }

  qPY_PRED = (qPYA + qPYB + 1)>>1;

  logtrace(LogTransform,"qPY_PRED = %d  (%d, %d)\n",qPY_PRED, qPYA, qPYB);

  int QPY = ((qPY_PRED + tctx->CuQpDelta + 52+2*sps->QpBdOffset_Y) %
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

  int log2CbSize = tctx->img->get_log2CbSize(xCUBase, yCUBase);
  tctx->img->set_QPY(xCUBase, yCUBase, log2CbSize, QPY);
  tctx->currentQPY = QPY;

  /*
  printf("SET QPY POC=%d %d;%d-%d;%d = %d\n",ctx->img->PicOrderCntVal,xCUBase,yCUBase,
         xCUBase+(1<<log2CbSize),yCUBase+(1<<log2CbSize), QPY);
  */

  logtrace(LogTransform,"qPY(%d,%d,%d)= %d, qPYPrime=%d\n",
           xCUBase,yCUBase,1<<log2CbSize,QPY,tctx->qPYPrime);
}



void transform_coefficients(decoder_context* ctx,
                            int16_t* coeff, int coeffStride, int nT, int trType, int postShift,
                            uint8_t* dst, int dstStride)
{
  logtrace(LogTransform,"transform --- trType: %d nT: %d\n",trType,nT);

  if (trType==1) {

    ctx->acceleration.transform_4x4_luma_add_8(dst, coeff, dstStride);
    nDST_4x4++;

  } else {

    /**/ if (nT==4)  { ctx->acceleration.transform_4x4_add_8(dst,coeff,dstStride); nDCT_4x4++; }
    else if (nT==8)  { ctx->acceleration.transform_8x8_add_8(dst,coeff,dstStride); nDCT_8x8++; }
    else if (nT==16) { ctx->acceleration.transform_16x16_add_8(dst,coeff,dstStride); nDCT_16x16++; }
    else             { ctx->acceleration.transform_32x32_add_8(dst,coeff,dstStride); nDCT_32x32++; }
  }
}


static const int levelScale[] = { 40,45,51,57,64,72 };

// (8.6.2) and (8.6.3)
void scale_coefficients(thread_context* tctx,
                        int xT,int yT, // position of TU in frame (chroma adapted)
                        int x0,int y0, // position of CU in frame (chroma adapted)
                        int nT, int cIdx,
                        bool transform_skip_flag, bool intra)
{
  seq_parameter_set* sps = &tctx->img->sps;
  pic_parameter_set* pps = &tctx->img->pps;

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
  pred = tctx->img->get_image_plane_at_pos(cIdx, xT,yT);
  stride = tctx->img->get_image_stride(cIdx);

  //fprintf(stderr,"POC=%d pred: %p (%d;%d stride=%d)\n",ctx->img->PicOrderCntVal,pred,xT,yT,stride);

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

  if (tctx->cu_transquant_bypass_flag) {
    //assert(false); // TODO

    for (int i=0;i<tctx->nCoeff[cIdx];i++) {
      int32_t currCoeff  = tctx->coeffList[cIdx][i];
      tctx->coeffBuf[ tctx->coeffPos[cIdx][i] ] = currCoeff;
    }

    tctx->decctx->acceleration.transform_bypass_8(pred, coeff, nT, stride);
  }
  else {
    // (8.6.3)

    int bdShift = (cIdx==0 ? sps->BitDepth_Y : sps->BitDepth_C) + Log2(nT) - 5;

    logtrace(LogTransform,"bdShift=%d\n",bdShift);

    logtrace(LogTransform,"dequant %d;%d cIdx=%d qp=%d\n",xT*(cIdx?2:1),yT*(cIdx?2:1),cIdx,qP);


    if (sps->scaling_list_enable_flag==0) {

      //const int m_x_y = 16;
      const int m_x_y = 1;
      bdShift -= 4;  // this is equivalent to having a m_x_y of 16 and we can use 32bit integers

      const int offset = (1<<(bdShift-1));
      const int fact = m_x_y * levelScale[qP%6] << (qP/6);

      for (int i=0;i<tctx->nCoeff[cIdx];i++) {

        // usually, this needs to be 64bit, but because we modify the shift above, we can use 16 bit
        int32_t currCoeff  = tctx->coeffList[cIdx][i];

        logtrace(LogTransform,"coefficient[%d] = %d\n",tctx->coeffPos[cIdx][i],tctx->coeffList[cIdx][i]);

        currCoeff = Clip3(-32768,32767,
                          ( (currCoeff * fact + offset ) >> bdShift));

        tctx->coeffBuf[ tctx->coeffPos[cIdx][i] ] = currCoeff;
      }
    }
    else {
      const int offset = (1<<(bdShift-1));

      uint8_t* sclist;
      int matrixID = cIdx;
      if (!intra) {
        if (nT<32) { matrixID += 3; }
        else { matrixID++; }
      }

      switch (nT) {
      case  4: sclist = &pps->scaling_list.ScalingFactor_Size0[matrixID][0][0]; break;
      case  8: sclist = &pps->scaling_list.ScalingFactor_Size1[matrixID][0][0]; break;
      case 16: sclist = &pps->scaling_list.ScalingFactor_Size2[matrixID][0][0]; break;
      case 32: sclist = &pps->scaling_list.ScalingFactor_Size3[matrixID][0][0]; break;
      default: assert(0);
      }

      for (int i=0;i<tctx->nCoeff[cIdx];i++) {
        int pos = tctx->coeffPos[cIdx][i];
        int x = pos%nT;
        int y = pos/nT;

        const int m_x_y = sclist[x+y*nT];
        const int fact = m_x_y * levelScale[qP%6] << (qP/6);

        int64_t currCoeff  = tctx->coeffList[cIdx][i];

        currCoeff = Clip3(-32768,32767,
                          ( (currCoeff * fact + offset ) >> bdShift));

        tctx->coeffBuf[ tctx->coeffPos[cIdx][i] ] = currCoeff;
      }
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
             transform_skip_flag);

    if (transform_skip_flag) {

      tctx->decctx->acceleration.transform_skip_8(pred, coeff, stride);

      nSkip_4x4++;
    }
    else {
      int trType;

      if (nT==4 && cIdx==0 && tctx->img->get_pred_mode(xT,yT)==MODE_INTRA) {
        trType=1;
      }
      else {
        trType=0;
      }

      transform_coefficients(tctx->decctx, coeff, coeffStride, nT, trType, bdShift2,
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
