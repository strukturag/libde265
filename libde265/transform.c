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


static const int tab8_22[] = { 29,30,31,32,33,33,34,34,35,35,36,36,37 /*,37*/ };

int table8_22(int qPi)
{
  if (qPi<30) return qPi;
  if (qPi>=43) return qPi-6;
  return tab8_22[qPi-30];
}


// (8.6.1)
void decode_quantization_parameters(decoder_context* ctx,
                                    slice_segment_header* shdr, int xC,int yC)
{
  pic_parameter_set* pps = ctx->current_pps;
  seq_parameter_set* sps = ctx->current_sps;

  // top left pixel position of current quantization group
  int xQG = xC - (xC & ((1<<pps->Log2MinCuQpDeltaSize)-1));
  int yQG = yC - (yC & ((1<<pps->Log2MinCuQpDeltaSize)-1));

  // if first QG in CU, remember last QPY of last CU previous QG

  if ((xQG & ((1<<sps->Log2CtbSizeY)-1)) == 0 &&
      (yQG & ((1<<sps->Log2CtbSizeY)-1)) == 0) {
    shdr->lastQPYinPreviousQG = shdr->currentQPY;
  }

  int qPY_PRED;
  bool firstQGInSlice;
  bool firstQGInTile = false; // TODO
  bool firstInCTBRow = false; // TODO
  
  int first_ctb_in_slice_RS = shdr->slice_segment_address;

  int SliceStartX = (first_ctb_in_slice_RS % sps->PicWidthInCtbsY) * sps->CtbSizeY;
  int SliceStartY = (first_ctb_in_slice_RS / sps->PicWidthInCtbsY) * sps->CtbSizeY;

  firstQGInSlice = (SliceStartX == xQG && SliceStartY == yQG);

  if (firstQGInSlice || firstQGInTile ||
      (firstInCTBRow && pps->entropy_coding_sync_enabled_flag)) {
    qPY_PRED = shdr->SliceQPY;
  }
  else {
    qPY_PRED = shdr->lastQPYinPreviousQG;
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

  shdr->qPYPrime = QPY + sps->QpBdOffset_Y;

  int qPiCb = Clip3(-sps->QpBdOffset_C,57, QPY+pps->pic_cb_qp_offset + shdr->slice_cb_qp_offset);
  int qPiCr = Clip3(-sps->QpBdOffset_C,57, QPY+pps->pic_cr_qp_offset + shdr->slice_cr_qp_offset);

  logtrace(LogTransform,"qPiCb:%d (%d %d), qPiCr:%d (%d %d)\n",
         qPiCb, pps->pic_cb_qp_offset, shdr->slice_cb_qp_offset,
         qPiCr, pps->pic_cr_qp_offset, shdr->slice_cr_qp_offset);

  int qPCb = table8_22(qPiCb);
  int qPCr = table8_22(qPiCr);

  shdr->qPCbPrime = qPCb + sps->QpBdOffset_C;
  shdr->qPCrPrime = qPCr + sps->QpBdOffset_C;

  set_QPY(ctx,xQG,yQG, QPY);
  shdr->currentQPY = QPY;

  logtrace(LogTransform,"qPY(%d,%d)= %d\n",xC,yC,QPY);
}


static int8_t mat_8_357[4][4] = {
  { 29, 55, 74, 84 },
  { 74, 74,  0,-74 },
  { 84,-29,-74, 55 },
  { 55,-84, 74,-29 }
};

void transform_dst(int16_t* in, int32_t* out, int shift)
{
  int rnd = 1<<(shift-1);

  logtrace(LogTransform,"");
  for (int i=0;i<4;i++) {
    logtrace(LogTransform,"%d ",in[i]);
  }
  logtrace(LogTransform,"* -> ");

  for (int i=0;i<4;i++) {
    int sum=0;

    for (int j=0;j<4;j++) {
      sum += mat_8_357[j][i] * in[j];
    }

    out[i] = Clip3(-32768,32767, (sum+rnd)>>shift);
  }

  for (int y=0;y<4;y++) {
    logtrace(LogTransform,"*%d ",out[y]);
  }
  logtrace(LogTransform,"*\n");
}


static int8_t mat_dct[32][32] = {
  { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
  { 90, 90, 88, 85, 82, 78, 73, 67, 61, 54, 46, 38, 31, 22, 13,  4,      -4,-13,-22,-31,-38,-46,-54,-61,-67,-73,-78,-82,-85,-88,-90,-90},
  { 90, 87, 80, 70, 57, 43, 25,  9, -9,-25,-43,-57,-70,-80,-87,-90,     -90,-87,-80,-70,-57,-43,-25, -9,  9, 25, 43, 57, 70, 80, 87, 90},
  { 90, 82, 67, 46, 22, -4,-31,-54,-73,-85,-90,-88,-78,-61,-38,-13,      13, 38, 61, 78, 88, 90, 85, 73, 54, 31,  4,-22,-46,-67,-82,-90},
  { 89, 75, 50, 18,-18,-50,-75,-89,-89,-75,-50,-18, 18, 50, 75, 89,      89, 75, 50, 18,-18,-50,-75,-89,-89,-75,-50,-18, 18, 50, 75, 89},
  { 88, 67, 31,-13,-54,-82,-90,-78,-46, -4, 38, 73, 90, 85, 61, 22,     -22,-61,-85,-90,-73,-38,  4, 46, 78, 90, 82, 54, 13,-31,-67,-88},
  { 87, 57,  9,-43,-80,-90,-70,-25, 25, 70, 90, 80, 43, -9,-57,-87,     -87,-57, -9, 43, 80, 90, 70, 25,-25,-70,-90,-80,-43,  9, 57, 87},
  { 85, 46,-13,-67,-90,-73,-22, 38, 82, 88, 54, -4,-61,-90,-78,-31,      31, 78, 90, 61,  4,-54,-88,-82,-38, 22, 73, 90, 67, 13,-46,-85},
  { 83, 36,-36,-83,-83,-36, 36, 83, 83, 36,-36,-83,-83,-36, 36, 83,      83, 36,-36,-83,-83,-36, 36, 83, 83, 36,-36,-83,-83,-36, 36, 83},
  { 82, 22,-54,-90,-61, 13, 78, 85, 31,-46,-90,-67,  4, 73, 88, 38,     -38,-88,-73, -4, 67, 90, 46,-31,-85,-78,-13, 61, 90, 54,-22,-82},
  { 80,  9,-70,-87,-25, 57, 90, 43,-43,-90,-57, 25, 87, 70, -9,-80,     -80, -9, 70, 87, 25,-57,-90,-43, 43, 90, 57,-25,-87,-70,  9, 80},
  { 78, -4,-82,-73, 13, 85, 67,-22,-88,-61, 31, 90, 54,-38,-90,-46,      46, 90, 38,-54,-90,-31, 61, 88, 22,-67,-85,-13, 73, 82,  4,-78},
  { 75,-18,-89,-50, 50, 89, 18,-75,-75, 18, 89, 50,-50,-89,-18, 75,      75,-18,-89,-50, 50, 89, 18,-75,-75, 18, 89, 50,-50,-89,-18, 75},
  { 73,-31,-90,-22, 78, 67,-38,-90,-13, 82, 61,-46,-88, -4, 85, 54,     -54,-85,  4, 88, 46,-61,-82, 13, 90, 38,-67,-78, 22, 90, 31,-73},
  { 70,-43,-87,  9, 90, 25,-80,-57, 57, 80,-25,-90, -9, 87, 43,-70,     -70, 43, 87, -9,-90,-25, 80, 57,-57,-80, 25, 90,  9,-87,-43, 70},
  { 67,-54,-78, 38, 85,-22,-90,  4, 90, 13,-88,-31, 82, 46,-73,-61,      61, 73,-46,-82, 31, 88,-13,-90, -4, 90, 22,-85,-38, 78, 54,-67},
  { 64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64,      64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64},
  { 61,-73,-46, 82, 31,-88,-13, 90, -4,-90, 22, 85,-38,-78, 54, 67,     -67,-54, 78, 38,-85,-22, 90,  4,-90, 13, 88,-31,-82, 46, 73,-61},
  { 57,-80,-25, 90, -9,-87, 43, 70,-70,-43, 87,  9,-90, 25, 80,-57,     -57, 80, 25,-90,  9, 87,-43,-70, 70, 43,-87, -9, 90,-25,-80, 57},
  { 54,-85, -4, 88,-46,-61, 82, 13,-90, 38, 67,-78,-22, 90,-31,-73,      73, 31,-90, 22, 78,-67,-38, 90,-13,-82, 61, 46,-88,  4, 85,-54},
  { 50,-89, 18, 75,-75,-18, 89,-50,-50, 89,-18,-75, 75, 18,-89, 50,      50,-89, 18, 75,-75,-18, 89,-50,-50, 89,-18,-75, 75, 18,-89, 50},
  { 46,-90, 38, 54,-90, 31, 61,-88, 22, 67,-85, 13, 73,-82,  4, 78,     -78, -4, 82,-73,-13, 85,-67,-22, 88,-61,-31, 90,-54,-38, 90,-46},
  { 43,-90, 57, 25,-87, 70,  9,-80, 80, -9,-70, 87,-25,-57, 90,-43,     -43, 90,-57,-25, 87,-70, -9, 80,-80,  9, 70,-87, 25, 57,-90, 43},
  { 38,-88, 73, -4,-67, 90,-46,-31, 85,-78, 13, 61,-90, 54, 22,-82,      82,-22,-54, 90,-61,-13, 78,-85, 31, 46,-90, 67,  4,-73, 88,-38},
  { 36,-83, 83,-36,-36, 83,-83, 36, 36,-83, 83,-36,-36, 83,-83, 36,      36,-83, 83,-36,-36, 83,-83, 36, 36,-83, 83,-36,-36, 83,-83, 36},
  { 31,-78, 90,-61,  4, 54,-88, 82,-38,-22, 73,-90, 67,-13,-46, 85,     -85, 46, 13,-67, 90,-73, 22, 38,-82, 88,-54, -4, 61,-90, 78,-31},
  { 25,-70, 90,-80, 43,  9,-57, 87,-87, 57, -9,-43, 80,-90, 70,-25,     -25, 70,-90, 80,-43, -9, 57,-87, 87,-57,  9, 43,-80, 90,-70, 25},
  { 22,-61, 85,-90, 73,-38, -4, 46,-78, 90,-82, 54,-13,-31, 67,-88,      88,-67, 31, 13,-54, 82,-90, 78,-46,  4, 38,-73, 90,-85, 61,-22},
  { 18,-50, 75,-89, 89,-75, 50,-18,-18, 50,-75, 89,-89, 75,-50, 18,      18,-50, 75,-89, 89,-75, 50,-18,-18, 50,-75, 89,-89, 75,-50, 18},
  { 13,-38, 61,-78, 88,-90, 85,-73, 54,-31,  4, 22,-46, 67,-82, 90,     -90, 82,-67, 46,-22, -4, 31,-54, 73,-85, 90,-88, 78,-61, 38,-13},
  {  9,-25, 43,-57, 70,-80, 87,-90, 90,-87, 80,-70, 57,-43, 25, -9,      -9, 25,-43, 57,-70, 80,-87, 90,-90, 87,-80, 70,-57, 43,-25,  9},
  {  4,-13, 22,-31, 38,-46, 54,-61, 67,-73, 78,-82, 85,-88, 90,-90,      90,-90, 88,-85, 82,-78, 73,-67, 61,-54, 46,-38, 31,-22, 13, -4}
};


void transform_dct(int16_t* in, int32_t* out, int nT, int shift)
{
  int rnd = 1<<(shift-1);

  logtrace(LogTransform,"DCT: ");
  for (int i=0;i<nT;i++) {
    logtrace(LogTransform,"*%d ",in[i]);
  }
  logtrace(LogTransform,"* -> ");

  int fact = (1<<(5-Log2(nT)));

  for (int i=0;i<nT;i++) {
    int sum=0;

    for (int j=0;j<nT;j++) {
      sum += mat_dct[fact*j][i] * in[j];
    }

    out[i] = Clip3(-32768,32767, (sum+rnd)>>shift);
  }

  for (int y=0;y<nT;y++) {
    logtrace(LogTransform,"*%d ",out[y]);
  }
  logtrace(LogTransform,"*\n");
}


void transform_coefficients(decoder_context* ctx, slice_segment_header* shdr,
                            int16_t* coeff, int coeffStride, int nT, int trType, int postShift)
{
  logtrace(LogTransform,"transform --- trType: %d nT: %d\n",trType,nT);

  if (trType==1) {
    int16_t g[4][4];
    int16_t col[4];
    int32_t out[4];

    for (int c=0;c<4;c++) {
      for (int y=0;y<4;y++) {
        col[y] = coeff[c+y*coeffStride];
      }

      transform_dst(col,out, 7);

      for (int y=0;y<4;y++) {
        g[y][c] = out[y];
      }
    }

    for (int y=0;y<4;y++) {
      transform_dst(&g[y][0], out,postShift);

      for (int x=0;x<4;x++) {
        coeff[x+y*coeffStride] = out[x];
      }
    }

  } else {

    int16_t g[nT][nT];
    int16_t col[nT];
    int32_t out[nT];

    for (int c=0;c<nT;c++) {
      for (int y=0;y<nT;y++) {
        col[y] = coeff[c+y*coeffStride];
      }

      transform_dct(col,out, nT, 7);

      for (int y=0;y<nT;y++) {
        g[y][c] = out[y];
      }
    }

    for (int y=0;y<nT;y++) {
      transform_dct(&g[y][0], out,nT,postShift);

      for (int x=0;x<nT;x++) {
        coeff[x+y*coeffStride] = out[x];
      }
    }
  }
}


static const int levelScale[] = { 40,45,51,57,64,72 };

// (8.6.2) and (8.6.3)
void scale_coefficients(decoder_context* ctx, slice_segment_header* shdr,
                        int xT,int yT, int nT, int cIdx)
{
  seq_parameter_set* sps = ctx->current_sps;

  int qP;
  switch (cIdx) {
  case 0: qP = shdr->qPYPrime;  break;
  case 1: qP = shdr->qPCbPrime; break;
  case 2: qP = shdr->qPCrPrime; break;
  default: qP = 0; assert(0); break; // should never happen
  }

  logtrace(LogTransform,"qP: %d\n",qP);

  int16_t* coeff;
  int      coeffStride;
  get_coeff_plane(ctx,cIdx, &coeff,&coeffStride);


  if (shdr->cu_transquant_bypass_flag) {
    assert(false); // TODO
  }
  else {
    // (8.6.3)

    int bdShift = (cIdx==0 ? sps->BitDepth_Y : sps->BitDepth_C) + Log2(nT) - 5;

    logtrace(LogTransform,"coefficients IN:\n");
    for (int y=0;y<nT;y++) {
      logtrace(LogTransform,"  ");
      for (int x=0;x<nT;x++) {
        logtrace(LogTransform,"*%3d ", coeff[x+xT+(y+yT)*coeffStride]);
      }
      logtrace(LogTransform,"*\n");
    }

    if (sps->scaling_list_enable_flag==0) {
      for (int y=0;y<nT;y++)
        for (int x=0;x<nT;x++) {
          coeff[xT+x+(y+yT)*coeffStride] = Clip3(-32768,32767,
                                                 ( (coeff[x+xT+(y+yT)*coeffStride]
                                                    * 16 * levelScale[qP%6] << (qP/6))
                                                   + (1<<(bdShift-1)) ) >> bdShift);
        }
    }
    else {
      assert(false); // TODO
    }

    logtrace(LogTransform,"coefficients OUT:\n");
    for (int y=0;y<nT;y++) {
      logtrace(LogTransform,"  ");
      for (int x=0;x<nT;x++) {
        logtrace(LogTransform,"*%3d ", coeff[x+xT+(y+yT)*coeffStride]);
      }
      logtrace(LogTransform,"*\n");
    }

    int bdShift2 = (cIdx==0) ? 20-sps->BitDepth_Y : 20-sps->BitDepth_C;

    if (get_transform_skip_flag(ctx,xT,yT,cIdx)) {
      for (int y=0;y<nT;y++)
        for (int x=0;x<nT;x++) {
          int16_t c = coeff[x+xT+(y+yT)*coeffStride] << 7;
          coeff[x+xT+(y+yT)*coeffStride] = (c+(1<<(bdShift2-1)))>>bdShift2;
        }
    }
    else {
      int trType;

      if (nT==4 && cIdx==0 && get_pred_mode(ctx,xT,yT)==MODE_INTRA) {
        trType=1;
      }
      else {
        trType=0;
      }

      transform_coefficients(ctx,shdr, &coeff[xT+yT*coeffStride], coeffStride, nT, trType, bdShift2);

      /*
        for (int y=0;y<nT;y++)
        for (int x=0;x<nT;x++) {
        coeff[xT+x+(yT+y)*coeffStride] = (coeff[xT+x+(yT+y)*coeffStride] + (1<<(bdShift2-1))) >> bdShift2;
        }
      */
    }
  }

  logtrace(LogTransform,"transform OUT:\n");
  for (int y=0;y<nT;y++) {
    logtrace(LogTransform,"  ");
    for (int x=0;x<nT;x++) {
      logtrace(LogTransform,"*%3d ", coeff[x+xT+(y+yT)*coeffStride]);
    }
    logtrace(LogTransform,"*\n");
  }


  uint8_t* pred;
  int      stride;
  get_image_plane(ctx,cIdx,&pred,&stride);
  pred += xT + yT*stride;

  logtrace(LogTransform,"prediction (cIdx:%d):\n",cIdx);
  for (int y=0;y<nT;y++) {
    for (int x=0;x<nT;x++) {
      logtrace(LogTransform,"*%02x ", pred[x+y*stride]);
    }

    logtrace(LogTransform,"*\n");
  }  

  for (int y=0;y<nT;y++)
    for (int x=0;x<nT;x++) {
      pred[x+y*stride] = Clip1_8bit(pred[x+y*stride] + coeff[xT+x+(yT+y)*coeffStride]);
    }


  logtrace(LogTransform,"pixels (cIdx:%d), position %d %d:\n",cIdx, xT,yT);

  for (int y=0;y<nT;y++) {
    logtrace(LogTransform,"RECO-%d-%d-%d ",xT,yT+y,cIdx);

    for (int x=0;x<nT;x++) {
      logtrace(LogTransform,"*%02x ", pred[x+y*stride]);
    }

    logtrace(LogTransform,"*\n");
  }  
}
