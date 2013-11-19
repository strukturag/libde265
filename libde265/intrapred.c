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

#include "intrapred.h"
#include "transform.h"
#include "util.h"
#include <assert.h>


#include <sys/types.h>
#include <string.h>


int nIntraPredictions;

void showIntraPredictionProfile()
{
  printf("nIntraPredictions: %d\n", nIntraPredictions);
}


void print_border(uint8_t* data, int nT)
{
  for (int i=-2*nT ; i<=2*nT ; i++) {
    if (i==0 || i==1 || i==-nT || i==nT+1) {
      logtrace(LogIntraPred,"|");
    } else {
      logtrace(LogIntraPred," ");
    }

    logtrace(LogIntraPred,"%02x",data[i]);
  }
}


// (8.4.4.2.2)
void fill_border_samples(decoder_context* ctx, int xB,int yB,
                         int nT, int cIdx,
                         uint8_t* out_border)
{
  seq_parameter_set* sps = ctx->current_sps;

  uint8_t available_data[2*64 + 1];
  uint8_t* available = &available_data[64];

  uint8_t* image;
  int stride;
  get_image_plane(ctx, cIdx,  &image, &stride);


  // HACK: init usually not needed
      for (int i=-2*nT ; i<=2*nT ; i++) {
        out_border[i] = 1<<(sps->bit_depth_luma-1);
      }


  for (int y=-1 ; y<nT*2 ; y++)
    {
      bool availableN;
      if (cIdx==0)
        availableN = available_zscan(ctx, xB,yB, xB-1,yB+y);
      else
        availableN = available_zscan(ctx, 2*xB,2*yB, 2*(xB-1),2*(yB+y));

      available[-(y+1)] = availableN;

      if (ctx->current_pps->constrained_intra_pred_flag) {
        if (get_pred_mode(ctx,xB-1,yB+y)!=MODE_INTRA)
          available[-(y+1)] = false;
      }

      if (available[-(y+1)]) {
        out_border[-(y+1)] = image[xB-1 + (yB+y)*stride];
      }
    }


  for (int x=0 ; x<nT*2 ; x++)
    {
      bool availableN;
      if (cIdx==0)
        availableN = available_zscan(ctx, xB,yB, xB+x,yB-1);
      else
        availableN = available_zscan(ctx, 2*xB,2*yB, 2*(xB+x),2*(yB-1));
      available[x+1] = availableN;

      if (ctx->current_pps->constrained_intra_pred_flag) {
        if (get_pred_mode(ctx,xB+x,yB-1)!=MODE_INTRA)
          available[x+1] = false;
      }

      if (available[x+1]) {
        out_border[x+1] = image[xB+x + (yB-1)*stride];
      }
    }


  int nAvail=0;
  for (int i = -2*nT ; i<=2*nT ; i++) {
    if (available[i]) {
      nAvail++;
    }
  }


  logtrace(LogIntraPred,"availableN: ");
  print_border(available,nT);
  logtrace(LogIntraPred,"\n");


  logtrace(LogIntraPred,"input: ");
  print_border(out_border,nT);
  logtrace(LogIntraPred,"\n");



  if (nAvail < 4*nT+1) {
    // 8.4.4.2.2 sample substitution process

    if (nAvail == 0) {
      for (int i=-2*nT ; i<=2*nT ; i++) {
        out_border[i] = 1<<(sps->bit_depth_luma-1);
      }
    } else {
      if (!available[-2*nT]) {
        for (int i=-2*nT+1 ; ; i++) {
          if (available[i]) {
            available[-2*nT] = true;
            out_border[-2*nT] = out_border[i];
            break;
          }
        }
      }

      for (int i=-2*nT+1 ; i<=2*nT ; i++) {
        if (!available[i]) {
          out_border[i] = out_border[i-1];
        }
        else {
          // break; // HACK, wrong according to standard
        }
      }
    }
  }


  // HACK
  /*
  for (int i=-2*nT ; i<=2*nT ; i++) {
    out_border[i] = (i+2*nT)*10;
  }
  */


  logtrace(LogIntraPred,"output: ");
  print_border(out_border,nT);
  logtrace(LogIntraPred,"\n");
}


// (8.4.4.2.3)
void intra_prediction_sample_filtering(decoder_context* ctx,
                                       uint8_t* p,
                                       int nT,
                                       enum IntraPredMode intraPredMode)
{
  int filterFlag;

  if (intraPredMode==INTRA_DC || nT==4) {
    filterFlag = 0;
  } else {
    // int-cast below prevents a typing problem that leads to wrong results when abs_value is a macro
    int minDistVerHor = min( abs_value((int)intraPredMode-26), abs_value((int)intraPredMode-10) );
    switch (nT) {
    case 8:  filterFlag = (minDistVerHor>7) ? 1 : 0; break;
    case 16: filterFlag = (minDistVerHor>1) ? 1 : 0; break;
    case 32: filterFlag = (minDistVerHor>0) ? 1 : 0; break;
    default: filterFlag = -1; assert(false); break; // should never happen
    }
  }


  if (filterFlag) {
    int biIntFlag = (ctx->current_sps->strong_intra_smoothing_enable_flag &&
                     nT==32 &&
                     abs_value(p[0]+p[ 64]-2*p[ 32]) < (1<<(ctx->current_sps->bit_depth_luma-5)) &&
                     abs_value(p[0]+p[-64]-2*p[-32]) < (1<<(ctx->current_sps->bit_depth_luma-5)))
      ? 1 : 0;

    uint8_t  pF_mem[2*64+1];
    uint8_t* pF = &pF_mem[64];

    if (biIntFlag) {
      pF[-2*nT] = p[-2*nT];
      pF[ 2*nT] = p[ 2*nT];
      pF[    0] = p[    0];

      for (int i=1;i<=63;i++) {
        pF[-i] = p[0] + ((i*(p[-64]-p[0])+32)>>6);
        pF[ i] = p[0] + ((i*(p[ 64]-p[0])+32)>>6);
      }
    } else {
      pF[-2*nT] = p[-2*nT];
      pF[ 2*nT] = p[ 2*nT];

      for (int i=-(2*nT-1) ; i<=2*nT-1 ; i++)
        {
          pF[i] = (p[i+1] + 2*p[i] + p[i-1] + 2) >> 2;
        }
    }


    // copy back to original array

    memcpy(p-2*nT, pF-2*nT, 4*nT+1);
  }
  else {
    // do nothing ?
  }


  logtrace(LogIntraPred,"post filtering: ");
  print_border(p,nT);
  logtrace(LogIntraPred,"\n");
}


const int intraPredAngle_table[1+34] =
  { 0, 0,32,26,21,17,13, 9, 5, 2, 0,-2,-5,-9,-13,-17,-21,-26,
    -32,-26,-21,-17,-13,-9,-5,-2,0,2,5,9,13,17,21,26,32 };

static const int invAngle_table[25-10] =
  { -4096,-1638,-910,-630,-482,-390,-315,-256,
    -315,-390,-482,-630,-910,-1638,-4096 };


// TODO: clip to read BitDepthY
int Clip1Y(int x) { if (x<0) return 0; else if (x>255) return 255; else return x; }


// (8.4.4.2.6)
void intra_prediction_angular(decoder_context* ctx,
                              int xB0,int yB0,
                              enum IntraPredMode intraPredMode,
                              int nT,int cIdx,
                              uint8_t* border)
{
  uint8_t  ref_mem[2*64+1];
  uint8_t* ref=&ref_mem[64];

  uint8_t* pred;
  int      stride;
  get_image_plane(ctx,cIdx,&pred,&stride);
  pred += xB0 + yB0*stride;

  int intraPredAngle = intraPredAngle_table[intraPredMode];

  if (intraPredMode >= 18) {

    for (int x=0;x<=nT;x++)
      { ref[x] = border[x]; }

    if (intraPredAngle<0) {
      int invAngle = invAngle_table[intraPredMode-11];

      if ((nT*intraPredAngle)>>5 < -1) {
        for (int x=(nT*intraPredAngle)>>5; x<=-1; x++) {
          ref[x] = border[0-((x*invAngle+128)>>8)];
        }
      }
    } else {
      for (int x=nT+1; x<=2*nT;x++) {
        ref[x] = border[x];
      }
    }

    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++)
        {
          int iIdx = ((y+1)*intraPredAngle)>>5;
          int iFact= ((y+1)*intraPredAngle)&31;

          if (iFact != 0) {
            pred[x+y*stride] = ((32-iFact)*ref[x+iIdx+1] + iFact*ref[x+iIdx+2] + 16)>>5;
          } else {
            pred[x+y*stride] = ref[x+iIdx+1];
          }
        }

    if (intraPredMode==26 && cIdx==0 && nT<32) {
      for (int y=0;y<nT;y++) {
        pred[0+y*stride] = Clip1Y(border[1] + ((border[-1-y] - border[0])>>1));
      }
    }
  }
  else { // intraPredAngle < 18

    for (int x=0;x<=nT;x++)
      { ref[x] = border[-x]; }  // DIFF (neg)

    if (intraPredAngle<0) {
      int invAngle = invAngle_table[intraPredMode-11];

      if ((nT*intraPredAngle)>>5 < -1) {
        for (int x=(nT*intraPredAngle)>>5; x<=-1; x++) {
          ref[x] = border[((x*invAngle+128)>>8)]; // DIFF (neg)
        }
      }
    } else {
      for (int x=nT+1; x<=2*nT;x++) {
        ref[x] = border[-x]; // DIFF (neg)
      }
    }

    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++)
        {
          int iIdx = ((x+1)*intraPredAngle)>>5;  // DIFF (x<->y)
          int iFact= ((x+1)*intraPredAngle)&31;  // DIFF (x<->y)

          if (iFact != 0) {
            pred[x+y*stride] = ((32-iFact)*ref[y+iIdx+1] + iFact*ref[y+iIdx+2] + 16)>>5; // DIFF (x<->y)
          } else {
            pred[x+y*stride] = ref[y+iIdx+1]; // DIFF (x<->y)
          }
        }

    if (intraPredMode==10 && cIdx==0 && nT<32) {  // DIFF 26->10
      for (int x=0;x<nT;x++) { // DIFF (x<->y)
        pred[x] = Clip1Y(border[-1] + ((border[1+x] - border[0])>>1)); // DIFF (x<->y && neg)
      }
    }
  }


  logtrace(LogIntraPred,"result of angular intra prediction (mode=%d):\n",intraPredMode);

  for (int y=0;y<nT;y++)
    {
      for (int x=0;x<nT;x++)
        logtrace(LogIntraPred,"%02x ", pred[x+y*stride]);

      logtrace(LogIntraPred,"\n");
    }
}


void intra_prediction_planar(decoder_context* ctx,int xB0,int yB0,int nT,int cIdx,
                             uint8_t* border)
{
  uint8_t* pred;
  int      stride;
  get_image_plane(ctx,cIdx,&pred,&stride);
  pred += xB0 + yB0*stride;

  int Log2_nT = Log2(nT);

  for (int y=0;y<nT;y++)
    for (int x=0;x<nT;x++)
      {
        pred[x+y*stride] = ((nT-1-x)*border[-1-y] + (x+1)*border[ 1+nT] +
                            (nT-1-y)*border[ 1+x] + (y+1)*border[-1-nT] + nT) >> (Log2_nT+1);
      }
}


void intra_prediction_DC(decoder_context* ctx,int xB0,int yB0,int nT,int cIdx,
                         uint8_t* border)
{
  uint8_t* pred;
  int      stride;
  get_image_plane(ctx,cIdx,&pred,&stride);
  pred += xB0 + yB0*stride;

  int Log2_nT = Log2(nT);

  int dcVal = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal += border[ i+1];
      dcVal += border[-i-1];
    }

  dcVal += nT;
  dcVal >>= Log2_nT+1;

  if (cIdx==0 && nT<32) {
    pred[0] = (border[-1] + 2*dcVal + border[1] +2) >> 2;

    for (int x=1;x<nT;x++) { pred[x]        = (border[ x+1] + 3*dcVal+2)>>2; }
    for (int y=1;y<nT;y++) { pred[y*stride] = (border[-y-1] + 3*dcVal+2)>>2; }
    for (int y=1;y<nT;y++)
      for (int x=1;x<nT;x++)
        {
          pred[x+y*stride] = dcVal;
        }
  } else {
    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++)
        {
          pred[x+y*stride] = dcVal;
        }
  }
}



// (8.4.4.2.1)
void decode_intra_prediction(decoder_context* ctx,
                             int xB0,int yB0,
                             enum IntraPredMode intraPredMode,
                             int nT, int cIdx)
{
  logtrace(LogIntraPred,"decode_intra_prediction xy0:%d/%d mode=%d nT=%d, cIdx=%d\n",
           xB0,yB0, intraPredMode, nT,cIdx);
  /*
  printf("decode_intra_prediction xy0:%d/%d mode=%d nT=%d, cIdx=%d\n",
           xB0,yB0, intraPredMode, nT,cIdx);
  */

  nIntraPredictions++;

    uint8_t  border_pixels_mem[2*64+1];
    uint8_t* border_pixels = &border_pixels_mem[64];

    fill_border_samples(ctx, xB0,yB0, nT, cIdx, border_pixels);

    if (cIdx==0) {
      intra_prediction_sample_filtering(ctx, border_pixels, nT, intraPredMode);
    }


    switch (intraPredMode) {
    case INTRA_PLANAR:
      intra_prediction_planar(ctx,xB0,yB0,nT,cIdx, border_pixels);
      break;
    case INTRA_DC:
      intra_prediction_DC(ctx,xB0,yB0,nT,cIdx, border_pixels);
      break;
    default:
      intra_prediction_angular(ctx,xB0,yB0,intraPredMode,nT,cIdx, border_pixels);
      break;
    }
}


void decode_intra_block(decoder_context* ctx,
                        thread_context* tctx,
                        int cIdx,
                        int xB0,int yB0,  // position of TU in frame (chroma adapted)
                        int x0,int y0,    // position of CU in frame (chroma adapted)
                        int log2TrafoSize, int trafoDepth,
                        enum IntraPredMode intraPredMode)
{
  //printf("decode_intra_block: xB0:%d/%d x0:%d/%d\n",xB0,yB0,x0,y0);

  slice_segment_header* shdr = tctx->shdr;

  int splitFlag;

  if (cIdx==0) {
    splitFlag = get_split_transform_flag(ctx,xB0,yB0,trafoDepth);

    logtrace(LogIntraPred,"get_split_transform_flag(%d,%d, %d)=%d\n",xB0,yB0,trafoDepth,splitFlag);
  }
  else {
    assert(cIdx>0);

    // for chroma, ignore split flag when we reach 4x4 transform

    if (log2TrafoSize == 2) {
      splitFlag = 0;
    }
    else {
      splitFlag = get_split_transform_flag(ctx,xB0<<1,yB0<<1,trafoDepth);
    }
  }


  logtrace(LogIntraPred,"splitFlag=%d\n",splitFlag);

  if (splitFlag==1) {
    int xB1 = xB0 + ((1<<log2TrafoSize)>>1);
    int yB1 = yB0 + ((1<<log2TrafoSize)>>1);

    decode_intra_block(ctx,tctx,cIdx,xB0,yB0,x0,y0,log2TrafoSize-1,trafoDepth+1,intraPredMode);
    decode_intra_block(ctx,tctx,cIdx,xB1,yB0,x0,y0,log2TrafoSize-1,trafoDepth+1,intraPredMode);
    decode_intra_block(ctx,tctx,cIdx,xB0,yB1,x0,y0,log2TrafoSize-1,trafoDepth+1,intraPredMode);
    decode_intra_block(ctx,tctx,cIdx,xB1,yB1,x0,y0,log2TrafoSize-1,trafoDepth+1,intraPredMode);
  }
  else {
    int nT = 1<<log2TrafoSize;

    logtrace(LogIntraPred,"decode block %d,%d cIdx=%d (size %d)\n",xB0,yB0,cIdx,nT);

    decode_intra_prediction(ctx, xB0,yB0, intraPredMode, nT, cIdx);

    // (8.6.2)

    scale_coefficients(ctx, tctx, xB0,yB0, x0,y0, nT,cIdx);
  }
}
