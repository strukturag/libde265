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

#include "fallback-intra-dc.h"
#include "util.h"

#include <assert.h>
#include <string.h>


template <class pixel_t>
void intra_dc_fallback_8bit(pixel_t* dst,int dstStride, pixel_t* border, int Log2_nT, bool avg)
{
  int nT = 1<<Log2_nT;

  int dcVal = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal += border[ i+1];
      dcVal += border[-i-1];
    }

  dcVal += nT;
  dcVal >>= Log2_nT+1;

  if (avg) {
    dst[0] = (border[-1] + 2*dcVal + border[1] +2) >> 2;

    for (int x=1;x<nT;x++) {
      dst[x] = (border[ x+1] + 3*dcVal+2)>>2;
    }

    for (int y=1;y<nT;y++) {
      dst[y*dstStride] = (border[-y-1] + 3*dcVal+2)>>2;

      for (int x=1;x<nT;x++)
        {
          dst[x+y*dstStride] = dcVal;
        }
    }
  } else {

    if (sizeof(pixel_t)==1) {
      for (int y=0;y<nT;y++)
        memset(dst+y*dstStride,dcVal, nT);
    }
    else {
      for (int y=0;y<nT;y++)
        for (int x=0;x<nT;x++)
          {
            dst[x+y*dstStride] = dcVal;
          }
    }
  }
}


void intra_dc_noavg_8_4x4_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 2,false); }
void intra_dc_avg_8_4x4_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 2,true); }

void intra_dc_noavg_8_8x8_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 3,false); }
void intra_dc_avg_8_8x8_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 3,true); }

void intra_dc_noavg_8_16x16_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 4,false); }
void intra_dc_avg_8_16x16_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 4,true); }

void intra_dc_noavg_8_32x32_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback_8bit(dst,dstStride,border, 5,false); }
