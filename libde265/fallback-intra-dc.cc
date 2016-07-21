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
//#include "iacaMarks.h"


// 4x4, avg=0, SNB : 11+imul cyc
// 4x4, avg=0, HSW : 11+imul cyc
// 4x4, avg=1, SNB : 29 cyc
// 4x4, avg=1, HSW : 28 cyc
template <class pixel_t>
inline void intra_dc_fallback(pixel_t* dst,int dstStride, pixel_t* border, int Log2_nT, bool avg)
{
  int nT = 1<<Log2_nT;

  int dcVal_a = 0;
  int dcVal_b = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal_a += border[ i+1];
      dcVal_b += border[-i-1];
    }

  int dcVal =dcVal_a + dcVal_b;
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


// avg=0, SNB : 23 cyc
// avg=0, HSW : 20 cyc
// avg=1, SNB : 35 cyc
// avg=1, HSW : 34 cyc
template <bool avg>
inline void intra_dc_fallback_8x8_8bit(uint8_t* dst,int dstStride,
                                       const uint8_t* border)
{
  const int Log2_nT = 3;
  const int nT = 1<<Log2_nT;

  int dcVal_a = 0;
  int dcVal_b = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal_a += border[ i+1];
      dcVal_b += border[-i-1];
    }

  int dcVal =dcVal_a + dcVal_b;
  dcVal += nT;
  dcVal >>= Log2_nT+1;

  if (1 || avg) {
    dst[0] = (border[-1] + 2*dcVal + border[1] +2) >> 2;

    for (int x=1;x<nT;x++) {
      dst[x] = (border[ x+1] + 3*dcVal+2)>>2;
    }

    // this code assumes we are running on a little-endian machine
    uint64_t dcFlat = dcVal * 0x0101010101010100;
    for (int y=1;y<nT;y++) {
      *(uint64_t*)&dst[y*dstStride] = dcFlat | ((border[-y-1] + 3*dcVal+2)>>2);
    }
  } else {
    uint8_t* dst4 = dst+4*dstStride;

    memset(dst+0*dstStride,dcVal, nT);
    memset(dst+1*dstStride,dcVal, nT);
    memset(dst+2*dstStride,dcVal, nT);
    memset(dst+3*dstStride,dcVal, nT);

    memset(dst4+0*dstStride,dcVal, nT);
    memset(dst4+1*dstStride,dcVal, nT);
    memset(dst4+2*dstStride,dcVal, nT);
    memset(dst4+3*dstStride,dcVal, nT);
  }
}


// SNB, avg=0 : 11+imul cyc
// HSW, avg=0 : 12+imul cyc
// SNB, avg=1 : 18+imul cyc
// HSW, avg=1 : 18+imul cyc
template <bool avg>
inline void intra_dc_fallback_4x4_8bit_littleEndian(uint8_t* dst,int dstStride,
                                                    const uint8_t* border)
{
  const int Log2_nT = 2;
  const int nT = 4;

  int dcVal = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal += border[ i+1];
      dcVal += border[-i-1];
    }

  // gcc produces faster code when the offset 'nT' is added later,
  // contrary to initializing dcVal with it
  dcVal += nT;
  dcVal >>= Log2_nT+1;

  if (avg) {
    // when building dcFlat, we assume to run on a little-endian machine
    uint32_t dcFlat = dcVal * 0x01010100;

    dst[0] = (border[-1] + 2*dcVal + border[1] +2) >> 2;

    for (int x=1;x<nT;x++) {
      dst[x] = (border[ x+1] + 3*dcVal+2)>>2;
    }

    for (int y=1;y<nT;y++) {
      uint32_t v = dcFlat | ((border[-y-1] + 3*dcVal+2)>>2);
      *(uint32_t*)&dst[y*dstStride] = v;
    }
  } else {
    uint32_t dcFlat = dcVal * 0x01010101;

    for (int y=0;y<nT;y++) {
      //memset(dst+y*dstStride,dcVal, nT);
      *(uint32_t*)&dst[y*dstStride] = dcFlat;
    }
  }
}


void intra_dc_noavg_8_4x4_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{
  if (IS_LITTLE_ENDIAN) {
    intra_dc_fallback_4x4_8bit_littleEndian<false>(dst,dstStride,border);
  }
  else {
    intra_dc_fallback(dst,dstStride,border, 2,false);
  }
}

void intra_dc_avg_8_4x4_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{
  if (IS_LITTLE_ENDIAN) {
    intra_dc_fallback_4x4_8bit_littleEndian<true>(dst,dstStride,border);
  }
  else {
    intra_dc_fallback(dst,dstStride,border, 2,true);
  }
}

void intra_dc_noavg_8_8x8_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{
  //intra_dc_fallback(dst,dstStride,border, 3,false);
  intra_dc_fallback_8x8_8bit<false>(dst,dstStride,border);
}
void intra_dc_avg_8_8x8_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{
  //intra_dc_fallback(dst,dstStride,border, 3,true);
  intra_dc_fallback_8x8_8bit<true>(dst,dstStride,border);
}

void intra_dc_noavg_8_16x16_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback(dst,dstStride,border, 4,false); }
void intra_dc_avg_8_16x16_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback(dst,dstStride,border, 4,true); }

void intra_dc_noavg_8_32x32_fallback(uint8_t* dst,int dstStride, uint8_t* border)
{ intra_dc_fallback(dst,dstStride,border, 5,false); }
