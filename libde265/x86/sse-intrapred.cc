/*
 * H.265 video codec.
 * Copyright (c) 2026 Dirk Farin <dirk.farin@gmail.com>
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

#include "x86/sse-intrapred.h"
#include "libde265/util.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <emmintrin.h> // SSE2

#if HAVE_SSE4_1
#include <smmintrin.h> // SSE4.1

// angle / inverse-angle lookup tables, defined in intrapred.cc
extern const int intraPredAngle_table[1+34];
extern const int invAngle_table[25-10];

namespace {

const int kMaxBlk = 64; // == MAX_INTRA_PRED_BLOCK_SIZE

// load 4 / 8 consecutive uint8_t and zero-extend to 8x int16
inline __m128i load4_epi16(const uint8_t* p) {
  int32_t t; memcpy(&t, p, 4);
  return _mm_cvtepu8_epi16(_mm_cvtsi32_si128(t));
}
inline __m128i load8_epi16(const uint8_t* p) {
  return _mm_cvtepu8_epi16(_mm_loadl_epi64((const __m128i*)p));
}

// store nT bytes from the low lanes of an u8-packed register (nT in {4,8,16,32})
inline void store_row(uint8_t* d, __m128i v, int nT) {
  switch (nT) {
    case 4:  { int32_t t = _mm_cvtsi128_si32(v); memcpy(d, &t, 4); } break;
    case 8:  _mm_storel_epi64((__m128i*)d, v); break;
    case 16: _mm_storeu_si128((__m128i*)d, v); break;
    default: _mm_storeu_si128((__m128i*)d, v);
             _mm_storeu_si128((__m128i*)(d+16), v); break;
  }
}

// copy exactly nT bytes (nT in {4,8,16,32})
inline void copy_row(uint8_t* d, const uint8_t* s, int nT) {
  switch (nT) {
    case 4:  memcpy(d, s, 4); break;
    case 8:  memcpy(d, s, 8); break;
    case 16: _mm_storeu_si128((__m128i*)d, _mm_loadu_si128((const __m128i*)s)); break;
    default: _mm_storeu_si128((__m128i*)d,      _mm_loadu_si128((const __m128i*)s));
             _mm_storeu_si128((__m128i*)(d+16), _mm_loadu_si128((const __m128i*)(s+16))); break;
  }
}

inline int shift_for(int nT) { return (nT==4)?3 : (nT==8)?4 : (nT==16)?5 : 6; } // Log2(nT)+1

} // namespace


void intra_pred_dc_8_sse4(uint8_t* dst, ptrdiff_t stride, int nT, int cIdx, const uint8_t* border)
{
  const int shift = shift_for(nT);

  int dcVal = 0;
  for (int i=0;i<nT;i++) { dcVal += border[i+1]; dcVal += border[-i-1]; }
  dcVal += nT;
  dcVal >>= shift;

  const __m128i v = _mm_set1_epi8((char)dcVal);
  for (int y=0;y<nT;y++) store_row(dst + y*stride, v, nT);

  // luma edge smoothing overwrites first row and first column (disjoint cells)
  if (cIdx==0 && nT<32) {
    dst[0] = (uint8_t)((border[-1] + 2*dcVal + border[1] + 2) >> 2);
    for (int x=1;x<nT;x++) dst[x]          = (uint8_t)((border[ x+1] + 3*dcVal + 2) >> 2);
    for (int y=1;y<nT;y++) dst[y*stride]   = (uint8_t)((border[-y-1] + 3*dcVal + 2) >> 2);
  }
}


void intra_pred_planar_8_sse4(uint8_t* dst, ptrdiff_t stride, int nT, int cIdx, const uint8_t* border)
{
  const int shift = shift_for(nT);
  const int TR = border[ 1+nT];   // top-right corner sample
  const int BL = border[-1-nT];   // bottom-left corner sample

  const __m128i base  = _mm_setr_epi16(0,1,2,3,4,5,6,7);
  const __m128i vTR   = _mm_set1_epi16((short)TR);
  const __m128i vNTm1 = _mm_set1_epi16((short)(nT-1));
  const __m128i one   = _mm_set1_epi16(1);

  for (int y=0;y<nT;y++) {
    const int left_y = border[-1-y];
    const int Cy = (y+1)*BL + nT;                 // constant term for this row
    const __m128i vL    = _mm_set1_epi16((short)left_y);
    const __m128i vNT1Y = _mm_set1_epi16((short)(nT-1-y));
    const __m128i vC    = _mm_set1_epi16((short)Cy);

    for (int x=0;x<nT;x+=8) {
      const __m128i xidx = _mm_add_epi16(_mm_set1_epi16((short)x), base);
      const __m128i vA   = _mm_sub_epi16(vNTm1, xidx);     // (nT-1-x)
      const __m128i vB   = _mm_add_epi16(xidx, one);       // (x+1)
      const __m128i top  = (nT==4) ? load4_epi16(border+1+x) : load8_epi16(border+1+x);

      __m128i acc = _mm_mullo_epi16(vA,  vL);              // (nT-1-x)*border[-1-y]
      acc = _mm_add_epi16(acc, _mm_mullo_epi16(vB,  vTR)); // (x+1)*border[1+nT]
      acc = _mm_add_epi16(acc, _mm_mullo_epi16(top, vNT1Y));// (nT-1-y)*border[1+x]
      acc = _mm_add_epi16(acc, vC);                        // (y+1)*border[-1-nT] + nT
      acc = _mm_srli_epi16(acc, shift);

      const __m128i p = _mm_packus_epi16(acc, acc);
      store_row(dst + y*stride + x, p, (nT<8)?nT:8);
    }
  }
}


void intra_pred_angular_8_sse4(uint8_t* dst, ptrdiff_t stride, int bit_depth, int disableBoundaryFilter,
                               int xB0, int yB0, int mode, int nT, int cIdx, const uint8_t* border)
{
  const int intraPredAngle = intraPredAngle_table[mode];

  uint8_t  ref_mem[4*kMaxBlk+1];
  uint8_t* ref = &ref_mem[2*kMaxBlk];

  if (mode >= 18) {
    for (int x=0;x<=nT;x++) ref[x] = border[x];

    if (intraPredAngle<0) {
      const int invAngle = invAngle_table[mode-11];
      if (((nT*intraPredAngle)>>5) < -1) {
        for (int x=(nT*intraPredAngle)>>5; x<=-1; x++)
          ref[x] = border[0-((x*invAngle+128)>>8)];
      }
    } else {
      for (int x=nT+1; x<=2*nT; x++) ref[x] = border[x];
    }

    for (int y=0;y<nT;y++) {
      const int iIdx  = ((y+1)*intraPredAngle)>>5;
      const int iFact = ((y+1)*intraPredAngle)&31;
      const uint8_t* src = ref + iIdx + 1;
      uint8_t* d = dst + y*stride;

      if (iFact==0) {
        copy_row(d, src, nT);                     // dst[x] = ref[x+iIdx+1]
      } else {
        const __m128i w0  = _mm_set1_epi16((short)(32-iFact));
        const __m128i w1  = _mm_set1_epi16((short)iFact);
        const __m128i r16 = _mm_set1_epi16(16);
        if (nT==4) {
          __m128i a = load4_epi16(src), b = load4_epi16(src+1);
          __m128i acc = _mm_add_epi16(_mm_add_epi16(_mm_mullo_epi16(a,w0), _mm_mullo_epi16(b,w1)), r16);
          acc = _mm_srli_epi16(acc, 5);
          store_row(d, _mm_packus_epi16(acc,acc), 4);
        } else {
          for (int x=0;x<nT;x+=8) {
            __m128i a = load8_epi16(src+x), b = load8_epi16(src+x+1);
            __m128i acc = _mm_add_epi16(_mm_add_epi16(_mm_mullo_epi16(a,w0), _mm_mullo_epi16(b,w1)), r16);
            acc = _mm_srli_epi16(acc, 5);
            store_row(d+x, _mm_packus_epi16(acc,acc), 8);
          }
        }
      }
    }

    if (mode==26 && cIdx==0 && nT<32 && !disableBoundaryFilter) {
      for (int y=0;y<nT;y++)
        dst[y*stride] = (uint8_t)Clip_BitDepth(border[1] + ((border[-1-y] - border[0])>>1), bit_depth);
    }
  }
  else {
    // Modes 2..17: the reference projection is transposed (per-column iIdx/iFact and
    // a row-indexed reference fetch), which does not map onto contiguous SIMD loads or
    // stores. Use the scalar reference path here -- kept bit-identical to
    // intra_prediction_angular() in intrapred.h.
    for (int x=0;x<=nT;x++) ref[x] = border[-x];

    if (intraPredAngle<0) {
      const int invAngle = invAngle_table[mode-11];
      if (((nT*intraPredAngle)>>5) < -1) {
        for (int x=(nT*intraPredAngle)>>5; x<=-1; x++)
          ref[x] = border[(x*invAngle+128)>>8];
      }
    } else {
      for (int x=nT+1; x<=2*nT; x++) ref[x] = border[-x];
    }

    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++) {
        const int iIdx  = ((x+1)*intraPredAngle)>>5;
        const int iFact = ((x+1)*intraPredAngle)&31;
        if (iFact != 0)
          dst[x+y*stride] = (uint8_t)(((32-iFact)*ref[y+iIdx+1] + iFact*ref[y+iIdx+2] + 16)>>5);
        else
          dst[x+y*stride] = ref[y+iIdx+1];
      }

    if (mode==10 && cIdx==0 && nT<32 && !disableBoundaryFilter) {
      for (int x=0;x<nT;x++)
        dst[x] = (uint8_t)Clip_BitDepth(border[-1] + ((border[1+x] - border[0])>>1), bit_depth);
    }
  }
}

#endif // HAVE_SSE4_1
