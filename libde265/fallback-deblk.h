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

#ifndef DE265_FALLBACK_DEBLK_H
#define DE265_FALLBACK_DEBLK_H

#include <stddef.h>
#include <stdint.h>
#include "util.h"

// One luma edge-filter segment (4 lines along the edge), spec 8.7.2.4.4.
// 'ptr' points at the q0 sample of line 0. dE in {1,2} (weak/strong); the
// caller guarantees dE != 0. The filterP/filterQ flags disable the p- resp.
// q-side (PCM / transquant-bypass).
template <class pixel_t>
void deblock_luma_kernel(pixel_t* ptr, ptrdiff_t stride, bool vertical,
                         int dE, int dEp, int dEq, int tc,
                         bool filterP, bool filterQ, int bitDepth)
{
  for (int k=0;k<4;k++) {
    pixel_t p0,p1,p2,p3,q0,q1,q2,q3;
    if (vertical) {
      p0=ptr[-1+k*stride]; p1=ptr[-2+k*stride]; p2=ptr[-3+k*stride]; p3=ptr[-4+k*stride];
      q0=ptr[ 0+k*stride]; q1=ptr[ 1+k*stride]; q2=ptr[ 2+k*stride]; q3=ptr[ 3+k*stride];
    } else {
      p0=ptr[k-1*stride]; p1=ptr[k-2*stride]; p2=ptr[k-3*stride]; p3=ptr[k-4*stride];
      q0=ptr[k+0*stride]; q1=ptr[k+1*stride]; q2=ptr[k+2*stride]; q3=ptr[k+3*stride];
    }

    if (dE==2) {
      // strong filtering
      pixel_t pnew[3],qnew[3];
      pnew[0] = Clip3(p0-2*tc,p0+2*tc, (p2 + 2*p1 + 2*p0 + 2*q0 + q1 +4)>>3);
      pnew[1] = Clip3(p1-2*tc,p1+2*tc, (p2 + p1 + p0 + q0+2)>>2);
      pnew[2] = Clip3(p2-2*tc,p2+2*tc, (2*p3 + 3*p2 + p1 + p0 + q0 + 4)>>3);
      qnew[0] = Clip3(q0-2*tc,q0+2*tc, (p1+2*p0+2*q0+2*q1+q2+4)>>3);
      qnew[1] = Clip3(q1-2*tc,q1+2*tc, (p0+q0+q1+q2+2)>>2);
      qnew[2] = Clip3(q2-2*tc,q2+2*tc, (p0+q0+q1+3*q2+2*q3+4)>>3);

      if (vertical) {
        for (int i=0;i<3;i++) {
          if (filterP) { ptr[-i-1+k*stride] = pnew[i]; }
          if (filterQ) { ptr[ i + k*stride] = qnew[i]; }
        }
      } else {
        for (int i=0;i<3;i++) {
          if (filterP) { ptr[ k -(i+1)*stride] = pnew[i]; }
          if (filterQ) { ptr[ k + i   *stride] = qnew[i]; }
        }
      }
    }
    else {
      // weak filtering
      int delta = (9*(q0-p0) - 3*(q1-p1) + 8)>>4;

      if (std::abs(delta) < tc*10) {
        delta = Clip3(-tc,tc,delta);

        if (vertical) {
          if (filterP) { ptr[-0-1+k*stride] = Clip_BitDepth(p0+delta, bitDepth); }
          if (filterQ) { ptr[ 0  +k*stride] = Clip_BitDepth(q0-delta, bitDepth); }
        } else {
          if (filterP) { ptr[ k -1*stride] = Clip_BitDepth(p0+delta, bitDepth); }
          if (filterQ) { ptr[ k +0*stride] = Clip_BitDepth(q0-delta, bitDepth); }
        }

        if (dEp==1 && filterP) {
          int delta_p = Clip3(-(tc>>1), tc>>1, (((p2+p0+1)>>1)-p1+delta)>>1);
          if (vertical) { ptr[-1-1+k*stride] = Clip_BitDepth(p1+delta_p, bitDepth); }
          else          { ptr[ k  -2*stride] = Clip_BitDepth(p1+delta_p, bitDepth); }
        }

        if (dEq==1 && filterQ) {
          int delta_q = Clip3(-(tc>>1), tc>>1, (((q2+q0+1)>>1)-q1-delta)>>1);
          if (vertical) { ptr[ 1  +k*stride] = Clip_BitDepth(q1+delta_q, bitDepth); }
          else          { ptr[ k  +1*stride] = Clip_BitDepth(q1+delta_q, bitDepth); }
        }
      }
    }
  }
}


// One chroma edge-filter segment (4 lines), spec 8.7.2.4.5.
template <class pixel_t>
void deblock_chroma_kernel(pixel_t* ptr, ptrdiff_t stride, bool vertical,
                           int tc, bool filterP, bool filterQ, int bitDepth)
{
  for (int k=0;k<4;k++) {
    pixel_t p0,p1,q0,q1;
    if (vertical) {
      q0=ptr[ 0+k*stride]; q1=ptr[ 1+k*stride]; p0=ptr[-1+k*stride]; p1=ptr[-2+k*stride];
    } else {
      q0=ptr[k+0*stride]; q1=ptr[k+1*stride]; p0=ptr[k-1*stride]; p1=ptr[k-2*stride];
    }

    int delta = Clip3(-tc,tc, ((((q0-p0)*4)+p1-q1+4)>>3));

    if (vertical) {
      if (filterP) { ptr[-1+k*stride] = Clip_BitDepth(p0+delta, bitDepth); }
      if (filterQ) { ptr[ 0+k*stride] = Clip_BitDepth(q0-delta, bitDepth); }
    } else {
      if (filterP) { ptr[ k-1*stride] = Clip_BitDepth(p0+delta, bitDepth); }
      if (filterQ) { ptr[ k+0*stride] = Clip_BitDepth(q0-delta, bitDepth); }
    }
  }
}


// 8-bit fallback wrappers stored in the acceleration table.
void deblock_luma_8_fallback(uint8_t* ptr, ptrdiff_t stride, int vertical,
                             int dE, int dEp, int dEq, int tc, int filterP, int filterQ);
void deblock_chroma_8_fallback(uint8_t* ptr, ptrdiff_t stride, int vertical,
                               int tc, int filterP, int filterQ);

#endif
