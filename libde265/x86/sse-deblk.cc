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

// SSE4.1 8-bit deblocking. One edge segment = 4 lines along the edge. The four
// lines all use the same per-edge parameters and (for luma) the same strong/
// weak choice, so they are processed in parallel as the 4 int32 lanes of an
// xmm register. Each sample position (p3..q3) becomes one vector-of-4-lines.
// For horizontal edges those vectors are 4 contiguous bytes (one per line);
// for vertical edges the 4 lines are strided, so load/store transpose a small
// block. The arithmetic is identical to the scalar kernels -> bit-exact.

#include "x86/sse-deblk.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_SSE4_1

#include <string.h>
#include <smmintrin.h> // SSE4.1

namespace {

inline __m128i load4(const uint8_t* p) {           // 4 bytes -> 4 int32
  int32_t t; memcpy(&t, p, 4);
  return _mm_cvtepu8_epi32(_mm_cvtsi32_si128(t));
}
inline void store4(uint8_t* p, __m128i v) {        // 4 int32 (0..255) -> 4 bytes
  __m128i b = _mm_packus_epi16(_mm_packus_epi32(v, v), _mm_setzero_si128());
  int32_t t = _mm_cvtsi128_si32(b);
  memcpy(p, &t, 4);
}

inline __m128i clip3(__m128i lo, __m128i hi, __m128i v) {
  return _mm_min_epi32(_mm_max_epi32(v, lo), hi);
}
inline __m128i clip_u8(__m128i v) {
  return _mm_min_epi32(_mm_max_epi32(v, _mm_setzero_si128()), _mm_set1_epi32(255));
}
inline __m128i x2(__m128i a){ return _mm_add_epi32(a,a); }
inline __m128i add3(__m128i a,__m128i b,__m128i c){ return _mm_add_epi32(_mm_add_epi32(a,b),c); }

// --- vertical: load/store transpose a 4(lines) x 8(samples) block ----------

inline void load_vert(const uint8_t* ptr, ptrdiff_t stride, __m128i s[8]) {
  __m128i r0 = _mm_loadl_epi64((const __m128i*)(ptr-4));
  __m128i r1 = _mm_loadl_epi64((const __m128i*)(ptr-4+stride));
  __m128i r2 = _mm_loadl_epi64((const __m128i*)(ptr-4+2*stride));
  __m128i r3 = _mm_loadl_epi64((const __m128i*)(ptr-4+3*stride));
  __m128i e  = _mm_unpacklo_epi8(r0, r1);
  __m128i f  = _mm_unpacklo_epi8(r2, r3);
  __m128i lo = _mm_unpacklo_epi16(e, f);   // samples p3 p2 p1 p0 (4 bytes each, 4 lines)
  __m128i hi = _mm_unpackhi_epi16(e, f);   // samples q0 q1 q2 q3
  s[0]=_mm_cvtepu8_epi32(lo);
  s[1]=_mm_cvtepu8_epi32(_mm_srli_si128(lo,4));
  s[2]=_mm_cvtepu8_epi32(_mm_srli_si128(lo,8));
  s[3]=_mm_cvtepu8_epi32(_mm_srli_si128(lo,12));
  s[4]=_mm_cvtepu8_epi32(hi);
  s[5]=_mm_cvtepu8_epi32(_mm_srli_si128(hi,4));
  s[6]=_mm_cvtepu8_epi32(_mm_srli_si128(hi,8));
  s[7]=_mm_cvtepu8_epi32(_mm_srli_si128(hi,12));
}

inline void store_vert(uint8_t* ptr, ptrdiff_t stride, const __m128i s[8]) {
  __m128i lo = _mm_packus_epi16(_mm_packus_epi32(s[0],s[1]), _mm_packus_epi32(s[2],s[3]));
  __m128i hi = _mm_packus_epi16(_mm_packus_epi32(s[4],s[5]), _mm_packus_epi32(s[6],s[7]));
  const __m128i base = _mm_setr_epi8(0,4,8,12, (char)0x80,(char)0x80,(char)0x80,(char)0x80,
                                     (char)0x80,(char)0x80,(char)0x80,(char)0x80,
                                     (char)0x80,(char)0x80,(char)0x80,(char)0x80);
  for (int k=0;k<4;k++) {
    __m128i mask = _mm_add_epi8(base, _mm_set1_epi8((char)k));  // {k,4+k,8+k,12+k, 0x80..}
    __m128i a = _mm_shuffle_epi8(lo, mask);   // low4 = s0..s3 of line k
    __m128i b = _mm_shuffle_epi8(hi, mask);   // low4 = s4..s7 of line k
    __m128i row = _mm_unpacklo_epi32(a, b);   // low8 = the 8 samples of line k
    _mm_storel_epi64((__m128i*)(ptr-4+k*stride), row);
  }
}

inline void load_horiz(const uint8_t* ptr, ptrdiff_t stride, __m128i s[8]) {
  s[0]=load4(ptr-4*stride); s[1]=load4(ptr-3*stride); s[2]=load4(ptr-2*stride); s[3]=load4(ptr-1*stride);
  s[4]=load4(ptr+0*stride); s[5]=load4(ptr+1*stride); s[6]=load4(ptr+2*stride); s[7]=load4(ptr+3*stride);
}
inline void store_horiz(uint8_t* ptr, ptrdiff_t stride, const __m128i s[8]) {
  store4(ptr-4*stride,s[0]); store4(ptr-3*stride,s[1]); store4(ptr-2*stride,s[2]); store4(ptr-1*stride,s[3]);
  store4(ptr+0*stride,s[4]); store4(ptr+1*stride,s[5]); store4(ptr+2*stride,s[6]); store4(ptr+3*stride,s[7]);
}

} // namespace


void deblock_luma_8_sse4(uint8_t* ptr, ptrdiff_t stride, int vertical,
                         int dE, int dEp, int dEq, int tc, int filterP, int filterQ)
{
  __m128i s[8];
  if (vertical) load_vert(ptr, stride, s); else load_horiz(ptr, stride, s);

  const __m128i p3=s[0], p2=s[1], p1=s[2], p0=s[3];
  const __m128i q0=s[4], q1=s[5], q2=s[6], q3=s[7];

  if (dE==2) {
    // strong filtering
    const __m128i v2tc = _mm_set1_epi32(2*tc);
    const __m128i c4   = _mm_set1_epi32(4);
    const __m128i c2   = _mm_set1_epi32(2);

    __m128i pn0 = _mm_srai_epi32(_mm_add_epi32(add3(p2, x2(p1), x2(p0)), add3(x2(q0), q1, c4)), 3);
    pn0 = clip3(_mm_sub_epi32(p0,v2tc), _mm_add_epi32(p0,v2tc), pn0);
    __m128i pn1 = _mm_srai_epi32(_mm_add_epi32(add3(p2,p1,p0), _mm_add_epi32(q0,c2)), 2);
    pn1 = clip3(_mm_sub_epi32(p1,v2tc), _mm_add_epi32(p1,v2tc), pn1);
    __m128i pn2 = _mm_srai_epi32(_mm_add_epi32(add3(x2(p3), _mm_add_epi32(x2(p2),p2), p1), add3(p0,q0,c4)), 3);
    pn2 = clip3(_mm_sub_epi32(p2,v2tc), _mm_add_epi32(p2,v2tc), pn2);

    __m128i qn0 = _mm_srai_epi32(_mm_add_epi32(add3(p1, x2(p0), x2(q0)), add3(x2(q1), q2, c4)), 3);
    qn0 = clip3(_mm_sub_epi32(q0,v2tc), _mm_add_epi32(q0,v2tc), qn0);
    __m128i qn1 = _mm_srai_epi32(_mm_add_epi32(add3(p0,q0,q1), _mm_add_epi32(q2,c2)), 2);
    qn1 = clip3(_mm_sub_epi32(q1,v2tc), _mm_add_epi32(q1,v2tc), qn1);
    __m128i qn2 = _mm_srai_epi32(_mm_add_epi32(add3(p0,q0,q1), add3(_mm_add_epi32(x2(q2),q2), x2(q3), c4)), 3);
    qn2 = clip3(_mm_sub_epi32(q2,v2tc), _mm_add_epi32(q2,v2tc), qn2);

    if (filterP) { s[3]=pn0; s[2]=pn1; s[1]=pn2; }
    if (filterQ) { s[4]=qn0; s[5]=qn1; s[6]=qn2; }
  }
  else {
    // weak filtering
    const __m128i vtc  = _mm_set1_epi32(tc);
    const __m128i delta0 = _mm_srai_epi32(
        _mm_add_epi32(_mm_sub_epi32(_mm_mullo_epi32(_mm_set1_epi32(9), _mm_sub_epi32(q0,p0)),
                                    _mm_mullo_epi32(_mm_set1_epi32(3), _mm_sub_epi32(q1,p1))),
                      _mm_set1_epi32(8)), 4);
    // per-line mask: abs(delta) < tc*10
    __m128i mask = _mm_cmpgt_epi32(_mm_set1_epi32(tc*10), _mm_abs_epi32(delta0));
    __m128i delta = clip3(_mm_set1_epi32(-tc), vtc, delta0);

    if (filterP) {
      __m128i p0n = clip_u8(_mm_add_epi32(p0, delta));
      s[3] = _mm_blendv_epi8(p0, p0n, mask);
    }
    if (filterQ) {
      __m128i q0n = clip_u8(_mm_sub_epi32(q0, delta));
      s[4] = _mm_blendv_epi8(q0, q0n, mask);
    }
    if (dEp && filterP) {
      const __m128i htc = _mm_set1_epi32(tc>>1);
      __m128i dp = _mm_srai_epi32(_mm_add_epi32(_mm_sub_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(p2,p0),_mm_set1_epi32(1)),1), p1), delta), 1);
      dp = clip3(_mm_sub_epi32(_mm_setzero_si128(),htc), htc, dp);
      __m128i p1n = clip_u8(_mm_add_epi32(p1, dp));
      s[2] = _mm_blendv_epi8(p1, p1n, mask);
    }
    if (dEq && filterQ) {
      const __m128i htc = _mm_set1_epi32(tc>>1);
      __m128i dq = _mm_srai_epi32(_mm_sub_epi32(_mm_sub_epi32(_mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(q2,q0),_mm_set1_epi32(1)),1), q1), delta), 1);
      dq = clip3(_mm_sub_epi32(_mm_setzero_si128(),htc), htc, dq);
      __m128i q1n = clip_u8(_mm_add_epi32(q1, dq));
      s[5] = _mm_blendv_epi8(q1, q1n, mask);
    }
  }

  if (vertical) store_vert(ptr, stride, s); else store_horiz(ptr, stride, s);
}

// Note: an SSE chroma deblock filter was implemented and benchmarked too, but
// the chroma filter is a single delta per line -- so trivial that it is fully
// load/store-bound (the vertical case needs a strided 2-column scatter), and
// SSE measured slower than scalar (~0.5-0.9x). Chroma deblock therefore stays
// on the scalar fallback; only luma is accelerated here.

#endif // HAVE_SSE4_1
