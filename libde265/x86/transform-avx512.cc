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

// AVX-512 (F + BW) inverse transform for 32x32, 8-bit.
//
// A full 32-column row fits in one zmm (32 int16), so the vertical pass
// processes all 32 columns at once (twice the AVX2 width). The per-column
// butterfly is the same arithmetic as the AVX2/SSE versions: every 512-bit op
// acts independently per 128-bit lane, so each of the 4 lanes does what the SSE
// does for 8 columns -> 32 columns, bit-identical. The even part reuses a
// 16-point core (a 32-pt inverse DCT's even half is a 16-pt inverse DCT of the
// even rows). The 32x32 transpose between passes is done at 256-bit width using
// the proven AVX2 path (split each zmm into its two ymm halves, transpose,
// recombine) since the transpose is shuffle-bound, not the hot path.

#include "x86/transform-avx512.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_AVX512

#include <immintrin.h>
#include "x86/transform-dct-tables.h"   // idct_T16_{1,2,3}, idct_T32

namespace {

#define shift_1st 7
#define add_1st   (1 << (shift_1st - 1))   // 64
#define shift_2nd 12
#define add_2nd   (1 << (shift_2nd - 1))   // 2048

// broadcast a 128-bit (8 int16) madd pattern to all four 128-bit lanes
static inline __m512i bc(const int16_t* p) {
  return _mm512_broadcast_i32x4(_mm_load_si128((const __m128i*)p));
}

// --- 256-bit transpose helpers (identical to transform-avx2.cc) ----------

static inline void transpose8x8_lanes(__m256i a[8]) {
  __m256i b0=_mm256_unpacklo_epi16(a[0],a[1]), b1=_mm256_unpackhi_epi16(a[0],a[1]);
  __m256i b2=_mm256_unpacklo_epi16(a[2],a[3]), b3=_mm256_unpackhi_epi16(a[2],a[3]);
  __m256i b4=_mm256_unpacklo_epi16(a[4],a[5]), b5=_mm256_unpackhi_epi16(a[4],a[5]);
  __m256i b6=_mm256_unpacklo_epi16(a[6],a[7]), b7=_mm256_unpackhi_epi16(a[6],a[7]);
  __m256i c0=_mm256_unpacklo_epi32(b0,b2), c1=_mm256_unpackhi_epi32(b0,b2);
  __m256i c2=_mm256_unpacklo_epi32(b1,b3), c3=_mm256_unpackhi_epi32(b1,b3);
  __m256i c4=_mm256_unpacklo_epi32(b4,b6), c5=_mm256_unpackhi_epi32(b4,b6);
  __m256i c6=_mm256_unpacklo_epi32(b5,b7), c7=_mm256_unpackhi_epi32(b5,b7);
  a[0]=_mm256_unpacklo_epi64(c0,c4); a[1]=_mm256_unpackhi_epi64(c0,c4);
  a[2]=_mm256_unpacklo_epi64(c1,c5); a[3]=_mm256_unpackhi_epi64(c1,c5);
  a[4]=_mm256_unpacklo_epi64(c2,c6); a[5]=_mm256_unpackhi_epi64(c2,c6);
  a[6]=_mm256_unpacklo_epi64(c3,c7); a[7]=_mm256_unpackhi_epi64(c3,c7);
}
static inline void transpose16x16(__m256i S[16]) {
  __m256i top[8], bot[8];
  for (int r=0;r<8;r++){ top[r]=S[r]; bot[r]=S[r+8]; }
  transpose8x8_lanes(top); transpose8x8_lanes(bot);
  for (int r=0;r<8;r++){
    S[r]   = _mm256_permute2x128_si256(top[r], bot[r], 0x20);
    S[r+8] = _mm256_permute2x128_si256(top[r], bot[r], 0x31);
  }
}
static inline void transpose32x32_ymm(__m256i Lo[32], __m256i Hi[32]) {
  __m256i A[16],B[16],C[16],D[16];
  for (int r=0;r<16;r++){ A[r]=Lo[r]; B[r]=Hi[r]; C[r]=Lo[r+16]; D[r]=Hi[r+16]; }
  transpose16x16(A); transpose16x16(B); transpose16x16(C); transpose16x16(D);
  for (int r=0;r<16;r++){ Lo[r]=A[r]; Hi[r]=C[r]; Lo[r+16]=B[r]; Hi[r+16]=D[r]; }
}

// 32x32 transpose of zmm rows, via the 256-bit path.
static inline void transpose32x32_z(__m512i S[32]) {
  __m256i Lo[32], Hi[32];
  for (int r=0;r<32;r++){
    Lo[r]=_mm512_castsi512_si256(S[r]);
    Hi[r]=_mm512_extracti64x4_epi64(S[r], 1);
  }
  transpose32x32_ymm(Lo, Hi);
  for (int r=0;r<32;r++)
    S[r]=_mm512_inserti64x4(_mm512_castsi256_si512(Lo[r]), Hi[r], 1);
}

// --- 16-point core (32 columns) ------------------------------------------

static inline void idct16_core_512(const __m512i S[16], __m512i resL[16], __m512i resH[16]) {
  const __m512i T00=bc(idct_T16_1[0][0]),T01=bc(idct_T16_1[0][1]),T02=bc(idct_T16_1[0][2]),T03=bc(idct_T16_1[0][3]);
  const __m512i T04=bc(idct_T16_1[0][4]),T05=bc(idct_T16_1[0][5]),T06=bc(idct_T16_1[0][6]),T07=bc(idct_T16_1[0][7]);
  const __m512i T10=bc(idct_T16_1[1][0]),T11=bc(idct_T16_1[1][1]),T12=bc(idct_T16_1[1][2]),T13=bc(idct_T16_1[1][3]);
  const __m512i T14=bc(idct_T16_1[1][4]),T15=bc(idct_T16_1[1][5]),T16=bc(idct_T16_1[1][6]),T17=bc(idct_T16_1[1][7]);
  const __m512i T20=bc(idct_T16_1[2][0]),T21=bc(idct_T16_1[2][1]),T22=bc(idct_T16_1[2][2]),T23=bc(idct_T16_1[2][3]);
  const __m512i T24=bc(idct_T16_1[2][4]),T25=bc(idct_T16_1[2][5]),T26=bc(idct_T16_1[2][6]),T27=bc(idct_T16_1[2][7]);
  const __m512i T30=bc(idct_T16_1[3][0]),T31=bc(idct_T16_1[3][1]),T32_=bc(idct_T16_1[3][2]),T33=bc(idct_T16_1[3][3]);
  const __m512i T34=bc(idct_T16_1[3][4]),T35=bc(idct_T16_1[3][5]),T36=bc(idct_T16_1[3][6]),T37=bc(idct_T16_1[3][7]);
  const __m512i U00=bc(idct_T16_2[0][0]),U01=bc(idct_T16_2[0][1]),U02=bc(idct_T16_2[0][2]),U03=bc(idct_T16_2[0][3]);
  const __m512i U10=bc(idct_T16_2[1][0]),U11=bc(idct_T16_2[1][1]),U12=bc(idct_T16_2[1][2]),U13=bc(idct_T16_2[1][3]);
  const __m512i V00=bc(idct_T16_3[0][0]),V01=bc(idct_T16_3[0][1]),V10=bc(idct_T16_3[1][0]),V11=bc(idct_T16_3[1][1]);

  __m512i m0,m1,m2,m3,m4,m5,m6,m7;
  __m512i E0l,E1l,E2l,E3l,E0h,E1h,E2h,E3h;
  __m512i O0l,O1l,O2l,O3l,O4l,O5l,O6l,O7l,O0h,O1h,O2h,O3h,O4h,O5h,O6h,O7h;
  __m512i E00l,E01l,E00h,E01h,EE0l,EE1l,EE2l,EE3l,EE0h,EE1h,EE2h,EE3h;
  __m512i E4l,E5l,E6l,E7l,E4h,E5h,E6h,E7h;

  m0=_mm512_unpacklo_epi16(S[1],S[3]);  E0l=_mm512_madd_epi16(m0,T00);
  m1=_mm512_unpackhi_epi16(S[1],S[3]);  E0h=_mm512_madd_epi16(m1,T00);
  m2=_mm512_unpacklo_epi16(S[5],S[7]);  E1l=_mm512_madd_epi16(m2,T10);
  m3=_mm512_unpackhi_epi16(S[5],S[7]);  E1h=_mm512_madd_epi16(m3,T10);
  m4=_mm512_unpacklo_epi16(S[9],S[11]); E2l=_mm512_madd_epi16(m4,T20);
  m5=_mm512_unpackhi_epi16(S[9],S[11]); E2h=_mm512_madd_epi16(m5,T20);
  m6=_mm512_unpacklo_epi16(S[13],S[15]);E3l=_mm512_madd_epi16(m6,T30);
  m7=_mm512_unpackhi_epi16(S[13],S[15]);E3h=_mm512_madd_epi16(m7,T30);
  O0l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O0h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T01);E0h=_mm512_madd_epi16(m1,T01);E1l=_mm512_madd_epi16(m2,T11);E1h=_mm512_madd_epi16(m3,T11);
  E2l=_mm512_madd_epi16(m4,T21);E2h=_mm512_madd_epi16(m5,T21);E3l=_mm512_madd_epi16(m6,T31);E3h=_mm512_madd_epi16(m7,T31);
  O1l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O1h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T02);E0h=_mm512_madd_epi16(m1,T02);E1l=_mm512_madd_epi16(m2,T12);E1h=_mm512_madd_epi16(m3,T12);
  E2l=_mm512_madd_epi16(m4,T22);E2h=_mm512_madd_epi16(m5,T22);E3l=_mm512_madd_epi16(m6,T32_);E3h=_mm512_madd_epi16(m7,T32_);
  O2l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O2h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T03);E0h=_mm512_madd_epi16(m1,T03);E1l=_mm512_madd_epi16(m2,T13);E1h=_mm512_madd_epi16(m3,T13);
  E2l=_mm512_madd_epi16(m4,T23);E2h=_mm512_madd_epi16(m5,T23);E3l=_mm512_madd_epi16(m6,T33);E3h=_mm512_madd_epi16(m7,T33);
  O3l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O3h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T04);E0h=_mm512_madd_epi16(m1,T04);E1l=_mm512_madd_epi16(m2,T14);E1h=_mm512_madd_epi16(m3,T14);
  E2l=_mm512_madd_epi16(m4,T24);E2h=_mm512_madd_epi16(m5,T24);E3l=_mm512_madd_epi16(m6,T34);E3h=_mm512_madd_epi16(m7,T34);
  O4l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O4h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T05);E0h=_mm512_madd_epi16(m1,T05);E1l=_mm512_madd_epi16(m2,T15);E1h=_mm512_madd_epi16(m3,T15);
  E2l=_mm512_madd_epi16(m4,T25);E2h=_mm512_madd_epi16(m5,T25);E3l=_mm512_madd_epi16(m6,T35);E3h=_mm512_madd_epi16(m7,T35);
  O5l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O5h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T06);E0h=_mm512_madd_epi16(m1,T06);E1l=_mm512_madd_epi16(m2,T16);E1h=_mm512_madd_epi16(m3,T16);
  E2l=_mm512_madd_epi16(m4,T26);E2h=_mm512_madd_epi16(m5,T26);E3l=_mm512_madd_epi16(m6,T36);E3h=_mm512_madd_epi16(m7,T36);
  O6l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O6h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  E0l=_mm512_madd_epi16(m0,T07);E0h=_mm512_madd_epi16(m1,T07);E1l=_mm512_madd_epi16(m2,T17);E1h=_mm512_madd_epi16(m3,T17);
  E2l=_mm512_madd_epi16(m4,T27);E2h=_mm512_madd_epi16(m5,T27);E3l=_mm512_madd_epi16(m6,T37);E3h=_mm512_madd_epi16(m7,T37);
  O7l=_mm512_add_epi32(_mm512_add_epi32(E0l,E1l),_mm512_add_epi32(E2l,E3l));
  O7h=_mm512_add_epi32(_mm512_add_epi32(E0h,E1h),_mm512_add_epi32(E2h,E3h));

  // even part
  m0=_mm512_unpacklo_epi16(S[2],S[6]);  E0l=_mm512_madd_epi16(m0,U00);
  m1=_mm512_unpackhi_epi16(S[2],S[6]);  E0h=_mm512_madd_epi16(m1,U00);
  m2=_mm512_unpacklo_epi16(S[10],S[14]);E0l=_mm512_add_epi32(E0l,_mm512_madd_epi16(m2,U10));
  m3=_mm512_unpackhi_epi16(S[10],S[14]);E0h=_mm512_add_epi32(E0h,_mm512_madd_epi16(m3,U10));
  E1l=_mm512_madd_epi16(m0,U01);E1h=_mm512_madd_epi16(m1,U01);
  E1l=_mm512_add_epi32(E1l,_mm512_madd_epi16(m2,U11));E1h=_mm512_add_epi32(E1h,_mm512_madd_epi16(m3,U11));
  E2l=_mm512_madd_epi16(m0,U02);E2h=_mm512_madd_epi16(m1,U02);
  E2l=_mm512_add_epi32(E2l,_mm512_madd_epi16(m2,U12));E2h=_mm512_add_epi32(E2h,_mm512_madd_epi16(m3,U12));
  E3l=_mm512_madd_epi16(m0,U03);E3h=_mm512_madd_epi16(m1,U03);
  E3l=_mm512_add_epi32(E3l,_mm512_madd_epi16(m2,U13));E3h=_mm512_add_epi32(E3h,_mm512_madd_epi16(m3,U13));

  m0=_mm512_unpacklo_epi16(S[4],S[12]); E00l=_mm512_madd_epi16(m0,V00);
  m1=_mm512_unpackhi_epi16(S[4],S[12]); E00h=_mm512_madd_epi16(m1,V00);
  m2=_mm512_unpacklo_epi16(S[0],S[8]);  EE0l=_mm512_madd_epi16(m2,V10);
  m3=_mm512_unpackhi_epi16(S[0],S[8]);  EE0h=_mm512_madd_epi16(m3,V10);
  E01l=_mm512_madd_epi16(m0,V01);E01h=_mm512_madd_epi16(m1,V01);
  EE1l=_mm512_madd_epi16(m2,V11);EE1h=_mm512_madd_epi16(m3,V11);

  EE2l=_mm512_sub_epi32(EE1l,E01l);EE3l=_mm512_sub_epi32(EE0l,E00l);
  EE2h=_mm512_sub_epi32(EE1h,E01h);EE3h=_mm512_sub_epi32(EE0h,E00h);
  EE0l=_mm512_add_epi32(EE0l,E00l);EE1l=_mm512_add_epi32(EE1l,E01l);
  EE0h=_mm512_add_epi32(EE0h,E00h);EE1h=_mm512_add_epi32(EE1h,E01h);

  E4l=_mm512_sub_epi32(EE3l,E3l); E5l=_mm512_sub_epi32(EE2l,E2l);
  E6l=_mm512_sub_epi32(EE1l,E1l); E7l=_mm512_sub_epi32(EE0l,E0l);
  E4h=_mm512_sub_epi32(EE3h,E3h); E5h=_mm512_sub_epi32(EE2h,E2h);
  E6h=_mm512_sub_epi32(EE1h,E1h); E7h=_mm512_sub_epi32(EE0h,E0h);
  E0l=_mm512_add_epi32(EE0l,E0l); E1l=_mm512_add_epi32(EE1l,E1l);
  E2l=_mm512_add_epi32(EE2l,E2l); E3l=_mm512_add_epi32(EE3l,E3l);
  E0h=_mm512_add_epi32(EE0h,E0h); E1h=_mm512_add_epi32(EE1h,E1h);
  E2h=_mm512_add_epi32(EE2h,E2h); E3h=_mm512_add_epi32(EE3h,E3h);

  resL[0]=_mm512_add_epi32(E0l,O0l); resH[0]=_mm512_add_epi32(E0h,O0h);
  resL[1]=_mm512_add_epi32(E1l,O1l); resH[1]=_mm512_add_epi32(E1h,O1h);
  resL[2]=_mm512_add_epi32(E2l,O2l); resH[2]=_mm512_add_epi32(E2h,O2h);
  resL[3]=_mm512_add_epi32(E3l,O3l); resH[3]=_mm512_add_epi32(E3h,O3h);
  resL[4]=_mm512_add_epi32(E4l,O4l); resH[4]=_mm512_add_epi32(E4h,O4h);
  resL[5]=_mm512_add_epi32(E5l,O5l); resH[5]=_mm512_add_epi32(E5h,O5h);
  resL[6]=_mm512_add_epi32(E6l,O6l); resH[6]=_mm512_add_epi32(E6h,O6h);
  resL[7]=_mm512_add_epi32(E7l,O7l); resH[7]=_mm512_add_epi32(E7h,O7h);
  resL[15]=_mm512_sub_epi32(E0l,O0l); resH[15]=_mm512_sub_epi32(E0h,O0h);
  resL[14]=_mm512_sub_epi32(E1l,O1l); resH[14]=_mm512_sub_epi32(E1h,O1h);
  resL[13]=_mm512_sub_epi32(E2l,O2l); resH[13]=_mm512_sub_epi32(E2h,O2h);
  resL[12]=_mm512_sub_epi32(E3l,O3l); resH[12]=_mm512_sub_epi32(E3h,O3h);
  resL[11]=_mm512_sub_epi32(E4l,O4l); resH[11]=_mm512_sub_epi32(E4h,O4h);
  resL[10]=_mm512_sub_epi32(E5l,O5l); resH[10]=_mm512_sub_epi32(E5h,O5h);
  resL[9] =_mm512_sub_epi32(E6l,O6l); resH[9] =_mm512_sub_epi32(E6h,O6h);
  resL[8] =_mm512_sub_epi32(E7l,O7l); resH[8] =_mm512_sub_epi32(E7h,O7h);
}

static inline __m512i round_shift_pack(__m512i lo, __m512i hi, __m512i vr, int shift) {
  return _mm512_packs_epi32(
      _mm512_srai_epi32(_mm512_add_epi32(lo,vr),shift),
      _mm512_srai_epi32(_mm512_add_epi32(hi,vr),shift));
}

// 1-D 32-point inverse transform in place on S[0..31] (32 columns).
static inline void idct32_vpass_512(__m512i S[32], int add, int shift) {
  __m512i ev[16];
  for (int i=0;i<16;i++) ev[i]=S[2*i];
  __m512i EL[16], EH[16];
  idct16_core_512(ev, EL, EH);

  __m512i pl[8], ph[8];
  for (int g=0; g<8; g++) {
    pl[g]=_mm512_unpacklo_epi16(S[4*g+1], S[4*g+3]);
    ph[g]=_mm512_unpackhi_epi16(S[4*g+1], S[4*g+3]);
  }

  const __m512i vr = _mm512_set1_epi32(add);
  for (int k=0;k<16;k++) {
    __m512i OL=_mm512_madd_epi16(pl[0], bc(idct_T32[0][k]));
    __m512i OH=_mm512_madd_epi16(ph[0], bc(idct_T32[0][k]));
    for (int g=1; g<8; g++) {
      OL=_mm512_add_epi32(OL,_mm512_madd_epi16(pl[g], bc(idct_T32[g][k])));
      OH=_mm512_add_epi32(OH,_mm512_madd_epi16(ph[g], bc(idct_T32[g][k])));
    }
    S[k]    = round_shift_pack(_mm512_add_epi32(EL[k],OL), _mm512_add_epi32(EH[k],OH), vr, shift);
    S[31-k] = round_shift_pack(_mm512_sub_epi32(EL[k],OL), _mm512_sub_epi32(EH[k],OH), vr, shift);
  }
}

} // namespace


void transform_32x32_add_8_avx512(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride)
{
  __m512i S[32];
  for (int r=0;r<32;r++) S[r]=_mm512_loadu_si512((const void*)(coeffs + r*32));

  idct32_vpass_512(S, add_1st, shift_1st);
  transpose32x32_z(S);
  idct32_vpass_512(S, add_2nd, shift_2nd);
  transpose32x32_z(S);

  // index to gather the low qword of each 128-bit lane into a contiguous 256
  const __m512i gather = _mm512_setr_epi64(0,2,4,6, 1,3,5,7);
  for (int r=0;r<32;r++) {
    uint8_t* d = dst + r*stride;
    __m512i pred = _mm512_cvtepu8_epi16(_mm256_loadu_si256((const __m256i*)d)); // 32 int16
    __m512i sum  = _mm512_adds_epi16(S[r], pred);
    __m512i pk   = _mm512_packus_epi16(sum, sum);          // per lane: [c|c]
    pk = _mm512_permutexvar_epi64(gather, pk);             // low 256 = c0..c31
    _mm256_storeu_si256((__m256i*)d, _mm512_castsi512_si256(pk));
  }
}

#endif // HAVE_AVX512
