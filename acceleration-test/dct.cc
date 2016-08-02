

#include "libde265/x86/sse-dct.h"
#include "libde265/fallback-dct.h"
#include "iacaMarks.h"

#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <iostream>

//#include "cpucounters.h"

#include "libde265/util.h"

#define D 0

#if D
#define Deb(x) if (D) { print128(x); printf(" " #x "\n"); }
#else
#define Deb(x)
#endif

void print128(__m128i m);
void print128(__m128i m,int w);


/*
  64, 83, 64, 36
  64, 36,-64,-83
  64,-36,-64, 83
  64,-83, 64,-36
*/
ALIGNED_16(static const int16_t) transform4x4[][8] =
{
  // for vertical transforms

  { 64,64, 64,64, 64,64, 64,64 },
  { 83,36, 83,36, 83,36, 83,36 },
  { 64,-64,64,-64,64,-64,64,-64 },
  { 36,-83,36,-83,36,-83,36,-83 },
  /*
  { 64,-64,64,-64,64,-64,64,-64 },    // [4] ==  [2]
  { -36,83,-36,83,-36,83,-36,83 },    // [5] == -[3]
  { 64,64, 64,64, 64,64, 64,64 },     // [6] ==  [0]
  { -83,-36,-83,-36,-83,-36,-83,-36 } // [7] == -[1]
  */

  // for horizontal transforms

  { 64, 83, 64,36, 64, 36,-64,-83 },
  { 64,-36,-64,83, 64,-83, 64,-36 }
};


void idct_4x4_add_8_sse4_local(uint8_t *dst, const int16_t *coeffs,
                               ptrdiff_t stride,int maxColumn,int maxRow)
{
  /* vertical transforms
     -------------------

     Input:
     a1 b1 c1 d1
     a2 b2 c2 d2
     a3 b3 c3 d3
     a4 b4 c4 d4

     Matrix:
     A1 A2 A3 A4    64  83  64  36
     B1 B2 B3 B4    64  36 -64 -83
     C1 C2 C3 C4    64 -36 -64  83
     D1 D2 D3 D4    64 -83  64 -36

     a1A1+a2A2+a3A3+a4A4+r  b1A1+b2A2+b3A3+b4A4+r  c1A1+c2A2+c3A3+c4A4+r  d1A1+d2A2+d3A3+d4A4+r
     a1B1+a2B2+a3B3+a4B4+r  b1B1+b2B2+b3B3+b4B4+r  c1B1+c2B2+c3B3+c4B4+r  d1B1+d2B2+d3B3+d4B4+r
     a1C1+a2C2+a3C3+a4C4+r  b1C1+b2C2+b3C3+b4C4+r  c1C1+c2C2+c3C3+c4C4+r  d1C1+d2C2+d3C3+d4C4+r
     a1D1+a2D2+a3D3+a4D4+r  b1D1+b2D2+b3D3+b4D4+r  c1D1+c2D2+c3D3+c4D4+r  d1D1+d2D2+d3D3+d4D4+r
   */

  __m128i input16_a = _mm_load_si128((const __m128i*)(coeffs));
  __m128i input16_b = _mm_load_si128((const __m128i*)(coeffs + 8));

  Deb(input16_a); // d2 c2 b2 a2 d1 c1 b1 a1
  Deb(input16_b); // d4 c4 b4 a4 d3 c3 b3 a3

  __m128i coeff1_a = _mm_load_si128((const __m128i*)(transform4x4[0]));
  __m128i coeff1_b = _mm_load_si128((const __m128i*)(transform4x4[1]));
  Deb(coeff1_a); // A3 A1 A3 A1 A3 A1 A3 A1
  Deb(coeff1_b); // A4 A2 A4 A2 A4 A2 A4 A2

  __m128i in_a = _mm_unpacklo_epi16(input16_a, input16_b);
  __m128i in_b = _mm_unpackhi_epi16(input16_a, input16_b);
  Deb(in_a); // d3 d1 c3 c1 b3 b1 a3 a1
  Deb(in_b); // d4 d2 c4 c2 b4 b2 a4 a2

  __m128i coeff2_a = _mm_load_si128((const __m128i*)(transform4x4[2]));
  __m128i coeff2_b = _mm_load_si128((const __m128i*)(transform4x4[3]));

  __m128i mul1_a = _mm_madd_epi16(in_a, coeff1_a); // a1A1+a3A3 b1A1+b3A3 c1A1+c3A3 d1A1+d3A3
  __m128i mul1_b = _mm_madd_epi16(in_b, coeff1_b); // a2A2+a4A4 b2A2+b4A4 c2A2+c4A4 d2A2+d4A4

  __m128i rnd = _mm_set1_epi32(64);
  mul1_a = _mm_add_epi32(mul1_a, rnd);  // rounding goes into row 1 and row 4

  __m128i row_1 = _mm_add_epi32(mul1_a,mul1_b);
  Deb(mul1_a);
  Deb(mul1_b);
  Deb(row_1);

  __m128i mul2_a = _mm_madd_epi16(in_a, coeff2_a); // a1C1+a3C3 b1C1+b3C3 c1C1+c3C3 d1C1+d3C3
  __m128i mul2_b = _mm_madd_epi16(in_b, coeff2_b); // a2C2+a4C4 b2C2+b4C4 c2C2+c4C4 d2C2+d4C4

  mul2_a = _mm_add_epi32(mul2_a, rnd);  // rounding goes into row 2 and row 3

  __m128i row_2 = _mm_add_epi32(mul2_a,mul2_b);
  Deb(mul2_a);
  Deb(mul2_b);
  Deb(row_2);

  __m128i row_3 = _mm_sub_epi32(mul2_a,mul2_b);
  __m128i row_4 = _mm_sub_epi32(mul1_a,mul1_b);
  Deb(row_3);
  Deb(row_4);

  row_1 = _mm_srai_epi32(row_1, 7);
  row_2 = _mm_srai_epi32(row_2, 7);
  row_3 = _mm_srai_epi32(row_3, 7);
  row_4 = _mm_srai_epi32(row_4, 7);

  Deb(row_1);
  Deb(row_2);
  Deb(row_3);
  Deb(row_4);

  /*
  __m128i tmp_a = _mm_packs_epi32(row_1, row_2);
  __m128i tmp_b = _mm_packs_epi32(row_3, row_4);

  Deb(tmp_a);
  Deb(tmp_b);
  */

  row_1 = _mm_packs_epi32(row_1, row_1);
  row_2 = _mm_packs_epi32(row_2, row_2);
  row_3 = _mm_packs_epi32(row_3, row_3);
  row_4 = _mm_packs_epi32(row_4, row_4);

  // 27 ins

  Deb(row_1);
  Deb(row_2);
  Deb(row_3);
  Deb(row_4);


  // --- horizontal transforms ---

  // do two horizontal 4-element vector multiplications at once

  __m128i coeffH_a = _mm_load_si128((const __m128i*)(transform4x4[4]));
  __m128i coeffH_b = _mm_load_si128((const __m128i*)(transform4x4[5]));
  Deb(coeffH_a);
  Deb(coeffH_b);

  __m128i r1l = _mm_madd_epi16(row_1, coeffH_a);
  __m128i r1r = _mm_madd_epi16(row_1, coeffH_b);
  Deb(r1l);
  Deb(r1r);

  // 4-element vector multiplications are still split into 2 sums each. Add the sum-pairs.
  __m128i r1 = _mm_hadd_epi32(r1l,r1r);
  Deb(r1);

  __m128i r2l = _mm_madd_epi16(row_2, coeffH_a);
  __m128i r2r = _mm_madd_epi16(row_2, coeffH_b);
  Deb(r2l);
  Deb(r2r);

  __m128i r2 = _mm_hadd_epi32(r2l,r2r);
  Deb(r2);

  __m128i r3l = _mm_madd_epi16(row_3, coeffH_a);
  __m128i r3r = _mm_madd_epi16(row_3, coeffH_b);
  Deb(r3l);
  Deb(r3r);

  __m128i r3 = _mm_hadd_epi32(r3l,r3r);
  Deb(r3);

  __m128i r4l = _mm_madd_epi16(row_4, coeffH_a);
  __m128i r4r = _mm_madd_epi16(row_4, coeffH_b);
  Deb(r4l);
  Deb(r4r);

  __m128i r4 = _mm_hadd_epi32(r4l,r4r);
  Deb(r4);

  __m128i rnd2 = _mm_set1_epi32(1<<11);
  r1 = _mm_srai_epi32(_mm_add_epi32(r1,rnd2), 12);
  r2 = _mm_srai_epi32(_mm_add_epi32(r2,rnd2), 12);
  r3 = _mm_srai_epi32(_mm_add_epi32(r3,rnd2), 12);
  r4 = _mm_srai_epi32(_mm_add_epi32(r4,rnd2), 12);

  __m128i out_1 = _mm_packs_epi32(r1,r1);
  __m128i out_2 = _mm_packs_epi32(r2,r2);
  __m128i out_3 = _mm_packs_epi32(r3,r3);
  __m128i out_4 = _mm_packs_epi32(r4,r4);
  Deb(out_1);
  Deb(out_2);
  Deb(out_3);
  Deb(out_4);

  // + 23 ins = 50 ins

  uint32_t in32_1 = *(uint32_t*)(dst+0*stride);
  uint32_t in32_2 = *(uint32_t*)(dst+1*stride);
  uint32_t in32_3 = *(uint32_t*)(dst+2*stride);
  uint32_t in32_4 = *(uint32_t*)(dst+3*stride);

  __m128i in_1 = _mm_cvtsi32_si128(in32_1);
  __m128i in_2 = _mm_cvtsi32_si128(in32_2);
  __m128i in_3 = _mm_cvtsi32_si128(in32_3);
  __m128i in_4 = _mm_cvtsi32_si128(in32_4);

  __m128i zero = _mm_setzero_si128();
  in_1 = _mm_unpacklo_epi8(in_1,zero);
  in_2 = _mm_unpacklo_epi8(in_2,zero);
  in_3 = _mm_unpacklo_epi8(in_3,zero);
  in_4 = _mm_unpacklo_epi8(in_4,zero);
  Deb(in_1);
  Deb(in_2);
  Deb(in_3);
  Deb(in_4);

  out_1 = _mm_adds_epi16(out_1,in_1);
  out_2 = _mm_adds_epi16(out_2,in_2);
  out_3 = _mm_adds_epi16(out_3,in_3);
  out_4 = _mm_adds_epi16(out_4,in_4);

  out_1 = _mm_packus_epi16(out_1,out_1);
  out_2 = _mm_packus_epi16(out_2,out_2);
  out_3 = _mm_packus_epi16(out_3,out_3);
  out_4 = _mm_packus_epi16(out_4,out_4);
  Deb(out_1);
  Deb(out_2);
  Deb(out_3);
  Deb(out_4);

  *(uint32_t*)(dst+0*stride) = _mm_cvtsi128_si32(out_1);
  *(uint32_t*)(dst+1*stride) = _mm_cvtsi128_si32(out_2);
  *(uint32_t*)(dst+2*stride) = _mm_cvtsi128_si32(out_3);
  *(uint32_t*)(dst+3*stride) = _mm_cvtsi128_si32(out_4);

  // + 25 ins = 75 ins
}


void dct()
{
  const int w = 4;

  int16_t coeff[w*w];

  uint8_t dstc[w*w], dstsse[w*w];

  memset(dstc  ,0,w*w);
  memset(dstsse,0,w*w);
  memset(coeff ,0,w*w*2);

  if (!D)
  {
    srand(time(0));
  }

  for (int i=0;i<w*w;i++) {
    //coeff[i] = 30*i+1; //rand() % 210;// - 10;

    dstc[i] = i;
    dstsse[i] = i;
  }
  coeff[0] = 20;
  coeff[1] = 55;
  coeff[4] = 28;
  coeff[5] = 63;

  for (int i=0;i<w;i++) {
    printf("%02d: ",i);
    for (int x=0;x<w;x++) {
      printf("%04x ",coeff[i*w+x]);
    }
    printf("\n");
  }

  printf("\n");


  const uint64_t nIter = (D ? 1 : 100000000); //1000000000);
  for (int i=0;i<nIter;i++) {

#if 0
    for (int i=0;i<w*w;i++) {
      coeff[i] = rand() % 1000 - 500;
    }
#endif

    if (w==4) {
      transform_4x4_add_8_fallback(dstc,coeff,w,3,3);
      //ff_hevc_transform_4x4_add_8_sse4(dstsse,coeff,w,3,3);
      //idct_4x4_add_8_sse4(dstsse,coeff,w,3,3);
    }

    if (w==8) {
    }

    if (w==16) {
    }

    if (w==32) {
    }

    for (int i=0;i<w*w;i++) {
      //assert(dstsse[i] == dstc[i]);
    }
  }


  if (1) {
    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      for (int x=0;x<w;x++) {
        printf("%02x ",dstsse[i*w+x]);
      }
      printf("\n");
    }

    printf("\n");

    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      for (int x=0;x<w;x++) {
        printf("%02x ",dstc[i*w+x]);
      }
      printf("\n");
    }
  }
}
