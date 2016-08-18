

#include "libde265/x86/sse-dct.h"
#include "libde265/fallback-dct.h"
//#include "iacaMarks.h"

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

#define D 1

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



/*
  64  89  83  75  64  50  36  18
  64  75  36 -18 -64 -89 -83 -50
  64  50 -36 -89 -64  18  83  75
  64  18 -83 -50  64  75 -36 -89
  64 -18 -83  50  64 -75 -36  89
  64 -50 -36  89 -64 -18  83 -75
  64 -75  36  18 -64  89 -83  50
  64 -89  83 -75  64 -50  36 -18

  #A  #B  #D  #B  #A  #C  #D  #C
*/
ALIGNED_16(static const int16_t) transform8x8_sparse_V[][8] =
{
  // --- vertical transforms ---

  // first 4x2 block coefficients for each output line
  {  64, 89, 64, 89, 64, 89, 64, 89 },
  {  64, 75, 64, 75, 64, 75, 64, 75 },
  {  64, 50, 64, 50, 64, 50, 64, 50 },
  {  64, 18, 64, 18, 64, 18, 64, 18 },
  {  64,-18, 64,-18, 64,-18, 64,-18 },
  {  64,-50, 64,-50, 64,-50, 64,-50 },
  {  64,-75, 64,-75, 64,-75, 64,-75 },
  {  64,-89, 64,-89, 64,-89, 64,-89 },

  // second 4x2 block coefficients for each output line
  {  83, 75, 83, 75, 83, 75, 83, 75 },
  {  36,-18, 36,-18, 36,-18, 36,-18 },
  { -36,-89,-36,-89,-36,-89,-36,-89 },
  { -83,-50,-83,-50,-83,-50,-83,-50 },
  { -83, 50,-83, 50,-83, 50,-83, 50 },
  { -36, 89,-36, 89,-36, 89,-36, 89 },
  {  36, 18, 36, 18, 36, 18, 36, 18 },
  {  83,-75, 83,-75, 83,-75, 83,-75 },

  // third 4x2 block coefficients for each output line
  {  64, 50, 64, 50, 64, 50, 64, 50},
  { -64,-89,-64,-89,-64,-89,-64,-89},
  { -64, 18,-64, 18,-64, 18,-64, 18},
  {  64, 75, 64, 75, 64, 75, 64, 75},
  {  64,-75, 64,-75, 64,-75, 64,-75},
  { -64,-18,-64,-18,-64,-18,-64,-18},
  { -64, 89,-64, 89,-64, 89,-64, 89},
  {  64,-50, 64,-50, 64,-50, 64,-50},

  // fourth 4x2 block coefficients for each output line
  {  36, 18, 36, 18, 36, 18, 36, 18},
  { -83,-50,-83,-50,-83,-50,-83,-50},
  {  83, 75, 83, 75, 83, 75, 83, 75},
  { -36,-89,-36,-89,-36,-89,-36,-89},
  { -36, 89,-36, 89,-36, 89,-36, 89},
  {  83,-75, 83,-75, 83,-75, 83,-75},
  { -83, 50,-83, 50,-83, 50,-83, 50},
  {  36,-18, 36,-18, 36,-18, 36,-18}
};


/*
  64  89  83  75  64  50  36  18
  64  75  36 -18 -64 -89 -83 -50
  64  50 -36 -89 -64  18  83  75
  64  18 -83 -50  64  75 -36 -89
  64 -18 -83  50  64 -75 -36  89
  64 -50 -36  89 -64 -18  83 -75
  64 -75  36  18 -64  89 -83  50
  64 -89  83 -75  64 -50  36 -18

  #A  #B  #D  #B  #A  #C  #D  #C
*/
ALIGNED_16(static const int16_t) transform8x8_sparse_H[][8] =
{
  // horizontal transforms

  { 64,  89,  83,  75,
    64,  75,  36, -18 },
  { 64,  50, -36, -89,
    64,  18, -83, -50 },
  { 64, -18, -83,  50,
    64, -50, -36,  89, },
  { 64, -75,  36,  18,
    64, -89,  83, -75 },
  {                   64,  50,  36,  18,
                     -64, -89, -83, -50 },
  {                  -64,  18,  83,  75,
                      64,  75, -36, -89, },
  {                   64, -75, -36,  89,
                     -64, -18,  83, -75 },
  {                  -64,  89, -83,  50,
                      64, -50,  36, -18 }
};


template <int maxCol, int maxRow>
void __attribute__ ((noinline)) idct_8x8_add_8_sse4_local_ORIG(uint8_t *dst, const int16_t *coeffs,
                                                               ptrdiff_t stride)
{
  const bool process_rightHalf = (maxCol>=4);

  __m128i pass1_out32[8*2];

  __m128i rnd = _mm_set1_epi32(64);
  for (int i=0;i<8;i++) {
    pass1_out32[2*i] = rnd;

    if (process_rightHalf) {
      pass1_out32[2*i+1] = rnd;
    }
    else {
      pass1_out32[2*i+1] = _mm_setzero_si128();
    }
  }


  // --- vertical transform ---

#if 1
  for (int r=0; r<=maxRow; r+=2) {
    //printf("r=%d\n",r);

    __m128i input16_toprow = _mm_load_si128((const __m128i*)(coeffs + r*8));
    __m128i input16_botrow = _mm_load_si128((const __m128i*)(coeffs + r*8 + 8));
    Deb(input16_toprow);
    Deb(input16_botrow);

    __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
    Deb(in_left);

    __m128i in_right;
    if (process_rightHalf) {
      in_right = _mm_unpackhi_epi16(input16_toprow, input16_botrow);
      Deb(in_right);
    }

    for (int y=0; y<8; y++) {
      __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V[y + 8/2*r]));
      Deb(coeffpair);

      __m128i mul = _mm_madd_epi16(in_left, coeffpair);
      pass1_out32[2*y] = _mm_add_epi32(pass1_out32[2*y], mul);
      Deb(pass1_out32[2*y]);

      if (process_rightHalf) {
        __m128i mul_r = _mm_madd_epi16(in_right, coeffpair);
        pass1_out32[2*y+1] = _mm_add_epi32(pass1_out32[2*y+1], mul_r);
        Deb(pass1_out32[2*y+1]);
      }
    }
  }
#endif

#if 0
  {
    int r=0;

    __m128i input16_toprow = _mm_load_si128((const __m128i*)(coeffs + r*8));
    __m128i input16_botrow = _mm_load_si128((const __m128i*)(coeffs + r*8 + 8));
    __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
    __m128i in_right;
    if (process_rightHalf) {
      in_right = _mm_unpackhi_epi16(input16_toprow, input16_botrow);
    }

    for (int y=0; y<8; y++) {
      __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V[y + 8/2*r]));
      __m128i mul = _mm_madd_epi16(in_left, coeffpair);
      pass1_out32[2*y] = _mm_add_epi32(pass1_out32[2*y], mul);

      if (process_rightHalf) {
        __m128i mul_r = _mm_madd_epi16(in_right, coeffpair);
        pass1_out32[2*y+1] = _mm_add_epi32(pass1_out32[2*y+1], mul_r);
      }
    }
  }
  if (maxRow>=2) {
    int r=2;

    __m128i input16_toprow = _mm_load_si128((const __m128i*)(coeffs + r*8));
    __m128i input16_botrow = _mm_load_si128((const __m128i*)(coeffs + r*8 + 8));
    __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
    __m128i in_right;
    if (process_rightHalf) {
      in_right = _mm_unpackhi_epi16(input16_toprow, input16_botrow);
    }

    for (int y=0; y<8; y++) {
      __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V[y + 8/2*r]));
      __m128i mul = _mm_madd_epi16(in_left, coeffpair);
      pass1_out32[2*y] = _mm_add_epi32(pass1_out32[2*y], mul);

      if (process_rightHalf) {
        __m128i mul_r = _mm_madd_epi16(in_right, coeffpair);
        pass1_out32[2*y+1] = _mm_add_epi32(pass1_out32[2*y+1], mul_r);
      }
    }
  }
  if (maxRow>=4) {
    int r=4;

    __m128i input16_toprow = _mm_load_si128((const __m128i*)(coeffs + r*8));
    __m128i input16_botrow = _mm_load_si128((const __m128i*)(coeffs + r*8 + 8));
    __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
    __m128i in_right;
    if (process_rightHalf) {
      in_right = _mm_unpackhi_epi16(input16_toprow, input16_botrow);
    }

    for (int y=0; y<8; y++) {
      __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V[y + 8/2*r]));
      __m128i mul = _mm_madd_epi16(in_left, coeffpair);
      pass1_out32[2*y] = _mm_add_epi32(pass1_out32[2*y], mul);

      if (process_rightHalf) {
        __m128i mul_r = _mm_madd_epi16(in_right, coeffpair);
        pass1_out32[2*y+1] = _mm_add_epi32(pass1_out32[2*y+1], mul_r);
      }
    }
  }
  if (maxRow>=6) {
    int r=6;

    __m128i input16_toprow = _mm_load_si128((const __m128i*)(coeffs + r*8));
    __m128i input16_botrow = _mm_load_si128((const __m128i*)(coeffs + r*8 + 8));
    __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
    __m128i in_right;
    if (process_rightHalf) {
      in_right = _mm_unpackhi_epi16(input16_toprow, input16_botrow);
    }

    for (int y=0; y<8; y++) {
      __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V[y + 8/2*r]));
      __m128i mul = _mm_madd_epi16(in_left, coeffpair);
      pass1_out32[2*y] = _mm_add_epi32(pass1_out32[2*y], mul);

      if (process_rightHalf) {
        __m128i mul_r = _mm_madd_epi16(in_right, coeffpair);
        pass1_out32[2*y+1] = _mm_add_epi32(pass1_out32[2*y+1], mul_r);
      }
    }
  }
#endif


  __m128i pass1_16[8];

  for (int y=0; y<8; y++) {
    pass1_out32[2*y  ] = _mm_srai_epi32(pass1_out32[2*y  ], 7);

    if (process_rightHalf) {
      pass1_out32[2*y+1] = _mm_srai_epi32(pass1_out32[2*y+1], 7);
    }

    pass1_16[y] = _mm_packs_epi32(pass1_out32[2*y], pass1_out32[2*y+1]);
    Deb(pass1_16[y]);
  }


  // --- horizontal transforms ---

  __m128i rnd2 = _mm_set1_epi32(1<<11);

  for (int y=0;y<8;y++) {
    __m128i in_row = pass1_16[y];
    Deb(in_row);

    __m128i in_left = _mm_unpacklo_epi64(in_row,in_row);
    Deb(in_left);

    __m128i coeff0 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[0]));
    __m128i coeff2 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[1]));
    __m128i coeff4 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[2]));
    __m128i coeff6 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[3]));
    Deb(coeff0);
    Deb(coeff2);
    Deb(coeff4);
    Deb(coeff6);

    __m128i sum0 = _mm_madd_epi16(in_left, coeff0);
    __m128i sum2 = _mm_madd_epi16(in_left, coeff2);
    __m128i sum4 = _mm_madd_epi16(in_left, coeff4);
    __m128i sum6 = _mm_madd_epi16(in_left, coeff6);
    Deb(sum0);
    Deb(sum2);
    Deb(sum4);
    Deb(sum6);

    if (process_rightHalf) {
      __m128i in_right = _mm_unpackhi_epi64(in_row,in_row);
      Deb(in_right);

      __m128i coeff1 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[4]));
      __m128i coeff3 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[5]));
      __m128i coeff5 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[6]));
      __m128i coeff7 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[7]));
      Deb(coeff1);
      Deb(coeff3);
      Deb(coeff5);
      Deb(coeff7);

      __m128i sum1 = _mm_madd_epi16(in_right, coeff1);
      __m128i sum3 = _mm_madd_epi16(in_right, coeff3);
      __m128i sum5 = _mm_madd_epi16(in_right, coeff5);
      __m128i sum7 = _mm_madd_epi16(in_right, coeff7);
      Deb(sum1);
      Deb(sum3);
      Deb(sum5);
      Deb(sum7);

      sum0 = _mm_add_epi32(sum0,sum1);
      sum2 = _mm_add_epi32(sum2,sum3);
      sum4 = _mm_add_epi32(sum4,sum5);
      sum6 = _mm_add_epi32(sum6,sum7);
    }

    __m128i sum_left  = _mm_hadd_epi32(sum0, sum2);
    __m128i sum_right = _mm_hadd_epi32(sum4, sum6);
    Deb(sum_left);
    Deb(sum_right);


    sum_left  = _mm_srai_epi32(_mm_add_epi32(sum_left ,rnd2), 12);
    sum_right = _mm_srai_epi32(_mm_add_epi32(sum_right,rnd2), 12);

    __m128i residual_16 = _mm_packs_epi32(sum_left,sum_right);
    Deb(residual_16);


    __m128i out_8 = _mm_loadl_epi64((__m128i*)(dst+y*stride));
    __m128i out_16 = _mm_unpacklo_epi8(out_8, _mm_setzero_si128());

    out_16 = _mm_adds_epi16(out_16,residual_16);
    __m128i result = _mm_packus_epi16(out_16,out_16);
    Deb(result);

    _mm_storel_epi64((__m128i*)(dst+y*stride), result);
  }
}





static inline void trans8v_4col(const int16_t* coeffs,
                                const int16_t* transform8x8_sparse_V,
                                __m128i* pass1_out32)
{
  __m128i input16_toprow = _mm_loadl_epi64((const __m128i*)(coeffs));
  __m128i input16_botrow = _mm_loadl_epi64((const __m128i*)(coeffs + 8));
  Deb(input16_toprow);
  Deb(input16_botrow);

  __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
  Deb(in_left);

  for (int y=0; y<8; y++) {
    __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 8*y));
    Deb(coeffpair);

    __m128i mul = _mm_madd_epi16(in_left, coeffpair);
    pass1_out32[y] = _mm_add_epi32(pass1_out32[y], mul);
    Deb(pass1_out32[y]);
  }
}


static inline void trans8h_4col(const __m128i& input,
                                __m128i& coeff0,__m128i& coeff2,__m128i& coeff4,__m128i& coeff6,
                                uint8_t* dst, __m128i& rnd2)
{
  // duplicate the 4 input coefficients -> (I0 I1 I2 I3 I0 I1 I2 I3)
  __m128i in_left = _mm_unpacklo_epi64(input, input);

  // sum01 =  I0*C00+I1*C01  I2*C02+I3*C03  I0*C10+I1*C11  I2*C12+I3*C13
  //          \________ output 0 ________/  \________ output 1 ________/
  __m128i sum01 = _mm_madd_epi16(in_left, coeff0);
  __m128i sum23 = _mm_madd_epi16(in_left, coeff2);
  __m128i sum45 = _mm_madd_epi16(in_left, coeff4);
  __m128i sum67 = _mm_madd_epi16(in_left, coeff6);

  // sum remaining pair
  __m128i sum_0123 = _mm_hadd_epi32(sum01, sum23);
  __m128i sum_4567 = _mm_hadd_epi32(sum45, sum67);
  sum_0123  = _mm_srai_epi32(_mm_add_epi32(sum_0123,rnd2), 12);
  sum_4567  = _mm_srai_epi32(_mm_add_epi32(sum_4567,rnd2), 12);

  // pack residuals to 8 x 16bit
  __m128i residual_16 = _mm_packs_epi32(sum_0123,sum_4567);

  // add residuals to prediction
  __m128i out_8 = _mm_loadl_epi64((__m128i*)(dst));
  __m128i out_16 = _mm_unpacklo_epi8(out_8, _mm_setzero_si128());

  out_16 = _mm_adds_epi16(out_16,residual_16);
  __m128i result = _mm_packus_epi16(out_16,out_16);
  _mm_storel_epi64((__m128i*)(dst), result);
}




template <int maxRow>
void __attribute__ ((noinline)) idct_8x8_add_8_sse4_local_left(uint8_t *dst, const int16_t *coeffs,
                                                               ptrdiff_t stride)
{
  __m128i pass1_out32[8];

  __m128i rnd = _mm_set1_epi32(64);
  for (int i=0;i<8;i++) {
    pass1_out32[i] = rnd;
  }


  // --- vertical transform ---

  /*          */   trans8v_4col(coeffs+0*8, transform8x8_sparse_V[ 0], pass1_out32);
  if (maxRow>=2) { trans8v_4col(coeffs+2*8, transform8x8_sparse_V[ 8], pass1_out32); }
  if (maxRow>=4) { trans8v_4col(coeffs+4*8, transform8x8_sparse_V[16], pass1_out32); }
  if (maxRow>=6) { trans8v_4col(coeffs+6*8, transform8x8_sparse_V[24], pass1_out32); }

  __m128i pass1_16_0 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[0], 7), _mm_setzero_si128());
  __m128i pass1_16_1 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[1], 7), _mm_setzero_si128());
  __m128i pass1_16_2 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[2], 7), _mm_setzero_si128());
  __m128i pass1_16_3 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[3], 7), _mm_setzero_si128());
  __m128i pass1_16_4 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[4], 7), _mm_setzero_si128());
  __m128i pass1_16_5 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[5], 7), _mm_setzero_si128());
  __m128i pass1_16_6 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[6], 7), _mm_setzero_si128());
  __m128i pass1_16_7 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[7], 7), _mm_setzero_si128());



  // --- horizontal transforms ---

  __m128i coeff0 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[0]));
  __m128i coeff1 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[1]));
  __m128i coeff2 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[2]));
  __m128i coeff3 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[3]));

  __m128i rnd2 = _mm_set1_epi32(1<<11);

  trans8h_4col(pass1_16_0, coeff0,coeff1,coeff2,coeff3, dst+0*stride, rnd2);
  trans8h_4col(pass1_16_1, coeff0,coeff1,coeff2,coeff3, dst+1*stride, rnd2);
  trans8h_4col(pass1_16_2, coeff0,coeff1,coeff2,coeff3, dst+2*stride, rnd2);
  trans8h_4col(pass1_16_3, coeff0,coeff1,coeff2,coeff3, dst+3*stride, rnd2);
  trans8h_4col(pass1_16_4, coeff0,coeff1,coeff2,coeff3, dst+4*stride, rnd2);
  trans8h_4col(pass1_16_5, coeff0,coeff1,coeff2,coeff3, dst+5*stride, rnd2);
  trans8h_4col(pass1_16_6, coeff0,coeff1,coeff2,coeff3, dst+6*stride, rnd2);
  trans8h_4col(pass1_16_7, coeff0,coeff1,coeff2,coeff3, dst+7*stride, rnd2);
}



// max coefficient array size 4x2  (4 columns, 2 rows)
void __attribute__ ((noinline)) idct_8x8_add_8_sse4_local_4x2(uint8_t *dst, const int16_t *coeffs,
                                                              ptrdiff_t stride)
{
  __m128i rnd = _mm_set1_epi32(64);


  // --- vertical transform ---

  __m128i input16_toprow = _mm_loadl_epi64((const __m128i*)(coeffs));
  __m128i input16_botrow = _mm_loadl_epi64((const __m128i*)(coeffs + 8));
  Deb(input16_toprow);
  Deb(input16_botrow);

  __m128i in_left = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
  Deb(in_left);


  __m128i coeffpair0 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 0));
  __m128i mul = _mm_madd_epi16(in_left, coeffpair0);
  __m128i coeffpair1 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 1));
  __m128i sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_0 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair1);
  __m128i coeffpair2 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 2));
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_1 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair2);
  __m128i coeffpair3 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 3));
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_2 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair3);
  __m128i coeffpair4 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 4));
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_3 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair4);
  __m128i coeffpair5 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 5));
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_4 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair5);
  __m128i coeffpair6 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 6));
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_5 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair6);
  __m128i coeffpair7 = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 7));
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_6 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());

  mul = _mm_madd_epi16(in_left, coeffpair7);
  sum32 = _mm_add_epi32(rnd, mul);
  __m128i pass1_16_7 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  // --- horizontal transforms ---

  __m128i coeff0 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[0]));
  __m128i coeff1 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[1]));
  __m128i coeff2 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[2]));
  __m128i coeff3 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[3]));

  __m128i rnd2 = _mm_set1_epi32(1<<11);

  trans8h_4col(pass1_16_0, coeff0,coeff1,coeff2,coeff3, dst+0*stride, rnd2);
  trans8h_4col(pass1_16_1, coeff0,coeff1,coeff2,coeff3, dst+1*stride, rnd2);
  trans8h_4col(pass1_16_2, coeff0,coeff1,coeff2,coeff3, dst+2*stride, rnd2);
  trans8h_4col(pass1_16_3, coeff0,coeff1,coeff2,coeff3, dst+3*stride, rnd2);
  trans8h_4col(pass1_16_4, coeff0,coeff1,coeff2,coeff3, dst+4*stride, rnd2);
  trans8h_4col(pass1_16_5, coeff0,coeff1,coeff2,coeff3, dst+5*stride, rnd2);
  trans8h_4col(pass1_16_6, coeff0,coeff1,coeff2,coeff3, dst+6*stride, rnd2);
  trans8h_4col(pass1_16_7, coeff0,coeff1,coeff2,coeff3, dst+7*stride, rnd2);
}




// NOTE: this is not any faster than the generic template function
void __attribute__ ((noinline)) idct_8x8_add_8_sse4_local_4x4(uint8_t *dst, const int16_t *coeffs,
                                                              ptrdiff_t stride)
{
  __m128i rnd = _mm_set1_epi32(64);


  // --- vertical transform ---


  __m128i input16_toprowA = _mm_loadl_epi64((const __m128i*)(coeffs));
  __m128i input16_botrowA = _mm_loadl_epi64((const __m128i*)(coeffs + 8));
  Deb(input16_toprowA);
  Deb(input16_botrowA);

  __m128i input16_toprowB = _mm_loadl_epi64((const __m128i*)(coeffs +16));
  __m128i input16_botrowB = _mm_loadl_epi64((const __m128i*)(coeffs +24));

  __m128i in_leftA = _mm_unpacklo_epi16(input16_toprowA, input16_botrowA);
  __m128i in_leftB = _mm_unpacklo_epi16(input16_toprowB, input16_botrowB);
  Deb(in_leftA);


  __m128i coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 0));
  __m128i coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 0));

  __m128i mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  __m128i mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  __m128i sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_0 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 1));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 1));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_1 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 2));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 2));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_2 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 3));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 3));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_3 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 4));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 4));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_4 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 5));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 5));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_5 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 6));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 6));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_6 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());


  coeffpairA = _mm_load_si128((const __m128i*)(transform8x8_sparse_V   + 7));
  coeffpairB = _mm_load_si128((const __m128i*)(transform8x8_sparse_V+8 + 7));

  mulA = _mm_madd_epi16(in_leftA, coeffpairA);
  mulB = _mm_madd_epi16(in_leftB, coeffpairB);
  sum32 = _mm_add_epi32(_mm_add_epi32(rnd, mulA), mulB);

  __m128i pass1_16_7 = _mm_packs_epi32( _mm_srai_epi32(sum32, 7), _mm_setzero_si128());









  // --- horizontal transforms ---

  __m128i coeff0 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[0]));
  __m128i coeff1 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[1]));
  __m128i coeff2 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[2]));
  __m128i coeff3 = _mm_load_si128((const __m128i*)(transform8x8_sparse_H[3]));

  __m128i rnd2 = _mm_set1_epi32(1<<11);

  trans8h_4col(pass1_16_0, coeff0,coeff1,coeff2,coeff3, dst+0*stride, rnd2);
  trans8h_4col(pass1_16_1, coeff0,coeff1,coeff2,coeff3, dst+1*stride, rnd2);
  trans8h_4col(pass1_16_2, coeff0,coeff1,coeff2,coeff3, dst+2*stride, rnd2);
  trans8h_4col(pass1_16_3, coeff0,coeff1,coeff2,coeff3, dst+3*stride, rnd2);
  trans8h_4col(pass1_16_4, coeff0,coeff1,coeff2,coeff3, dst+4*stride, rnd2);
  trans8h_4col(pass1_16_5, coeff0,coeff1,coeff2,coeff3, dst+5*stride, rnd2);
  trans8h_4col(pass1_16_6, coeff0,coeff1,coeff2,coeff3, dst+6*stride, rnd2);
  trans8h_4col(pass1_16_7, coeff0,coeff1,coeff2,coeff3, dst+7*stride, rnd2);
}





// 24 instructions
static inline void transpose_8x8_16bit(const __m128i& in0, const __m128i& in1,
                                       const __m128i& in2, const __m128i& in3,
                                       const __m128i& in4, const __m128i& in5,
                                       const __m128i& in6, const __m128i& in7,
                                       __m128i& out0, __m128i& out1, __m128i& out2, __m128i& out3,
                                       __m128i& out4, __m128i& out5, __m128i& out6, __m128i& out7)
{
  __m128i mix04l = _mm_unpacklo_epi16(in0, in4);
  __m128i mix15l = _mm_unpacklo_epi16(in1, in5);
  __m128i mix26l = _mm_unpacklo_epi16(in2, in6);
  __m128i mix37l = _mm_unpacklo_epi16(in3, in7);
  __m128i mix0246ll = _mm_unpacklo_epi16(mix04l, mix26l);
  __m128i mix1357ll = _mm_unpacklo_epi16(mix15l, mix37l);
  out0 = _mm_unpacklo_epi16(mix0246ll, mix1357ll);
  out1 = _mm_unpackhi_epi16(mix0246ll, mix1357ll);
  __m128i mix0246lh = _mm_unpackhi_epi16(mix04l, mix26l);
  __m128i mix1357lh = _mm_unpackhi_epi16(mix15l, mix37l);
  out2 = _mm_unpacklo_epi16(mix0246lh, mix1357lh);
  out3 = _mm_unpackhi_epi16(mix0246lh, mix1357lh);

  __m128i mix04h = _mm_unpackhi_epi16(in0, in4);
  __m128i mix15h = _mm_unpackhi_epi16(in1, in5);
  __m128i mix26h = _mm_unpackhi_epi16(in2, in6);
  __m128i mix37h = _mm_unpackhi_epi16(in3, in7);
  __m128i mix0246hl = _mm_unpacklo_epi16(mix04h, mix26h);
  __m128i mix1357hl = _mm_unpacklo_epi16(mix15h, mix37h);
  out4 = _mm_unpacklo_epi16(mix0246hl, mix1357hl);
  out5 = _mm_unpackhi_epi16(mix0246hl, mix1357hl);
  __m128i mix0246hh = _mm_unpackhi_epi16(mix04h, mix26h);
  __m128i mix1357hh = _mm_unpackhi_epi16(mix15h, mix37h);
  out6 = _mm_unpacklo_epi16(mix0246hh, mix1357hh);
  out7 = _mm_unpackhi_epi16(mix0246hh, mix1357hh);
}


/*
  EE  ..  EE  ..  EE  ..  EE  ..
  ..  OO  ..  OO  ..  OO  ..  OO

  64  89  83  75  64  50  36  18
  64  75  36 -18 -64 -89 -83 -50
  64  50 -36 -89 -64  18  83  75
  64  18 -83 -50  64  75 -36 -89
  64 -18 -83  50  64 -75 -36  89
  64 -50 -36  89 -64 -18  83 -75
  64 -75  36  18 -64  89 -83  50
  64 -89  83 -75  64 -50  36 -18

  #A  #B  #D  #B  #A  #C  #D  #C
*/
ALIGNED_16(static const int16_t) transform8x8_full_H[][8] =
{
  // horizontal transforms

  { 64,  89,  83,  75,  64,  50,  36,  18 },
  { 64,  75,  36, -18, -64, -89, -83, -50 },
  { 64,  50, -36, -89, -64,  18,  83,  75 },
  { 64,  18, -83, -50,  64,  75, -36, -89 },
  { 64, -18, -83,  50,  64, -75, -36,  89 },
  { 64, -50, -36,  89, -64, -18,  83, -75 },
  { 64, -75,  36,  18, -64,  89, -83,  50 },
  { 64, -89,  83, -75,  64, -50,  36, -18 }
};



static inline void trans8_v(const int16_t* coeffs,
                            const int16_t* transform8x8_sparse_V,
                            __m128i* pass1_out32)
{
  __m128i input16_toprow = _mm_load_si128((const __m128i*)(coeffs));
  __m128i input16_botrow = _mm_load_si128((const __m128i*)(coeffs + 8));
  Deb(input16_toprow);
  Deb(input16_botrow);

  __m128i in_left  = _mm_unpacklo_epi16(input16_toprow, input16_botrow);
  __m128i in_right = _mm_unpackhi_epi16(input16_toprow, input16_botrow);
  Deb(in_left);
  Deb(in_right);

  for (int y=0; y<8; y++) {
    __m128i coeffpair = _mm_load_si128((const __m128i*)(transform8x8_sparse_V + 8*y));
    Deb(coeffpair);

    __m128i mul_left  = _mm_madd_epi16(in_left,  coeffpair);
    __m128i mul_right = _mm_madd_epi16(in_right, coeffpair);

    pass1_out32[2*y  ] = _mm_add_epi32(pass1_out32[2*y  ], mul_left);
    pass1_out32[2*y+1] = _mm_add_epi32(pass1_out32[2*y+1], mul_right);
    Deb(pass1_out32[2*y  ]);
    Deb(pass1_out32[2*y+1]);
  }
}


static inline void trans8_h(const __m128i& input, uint8_t* dst, __m128i& rnd2)
{
  __m128i sum0 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[0])));
  __m128i sum1 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[1])));
  __m128i sum2 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[2])));
  __m128i sum3 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[3])));
  __m128i sum01 = _mm_hadd_epi32(sum0,sum1);
  __m128i sum23 = _mm_hadd_epi32(sum2,sum3);

  __m128i sum4 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[4])));
  __m128i sum5 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[5])));
  __m128i sum6 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[6])));
  __m128i sum7 = _mm_madd_epi16(input, _mm_load_si128((const __m128i*)(transform8x8_full_H[7])));
  __m128i sum0123 = _mm_hadd_epi32(sum01,sum23);

  __m128i sum45 = _mm_hadd_epi32(sum4,sum5);
  __m128i sum67 = _mm_hadd_epi32(sum6,sum7);

  __m128i sum4567 = _mm_hadd_epi32(sum45,sum67);

  sum0123 = _mm_srai_epi32(_mm_add_epi32(sum0123,rnd2), 12);
  sum4567 = _mm_srai_epi32(_mm_add_epi32(sum4567,rnd2), 12);

  __m128i residual_16 = _mm_packs_epi32(sum0123,sum4567);
  __m128i out_8 = _mm_loadl_epi64((__m128i*)(dst));
  __m128i out_16 = _mm_unpacklo_epi8(out_8, _mm_setzero_si128());

  out_16 = _mm_adds_epi16(out_16,residual_16);
  __m128i result = _mm_packus_epi16(out_16,out_16);
  _mm_storel_epi64((__m128i*)(dst), result);
}


// Note: not good. Too slow to be useful.
// TODO: replace V-transform with transpose/sym-trans/transpose and test again
template <int maxRow>
void __attribute__ ((noinline)) idct_8x8_add_8_sse4_local_fullwidth(uint8_t *dst,
                                                                    const int16_t *coeffs,
                                                                    ptrdiff_t stride)
{
  __m128i pass1_out32[8*2];

  __m128i rnd = _mm_set1_epi32(64);
  for (int i=0;i<8;i++) {
    pass1_out32[2*i] = rnd;
    pass1_out32[2*i+1] = rnd;
  }



  // --- vertical transform ---

  /*          */   trans8_v(coeffs+0*8, transform8x8_sparse_V[ 0], pass1_out32);
  if (maxRow>=2) { trans8_v(coeffs+2*8, transform8x8_sparse_V[ 8], pass1_out32); }
  if (maxRow>=4) { trans8_v(coeffs+4*8, transform8x8_sparse_V[16], pass1_out32); }
  if (maxRow>=6) { trans8_v(coeffs+6*8, transform8x8_sparse_V[24], pass1_out32); }

  __m128i pass1_16_0 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 0], 7),
                                        _mm_srai_epi32(pass1_out32[ 1], 7) );
  __m128i pass1_16_1 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 2], 7),
                                        _mm_srai_epi32(pass1_out32[ 3], 7) );
  __m128i pass1_16_2 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 4], 7),
                                        _mm_srai_epi32(pass1_out32[ 5], 7) );
  __m128i pass1_16_3 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 6], 7),
                                        _mm_srai_epi32(pass1_out32[ 7], 7) );
  __m128i pass1_16_4 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 8], 7),
                                        _mm_srai_epi32(pass1_out32[ 9], 7) );
  __m128i pass1_16_5 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[10], 7),
                                        _mm_srai_epi32(pass1_out32[11], 7) );
  __m128i pass1_16_6 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[12], 7),
                                        _mm_srai_epi32(pass1_out32[13], 7) );
  __m128i pass1_16_7 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[14], 7),
                                        _mm_srai_epi32(pass1_out32[15], 7) );



  // --- horizontal transforms ---

  __m128i rnd2 = _mm_set1_epi32(1<<11);

  trans8_h(pass1_16_0, dst+0*stride, rnd2);
  trans8_h(pass1_16_1, dst+1*stride, rnd2);
  trans8_h(pass1_16_2, dst+2*stride, rnd2);
  trans8_h(pass1_16_3, dst+3*stride, rnd2);
  trans8_h(pass1_16_4, dst+4*stride, rnd2);
  trans8_h(pass1_16_5, dst+5*stride, rnd2);
  trans8_h(pass1_16_6, dst+6*stride, rnd2);
  trans8_h(pass1_16_7, dst+7*stride, rnd2);
}



ALIGNED_16(static const int16_t) transform8x8[12][8] =
{
  // odd
  {  89,  75,  89,  75, 89,  75, 89,  75 },
  {  50,  18,  50,  18, 50,  18, 50,  18 },
  {  75, -18,  75, -18, 75, -18, 75, -18 },
  { -89, -50, -89, -50,-89, -50,-89, -50 },
  {  50, -89,  50, -89, 50, -89, 50, -89 },
  {  18,  75,  18,  75, 18,  75, 18,  75 },
  {  18, -50,  18, -50, 18, -50, 18, -50 },
  {  75, -89,  75, -89, 75, -89, 75, -89 },

  // even
  {  64,  64,  64,  64, 64,  64, 64,  64 },
  {  64, -64,  64, -64, 64, -64, 64, -64 },
  {  83,  36,  83,  36, 83,  36, 83,  36 },
  {  36, -83,  36, -83, 36, -83, 36, -83 }
};


static inline void transform_8x8_v(__m128i& out0,__m128i& out1,__m128i& out2,__m128i& out3,
                                   __m128i& out4,__m128i& out5,__m128i& out6,__m128i& out7,
                                   __m128i in0, __m128i in1, __m128i in2, __m128i in3,
                                   __m128i in4, __m128i in5, __m128i in6, __m128i in7,
                                   __m128i rnd, int shift)
{
  __m128i in01l = _mm_unpacklo_epi16(in0,in1);
  __m128i in01h = _mm_unpackhi_epi16(in0,in1);
  __m128i in23l = _mm_unpacklo_epi16(in2,in3);
  __m128i in23h = _mm_unpackhi_epi16(in2,in3);
  __m128i in45l = _mm_unpacklo_epi16(in4,in5);
  __m128i in45h = _mm_unpackhi_epi16(in4,in5);
  __m128i in67l = _mm_unpacklo_epi16(in6,in7);
  __m128i in67h = _mm_unpackhi_epi16(in6,in7);

  // QWE
}


void __attribute__ ((noinline)) idct_8x8_add_8_sse4_local_full(uint8_t *dst,
                                                               const int16_t *coeffs,
                                                               ptrdiff_t stride)
{
  __m128i rnd1 = _mm_set1_epi32(64);


  // --- vertical transform ---

  __m128i in0 = _mm_load_si128((__m128i*)(dst+0*stride));
  __m128i in1 = _mm_load_si128((__m128i*)(dst+1*stride));
  __m128i in2 = _mm_load_si128((__m128i*)(dst+2*stride));
  __m128i in3 = _mm_load_si128((__m128i*)(dst+3*stride));
  __m128i in4 = _mm_load_si128((__m128i*)(dst+4*stride));
  __m128i in5 = _mm_load_si128((__m128i*)(dst+5*stride));
  __m128i in6 = _mm_load_si128((__m128i*)(dst+6*stride));
  __m128i in7 = _mm_load_si128((__m128i*)(dst+7*stride));

  __m128i tmp0,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7;

  transform_8x8_v(tmp0,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7,
                  in0, in1, in2, in3, in4, in5, in6, in7,
                  rnd1, 7);

#if 0
  int maxRow = 999; // TODO
  /*          */   trans8_v(coeffs+0*8, transform8x8_sparse_V[ 0], pass1_out32);
  if (maxRow>=2) { trans8_v(coeffs+2*8, transform8x8_sparse_V[ 8], pass1_out32); }
  if (maxRow>=4) { trans8_v(coeffs+4*8, transform8x8_sparse_V[16], pass1_out32); }
  if (maxRow>=6) { trans8_v(coeffs+6*8, transform8x8_sparse_V[24], pass1_out32); }

  __m128i pass1_16_0 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 0], 7),
                                        _mm_srai_epi32(pass1_out32[ 1], 7) );
  __m128i pass1_16_1 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 2], 7),
                                        _mm_srai_epi32(pass1_out32[ 3], 7) );
  __m128i pass1_16_2 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 4], 7),
                                        _mm_srai_epi32(pass1_out32[ 5], 7) );
  __m128i pass1_16_3 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 6], 7),
                                        _mm_srai_epi32(pass1_out32[ 7], 7) );
  __m128i pass1_16_4 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[ 8], 7),
                                        _mm_srai_epi32(pass1_out32[ 9], 7) );
  __m128i pass1_16_5 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[10], 7),
                                        _mm_srai_epi32(pass1_out32[11], 7) );
  __m128i pass1_16_6 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[12], 7),
                                        _mm_srai_epi32(pass1_out32[13], 7) );
  __m128i pass1_16_7 = _mm_packs_epi32( _mm_srai_epi32(pass1_out32[14], 7),
                                        _mm_srai_epi32(pass1_out32[15], 7) );



  // --- horizontal transforms ---

  __m128i rnd2 = _mm_set1_epi32(1<<11);

  trans8_h(pass1_16_0, dst+0*stride, rnd2);
  trans8_h(pass1_16_1, dst+1*stride, rnd2);
  trans8_h(pass1_16_2, dst+2*stride, rnd2);
  trans8_h(pass1_16_3, dst+3*stride, rnd2);
  trans8_h(pass1_16_4, dst+4*stride, rnd2);
  trans8_h(pass1_16_5, dst+5*stride, rnd2);
  trans8_h(pass1_16_6, dst+6*stride, rnd2);
  trans8_h(pass1_16_7, dst+7*stride, rnd2);
#endif
}




template <bool fullyCompliant>
void idct_8x8_add_8_sse4_dc(uint8_t* dst, const int16_t* coeffs, int stride)
{
  int g = (coeffs[0]+1)>>1; // 15 bit (14 bit + sign)
  int r = (g+32)>>(12-6);   // 9 bit (8 bit + sign)
  // TODO: this can theoretically overflow when (g+32) exceeds 0x3FFF

  if (fullyCompliant) {
    r = Clip3(-255,255,r); // TODO: check against conformance streams whether this is correct and needed
  }

  if (r>0) {
    __m128i dc8  = _mm_set1_epi8(r); // 8 bit (no sign)

    for (int y=0;y<8;y++) {
      __m128i input8_a = _mm_loadl_epi64((const __m128i*)(dst + y*stride));
      __m128i output8_a = _mm_adds_epu8(input8_a, dc8);
      _mm_storel_epi64((__m128i*)(dst+y*stride),    output8_a);
    }
  }
  else if (r<0) {
    __m128i dc8  = _mm_set1_epi8(-r); // 8 bit (no sign)

    for (int y=0;y<8;y++) {
      __m128i input8_a = _mm_loadl_epi64((const __m128i*)(dst + y*stride));
      __m128i output8_a = _mm_subs_epu8(input8_a, dc8);
      _mm_storel_epi64((__m128i*)(dst+y*stride),    output8_a);
    }
  }
  else {
    // r==0
  }
}


void dct()
{
  const int w = 8;

  int16_t coeff[w*w];

  uint8_t dstc[w*w], dstsse[w*w];

  memset(dstc  ,0,w*w);
  memset(dstsse,0,w*w);
  memset(coeff ,0,w*w*2);

  //if (!D)
    {
      srand(time(0));
    }

  for (int i=0;i<w*w;i++) {
    dstc[i] = i;
    dstsse[i] = i;
  }
  coeff[0] = 250;
  //coeff[1] = 155;
  //coeff[8] = 100;
  //coeff[9] =  63;
  //coeff[6*8] =  63;

#if 0
    for (int y=0;y<=7;y++) {
      for (int x=0;x<=3;x++) {
        //coeff[y*8+x] = x+ y*16;
        coeff[y*8+x] = rand() % 10000 - 5000;
      }
    }
#endif

  for (int i=0;i<w;i++) {
    printf("%02d: ",i);
    for (int x=0;x<w;x++) {
      printf("%04x ",coeff[i*w+x]);
    }
    printf("\n");
  }

  printf("\n");


  const uint64_t nIter = (D ? 1 : 500000000); //1000000000);
  for (int i=0;i<nIter;i++) {

    //coeff[i%64]++;

#if D
    for (int x=0;x<7;x++) {
      for (int y=0;y<7;y++) {
        //coeff[y*8+x] = x+ y*16; //rand() % 10000 - 5000;
        coeff[y*8+x] = rand() % 10000 - 5000;
      }
    }
#endif

    if (w==4) {
      //transform_4x4_add_8_fallback(dstc,coeff,w,3,3);
      //ff_hevc_transform_4x4_add_8_sse4(dstsse,coeff,w,3,3);
      //idct_4x4_add_8_sse4(dstsse,coeff,w,3,3);
    }

    if (w==8) {
      if (D) transform_8x8_add_8_fallback(dstc,coeff,w,7,7);
      //ff_hevc_transform_8x8_add_8_sse4(dstsse,coeff,w,7,7);
      //idct_8x8_add_8_sse4_local_ORIG<3,1>(dstsse,coeff,w);  // my original SSE DCT implementation
      //idct_8x8_add_8_sse4_local_left<3>(dstsse,coeff,w);    // for coefficients only in left side
      //idct_8x8_add_8_sse4_dc<false>(dstsse,coeff,w);        // DC only
      //idct_8x8_add_8_sse4_local_4x2(dstsse,coeff,w);
      //idct_8x8_add_8_sse4_local_4x4(dstsse,coeff,w);
      idct_8x8_add_8_sse4_local_fullwidth<6>(dstsse,coeff,w);
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

  if (D)
    for (int i=0;i<w*w;i++) {
      assert(dstsse[i] == dstc[i]);
    }
}
