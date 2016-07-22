/*
 * H.265 video codec.
 * Copyright (c) 2013-2016 struktur AG, Dirk Farin <farin@struktur.de>
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <emmintrin.h>
#include <tmmintrin.h> // SSSE3
#if HAVE_SSE4_1
#include <smmintrin.h>
#endif

#include <assert.h>

#include "sse-motion-new.h"
#include "libde265/util.h"

#include "sse-motion.h"


inline void store_2_4_6(uint8_t* dst, __m128i& result, int width)
{
  uint32_t result32 = _mm_cvtsi128_si32(result);

  if (width==4) {
    *(uint32_t*)(dst) = result32;
  }
  else if (width==2) {
    *(uint16_t*)(dst) = result32;
  }
  else { // width==6
    *(uint32_t*)(dst) = result32;

    // remaining two pixels
    __m128i result2 = _mm_srli_si128(result, 4);  // SSE2

    *(uint16_t*)(dst+4) = _mm_cvtsi128_si32(result2); // SSE2
  }
}


void put_pred_8_sse2(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                     const int16_t __restrict__ *src, ptrdiff_t srcstride,
                     int width, int height)
{
  __m128i round_offset = _mm_set1_epi16(32);

  while (width >= 16) {
    for (int y=0; y < height; y++) {
      __m128i input1 = _mm_load_si128((__m128i *) (src  +y*srcstride));
      __m128i input2 = _mm_load_si128((__m128i *) (src+8+y*srcstride));
      __m128i round1 = _mm_adds_epi16(input1, round_offset); // SSE2
      __m128i round2 = _mm_adds_epi16(input2, round_offset);
      __m128i shifted1 = _mm_srai_epi16(round1, 6); // SSE2
      __m128i shifted2 = _mm_srai_epi16(round2, 6);
      __m128i result8  = _mm_packus_epi16(shifted1, shifted2);

      _mm_storeu_si128((__m128i *) (dst + y*dststride), result8); // SSE2
    }

    width -= 16;
    dst += 16;
    src += 16;
  }

  if (width >= 8) {
    for (int y=0; y < height; y++) {
      __m128i input = _mm_load_si128((__m128i *) (src + y*srcstride));
      __m128i round = _mm_adds_epi16(input, round_offset);
      __m128i shifted = _mm_srai_epi16(round, 6);
      __m128i result8 = _mm_packus_epi16(shifted, shifted);
      _mm_storel_epi64((__m128i *) (dst + y*dststride), result8);
    }

    width -= 8;
    dst += 8;
    src += 8;
  }

  if (width > 0) {
    for (int y=0; y < height; y++) {
      __m128i input = _mm_load_si128((__m128i *) (src + y*srcstride));
      __m128i round = _mm_adds_epi16(input, round_offset);
      __m128i shifted = _mm_srai_epi16(round, 6);
      __m128i result8 = _mm_packus_epi16(shifted, shifted);

      store_2_4_6(dst+y*dststride, result8, width);
    }
  }
}


void put_bipred_8_sse2(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                       const int16_t __restrict__ *src1,
                       const int16_t __restrict__ *src2, ptrdiff_t srcstride,
                       int width, int height)
{
  __m128i round_offset = _mm_set1_epi16(64);

  while (width >= 16) {
    for (int y=0; y < height; y++) {
      __m128i input1A = _mm_load_si128((__m128i *) (src1  +y*srcstride));
      __m128i input1B = _mm_load_si128((__m128i *) (src1+8+y*srcstride));
      __m128i input2A = _mm_load_si128((__m128i *) (src2  +y*srcstride));
      __m128i input2B = _mm_load_si128((__m128i *) (src2+8+y*srcstride));
      __m128i round1A = _mm_adds_epi16(input1A, round_offset);
      __m128i round1B = _mm_adds_epi16(input1B, round_offset);
      __m128i sumA    = _mm_adds_epi16(round1A, input2A);
      __m128i sumB    = _mm_adds_epi16(round1B, input2B);
      __m128i shiftedA = _mm_srai_epi16(sumA, 7);
      __m128i shiftedB = _mm_srai_epi16(sumB, 7);
      __m128i result8  = _mm_packus_epi16(shiftedA, shiftedB);

      _mm_storeu_si128((__m128i *) (dst + y*dststride), result8);
    }

    width -= 16;
    dst  += 16;
    src1 += 16;
    src2 += 16;
  }

  if (width >= 8) {
    for (int y=0; y < height; y++) {
      __m128i inputA = _mm_load_si128((__m128i *) (src1 + y*srcstride));
      __m128i inputB = _mm_load_si128((__m128i *) (src2 + y*srcstride));
      __m128i round  = _mm_adds_epi16(inputA, round_offset);
      __m128i sum    = _mm_adds_epi16(round, inputB);
      __m128i shifted = _mm_srai_epi16(sum, 7);
      __m128i result8 = _mm_packus_epi16(shifted, shifted);
      _mm_storel_epi64((__m128i *) (dst + y*dststride), result8);
    }

    width -= 8;
    dst  += 8;
    src1 += 8;
    src2 += 8;
  }

  if (width > 0) {
    for (int y=0; y < height; y++) {
      __m128i inputA = _mm_load_si128((__m128i *) (src1 + y*srcstride));
      __m128i inputB = _mm_load_si128((__m128i *) (src2 + y*srcstride));
      __m128i round  = _mm_adds_epi16(inputA, round_offset);
      __m128i sum    = _mm_adds_epi16(round, inputB);
      __m128i shifted = _mm_srai_epi16(sum, 7);
      __m128i result8 = _mm_packus_epi16(shifted, shifted);
      store_2_4_6(dst+y*dststride, result8, width);
    }
  }
}



void put_weighted_pred_8_ssse3(uint8_t *dst, ptrdiff_t dststride,
                               const int16_t *src, ptrdiff_t srcstride,
                               int width, int height,
                               int w,int o,int log2WD)
{
  __m128i flat_w = _mm_set1_epi16( w<<(15-log2WD) );
  __m128i offset = _mm_set1_epi16( o );

  while (width>=16) {
    for (int y=0;y<height;y++) {
      __m128i in_a = _mm_load_si128((const __m128i*)(&src[y*srcstride]));
      __m128i in_b = _mm_load_si128((const __m128i*)(&src[y*srcstride+8]));
      __m128i mul_a = _mm_mulhrs_epi16(in_a, flat_w); // SSSE3
      __m128i mul_b = _mm_mulhrs_epi16(in_b, flat_w);
      __m128i result_a16 = _mm_add_epi16(mul_a,offset); // SSE2
      __m128i result_b16 = _mm_add_epi16(mul_b,offset);
      __m128i result_8  = _mm_packus_epi16(result_a16, result_b16);
      _mm_storeu_si128((__m128i*)(dst+y*dststride),   result_8);
    }

    width -= 16;
    src += 16;
    dst += 16;
  }

  if (width>=8) {
    for (int y=0;y<height;y++) {
      __m128i in = _mm_load_si128((const __m128i*)(&src[y*srcstride]));
      __m128i mul = _mm_mulhrs_epi16(in, flat_w);
      __m128i result16 = _mm_add_epi16(mul,offset);
      __m128i result8  = _mm_packus_epi16(result16, result16);
      _mm_storel_epi64 ((__m128i*)(dst+y*dststride), result8);
    }

    width -= 8;
    src += 8;
    dst += 8;
  }

  if (width>0) {
    for (int y=0;y<height;y++) {
      // for width<=4, a loadl_epi64 would be sufficient
      __m128i in = _mm_load_si128((const __m128i*)(&src[y*srcstride]));
      __m128i mul = _mm_mulhrs_epi16(in, flat_w);
      __m128i result16 = _mm_add_epi16(mul,offset);
      __m128i result8  = _mm_packus_epi16(result16, result16);

      store_2_4_6(dst+y*dststride, result8, width);
    }
  }
}


#define D 0

#if D
#define Deb(x) if (D) { print128(x); printf(" " #x "\n"); }
#else
#define Deb(x)
#endif


static void print128(__m128i m)
{
  for (int i=0;i<16;i++) {
    uint8_t v = ((uint8_t*)&m)[15-i];
    printf("%02x",v);
  }
}


void put_weighted_bipred_8_sse2(uint8_t *dst, ptrdiff_t dststride,
                                const int16_t *src1, const int16_t *src2,
                                ptrdiff_t srcstride,
                                int width, int height,
                                int w1,int o1, int w2,int o2, int log2WD)
{
  if (D) printf("w1:%d w2:%d  o1:%d o2:%d log2WD:%d\n",w1,w2,o1,o2,log2WD);

  /* Here is a worst case computation:
     11a110400cc6101d163d0fdb3016 3fef input1
     10fb0fa50f6f124f19bc0ffc2e9b 40ab input2
     11a110400cc6101d163d0fdb3016 3fef mul1
     10fb0fa50f6f124f19bc0ffc2e9b 40ab mul2
     229c1fe51c35226c2ff91fd75eb1 809a mul
     22dc20251c7522ac303920175ef1 80da result
     00450040003800450060004000bd ff01 result16
     454038456040bd00454038456040bd 00 result8
     .               ffbd406045384045 reference
     .               00bd406045384045 sse

     One of the input pixels already comes in with one bit overflow.
     This leads to a total overflow of the sum into the sign bit.

     -> we now solve this by using saturating 'add's
  */


  // input bit width (8 bits + 1 overflow + 1 sign bit)
  // The case that the weighted sum overflows can only happen when
  // one input overflows and the other is also near the maximum.
  // Without saturating 'add's, this resulted in outputting 0x00 instead of 0xff.

  if (log2WD+1 // additional precision
      + 8+1 // input bit width (8 bits + 1 sign bit) + 1 overflow, which is clipped away in 'adds'
      > 16) {  // available precision

    // need 32bit computation
    // precision computation: last shift is log2WD+1
    // final precision = 8 bit + 1 sign bit from possible negative overflow

    __m128i flat_w1 = _mm_set1_epi16( w1 );
    __m128i flat_w2 = _mm_set1_epi16( w2 );
    __m128i offset  = _mm_set1_epi32( (o1+o2+1)<<log2WD );
    Deb(flat_w1);
    Deb(flat_w2);
    Deb(offset);

    __m128i zero = _mm_setzero_si128();

    while (width>0) {
      for (int y=0;y<height;y++) {
        __m128i input1 = _mm_loadu_si128((const __m128i*)(&src1[y*srcstride])); // 8*16 bit
        __m128i input2 = _mm_loadu_si128((const __m128i*)(&src2[y*srcstride]));
        Deb(input1);
        Deb(input2);
        __m128i mul1low  = _mm_mullo_epi16( flat_w1, input1 ); // SSE2
        __m128i mul2low  = _mm_mullo_epi16( flat_w2, input2 );
        __m128i mul1high = _mm_mulhi_epi16( flat_w1, input1 );
        __m128i mul2high = _mm_mulhi_epi16( flat_w2, input2 );
        Deb(mul1low);
        Deb(mul2low);
        Deb(mul1high);
        Deb(mul2high);

        __m128i mul1_32_lower = _mm_unpacklo_epi16(mul1low,mul1high);
        __m128i mul1_32_upper = _mm_unpackhi_epi16(mul1low,mul1high);
        __m128i mul2_32_lower = _mm_unpacklo_epi16(mul2low,mul2high);
        __m128i mul2_32_upper = _mm_unpackhi_epi16(mul2low,mul2high);
        Deb(mul1_32_lower);
        Deb(mul1_32_upper);
        Deb(mul2_32_lower);
        Deb(mul2_32_upper);

        __m128i mul32_lower = _mm_add_epi32(mul1_32_lower, mul2_32_lower);
        __m128i mul32_upper = _mm_add_epi32(mul1_32_upper, mul2_32_upper);
        Deb(mul32_lower);
        Deb(mul32_upper);

        __m128i result32_lower = _mm_add_epi32(mul32_lower,offset);
        __m128i result32_upper = _mm_add_epi32(mul32_upper,offset);
        Deb(result32_lower);
        Deb(result32_upper);

        result32_lower = _mm_srai_epi32(result32_lower,log2WD+1);
        result32_upper = _mm_srai_epi32(result32_upper,log2WD+1);
        Deb(result32_lower);
        Deb(result32_upper);

        __m128i result16 = _mm_packus_epi32(result32_lower, result32_upper);
        __m128i result8  = _mm_packus_epi16(result16, result16);

        if (width>=8) {
          _mm_storel_epi64 ((__m128i*)(dst+y*dststride), result8);
        }
        else {
          store_2_4_6(dst+y*dststride, result8, width);
        }

        Deb(result16);
        Deb(result8);
      }

      src1 += 8;
      src2 += 8;
      dst  += 8;
      width -= 8;
    }

    return;
  }


  __m128i flat_w1 = _mm_set1_epi16( w1 );
  __m128i flat_w2 = _mm_set1_epi16( w2 );
  __m128i offset  = _mm_set1_epi16( (o1+o2+1)<<log2WD );
  Deb(flat_w1);
  Deb(flat_w2);
  Deb(offset);

  while (width>=16) {
    for (int y=0;y<height;y++) {
      __m128i input1A = _mm_loadu_si128((const __m128i*)(&src1[y*srcstride]));
      __m128i input2A = _mm_loadu_si128((const __m128i*)(&src2[y*srcstride]));
      __m128i mul1A = _mm_mullo_epi16( flat_w1, input1A );
      __m128i mul2A = _mm_mullo_epi16( flat_w2, input2A );

      __m128i input1B = _mm_loadu_si128((const __m128i*)(&src1[y*srcstride+8]));
      __m128i input2B = _mm_loadu_si128((const __m128i*)(&src2[y*srcstride+8]));
      __m128i mul1B = _mm_mullo_epi16( flat_w1, input1B );
      __m128i mul2B = _mm_mullo_epi16( flat_w2, input2B );

      __m128i mulA  = _mm_adds_epi16(mul1A,mul2A);
      __m128i mulB  = _mm_adds_epi16(mul1B,mul2B);

      __m128i resultA   = _mm_adds_epi16(mulA,offset);
      __m128i resultB   = _mm_adds_epi16(mulB,offset);
      __m128i resultA16 = _mm_srai_epi16(resultA, log2WD+1);
      __m128i resultB16 = _mm_srai_epi16(resultB, log2WD+1);
      __m128i result8  = _mm_packus_epi16(resultA16, resultB16);
      _mm_storeu_si128((__m128i*)(dst+y*dststride), result8);
    }

    src1 += 16;
    src2 += 16;
    dst  += 16;
    width -= 16;
  }

  if (width>=8) {
    for (int y=0;y<height;y++) {
      __m128i input1 = _mm_loadu_si128((const __m128i*)(&src1[y*srcstride]));
      __m128i input2 = _mm_loadu_si128((const __m128i*)(&src2[y*srcstride]));
      Deb(input1);
      Deb(input2);
      __m128i mul1 = _mm_mullo_epi16( flat_w1, input1 );
      __m128i mul2 = _mm_mullo_epi16( flat_w2, input2 );
      __m128i mul  = _mm_adds_epi16(mul1,mul2);
      Deb(mul1);
      Deb(mul2);
      Deb(mul);
      __m128i result   = _mm_adds_epi16(mul,offset);
      __m128i result16 = _mm_srai_epi16(result, log2WD+1);
      __m128i result8  = _mm_packus_epi16(result16, result16);
      _mm_storel_epi64 ((__m128i*)(dst+y*dststride), result8);

      Deb(result);
      Deb(result16);
      Deb(result8);
    }

    src1 += 8;
    src2 += 8;
    dst  += 8;
    width -= 8;
  }

  if (width>0) {
    for (int y=0;y<height;y++) {
      // for width<=4, a loadl_epi64 would be sufficient
      __m128i input1 = _mm_loadu_si128((const __m128i*)(&src1[y*srcstride]));
      __m128i input2 = _mm_loadu_si128((const __m128i*)(&src2[y*srcstride]));
      __m128i mul1 = _mm_mullo_epi16( flat_w1, input1 );
      __m128i mul2 = _mm_mullo_epi16( flat_w2, input2 );
      __m128i mul  = _mm_adds_epi16(mul1,mul2);
      __m128i result_tmp  = _mm_adds_epi16(mul,offset);
      __m128i result16 = _mm_srai_epi16(result_tmp, log2WD+1);
      __m128i result8  = _mm_packus_epi16(result16, result16);

      store_2_4_6(dst+y*dststride, result8, width);
    }
  }
}




template <bool chroma>
inline void put_hevc_direct_8_sse2(int16_t *dst, ptrdiff_t dststride,
                                   const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                                   int16_t* mcbuffer)
{
  __m128i zero = _mm_setzero_si128();

  while (width>=16) {
    for (int y=0; y<height; y++) {
      __m128i input = _mm_loadu_si128((__m128i *) &src[y*srcstride]);

      __m128i input16low  = _mm_unpacklo_epi8(input, zero);
      __m128i input16high = _mm_unpackhi_epi8(input, zero);

      __m128i shifted_low = _mm_slli_epi16(input16low,  6); // SSE2
      __m128i shifted_high= _mm_slli_epi16(input16high, 6);

      _mm_storeu_si128((__m128i *) &dst[y*dststride  ], shifted_low);
      _mm_storeu_si128((__m128i *) &dst[y*dststride+8], shifted_high);
    }

    width -= 16;
    src += 16;
    dst += 16;
  }

  if (width>=8) {
    for (int y=0; y<height; y++) {
      __m128i input = _mm_loadl_epi64((__m128i *) &src[y*srcstride]);

      __m128i input16 = _mm_unpacklo_epi8(input, zero);
      __m128i shifted = _mm_slli_epi16(input16, 6);

      _mm_storeu_si128((__m128i *) &dst[y*dststride], shifted);
    }

    width -= 8;
    src += 8;
    dst += 8;
  }


  if (width>0) {
    for (int y=0; y<height; y++) {
      __m128i input = _mm_loadl_epi64((__m128i *) &src[y*srcstride]);

      __m128i input16 = _mm_unpacklo_epi8(input, zero);
      __m128i shifted = _mm_slli_epi16(input16, 6);

      if (chroma && width==2) {
        uint32_t result = _mm_cvtsi128_si32(shifted);
        *(uint32_t*)(dst+y*dststride) = result;
      }
      else {
        _mm_storel_epi64((__m128i*)&dst[y*dststride], shifted);

        if (chroma && width==6) {
          // remaining two pixels
          __m128i result8 = _mm_srli_si128(shifted, 4*2);

          uint32_t result2 = _mm_cvtsi128_si32(result8);
          *(uint32_t*)(dst+4+y*dststride) = result2;
        }
      }
    }
  }
}


void put_hevc_luma_direct_8_sse2(int16_t *dst, ptrdiff_t dststride,
                                 const uint8_t *src, ptrdiff_t srcstride,
                                 int width, int height,
                                 int16_t* mcbuffer)
{
  put_hevc_direct_8_sse2<false>(dst,dststride, src,srcstride, width,height, mcbuffer);
}


void put_hevc_chroma_direct_8_sse2(int16_t *dst, ptrdiff_t dststride,
                                   const uint8_t *src, ptrdiff_t srcstride,
                                   int width, int height, int mx,
                                   int my, int16_t* mcbuffer)
{
  put_hevc_direct_8_sse2<true>(dst,dststride, src,srcstride, width,height, mcbuffer);
}
