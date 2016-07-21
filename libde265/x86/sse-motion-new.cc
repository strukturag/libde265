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


void put_weighted_pred_8_sse(uint8_t *dst, ptrdiff_t dststride,
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
      __m128i mul_a = _mm_mulhrs_epi16(in_a, flat_w);
      __m128i mul_b = _mm_mulhrs_epi16(in_b, flat_w);
      __m128i result_a16 = _mm_add_epi16(mul_a,offset);
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

      uint32_t result = _mm_cvtsi128_si32(result8);

      if (width==4) {
        *(uint32_t*)(dst+y*dststride) = result;
      }
      else if (width==2) {
        *(uint16_t*)(dst+y*dststride) = result;
      }
      else { // width==6
        *(uint32_t*)(dst+y*dststride) = result;

        // remaining two pixels
        result8 = _mm_srli_si128(result8, 4);

        uint16_t result2 = _mm_cvtsi128_si32(result8);
        *(uint16_t*)(dst+4+y*dststride) = result2;
      }
    }
  }
}

/*
  void put_weighted_pred_8_w32_sse4(uint8_t *dst, ptrdiff_t dststride,
  const int16_t *src, ptrdiff_t srcstride,
  int height,
  int w,int o,int log2WD)
  {
  __m128i flat_w = _mm_set1_epi16( w<<(15-log2WD) );
  __m128i offset = _mm_set1_epi16( o );

  for (int y=0;y<height;y++) {
  __m128i in_a = _mm_load_si128((const __m128i*)(&src[y*srcstride]));
  __m128i in_b = _mm_load_si128((const __m128i*)(&src[y*srcstride+8]));
  __m128i in_c = _mm_load_si128((const __m128i*)(&src[y*srcstride+16]));
  __m128i in_d = _mm_load_si128((const __m128i*)(&src[y*srcstride+24]));
  __m128i mul_a = _mm_mulhrs_epi16(in_a, flat_w);
  __m128i mul_b = _mm_mulhrs_epi16(in_b, flat_w);
  __m128i mul_c = _mm_mulhrs_epi16(in_c, flat_w);
  __m128i mul_d = _mm_mulhrs_epi16(in_d, flat_w);
  __m128i result_a16 = _mm_add_epi16(mul_a,offset);
  __m128i result_b16 = _mm_add_epi16(mul_b,offset);
  __m128i result_c16 = _mm_add_epi16(mul_c,offset);
  __m128i result_d16 = _mm_add_epi16(mul_d,offset);
  __m128i result_A8  = _mm_packus_epi16(result_a16, result_b16);
  _mm_store_si128((__m128i*)(dst+y*dststride),   result_A8);
  __m128i result_B8  = _mm_packus_epi16(result_c16, result_d16);
  _mm_store_si128((__m128i*)(dst+16+y*dststride),   result_B8);
  }
  }
*/


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


void put_weighted_bipred_8_sse(uint8_t *dst, ptrdiff_t dststride,
                               const int16_t *src1, const int16_t *src2, ptrdiff_t srcstride,
                               int width, int height,
                               int w1,int o1, int w2,int o2, int log2WD)
{
  if (D) printf("w1:%d w2:%d  o1:%d o2:%d log2WD:%d\n",w1,w2,o1,o2,log2WD);

  __m128i flat_w1 = _mm_set1_epi16( w1 );
  __m128i flat_w2 = _mm_set1_epi16( w2 );
  __m128i offset  = _mm_set1_epi32( (o1+o2+1)<<log2WD );
  Deb(flat_w1);
  Deb(flat_w2);
  Deb(offset);

  if (log2WD+1 + 8+1 > 16) {
    // need 32bit computation
    // precision computation: last shift is log2WD+1
    // final precision = 8 bit + 1 sign bit from possible negative overflow

    __m128i zero = _mm_setzero_si128();

    while (width>0) {
      for (int y=0;y<height;y++) {
        __m128i input1 = _mm_loadu_si128((const __m128i*)(&src1[y*srcstride])); // 8*16 bit
        __m128i input2 = _mm_loadu_si128((const __m128i*)(&src2[y*srcstride]));
        Deb(input1);
        Deb(input2);
        __m128i mul1low  = _mm_mullo_epi16( flat_w1, input1 );
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

        __m128i result32_upper = _mm_add_epi16(mul32_upper,offset);
        __m128i result32_lower = _mm_add_epi16(mul32_lower,offset);
        Deb(result32_upper);
        Deb(result32_upper);

        result32_upper = _mm_srai_epi32(result32_upper,log2WD+1);
        result32_lower = _mm_srai_epi32(result32_lower,log2WD+1);
        Deb(result32_upper);
        Deb(result32_upper);

        __m128i result16 = _mm_packus_epi32(result32_lower, result32_upper);
        __m128i result8  = _mm_packus_epi16(result16, result16);

        if (width>=8) {
          _mm_storel_epi64 ((__m128i*)(dst+y*dststride), result8);
        }
        else {
          uint32_t result = _mm_cvtsi128_si32(result8);

          if (width==4) {
            *(uint32_t*)(dst+y*dststride) = result;
          }
          else if (width==2) {
            *(uint16_t*)(dst+y*dststride) = result;
          }
          else { // width==6
            *(uint32_t*)(dst+  y*dststride) = result;

            // remaining two pixels
            result8 = _mm_srli_si128(result8, 4);

            uint16_t result2 = _mm_cvtsi128_si32(result8);
            *(uint16_t*)(dst+4+y*dststride) = result2;
          }
        }

        Deb(result16);
        Deb(result8);
        if (D) printf("\n");
      }

      src1 += 8;
      src2 += 8;
      dst  += 8;
      width -= 8;
    }

    /*
    const int rnd = ((o1+o2+1) << log2WD);

    for (int y=0;y<height;y++) {
      const int16_t* in1 = &src1[y*srcstride];
      const int16_t* in2 = &src2[y*srcstride];
      uint8_t* out = &dst[y*dststride];

      for (int x=0;x<width;x++) {
        out[0] = Clip1_8bit((in1[0]*w1 + in2[0]*w2 + rnd)>>(log2WD+1));
        out++; in1++; in2++;
      }
    }
    */

    return;
  }

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

      __m128i mulA  = _mm_add_epi16(mul1A,mul2A);
      __m128i mulB  = _mm_add_epi16(mul1B,mul2B);

      __m128i resultA   = _mm_add_epi16(mulA,offset);
      __m128i resultB   = _mm_add_epi16(mulB,offset);
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
      __m128i mul1 = _mm_mullo_epi16( flat_w1, input1 );
      __m128i mul2 = _mm_mullo_epi16( flat_w2, input2 );
      __m128i mul  = _mm_add_epi16(mul1,mul2);
      __m128i result   = _mm_add_epi16(mul,offset);
      __m128i result16 = _mm_srai_epi16(result, log2WD+1);
      __m128i result8  = _mm_packus_epi16(result16, result16);
      _mm_storel_epi64 ((__m128i*)(dst+y*dststride), result8);
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
      __m128i mul  = _mm_add_epi16(mul1,mul2);
      __m128i result_tmp  = _mm_add_epi16(mul,offset);
      __m128i result16 = _mm_srai_epi16(result_tmp, log2WD+1);
      __m128i result8  = _mm_packus_epi16(result16, result16);

      uint32_t result = _mm_cvtsi128_si32(result8);

      if (width==4) {
        *(uint32_t*)(dst+y*dststride) = result;
      }
      else if (width==2) {
        *(uint16_t*)(dst+y*dststride) = result;
      }
      else { // width==6
        *(uint32_t*)(dst+  y*dststride) = result;

        // remaining two pixels
        result8 = _mm_srli_si128(result8, 4);

        uint16_t result2 = _mm_cvtsi128_si32(result8);
        *(uint16_t*)(dst+4+y*dststride) = result2;
      }
    }
  }
}



template <bool chroma>
inline void put_hevc_direct_8_sse(int16_t *dst, ptrdiff_t dststride,
                                  const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                                  int16_t* mcbuffer)
{
  __m128i zero = _mm_setzero_si128();

  while (width>=16) {
    for (int y=0; y<height; y++) {
      __m128i input = _mm_loadu_si128((__m128i *) &src[y*srcstride]);

      __m128i input16low  = _mm_unpacklo_epi8(input, zero);
      __m128i input16high = _mm_unpackhi_epi8(input, zero);

      __m128i shifted_low = _mm_slli_epi16(input16low,  6);
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


void put_hevc_luma_direct_8_sse(int16_t *dst, ptrdiff_t dststride,
                                const uint8_t *src, ptrdiff_t srcstride,
                                int width, int height,
                                int16_t* mcbuffer)
{
  put_hevc_direct_8_sse<false>(dst,dststride, src,srcstride, width,height, mcbuffer);
}


void put_hevc_chroma_direct_8_sse(int16_t *dst, ptrdiff_t dststride,
                                  const uint8_t *src, ptrdiff_t srcstride,
                                  int width, int height, int mx,
                                  int my, int16_t* mcbuffer)
{
  put_hevc_direct_8_sse<true>(dst,dststride, src,srcstride, width,height, mcbuffer);
}
