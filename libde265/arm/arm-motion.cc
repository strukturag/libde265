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
#include <assert.h>

#include <arm_neon.h>

#include "arm-motion.h"
#include "util.h"


static const uint16_t rnd_8bit_values[8] = { 32,32,32,32, 32,32,32,32 };

void put_pred_8_neon(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                     const int16_t __restrict__ *src, ptrdiff_t srcstride,
                     int width, int height)
{
  //printf("PUT-PRED %d %d\n",width,height);

#if 1
  while (width >= 16) {
    //uint16x8_t rnd = vmovq_n_u16(32);

    for (int y=0;y<height;y++) {
      uint16x8_t input1 = vld1q_u16((uint16_t*)(src+y*srcstride));
      uint16x8_t input2 = vld1q_u16((uint16_t*)(src+y*srcstride + 8));
      /*
      uint16x8_t sum1   = vaddq_u16(input1, rnd);
      uint16x8_t sum2   = vaddq_u16(input2, rnd);
      uint16x8_t shiftedsum1 = vshrq_n_u16(sum1, 6);
      uint16x8_t shiftedsum2 = vshrq_n_u16(sum2, 6);
      uint8x8_t  output1 = vqmovn_u16(shiftedsum1);
      uint8x8_t  output2 = vqmovn_u16(shiftedsum2);
      */
      uint8x8_t  output1 = vqrshrn_n_u16(input1, 6);
      uint8x8_t  output2 = vqrshrn_n_u16(input2, 6);

      uint8x16_t combined_output = vcombine_u8(output1, output2);

      vst1q_u8((uint8_t*)(dst+dststride*y), combined_output);
    }

    width -= 16;
    src += 16;
    dst += 16;
  }
#endif

#if 1
  if (width >= 8) {
    //uint16x8_t rnd = vmovq_n_u16(32);

    for (int y=0;y<height;y++) {
      uint16x8_t input = vld1q_u16((uint16_t*)(src+y*srcstride));
      //uint16x8_t sum   = vaddq_u16(input, rnd);
      //uint16x8_t shiftedsum = vshrq_n_u16(sum, 6);
      //uint8x8_t  output = vqmovn_u16(shiftedsum);

      uint8x8_t  output = vqrshrn_n_u16(input, 6);

      vst1_u8((uint8_t*)(dst+dststride*y), output);
    }

    width -= 8;
    src += 8;
    dst += 8;
  }
#endif

#if 1
  if (width >= 4) {
    //uint16x4_t rnd = vmov_n_u16(32);

    for (int y=0;y<height;y++) {
      uint16x4_t input = vld1_u16((uint16_t*)(src+y*srcstride));
      uint16x8_t extended_input = vcombine_u16(input, input); // extend to 128 bit
      uint8x8_t  output = vqrshrn_n_u16(extended_input, 6);

      /*
      uint16x4_t sum   = vadd_u16(input, rnd);
      uint16x4_t shiftedsum = vshr_n_u16(sum, 6);
      uint16x8_t extended_sum = vcombine_u16(shiftedsum, shiftedsum); // extend to 128 bit
      uint8x8_t  output = vqmovn_u16(extended_sum);
      */

      uint32x2_t output32 = vreinterpret_u32_u8(output);

      vst1_lane_u32((uint32_t*)(dst+dststride*y), output32, 0);
    }

    width -= 4;
    src += 4;
    dst += 4;
  }
#endif

  if (width > 0) {
    int offset8bit = 32;
    int shift8bit = 6;

    assert((width&1)==0);

    for (int y=0;y<height;y++) {
      const int16_t* in  = &src[y*srcstride];
      uint8_t* out = &dst[y*dststride];

      for (int x=0;x<width;x+=2) {
        out[0] = Clip1_8bit((in[0] + offset8bit)>>shift8bit);
        out[1] = Clip1_8bit((in[1] + offset8bit)>>shift8bit);
        out+=2; in+=2;
      }
    }
  }
}


void put_bipred_8_neon(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                       const int16_t __restrict__ *src1,
                       const int16_t __restrict__ *src2, ptrdiff_t srcstride,
                       int width, int height)
{
  //printf("PUT-BI-PRED %d %d\n",width,height);

#if 1
  while (width >= 16) {
    //uint16x8_t rnd = vmovq_n_u16(64);

    for (int y=0;y<height;y++) {
      uint16x8_t input1a = vld1q_u16((uint16_t*)(src1+y*srcstride));
      uint16x8_t input2a = vld1q_u16((uint16_t*)(src2+y*srcstride));
      uint16x8_t input1b = vld1q_u16((uint16_t*)(src1+y*srcstride+8));
      uint16x8_t input2b = vld1q_u16((uint16_t*)(src2+y*srcstride+8));

      /*
      uint16x8_t suma    = vaddq_u16(input1a, rnd);
                 suma    = vaddq_u16(suma, input2a);
      uint16x8_t sumb    = vaddq_u16(input1b, rnd);
                 sumb    = vaddq_u16(sumb, input2b);
      uint16x8_t shiftedsuma = vshrq_n_u16(suma, 7);
      uint16x8_t shiftedsumb = vshrq_n_u16(sumb, 7);
      uint8x8_t  outputa = vqmovn_u16(shiftedsuma);
      uint8x8_t  outputb = vqmovn_u16(shiftedsumb);
      */

      uint16x8_t suma    = vaddq_u16(input1a, input2a);
      uint16x8_t sumb    = vaddq_u16(input1b, input2b);
      uint8x8_t  outputa = vqrshrn_n_u16(suma, 7);
      uint8x8_t  outputb = vqrshrn_n_u16(sumb, 7);

      uint8x16_t combined_output = vcombine_u8(outputa, outputb);

      vst1q_u8((uint8_t*)(dst+dststride*y), combined_output);
    }

    width -= 16;
    src1 += 16;
    src2 += 16;
    dst += 16;
  }

  if (width >= 8) {
    //uint16x8_t rnd = vmovq_n_u16(64);

    for (int y=0;y<height;y++) {
      uint16x8_t input1 = vld1q_u16((uint16_t*)(src1+y*srcstride));
      uint16x8_t input2 = vld1q_u16((uint16_t*)(src2+y*srcstride));
      /*
      uint16x8_t sum    = vaddq_u16(input1, rnd);
                 sum    = vaddq_u16(sum, input2);
      uint16x8_t shiftedsum = vshrq_n_u16(sum, 7);
      uint8x8_t  output = vqmovn_u16(shiftedsum);
      */

      uint16x8_t sum    = vaddq_u16(input1, input2);
      uint8x8_t  output = vqrshrn_n_u16(sum, 7);

      vst1_u8((uint8_t*)(dst+dststride*y), output);
    }

    width -= 8;
    src1 += 8;
    src2 += 8;
    dst += 8;
  }
#endif

#if 1
  if (width >= 4) {
    //uint16x4_t rnd = vmov_n_u16(64);

    for (int y=0;y<height;y++) {
      uint16x4_t input1 = vld1_u16((uint16_t*)(src1+y*srcstride));
      uint16x4_t input2 = vld1_u16((uint16_t*)(src2+y*srcstride));
      /*
      uint16x4_t sum   = vadd_u16(input1, rnd);
                 sum   = vadd_u16(sum, input2);
      uint16x4_t shiftedsum = vshr_n_u16(sum, 7);
      uint16x8_t extended_sum = vcombine_u16(shiftedsum, shiftedsum); // extend to 128 bit
      uint8x8_t  output = vqmovn_u16(extended_sum);
      */

      uint16x4_t sum    = vadd_u16(input1, input2);
      uint16x8_t extended_sum = vcombine_u16(sum,sum); // extend to 128 bit
      uint8x8_t  output = vqrshrn_n_u16(extended_sum, 7);

      uint32x2_t output32 = vreinterpret_u32_u8(output);

      vst1_lane_u32((uint32_t*)(dst+dststride*y), output32, 0);
    }

    width -= 4;
    src1 += 4;
    src2 += 4;
    dst += 4;
  }
#endif

  if (width>0) {
    int offset8bit = 64;
    int shift8bit = 7;

    for (int y=0;y<height;y++) {
      const int16_t* in1 = &src1[y*srcstride];
      const int16_t* in2 = &src2[y*srcstride];
      uint8_t* out = &dst[y*dststride];

      for (int x=0;x<width;x+=2) {
        out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
        out[1] = Clip1_8bit((in1[1] + in2[1] + offset8bit)>>shift8bit);
        out+=2; in1+=2; in2+=2;
      }
    }
  }
}


template <bool chroma>
void mc_get_noshift_8_neon(int16_t *dst, ptrdiff_t dststride,
                           const uint8_t *src, ptrdiff_t srcstride,
                           int width, int height)
{
  while (width >= 16) {
    for (int y=0;y<height;y++) {
      uint8x16_t  input  = vld1q_u8(src+y*srcstride);

      uint8x8_t  inputL = vget_low_u8(input);
      uint8x8_t  inputH = vget_high_u8(input);
      uint16x8_t inputL16 = vshll_n_u8(inputL, 6);
      uint16x8_t inputH16 = vshll_n_u8(inputH, 6);

      vst1q_u16((uint16_t*)(dst+dststride*y  ), inputL16);
      vst1q_u16((uint16_t*)(dst+dststride*y+8), inputH16);
    }

    width -= 16;
    src += 16;
    dst += 16;
  }


  if (width >= 8) {
    for (int y=0;y<height;y++) {
      uint8x8_t  input   = vld1_u8(src+y*srcstride);
      uint16x8_t input16 = vshll_n_u8(input, 6);

      vst1q_u16((uint16_t*)(dst+dststride*y), input16);
    }

    width -= 8;
    src += 8;
    dst += 8;
  }


  if (width >= 4) {
    for (int y=0;y<height;y++) {
      uint8x8_t  input   = vld1_u8(src+y*srcstride);
      uint16x8_t input16 = vshll_n_u8(input, 6);

      uint16x4_t output16 = vget_low_u16(input16);
      vst1_u16((uint16_t*)(dst+dststride*y), output16);
    }

    width -= 4;
    src += 4;
    dst += 4;
  }


  if (chroma && width>0) {
    const int shift = 6;

    for (int y=0;y<height;y++) {
      int16_t* o = &dst[y*dststride];
      const uint8_t* i = &src[y*srcstride];

      for (int x=0;x<width;x+=2) {
        o[x  ] = i[x  ] << shift;
        o[x+1] = i[x+1] << shift;
      }
    }
  }
}


void mc_get_noshift_8_luma_neon(int16_t *dst, ptrdiff_t dststride,
                                const uint8_t *src, ptrdiff_t srcstride,
                                int width, int height,
                                int16_t* mcbuffer)
{
  mc_get_noshift_8_neon<false>(dst,dststride, src,srcstride, width,height);
}


void mc_get_noshift_8_chroma_neon(int16_t *dst, ptrdiff_t dststride,
                                  const uint8_t *src, ptrdiff_t srcstride,
                                  int width, int height, int mx,
                                  int my, int16_t* mcbuffer)
{
  mc_get_noshift_8_neon<true>(dst,dststride, src,srcstride, width,height);
}
