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
    uint16x8_t rnd = vmovq_n_u16(32);

    for (int y=0;y<height;y++) {
      uint16x8_t input1 = vld1q_u16((uint16_t*)(src+y*srcstride));
      uint16x8_t input2 = vld1q_u16((uint16_t*)(src+y*srcstride + 8));
      uint16x8_t sum1   = vaddq_u16(input1, rnd);
      uint16x8_t sum2   = vaddq_u16(input2, rnd);
      uint16x8_t shiftedsum1 = vshrq_n_u16(sum1, 6);
      uint16x8_t shiftedsum2 = vshrq_n_u16(sum2, 6);
      uint8x8_t  output1 = vqmovn_u16(shiftedsum1);
      uint8x8_t  output2 = vqmovn_u16(shiftedsum2);

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
    uint16x8_t rnd = vmovq_n_u16(32);

    for (int y=0;y<height;y++) {
      uint16x8_t input = vld1q_u16((uint16_t*)(src+y*srcstride));
      uint16x8_t sum   = vaddq_u16(input, rnd);
      uint16x8_t shiftedsum = vshrq_n_u16(sum, 6);
      uint8x8_t  output = vqmovn_u16(shiftedsum);

      vst1_u8((uint8_t*)(dst+dststride*y), output);
    }

    width -= 8;
    src += 8;
    dst += 8;
  }
#endif

#if 1
  if (width >= 4) {
    uint16x4_t rnd = vmov_n_u16(32);

    for (int y=0;y<height;y++) {
      uint16x4_t input = vld1_u16((uint16_t*)(src+y*srcstride));
      uint16x4_t sum   = vadd_u16(input, rnd);
      uint16x4_t shiftedsum = vshr_n_u16(sum, 6);
      uint16x8_t extended_sum = vcombine_u16(shiftedsum, shiftedsum); // extend to 128 bit
      uint8x8_t  output = vqmovn_u16(extended_sum);

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
    uint16x8_t rnd = vmovq_n_u16(64);

    for (int y=0;y<height;y++) {
      uint16x8_t input1a = vld1q_u16((uint16_t*)(src1+y*srcstride));
      uint16x8_t input2a = vld1q_u16((uint16_t*)(src2+y*srcstride));
      uint16x8_t input1b = vld1q_u16((uint16_t*)(src1+y*srcstride+8));
      uint16x8_t input2b = vld1q_u16((uint16_t*)(src2+y*srcstride+8));
      uint16x8_t suma    = vaddq_u16(input1a, rnd);
                 suma    = vaddq_u16(suma, input2a);
      uint16x8_t sumb    = vaddq_u16(input1b, rnd);
                 sumb    = vaddq_u16(sumb, input2b);
      uint16x8_t shiftedsuma = vshrq_n_u16(suma, 7);
      uint16x8_t shiftedsumb = vshrq_n_u16(sumb, 7);
      uint8x8_t  outputa = vqmovn_u16(shiftedsuma);
      uint8x8_t  outputb = vqmovn_u16(shiftedsumb);

      uint8x16_t combined_output = vcombine_u8(outputa, outputb);

      vst1q_u8((uint8_t*)(dst+dststride*y), combined_output);
    }

    width -= 16;
    src1 += 16;
    src2 += 16;
    dst += 16;
  }

  if (width >= 8) {
    uint16x8_t rnd = vmovq_n_u16(64);

    for (int y=0;y<height;y++) {
      uint16x8_t input1 = vld1q_u16((uint16_t*)(src1+y*srcstride));
      uint16x8_t input2 = vld1q_u16((uint16_t*)(src2+y*srcstride));
      uint16x8_t sum    = vaddq_u16(input1, rnd);
                 sum    = vaddq_u16(sum, input2);
      uint16x8_t shiftedsum = vshrq_n_u16(sum, 7);
      uint8x8_t  output = vqmovn_u16(shiftedsum);

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
    uint16x4_t rnd = vmov_n_u16(64);

    for (int y=0;y<height;y++) {
      uint16x4_t input1 = vld1_u16((uint16_t*)(src1+y*srcstride));
      uint16x4_t input2 = vld1_u16((uint16_t*)(src2+y*srcstride));
      uint16x4_t sum   = vadd_u16(input1, rnd);
                 sum   = vadd_u16(sum, input2);
      uint16x4_t shiftedsum = vshr_n_u16(sum, 7);
      uint16x8_t extended_sum = vcombine_u16(shiftedsum, shiftedsum); // extend to 128 bit
      uint8x8_t  output = vqmovn_u16(extended_sum);

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
