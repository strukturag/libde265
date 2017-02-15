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

#include "arm-dct.h"
#include "util.h"

#include "fallback-dct.h"


#define D 0

#if D
#define Deb(x)  if (D) { print(x); printf(" " #x "\n"); }
#else
#define Deb(x)
#define print(x)
#endif


static void print128(const uint8_t* m,int group=16)
{
  for (int i=0;i<16;i++) {
    printf("%02x",m[i]);
    if ((i%group)==group-1) printf(" ");
  }
}

static void print64(const uint8_t* m, int group=8)
{
  for (int i=0;i<8;i++) {
    printf("%02x",m[i]);
    if ((i%group)==group-1) printf(" ");
  }
}

#if D
inline void print(uint8x8_t& v)
{
  uint8_t m[8];
  vst1_u8(m, v);
  print64(m,1);
}

inline void print(uint16x8_t& v)
{
  uint16_t m[8];
  vst1q_u16(m, v);
  print128((uint8_t*)m,2);
}

inline void print(int16x8_t& v)
{
  int16_t m[8];
  vst1q_s16(m, v);
  print128((uint8_t*)m,2);
}

inline void print(int16x4_t& v)
{
  int16_t m[4];
  vst1_s16(m, v);
  print64((uint8_t*)m,2);
}
#endif



void idct_4x4_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                         int maxColumn,int maxRow)
{
  //printf("%d %d\n",maxColumn,maxRow);

  if (maxColumn==0 && maxRow==0) {
    int g = (coeffs[0]+1)>>1; // 15 bit (14 bit + sign)
    int r = (g+32)>>(12-6);   // 9 bit (8 bit + sign)

    int16x8_t dc = vdupq_n_s16(r);

    Deb(dc);

    for (int y=0;y<4;y++) {
      uint32x2_t input32;
      input32 = vld1_lane_u32((uint32_t*)(dst + y*stride), input32, 0);

      uint8x8_t input_u8 = vreinterpret_u8_u32(input32);
      Deb(input_u8);
      uint16x8_t input_u16 = vmovl_u8(input_u8);
      int16x8_t  input_s16 = vreinterpretq_s16_u16(input_u16);
      //Deb(input_s16);
      int16x8_t  result_s16 = vaddq_s16(input_s16, dc);
      //Deb(result_s16);
      uint8x8_t  result_u8 = vqmovun_s16(result_s16);
      Deb(result_u8);

      vst1_lane_u32((uint32_t*)(dst + y*stride),
                    vreinterpret_u32_u8(result_u8), 0);
    }
  }
  else {
    transform_4x4_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}



void idct_8x8_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                         int maxColumn,int maxRow)
{
  //printf("%d %d\n",maxColumn,maxRow);

  if (maxColumn==0 && maxRow==0) {
    int g = (coeffs[0]+1)>>1; // 15 bit (14 bit + sign)
    int r = (g+32)>>(12-6);   // 9 bit (8 bit + sign)

#if 1
    int16x8_t dc = vdupq_n_s16(r);

    Deb(dc);

    for (int y=0;y<8;y++) {
      uint8x8_t  input_u8 = vld1_u8(dst + y*stride);
      Deb(input_u8);
      uint16x8_t input_u16 = vmovl_u8(input_u8);
      int16x8_t  input_s16 = vreinterpretq_s16_u16(input_u16);
      //Deb(input_s16);
      int16x8_t  result_s16 = vaddq_s16(input_s16, dc);
      //Deb(result_s16);
      uint8x8_t  result_u8 = vqmovun_s16(result_s16);
      Deb(result_u8);

      vst1_u8(dst + y*stride, result_u8);
    }
#else
    if (r>0) {
      uint8x8_t dc = vdup_n_u8(r);

      for (int y=0;y<8;y++) {
        uint8x8_t  input_u8  = vld1_u8(dst + y*stride);
        uint8x8_t  result_u8 = vqadd_u8(input_u8, dc);
        vst1_u8(dst + y*stride, result_u8);
      }
    }
    else {
      uint8x8_t dc = vdup_n_u8(-r);

      for (int y=0;y<8;y++) {
        uint8x8_t  input_u8  = vld1_u8(dst + y*stride);
        uint8x8_t  result_u8 = vqsub_u8(input_u8, dc);
        vst1_u8(dst + y*stride, result_u8);
      }
    }
#endif
  }
  else {
    transform_8x8_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}



void idct_16x16_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                           int maxColumn,int maxRow)
{
  //printf("%d %d\n",maxColumn,maxRow);

  if (maxColumn==0 && maxRow==0) {
    int g = (coeffs[0]+1)>>1; // 15 bit (14 bit + sign)
    int r = (g+32)>>(12-6);   // 9 bit (8 bit + sign)

    if (r>0) {
      uint8x16_t dc = vdupq_n_u8(r);

      for (int y=0;y<16;y++) {
        uint8x16_t  input_u8  = vld1q_u8(dst + y*stride);
        uint8x16_t  result_u8 = vqaddq_u8(input_u8, dc);
        vst1q_u8(dst + y*stride, result_u8);
      }
    }
    else {
      uint8x16_t dc = vdupq_n_u8(-r);

      for (int y=0;y<16;y++) {
        uint8x16_t  input_u8  = vld1q_u8(dst + y*stride);
        uint8x16_t  result_u8 = vqsubq_u8(input_u8, dc);
        vst1q_u8(dst + y*stride, result_u8);
      }
    }
  }
  else {
    transform_16x16_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}


void idct_32x32_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                           int maxColumn,int maxRow)
{
  //printf("%d %d\n",maxColumn,maxRow);

  if (maxColumn==0 && maxRow==0) {
    int g = (coeffs[0]+1)>>1; // 15 bit (14 bit + sign)
    int r = (g+32)>>(12-6);   // 9 bit (8 bit + sign)

    if (r>0) {
      uint8x16_t dc = vdupq_n_u8(r);

      for (int y=0;y<32;y++) {
        uint8x16_t  input_u8a  = vld1q_u8(dst + y*stride);
        uint8x16_t  input_u8b  = vld1q_u8(dst + y*stride + 16);
        uint8x16_t  result_u8a = vqaddq_u8(input_u8a, dc);
        uint8x16_t  result_u8b = vqaddq_u8(input_u8b, dc);
        vst1q_u8(dst + y*stride,      result_u8a);
        vst1q_u8(dst + y*stride + 16, result_u8b);
      }
    }
    else {
      uint8x16_t dc = vdupq_n_u8(-r);

      for (int y=0;y<32;y++) {
        uint8x16_t  input_u8a  = vld1q_u8(dst + y*stride);
        uint8x16_t  input_u8b  = vld1q_u8(dst + y*stride + 16);
        uint8x16_t  result_u8a = vqsubq_u8(input_u8a, dc);
        uint8x16_t  result_u8b = vqsubq_u8(input_u8b, dc);
        vst1q_u8(dst + y*stride,      result_u8a);
        vst1q_u8(dst + y*stride + 16, result_u8b);
      }
    }
  }
  else {
    transform_32x32_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}
