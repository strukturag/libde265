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

inline void print(int32x4_t& v)
{
  int32_t m[4];
  vst1q_s32(m, v);
  print128((uint8_t*)m,4);
}

inline void print(uint32x2_t& v)
{
  uint32_t m[2];
  vst1_u32(m, v);
  print64((uint8_t*)m,4);
}
#endif



/*
  64  83  64  36
  64  36 -64 -83
  64 -36 -64  83
  64 -83  64 -36
*/
#if 0
ALIGNED_16(static const int16_t) transform4x4_v[][8] =
{
  // for vertical transforms

  { 64,64, 64,64, 64,64, 64,64 },
  { 83,36, 83,36, 83,36, 83,36 },
  { 64,-64,64,-64,64,-64,64,-64 },
  { 36,-83,36,-83,36,-83,36,-83 }
};


ALIGNED_16(static const int16_t) transform4x4_h[][8] =
{
  // for horizontal transforms

  { 64, 83, 64,36, 64, 36,-64,-83 },
  { 64,-36,-64,83, 64,-83, 64,-36 }
};
#endif


inline void transpose_4x4(int32x4_t& out1, int32x4_t& out2, int32x4_t& out3, int32x4_t& out4,
                          int32x4_t  in1,  int32x4_t  in2,  int32x4_t  in3,  int32x4_t  in4)
{
  int32x4x2_t tmp12 = vtrnq_s32(in1,in2);
  int32x4x2_t tmp34 = vtrnq_s32(in3,in4);

  out1 = vreinterpretq_s32_s64(vcombine_s64( vget_low_s64(vreinterpretq_s64_s32(tmp12.val[0])),
                                             vget_low_s64(vreinterpretq_s64_s32(tmp34.val[0])) ));
  out2 = vreinterpretq_s32_s64(vcombine_s64( vget_low_s64(vreinterpretq_s64_s32(tmp12.val[1])),
                                             vget_low_s64(vreinterpretq_s64_s32(tmp34.val[1])) ));
  out3 = vreinterpretq_s32_s64(vcombine_s64( vget_high_s64(vreinterpretq_s64_s32(tmp12.val[0])),
                                             vget_high_s64(vreinterpretq_s64_s32(tmp34.val[0])) ));
  out4 = vreinterpretq_s32_s64(vcombine_s64( vget_high_s64(vreinterpretq_s64_s32(tmp12.val[1])),
                                             vget_high_s64(vreinterpretq_s64_s32(tmp34.val[1])) ));
}


inline void transpose_4x4(int16x4_t& out1, int16x4_t& out2, int16x4_t& out3, int16x4_t& out4,
                          int16x4_t  in1,  int16x4_t  in2,  int16x4_t  in3,  int16x4_t  in4)
{
  int16x4x2_t tmp12 = vtrn_s16(in1,in2);
  int16x4x2_t tmp34 = vtrn_s16(in3,in4);

  Deb(tmp12.val[0]);
  Deb(tmp12.val[1]);
  Deb(tmp34.val[0]);
  Deb(tmp34.val[1]);

  int32x2_t tmp1_32 = vreinterpret_s32_s16(tmp12.val[0]);
  int32x2_t tmp2_32 = vreinterpret_s32_s16(tmp12.val[1]);
  int32x2_t tmp3_32 = vreinterpret_s32_s16(tmp34.val[0]);
  int32x2_t tmp4_32 = vreinterpret_s32_s16(tmp34.val[1]);

  int32x2x2_t out13 = vtrn_s32(tmp1_32,tmp3_32);
  int32x2x2_t out24 = vtrn_s32(tmp2_32,tmp4_32);

  Deb(out13.val[0]);
  Deb(out24.val[0]);
  Deb(out13.val[1]);
  Deb(out24.val[1]);

  out1 = vreinterpret_s16_s32( out13.val[0] );
  out2 = vreinterpret_s16_s32( out24.val[0] );
  out3 = vreinterpret_s16_s32( out13.val[1] );
  out4 = vreinterpret_s16_s32( out24.val[1] );
}


void idct_4x4_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                         int maxColumn,int maxRow)
{
  //printf("%d %d\n",maxColumn,maxRow);

  if (maxColumn==0 && maxRow==0) {
    int g = (coeffs[0]+1)>>1; // 15 bit (14 bit + sign)
    int r = (g+32)>>(12-6);   // 9 bit (8 bit + sign)

    int16x8_t dc = vdupq_n_s16(r);

    for (int y=0;y<4;y++) {
      uint32x2_t input32;
      input32 = vld1_lane_u32((uint32_t*)(dst + y*stride), input32, 0);

      uint8x8_t  input_u8 = vreinterpret_u8_u32(input32);
      uint16x8_t input_u16 = vmovl_u8(input_u8);
      int16x8_t  input_s16 = vreinterpretq_s16_u16(input_u16);
      int16x8_t  result_s16 = vaddq_s16(input_s16, dc);
      uint8x8_t  result_u8 = vqmovun_s16(result_s16);

      vst1_lane_u32((uint32_t*)(dst + y*stride),
                    vreinterpret_u32_u8(result_u8), 0);
    }
  }
  else if (false) {
    /*
    for (int i=0;i<4*4;i++) {
      *(int16_t*)(&coeffs[i]) = (i==11)*256;
    }
    */

    int16x4_t  coeff1 = vld1_s16(coeffs);
    int16x4_t  coeff2 = vld1_s16(coeffs+4);
    int16x4_t  coeff3 = vld1_s16(coeffs+8);
    int16x4_t  coeff4 = vld1_s16(coeffs+12);

    Deb(coeff1);
    Deb(coeff2);
    Deb(coeff3);
    Deb(coeff4);

    int32x4_t  v1_A = vshll_n_s16(coeff1, 6);
    int32x4_t  v3_A = vshll_n_s16(coeff3, 6);
    int32x4_t  v2_B = vmull_n_s16(coeff2, 83);
    int32x4_t  v2_C = vmull_n_s16(coeff2, 36);
    int32x4_t  v4_B = vmull_n_s16(coeff4, 83);
    int32x4_t  v4_C = vmull_n_s16(coeff4, 36);

    int32x4_t  v_pApA = vaddq_s32(v1_A, v3_A);
    int32x4_t  v_pAmA = vsubq_s32(v1_A, v3_A);

    int32x4_t  v_pBpC = vaddq_s32(v2_B, v4_C);
    int32x4_t  v_pCmB = vsubq_s32(v2_C, v4_B);

    int32x4_t  v_pApApBpC = vaddq_s32(v_pApA, v_pBpC);
    int32x4_t  v_pAmApCmB = vaddq_s32(v_pAmA, v_pCmB);
    int32x4_t  v_pAmAmCpB = vsubq_s32(v_pAmA, v_pCmB);
    int32x4_t  v_pApAmBmC = vsubq_s32(v_pApA, v_pBpC);

    Deb(v_pApApBpC);
    Deb(v_pAmApCmB);
    Deb(v_pAmAmCpB);
    Deb(v_pApAmBmC);

    int32x4_t  v_row1 = vrshrq_n_s32( v_pApApBpC,7 );
    int32x4_t  v_row2 = vrshrq_n_s32( v_pAmApCmB,7 );
    int32x4_t  v_row3 = vrshrq_n_s32( v_pAmAmCpB,7 );
    int32x4_t  v_row4 = vrshrq_n_s32( v_pApAmBmC,7 );

    Deb(v_row1);
    Deb(v_row2);
    Deb(v_row3);
    Deb(v_row4);

    int32x4_t  h_in1,h_in2,h_in3,h_in4;
    transpose_4x4(h_in1,h_in2,h_in3,h_in4, v_row1,v_row2,v_row3,v_row4);

    Deb(h_in1);
    Deb(h_in2);
    Deb(h_in3);
    Deb(h_in4);

    int32x4_t  h1_A = vshlq_n_s32(h_in1, 6);
    int32x4_t  h3_A = vshlq_n_s32(h_in3, 6);
    int32x4_t  h2_B = vmulq_n_s32(h_in2, 83);
    int32x4_t  h2_C = vmulq_n_s32(h_in2, 36);
    int32x4_t  h4_B = vmulq_n_s32(h_in4, 83);
    int32x4_t  h4_C = vmulq_n_s32(h_in4, 36);

    Deb(h1_A);
    Deb(h3_A);
    Deb(h2_B);
    Deb(h2_C);
    Deb(h4_B);
    Deb(h4_C);

    int32x4_t  h_pApA = vaddq_s32(h1_A, h3_A);
    int32x4_t  h_pAmA = vsubq_s32(h1_A, h3_A);

    int32x4_t  h_pBpC = vaddq_s32(h2_B, h4_C);
    int32x4_t  h_pCmB = vsubq_s32(h2_C, h4_B);

    int32x4_t  h_pApApBpC = vaddq_s32(h_pApA, h_pBpC);
    int32x4_t  h_pAmApCmB = vaddq_s32(h_pAmA, h_pCmB);
    int32x4_t  h_pAmAmCpB = vsubq_s32(h_pAmA, h_pCmB);
    int32x4_t  h_pApAmBmC = vsubq_s32(h_pApA, h_pBpC);

    Deb(h_pApApBpC);
    Deb(h_pAmApCmB);
    Deb(h_pAmAmCpB);
    Deb(h_pApAmBmC);

    int16x4_t  h_row1 = vqrshrn_n_s32( h_pApApBpC,12 );
    int16x4_t  h_row2 = vqrshrn_n_s32( h_pAmApCmB,12 );
    int16x4_t  h_row3 = vqrshrn_n_s32( h_pAmAmCpB,12 );
    int16x4_t  h_row4 = vqrshrn_n_s32( h_pApAmBmC,12 );

    Deb(h_row1);
    Deb(h_row2);
    Deb(h_row3);
    Deb(h_row4);

    int16x4_t  out1,out2,out3,out4;
    transpose_4x4(out1,out2,out3,out4, h_row1,h_row2,h_row3,h_row4);

    Deb(out1);
    Deb(out2);
    Deb(out3);
    Deb(out4);


    uint32x2_t input1_32,input2_32,input3_32,input4_32;
    input1_32 = vld1_lane_u32((uint32_t*)(dst + 0*stride), input1_32, 0);
    input2_32 = vld1_lane_u32((uint32_t*)(dst + 1*stride), input2_32, 0);
    input3_32 = vld1_lane_u32((uint32_t*)(dst + 2*stride), input3_32, 0);
    input4_32 = vld1_lane_u32((uint32_t*)(dst + 3*stride), input4_32, 0);

    Deb(input1_32);
    Deb(input2_32);
    Deb(input3_32);
    Deb(input4_32);

    uint16x4_t input1_u16  = vget_low_u16(vmovl_u8( vreinterpret_u8_u32(input1_32) ));
    int16x4_t  result1_s16 = vadd_s16(vreinterpret_s16_u16(input1_u16), out1);
    uint8x8_t  result1_u8  = vqmovun_s16( vcombine_s16(result1_s16,result1_s16) );
    vst1_lane_u32((uint32_t*)(dst + 0*stride), vreinterpret_u32_u8(result1_u8), 0);

    uint16x4_t input2_u16  = vget_low_u16(vmovl_u8( vreinterpret_u8_u32(input2_32) ));
    int16x4_t  result2_s16 = vadd_s16(vreinterpret_s16_u16(input2_u16), out2);
    uint8x8_t  result2_u8  = vqmovun_s16( vcombine_s16(result2_s16,result2_s16) );
    vst1_lane_u32((uint32_t*)(dst + 1*stride), vreinterpret_u32_u8(result2_u8), 0);

    uint16x4_t input3_u16  = vget_low_u16(vmovl_u8( vreinterpret_u8_u32(input3_32) ));
    int16x4_t  result3_s16 = vadd_s16(vreinterpret_s16_u16(input3_u16), out3);
    uint8x8_t  result3_u8  = vqmovun_s16( vcombine_s16(result3_s16,result3_s16) );
    vst1_lane_u32((uint32_t*)(dst + 2*stride), vreinterpret_u32_u8(result3_u8), 0);

    uint16x4_t input4_u16  = vget_low_u16(vmovl_u8( vreinterpret_u8_u32(input4_32) ));
    int16x4_t  result4_s16 = vadd_s16(vreinterpret_s16_u16(input4_u16), out4);
    uint8x8_t  result4_u8  = vqmovun_s16( vcombine_s16(result4_s16,result4_s16) );
    vst1_lane_u32((uint32_t*)(dst + 3*stride), vreinterpret_u32_u8(result4_u8), 0);

    /*
    transform_4x4_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);

    printf("cmp[0]: %08x\n",cmp[0]);
    printf("cmp[1]: %08x\n",cmp[1]);
    printf("cmp[2]: %08x\n",cmp[2]);
    printf("cmp[3]: %08x\n",cmp[3]);

    printf("ref[0]: %08x\n",*(uint32_t*)(dst+0*stride));
    printf("ref[1]: %08x\n",*(uint32_t*)(dst+1*stride));
    printf("ref[2]: %08x\n",*(uint32_t*)(dst+2*stride));
    printf("ref[3]: %08x\n",*(uint32_t*)(dst+3*stride));

    printf("\n");
    */
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

    for (int y=0;y<8;y++) {
      uint8x8_t  input_u8 = vld1_u8(dst + y*stride);
      uint16x8_t input_u16 = vmovl_u8(input_u8);
      int16x8_t  input_s16 = vreinterpretq_s16_u16(input_u16);
      int16x8_t  result_s16 = vaddq_s16(input_s16, dc);
      uint8x8_t  result_u8 = vqmovun_s16(result_s16);

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
