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


int tcnt4[4][4];
int tcnt8[8][8];
int tcnt16[16][16];
int tcnt32[32][32];


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

  out1 = vreinterpret_s16_s32( out13.val[0] );
  out2 = vreinterpret_s16_s32( out24.val[0] );
  out3 = vreinterpret_s16_s32( out13.val[1] );
  out4 = vreinterpret_s16_s32( out24.val[1] );
}


void idct_4x4_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                         int maxColumn,int maxRow)
{
  tcnt4[maxColumn][maxRow]++;

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


static inline void idct_v8_x4_16bit_neon(const int16_t *coeffs, int maxRow,
                                         int32x4_t& out_row1,
                                         int32x4_t& out_row2,
                                         int32x4_t& out_row3,
                                         int32x4_t& out_row4,
                                         int32x4_t& out_row5,
                                         int32x4_t& out_row6,
                                         int32x4_t& out_row7,
                                         int32x4_t& out_row8)
{
  int16x4_t  coeff1 = vld1_s16(coeffs);
  int16x4_t  coeff2 = vld1_s16(coeffs+8);
  int16x4_t  coeff3 = vld1_s16(coeffs+16);
  int16x4_t  coeff4 = vld1_s16(coeffs+24);

  int32x4_t  v1_A = vshll_n_s16(coeff1, 6); // * 64
  int32x4_t  v2_B = vmull_n_s16(coeff2, 89);
  int32x4_t  v2_C = vmull_n_s16(coeff2, 75);
  int32x4_t  v2_D = vmull_n_s16(coeff2, 50);
  int32x4_t  v2_E = vmull_n_s16(coeff2, 18);
  int32x4_t  v3_F = vmull_n_s16(coeff3, 83);
  int32x4_t  v3_G = vmull_n_s16(coeff3, 36);
  int32x4_t  v4_H = vmull_n_s16(coeff4, 75);
  int32x4_t  v4_I = vmull_n_s16(coeff4, 18);
  int32x4_t  v4_J = vmull_n_s16(coeff4, 89);
  int32x4_t  v4_K = vmull_n_s16(coeff4, 50);

  int32x4_t  v_pApF = vaddq_s32(v1_A, v3_F);
  int32x4_t  v_pAmF = vsubq_s32(v1_A, v3_F);
  int32x4_t  v_pApG = vaddq_s32(v1_A, v3_G);
  int32x4_t  v_pAmG = vsubq_s32(v1_A, v3_G);

  int32x4_t  v_pBpH = vaddq_s32(v2_B, v4_H);
  int32x4_t  v_pCmI = vsubq_s32(v2_C, v4_I);
  int32x4_t  v_pDmJ = vsubq_s32(v2_D, v4_J);
  int32x4_t  v_pEmK = vsubq_s32(v2_E, v4_K);

  out_row1 = vaddq_s32(v_pApF, v_pBpH);
  out_row2 = vaddq_s32(v_pApG, v_pCmI);
  out_row3 = vaddq_s32(v_pAmG, v_pDmJ);
  out_row4 = vaddq_s32(v_pAmF, v_pEmK);

  out_row5 = vsubq_s32(v_pAmF, v_pEmK);
  out_row6 = vsubq_s32(v_pAmG, v_pDmJ);
  out_row7 = vsubq_s32(v_pApG, v_pCmI);
  out_row8 = vsubq_s32(v_pApF, v_pBpH);

  if (maxRow >= 4) {
    int16x4_t  coeff1b = vld1_s16(coeffs    + 4*8);
    int16x4_t  coeff2b = vld1_s16(coeffs+8  + 4*8);
    int16x4_t  coeff3b = vld1_s16(coeffs+16 + 4*8);
    int16x4_t  coeff4b = vld1_s16(coeffs+24 + 4*8);

    int32x4_t  v5_p64 = vshll_n_s16(coeff1b, 6); // * 64
    int32x4_t  v7_p36 = vmull_n_s16(coeff3b, 36);
    int32x4_t  v7_p83 = vmull_n_s16(coeff3b, 83);
    int32x4_t  v_p64p36 = vaddq_s32(v5_p64, v7_p36);
    int32x4_t  v_p64m36 = vsubq_s32(v5_p64, v7_p36);
    int32x4_t  v_p64p83 = vaddq_s32(v5_p64, v7_p83);
    int32x4_t  v_p64m83 = vsubq_s32(v5_p64, v7_p83);

    out_row1 = vaddq_s32(out_row1,v_p64p36);
    out_row2 = vsubq_s32(out_row2,v_p64p83);
    out_row3 = vsubq_s32(out_row3,v_p64m83);
    out_row4 = vaddq_s32(out_row4,v_p64m36);
    out_row5 = vaddq_s32(out_row5,v_p64m36);
    out_row6 = vsubq_s32(out_row6,v_p64m83);
    out_row7 = vsubq_s32(out_row7,v_p64p83);
    out_row8 = vaddq_s32(out_row8,v_p64p36);

    int32x4_t  v_p50p18 = vaddq_s32( vmull_n_s16(coeff2b, 50), vmull_n_s16(coeff4b, 18) );
    int32x4_t  v_m89m50 = vaddq_s32( vmull_n_s16(coeff2b,-89), vmull_n_s16(coeff4b,-50) );
    int32x4_t  v_p18p75 = vaddq_s32( vmull_n_s16(coeff2b, 18), vmull_n_s16(coeff4b, 75) );
    int32x4_t  v_p75m89 = vaddq_s32( vmull_n_s16(coeff2b, 75), vmull_n_s16(coeff4b,-89) );

    out_row1 = vaddq_s32(out_row1,v_p50p18);
    out_row2 = vaddq_s32(out_row2,v_m89m50);
    out_row3 = vaddq_s32(out_row3,v_p18p75);
    out_row4 = vaddq_s32(out_row4,v_p75m89);
    out_row5 = vsubq_s32(out_row5,v_p75m89);
    out_row6 = vsubq_s32(out_row6,v_p18p75);
    out_row7 = vsubq_s32(out_row7,v_m89m50);
    out_row8 = vsubq_s32(out_row8,v_p50p18);
  }

  out_row1 = vrshrq_n_s32( out_row1, 7 );
  out_row2 = vrshrq_n_s32( out_row2, 7 );
  out_row3 = vrshrq_n_s32( out_row3, 7 );
  out_row4 = vrshrq_n_s32( out_row4, 7 );
  out_row5 = vrshrq_n_s32( out_row5, 7 );
  out_row6 = vrshrq_n_s32( out_row6, 7 );
  out_row7 = vrshrq_n_s32( out_row7, 7 );
  out_row8 = vrshrq_n_s32( out_row8, 7 );
}


static inline void idct_h8_x4_32bit_neon(int32x4_t in1,
                                         int32x4_t in2,
                                         int32x4_t in3,
                                         int32x4_t in4,
                                         int32x4_t& out_row1,
                                         int32x4_t& out_row2,
                                         int32x4_t& out_row3,
                                         int32x4_t& out_row4,
                                         int32x4_t& out_row5,
                                         int32x4_t& out_row6,
                                         int32x4_t& out_row7,
                                         int32x4_t& out_row8)
{
  int32x4_t  h1L_A = vshlq_n_s32(in1, 6); // * 64
  int32x4_t  h2L_B = vmulq_n_s32(in2, 89);
  int32x4_t  h2L_C = vmulq_n_s32(in2, 75);
  int32x4_t  h2L_D = vmulq_n_s32(in2, 50);
  int32x4_t  h2L_E = vmulq_n_s32(in2, 18);
  int32x4_t  h3L_F = vmulq_n_s32(in3, 83);
  int32x4_t  h3L_G = vmulq_n_s32(in3, 36);
  int32x4_t  h4L_H = vmulq_n_s32(in4, 75);
  int32x4_t  h4L_I = vmulq_n_s32(in4, 18);
  int32x4_t  h4L_J = vmulq_n_s32(in4, 89);
  int32x4_t  h4L_K = vmulq_n_s32(in4, 50);

  int32x4_t  hL_pApF = vaddq_s32(h1L_A, h3L_F);
  int32x4_t  hL_pAmF = vsubq_s32(h1L_A, h3L_F);
  int32x4_t  hL_pApG = vaddq_s32(h1L_A, h3L_G);
  int32x4_t  hL_pAmG = vsubq_s32(h1L_A, h3L_G);

  int32x4_t  hL_pBpH = vaddq_s32(h2L_B, h4L_H);
  int32x4_t  hL_pCmI = vsubq_s32(h2L_C, h4L_I);
  int32x4_t  hL_pDmJ = vsubq_s32(h2L_D, h4L_J);
  int32x4_t  hL_pEmK = vsubq_s32(h2L_E, h4L_K);

  out_row1 = vaddq_s32(hL_pApF, hL_pBpH);
  out_row2 = vaddq_s32(hL_pApG, hL_pCmI);
  out_row3 = vaddq_s32(hL_pAmG, hL_pDmJ);
  out_row4 = vaddq_s32(hL_pAmF, hL_pEmK);

  out_row5 = vsubq_s32(hL_pAmF, hL_pEmK);
  out_row6 = vsubq_s32(hL_pAmG, hL_pDmJ);
  out_row7 = vsubq_s32(hL_pApG, hL_pCmI);
  out_row8 = vsubq_s32(hL_pApF, hL_pBpH);
}


static inline void idct_h8_x4_32bit_neon(int32x4_t in1,
                                         int32x4_t in2,
                                         int32x4_t in3,
                                         int32x4_t in4,
                                         int32x4_t in5,
                                         int32x4_t in6,
                                         int32x4_t in7,
                                         int32x4_t in8,
                                         int32x4_t& out_row1,
                                         int32x4_t& out_row2,
                                         int32x4_t& out_row3,
                                         int32x4_t& out_row4,
                                         int32x4_t& out_row5,
                                         int32x4_t& out_row6,
                                         int32x4_t& out_row7,
                                         int32x4_t& out_row8)
{
  idct_h8_x4_32bit_neon(in1, in2, in3, in4,
                        out_row1, out_row2, out_row3, out_row4,
                        out_row5, out_row6, out_row7, out_row8);

  int32x4_t  h5_p64 = vshlq_n_s32(in5, 6); // * 64
  int32x4_t  h7_p36 = vmulq_n_s32(in7, 36);
  int32x4_t  h7_p83 = vmulq_n_s32(in7, 83);

  int32x4_t  h_p64p36 = vaddq_s32(h5_p64, h7_p36);
  int32x4_t  h_p64m36 = vsubq_s32(h5_p64, h7_p36);
  int32x4_t  h_p64p83 = vaddq_s32(h5_p64, h7_p83);
  int32x4_t  h_p64m83 = vsubq_s32(h5_p64, h7_p83);

  out_row1 = vaddq_s32(out_row1,h_p64p36);
  out_row2 = vsubq_s32(out_row2,h_p64p83);
  out_row3 = vsubq_s32(out_row3,h_p64m83);
  out_row4 = vaddq_s32(out_row4,h_p64m36);
  out_row5 = vaddq_s32(out_row5,h_p64m36);
  out_row6 = vsubq_s32(out_row6,h_p64m83);
  out_row7 = vsubq_s32(out_row7,h_p64p83);
  out_row8 = vaddq_s32(out_row8,h_p64p36);

  int32x4_t  h_p50p18 = vaddq_s32( vmulq_n_s32(in6, 50), vmulq_n_s32(in8, 18) );
  int32x4_t  h_m89m50 = vaddq_s32( vmulq_n_s32(in6,-89), vmulq_n_s32(in8,-50) );
  int32x4_t  h_p18p75 = vaddq_s32( vmulq_n_s32(in6, 18), vmulq_n_s32(in8, 75) );
  int32x4_t  h_p75m89 = vaddq_s32( vmulq_n_s32(in6, 75), vmulq_n_s32(in8,-89) );

  out_row1 = vaddq_s32(out_row1,h_p50p18);
  out_row2 = vaddq_s32(out_row2,h_m89m50);
  out_row3 = vaddq_s32(out_row3,h_p18p75);
  out_row4 = vaddq_s32(out_row4,h_p75m89);
  out_row5 = vsubq_s32(out_row5,h_p75m89);
  out_row6 = vsubq_s32(out_row6,h_p18p75);
  out_row7 = vsubq_s32(out_row7,h_m89m50);
  out_row8 = vsubq_s32(out_row8,h_p50p18);
}


static inline void add_16x8_to_8x8_4rows_neon(int16x8_t out1,int16x8_t out2,
                                              int16x8_t out3,int16x8_t out4,
                                              uint8_t* dst, ptrdiff_t stride)
{

  uint8x8_t input1,input2,input3,input4;
  input1 = vld1_u8(dst + 0*stride);
  input2 = vld1_u8(dst + 1*stride);
  input3 = vld1_u8(dst + 2*stride);
  input4 = vld1_u8(dst + 3*stride);

  int16x8_t  result1_s16 = vaddq_s16(vreinterpretq_s16_u16( vmovl_u8(input1)), out1);
  vst1_u8(dst + 0*stride, vqmovun_s16(result1_s16));
  int16x8_t  result2_s16 = vaddq_s16(vreinterpretq_s16_u16( vmovl_u8(input2)), out2);
  vst1_u8(dst + 1*stride, vqmovun_s16(result2_s16));
  int16x8_t  result3_s16 = vaddq_s16(vreinterpretq_s16_u16( vmovl_u8(input3)), out3);
  vst1_u8(dst + 2*stride, vqmovun_s16(result3_s16));
  int16x8_t  result4_s16 = vaddq_s16(vreinterpretq_s16_u16( vmovl_u8(input4)), out4);
  vst1_u8(dst + 3*stride, vqmovun_s16(result4_s16));
}


static inline void idct_h8_x4_add_neon(int32x4_t in1,
                                       int32x4_t in2,
                                       int32x4_t in3,
                                       int32x4_t in4,
                                       uint8_t* dst, ptrdiff_t stride)
{
  int32x4_t  h_row1,h_row2,h_row3,h_row4;
  int32x4_t  h_row5,h_row6,h_row7,h_row8;

  idct_h8_x4_32bit_neon(in1, in2, in3, in4,
                        h_row1, h_row2, h_row3, h_row4,
                        h_row5, h_row6, h_row7, h_row8);


  int16x4_t row1 = vqrshrn_n_s32( h_row1,12 );
  int16x4_t row2 = vqrshrn_n_s32( h_row2,12 );
  int16x4_t row3 = vqrshrn_n_s32( h_row3,12 );
  int16x4_t row4 = vqrshrn_n_s32( h_row4,12 );
  int16x4_t row5 = vqrshrn_n_s32( h_row5,12 );
  int16x4_t row6 = vqrshrn_n_s32( h_row6,12 );
  int16x4_t row7 = vqrshrn_n_s32( h_row7,12 );
  int16x4_t row8 = vqrshrn_n_s32( h_row8,12 );


  int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8;
  transpose_4x4(t1,t2,t3,t4, row1, row2, row3, row4);
  transpose_4x4(t5,t6,t7,t8, row5, row6, row7, row8);

  int16x8_t  out1 = vcombine_s16(t1,t5);
  int16x8_t  out2 = vcombine_s16(t2,t6);
  int16x8_t  out3 = vcombine_s16(t3,t7);
  int16x8_t  out4 = vcombine_s16(t4,t8);

  add_16x8_to_8x8_4rows_neon(out1,out2,out3,out4, dst, stride);
}


static inline void idct_h8_x4_add_neon(int32x4_t in1,
                                       int32x4_t in2,
                                       int32x4_t in3,
                                       int32x4_t in4,
                                       int32x4_t in5,
                                       int32x4_t in6,
                                       int32x4_t in7,
                                       int32x4_t in8,
                                       uint8_t* dst, ptrdiff_t stride)
{
  int32x4_t  h_row1,h_row2,h_row3,h_row4;
  int32x4_t  h_row5,h_row6,h_row7,h_row8;

  idct_h8_x4_32bit_neon(in1, in2, in3, in4, in5, in6, in7, in8,
                        h_row1, h_row2, h_row3, h_row4,
                        h_row5, h_row6, h_row7, h_row8);

  int16x4_t row1 = vqrshrn_n_s32( h_row1,12 );
  int16x4_t row2 = vqrshrn_n_s32( h_row2,12 );
  int16x4_t row3 = vqrshrn_n_s32( h_row3,12 );
  int16x4_t row4 = vqrshrn_n_s32( h_row4,12 );
  int16x4_t row5 = vqrshrn_n_s32( h_row5,12 );
  int16x4_t row6 = vqrshrn_n_s32( h_row6,12 );
  int16x4_t row7 = vqrshrn_n_s32( h_row7,12 );
  int16x4_t row8 = vqrshrn_n_s32( h_row8,12 );


  int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8;
  transpose_4x4(t1,t2,t3,t4, row1, row2, row3, row4);
  transpose_4x4(t5,t6,t7,t8, row5, row6, row7, row8);

  int16x8_t  out1 = vcombine_s16(t1,t5);
  int16x8_t  out2 = vcombine_s16(t2,t6);
  int16x8_t  out3 = vcombine_s16(t3,t7);
  int16x8_t  out4 = vcombine_s16(t4,t8);

  add_16x8_to_8x8_4rows_neon(out1,out2,out3,out4, dst, stride);
}


void idct_8x8_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                         int maxColumn,int maxRow)
{
  tcnt8[maxColumn][maxRow]++;

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
  else if (maxColumn<=3) {

    int32x4_t  v_row1,v_row2,v_row3,v_row4;
    int32x4_t  v_row5,v_row6,v_row7,v_row8;

    idct_v8_x4_16bit_neon(coeffs, maxRow,
                          v_row1, v_row2, v_row3, v_row4,
                          v_row5, v_row6, v_row7, v_row8);

    int32x4_t  h_in1L,h_in2L,h_in3L,h_in4L;
    int32x4_t  h_in1R,h_in2R,h_in3R,h_in4R;
    transpose_4x4(h_in1L,h_in2L,h_in3L,h_in4L, v_row1,v_row2,v_row3,v_row4);
    transpose_4x4(h_in1R,h_in2R,h_in3R,h_in4R, v_row5,v_row6,v_row7,v_row8);

    int16x4_t  h_row1,h_row2,h_row3,h_row4;
    int16x4_t  h_row5,h_row6,h_row7,h_row8;

    // left half

    idct_h8_x4_add_neon(h_in1L, h_in2L, h_in3L, h_in4L, dst, stride);

    // right half

    idct_h8_x4_add_neon(h_in1R, h_in2R, h_in3R, h_in4R, dst + 4*stride, stride);
  }
  else if (true) {
    // Note: full 8x8 DCT in NEON does not give significant improvements.
    // It is not clear whether we should enable this.

    // coefficients on left matrix half

    int32x4_t  v_row1L,v_row2L,v_row3L,v_row4L;
    int32x4_t  v_row5L,v_row6L,v_row7L,v_row8L;

    idct_v8_x4_16bit_neon(coeffs, maxRow,
                          v_row1L, v_row2L, v_row3L, v_row4L,
                          v_row5L, v_row6L, v_row7L, v_row8L);

    // coefficients on right matrix half

    int32x4_t  v_row1R,v_row2R,v_row3R,v_row4R;
    int32x4_t  v_row5R,v_row6R,v_row7R,v_row8R;

    idct_v8_x4_16bit_neon(coeffs+4, maxRow,
                          v_row1R, v_row2R, v_row3R, v_row4R,
                          v_row5R, v_row6R, v_row7R, v_row8R);

    int32x4_t  h_in1L,h_in2L,h_in3L,h_in4L; // top left 4x4 submatrix
    int32x4_t  h_in1R,h_in2R,h_in3R,h_in4R; // top right 4x4 submatrix
    transpose_4x4(h_in1L,h_in2L,h_in3L,h_in4L, v_row1L,v_row2L,v_row3L,v_row4L);
    transpose_4x4(h_in1R,h_in2R,h_in3R,h_in4R, v_row5L,v_row6L,v_row7L,v_row8L);

    int32x4_t  h_in5L,h_in6L,h_in7L,h_in8L; // bottom left 4x4 submatrix
    int32x4_t  h_in5R,h_in6R,h_in7R,h_in8R; // bottom right 4x4 submatrix
    transpose_4x4(h_in5L,h_in6L,h_in7L,h_in8L, v_row1R,v_row2R,v_row3R,v_row4R);
    transpose_4x4(h_in5R,h_in6R,h_in7R,h_in8R, v_row5R,v_row6R,v_row7R,v_row8R);

    // left half

    idct_h8_x4_add_neon(h_in1L, h_in2L, h_in3L, h_in4L, h_in5L, h_in6L, h_in7L, h_in8L,
                        dst, stride);

    // right half

    idct_h8_x4_add_neon(h_in1R, h_in2R, h_in3R, h_in4R, h_in5R, h_in6R, h_in7R, h_in8R,
                        dst + 4*stride, stride);
  }
  else {
    transform_8x8_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}



static inline
void idct16_4A_neon(int32x4_t coeff1, int32x4_t coeff2,
                    int32x4_t coeff3, int32x4_t coeff4,
                    int32x4_t& row01,int32x4_t& row02,int32x4_t& row03,int32x4_t& row04,
                    int32x4_t& row05,int32x4_t& row06,int32x4_t& row07,int32x4_t& row08,
                    int32x4_t& row09,int32x4_t& row10,int32x4_t& row11,int32x4_t& row12,
                    int32x4_t& row13,int32x4_t& row14,int32x4_t& row15,int32x4_t& row16)
{
  int32x4_t  v1_A  = vshlq_n_s32(coeff1, 6); // * 64
  int32x4_t  v2_90 = vmulq_n_s32(coeff2, 90);
  int32x4_t  v2_87 = vmulq_n_s32(coeff2, 87);
  int32x4_t  v2_80 = vmulq_n_s32(coeff2, 80);
  int32x4_t  v2_70 = vmulq_n_s32(coeff2, 70);
  int32x4_t  v2_57 = vmulq_n_s32(coeff2, 57);
  int32x4_t  v2_43 = vmulq_n_s32(coeff2, 43);
  int32x4_t  v2_25 = vmulq_n_s32(coeff2, 25);
  int32x4_t  v2_09 = vmulq_n_s32(coeff2,  9);
  int32x4_t  v3_89 = vmulq_n_s32(coeff3, 89);
  int32x4_t  v3_75 = vmulq_n_s32(coeff3, 75);
  int32x4_t  v3_50 = vmulq_n_s32(coeff3, 50);
  int32x4_t  v3_18 = vmulq_n_s32(coeff3, 18);
  int32x4_t  v4_87 = vmulq_n_s32(coeff4, 87);
  int32x4_t  v4_57 = vmulq_n_s32(coeff4, 57);
  int32x4_t  v4_09 = vmulq_n_s32(coeff4,  9);
  int32x4_t  v4m43 = vmulq_n_s32(coeff4,-43);
  int32x4_t  v4m80 = vmulq_n_s32(coeff4,-80);
  int32x4_t  v4m90 = vmulq_n_s32(coeff4,-90);
  int32x4_t  v4m70 = vmulq_n_s32(coeff4,-70);
  int32x4_t  v4m25 = vmulq_n_s32(coeff4,-25);

  int32x4_t  v_pAp89 = vaddq_s32(v1_A, v3_89);
  int32x4_t  v_pAp75 = vaddq_s32(v1_A, v3_75);
  int32x4_t  v_pAp50 = vaddq_s32(v1_A, v3_50);
  int32x4_t  v_pAp18 = vaddq_s32(v1_A, v3_18);
  int32x4_t  v_pAm89 = vsubq_s32(v1_A, v3_89);
  int32x4_t  v_pAm75 = vsubq_s32(v1_A, v3_75);
  int32x4_t  v_pAm50 = vsubq_s32(v1_A, v3_50);
  int32x4_t  v_pAm18 = vsubq_s32(v1_A, v3_18);

  int32x4_t  v_p90p87 = vaddq_s32(v2_90, v4_87);
  int32x4_t  v_p87p57 = vaddq_s32(v2_87, v4_57);
  int32x4_t  v_p80p09 = vaddq_s32(v2_80, v4_09);
  int32x4_t  v_p70m43 = vaddq_s32(v2_70, v4m43);
  int32x4_t  v_p57m80 = vaddq_s32(v2_57, v4m80);
  int32x4_t  v_p43m90 = vaddq_s32(v2_43, v4m90);
  int32x4_t  v_p25m70 = vaddq_s32(v2_25, v4m70);
  int32x4_t  v_p09m25 = vaddq_s32(v2_09, v4m25);

  row01 = vaddq_s32(v_pAp89, v_p90p87);
  row02 = vaddq_s32(v_pAp75, v_p87p57);
  row03 = vaddq_s32(v_pAp50, v_p80p09);
  row04 = vaddq_s32(v_pAp18, v_p70m43);
  row05 = vaddq_s32(v_pAm18, v_p57m80);
  row06 = vaddq_s32(v_pAm50, v_p43m90);
  row07 = vaddq_s32(v_pAm75, v_p25m70);
  row08 = vaddq_s32(v_pAm89, v_p09m25);

  row09 = vsubq_s32(v_pAm89, v_p09m25);
  row10 = vsubq_s32(v_pAm75, v_p25m70);
  row11 = vsubq_s32(v_pAm50, v_p43m90);
  row12 = vsubq_s32(v_pAm18, v_p57m80);
  row13 = vsubq_s32(v_pAp18, v_p70m43);
  row14 = vsubq_s32(v_pAp50, v_p80p09);
  row15 = vsubq_s32(v_pAp75, v_p87p57);
  row16 = vsubq_s32(v_pAp89, v_p90p87);
}


static inline
void idct16_4B_neon(int32x4_t coeff5, int32x4_t coeff6,
                    int32x4_t coeff7, int32x4_t coeff8,
                    int32x4_t& row01,int32x4_t& row02,int32x4_t& row03,int32x4_t& row04,
                    int32x4_t& row05,int32x4_t& row06,int32x4_t& row07,int32x4_t& row08,
                    int32x4_t& row09,int32x4_t& row10,int32x4_t& row11,int32x4_t& row12,
                    int32x4_t& row13,int32x4_t& row14,int32x4_t& row15,int32x4_t& row16)
{
  int32x4_t  v5_83 = vmulq_n_s32(coeff5, 83);
  int32x4_t  v5_36 = vmulq_n_s32(coeff5, 36);

  int32x4_t  v7_75 = vmulq_n_s32(coeff7, 75);
  int32x4_t  v7m18 = vmulq_n_s32(coeff7,-18);
  int32x4_t  v7m89 = vmulq_n_s32(coeff7,-89);
  int32x4_t  v7m50 = vmulq_n_s32(coeff7,-50);

  int32x4_t  v_p83p75  = vaddq_s32(v5_83, v7_75);
  int32x4_t  v_p36m18  = vaddq_s32(v5_36, v7m18);
  int32x4_t  v_m36m89  = vsubq_s32(v7m89, v5_36);
  int32x4_t  v_m83m50  = vsubq_s32(v7m50, v5_83);
  int32x4_t  v_m83p50m = vaddq_s32(v5_83, v7m50);
  int32x4_t  v_m36p89m = vaddq_s32(v5_36, v7m89);
  int32x4_t  v_p36p18  = vsubq_s32( v5_36, v7m18 );
  int32x4_t  v_p83m75  = vsubq_s32( v5_83, v7_75 );

  row01 = vaddq_s32(row01, v_p83p75);
  row02 = vaddq_s32(row02, v_p36m18);
  row03 = vaddq_s32(row03, v_m36m89);
  row04 = vaddq_s32(row04, v_m83m50);
  row05 = vsubq_s32(row05, v_m83p50m);
  row06 = vsubq_s32(row06, v_m36p89m);
  row07 = vaddq_s32(row07, v_p36p18);
  row08 = vaddq_s32(row08, v_p83m75);
  row09 = vaddq_s32(row09, v_p83m75);
  row10 = vaddq_s32(row10, v_p36p18);
  row11 = vsubq_s32(row11, v_m36p89m);
  row12 = vsubq_s32(row12, v_m83p50m);
  row13 = vaddq_s32(row13, v_m83m50);
  row14 = vaddq_s32(row14, v_m36m89);
  row15 = vaddq_s32(row15, v_p36m18);
  row16 = vaddq_s32(row16, v_p83p75);

  int32x4_t v68_A = vaddq_s32( vmulq_n_s32(coeff6, 80), vmulq_n_s32(coeff8, 70) );
  int32x4_t v68_B = vaddq_s32( vmulq_n_s32(coeff6,  9), vmulq_n_s32(coeff8,-43) );
  int32x4_t v68_C = vaddq_s32( vmulq_n_s32(coeff6,-70), vmulq_n_s32(coeff8,-87) );
  int32x4_t v68_D = vaddq_s32( vmulq_n_s32(coeff6,-87), vmulq_n_s32(coeff8,  9) );
  int32x4_t v68_E = vaddq_s32( vmulq_n_s32(coeff6,-25), vmulq_n_s32(coeff8, 90) );
  int32x4_t v68_F = vaddq_s32( vmulq_n_s32(coeff6, 57), vmulq_n_s32(coeff8, 25) );
  int32x4_t v68_G = vaddq_s32( vmulq_n_s32(coeff6, 90), vmulq_n_s32(coeff8,-80) );
  int32x4_t v68_H = vaddq_s32( vmulq_n_s32(coeff6, 43), vmulq_n_s32(coeff8,-57) );

  row01 = vaddq_s32(row01, v68_A);
  row02 = vaddq_s32(row02, v68_B);
  row03 = vaddq_s32(row03, v68_C);
  row04 = vaddq_s32(row04, v68_D);
  row05 = vaddq_s32(row05, v68_E);
  row06 = vaddq_s32(row06, v68_F);
  row07 = vaddq_s32(row07, v68_G);
  row08 = vaddq_s32(row08, v68_H);
  row09 = vsubq_s32(row09, v68_H);
  row10 = vsubq_s32(row10, v68_G);
  row11 = vsubq_s32(row11, v68_F);
  row12 = vsubq_s32(row12, v68_E);
  row13 = vsubq_s32(row13, v68_D);
  row14 = vsubq_s32(row14, v68_C);
  row15 = vsubq_s32(row15, v68_B);
  row16 = vsubq_s32(row16, v68_A);
}


static inline
void idct16_4C_neon(int32x4_t coeff09, int32x4_t coeff10,
                    int32x4_t coeff11, int32x4_t coeff12,
                    int32x4_t& row01,int32x4_t& row02,int32x4_t& row03,int32x4_t& row04,
                    int32x4_t& row05,int32x4_t& row06,int32x4_t& row07,int32x4_t& row08,
                    int32x4_t& row09,int32x4_t& row10,int32x4_t& row11,int32x4_t& row12,
                    int32x4_t& row13,int32x4_t& row14,int32x4_t& row15,int32x4_t& row16)
{
  int32x4_t  v09_p64 = vshlq_n_s32(coeff09, 6); // * 64

  int32x4_t  v11_p50 = vmulq_n_s32(coeff11, 50);
  int32x4_t  v11_m89 = vmulq_n_s32(coeff11,-89);
  int32x4_t  v11_p18 = vmulq_n_s32(coeff11, 18);
  int32x4_t  v11_p75 = vmulq_n_s32(coeff11, 75);

  int32x4_t  v_p64p50  = vaddq_s32(v11_p50, v09_p64);
  int32x4_t  v_m64m89  = vsubq_s32(v11_m89, v09_p64);
  int32x4_t  v_m64p18  = vsubq_s32(v11_p18, v09_p64);
  int32x4_t  v_p64p75  = vaddq_s32(v11_p75, v09_p64);
  int32x4_t  v_p64m75m = vsubq_s32(v11_p75, v09_p64);
  int32x4_t  v_m64m18m = vaddq_s32(v11_p18, v09_p64);
  int32x4_t  v_m64p89m = vaddq_s32(v11_m89, v09_p64);
  int32x4_t  v_p64m50m = vsubq_s32(v11_p50, v09_p64);

  row01 = vaddq_s32(row01, v_p64p50);
  row02 = vaddq_s32(row02, v_m64m89);
  row03 = vaddq_s32(row03, v_m64p18);
  row04 = vaddq_s32(row04, v_p64p75);
  row05 = vsubq_s32(row05, v_p64m75m);
  row06 = vsubq_s32(row06, v_m64m18m);
  row07 = vsubq_s32(row07, v_m64p89m);
  row08 = vsubq_s32(row08, v_p64m50m);
  row09 = vsubq_s32(row09, v_p64m50m);
  row10 = vsubq_s32(row10, v_m64p89m);
  row11 = vsubq_s32(row11, v_m64m18m);
  row12 = vsubq_s32(row12, v_p64m75m);
  row13 = vaddq_s32(row13, v_p64p75);
  row14 = vaddq_s32(row14, v_m64p18);
  row15 = vaddq_s32(row15, v_m64m89);
  row16 = vaddq_s32(row16, v_p64p50);

  int32x4_t v1012_1 = vaddq_s32( vmulq_n_s32(coeff10, 57), vmulq_n_s32(coeff12, 43) );
  int32x4_t v1012_2 = vaddq_s32( vmulq_n_s32(coeff10,-80), vmulq_n_s32(coeff12,-90) );
  int32x4_t v1012_3 = vaddq_s32( vmulq_n_s32(coeff10,-25), vmulq_n_s32(coeff12, 57) );
  int32x4_t v1012_4 = vaddq_s32( vmulq_n_s32(coeff10, 90), vmulq_n_s32(coeff12, 25) );
  int32x4_t v1012_5 = vaddq_s32( vmulq_n_s32(coeff10, -9), vmulq_n_s32(coeff12,-87) );
  int32x4_t v1012_6 = vaddq_s32( vmulq_n_s32(coeff10,-87), vmulq_n_s32(coeff12, 70) );
  int32x4_t v1012_7 = vaddq_s32( vmulq_n_s32(coeff10, 43), vmulq_n_s32(coeff12,  9) );
  int32x4_t v1012_8 = vaddq_s32( vmulq_n_s32(coeff10, 70), vmulq_n_s32(coeff12,-80) );

  row01 = vaddq_s32(row01, v1012_1);
  row02 = vaddq_s32(row02, v1012_2);
  row03 = vaddq_s32(row03, v1012_3);
  row04 = vaddq_s32(row04, v1012_4);
  row05 = vaddq_s32(row05, v1012_5);
  row06 = vaddq_s32(row06, v1012_6);
  row07 = vaddq_s32(row07, v1012_7);
  row08 = vaddq_s32(row08, v1012_8);
  row09 = vsubq_s32(row09, v1012_8);
  row10 = vsubq_s32(row10, v1012_7);
  row11 = vsubq_s32(row11, v1012_6);
  row12 = vsubq_s32(row12, v1012_5);
  row13 = vsubq_s32(row13, v1012_4);
  row14 = vsubq_s32(row14, v1012_3);
  row15 = vsubq_s32(row15, v1012_2);
  row16 = vsubq_s32(row16, v1012_1);
}


void transform_16x16_add_4coeff_horiz(uint8_t* dst, ptrdiff_t stride,
                                      int32x4_t row1, int32x4_t row2,
                                      int32x4_t row3, int32x4_t row4)
{
  int32x4_t coeff1,coeff2,coeff3,coeff4;
  transpose_4x4(coeff1,coeff2,coeff3,coeff4, row1,row2,row3,row4);

  int32x4_t row01,row02,row03,row04;
  int32x4_t row05,row06,row07,row08;
  int32x4_t row09,row10,row11,row12;
  int32x4_t row13,row14,row15,row16;

  idct16_4A_neon(coeff1, coeff2, coeff3, coeff4,
                 row01, row02, row03, row04,
                 row05, row06, row07, row08,
                 row09, row10, row11, row12,
                 row13, row14, row15, row16);

  int16x4_t v_row01_16 = vqrshrn_n_s32( row01, 12 );
  int16x4_t v_row02_16 = vqrshrn_n_s32( row02, 12 );
  int16x4_t v_row03_16 = vqrshrn_n_s32( row03, 12 );
  int16x4_t v_row04_16 = vqrshrn_n_s32( row04, 12 );
  int16x4_t v_row05_16 = vqrshrn_n_s32( row05, 12 );
  int16x4_t v_row06_16 = vqrshrn_n_s32( row06, 12 );
  int16x4_t v_row07_16 = vqrshrn_n_s32( row07, 12 );
  int16x4_t v_row08_16 = vqrshrn_n_s32( row08, 12 );
  int16x4_t v_row09_16 = vqrshrn_n_s32( row09, 12 );
  int16x4_t v_row10_16 = vqrshrn_n_s32( row10, 12 );
  int16x4_t v_row11_16 = vqrshrn_n_s32( row11, 12 );
  int16x4_t v_row12_16 = vqrshrn_n_s32( row12, 12 );
  int16x4_t v_row13_16 = vqrshrn_n_s32( row13, 12 );
  int16x4_t v_row14_16 = vqrshrn_n_s32( row14, 12 );
  int16x4_t v_row15_16 = vqrshrn_n_s32( row15, 12 );
  int16x4_t v_row16_16 = vqrshrn_n_s32( row16, 12 );

  int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8, t9,t10,t11,t12,t13,t14,t15,t16;
  transpose_4x4(t1, t2, t3, t4,  v_row01_16, v_row02_16, v_row03_16, v_row04_16);
  transpose_4x4(t5, t6, t7, t8,  v_row05_16, v_row06_16, v_row07_16, v_row08_16);
  transpose_4x4(t9, t10,t11,t12, v_row09_16, v_row10_16, v_row11_16, v_row12_16);
  transpose_4x4(t13,t14,t15,t16, v_row13_16, v_row14_16, v_row15_16, v_row16_16);

  int16x8_t  out1L = vcombine_s16(t1, t5);
  int16x8_t  out2L = vcombine_s16(t2, t6);
  int16x8_t  out3L = vcombine_s16(t3, t7);
  int16x8_t  out4L = vcombine_s16(t4, t8);
  int16x8_t  out1R = vcombine_s16(t9, t13);
  int16x8_t  out2R = vcombine_s16(t10,t14);
  int16x8_t  out3R = vcombine_s16(t11,t15);
  int16x8_t  out4R = vcombine_s16(t12,t16);

  add_16x8_to_8x8_4rows_neon(out1L,out2L,out3L,out4L, dst,  stride);
  add_16x8_to_8x8_4rows_neon(out1R,out2R,out3R,out4R, dst+8,stride);
}


void transform_16x16_add_8coeff_horiz(uint8_t* dst, ptrdiff_t stride,
                                      int32x4_t row1,int32x4_t row2,int32x4_t row3,int32x4_t row4,
                                      int32x4_t row5,int32x4_t row6,int32x4_t row7,int32x4_t row8)
{
  int32x4_t coeff1,coeff2,coeff3,coeff4;
  transpose_4x4(coeff1,coeff2,coeff3,coeff4, row1,row2,row3,row4);
  int32x4_t coeff5,coeff6,coeff7,coeff8;
  transpose_4x4(coeff5,coeff6,coeff7,coeff8, row5,row6,row7,row8);

  int32x4_t row01,row02,row03,row04;
  int32x4_t row05,row06,row07,row08;
  int32x4_t row09,row10,row11,row12;
  int32x4_t row13,row14,row15,row16;

  idct16_4A_neon(coeff1, coeff2, coeff3, coeff4,
                 row01, row02, row03, row04,
                 row05, row06, row07, row08,
                 row09, row10, row11, row12,
                 row13, row14, row15, row16);

  idct16_4B_neon(coeff5, coeff6, coeff7, coeff8,
                 row01, row02, row03, row04,
                 row05, row06, row07, row08,
                 row09, row10, row11, row12,
                 row13, row14, row15, row16);

  int16x4_t v_row01_16 = vqrshrn_n_s32( row01, 12 );
  int16x4_t v_row02_16 = vqrshrn_n_s32( row02, 12 );
  int16x4_t v_row03_16 = vqrshrn_n_s32( row03, 12 );
  int16x4_t v_row04_16 = vqrshrn_n_s32( row04, 12 );
  int16x4_t v_row05_16 = vqrshrn_n_s32( row05, 12 );
  int16x4_t v_row06_16 = vqrshrn_n_s32( row06, 12 );
  int16x4_t v_row07_16 = vqrshrn_n_s32( row07, 12 );
  int16x4_t v_row08_16 = vqrshrn_n_s32( row08, 12 );
  int16x4_t v_row09_16 = vqrshrn_n_s32( row09, 12 );
  int16x4_t v_row10_16 = vqrshrn_n_s32( row10, 12 );
  int16x4_t v_row11_16 = vqrshrn_n_s32( row11, 12 );
  int16x4_t v_row12_16 = vqrshrn_n_s32( row12, 12 );
  int16x4_t v_row13_16 = vqrshrn_n_s32( row13, 12 );
  int16x4_t v_row14_16 = vqrshrn_n_s32( row14, 12 );
  int16x4_t v_row15_16 = vqrshrn_n_s32( row15, 12 );
  int16x4_t v_row16_16 = vqrshrn_n_s32( row16, 12 );

  int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8, t9,t10,t11,t12,t13,t14,t15,t16;
  transpose_4x4(t1, t2, t3, t4,  v_row01_16, v_row02_16, v_row03_16, v_row04_16);
  transpose_4x4(t5, t6, t7, t8,  v_row05_16, v_row06_16, v_row07_16, v_row08_16);
  transpose_4x4(t9, t10,t11,t12, v_row09_16, v_row10_16, v_row11_16, v_row12_16);
  transpose_4x4(t13,t14,t15,t16, v_row13_16, v_row14_16, v_row15_16, v_row16_16);

  int16x8_t  out1L = vcombine_s16(t1, t5);
  int16x8_t  out2L = vcombine_s16(t2, t6);
  int16x8_t  out3L = vcombine_s16(t3, t7);
  int16x8_t  out4L = vcombine_s16(t4, t8);
  int16x8_t  out1R = vcombine_s16(t9, t13);
  int16x8_t  out2R = vcombine_s16(t10,t14);
  int16x8_t  out3R = vcombine_s16(t11,t15);
  int16x8_t  out4R = vcombine_s16(t12,t16);

  add_16x8_to_8x8_4rows_neon(out1L,out2L,out3L,out4L, dst,  stride);
  add_16x8_to_8x8_4rows_neon(out1R,out2R,out3R,out4R, dst+8,stride);
}


void transform_16x16_add_12coeff_horiz(uint8_t* dst, ptrdiff_t stride,
                                       int32x4_t in1,int32x4_t in2, int32x4_t in3, int32x4_t in4,
                                       int32x4_t in5,int32x4_t in6, int32x4_t in7, int32x4_t in8,
                                       int32x4_t in9,int32x4_t in10,int32x4_t in11,int32x4_t in12)
{
  int32x4_t coeff1,coeff2,coeff3,coeff4;
  transpose_4x4(coeff1,coeff2,coeff3,coeff4, in1,in2,in3,in4);
  int32x4_t coeff5,coeff6,coeff7,coeff8;
  transpose_4x4(coeff5,coeff6,coeff7,coeff8, in5,in6,in7,in8);
  int32x4_t coeff9,coeff10,coeff11,coeff12;
  transpose_4x4(coeff9,coeff10,coeff11,coeff12, in9,in10,in11,in12);

  int32x4_t row01,row02,row03,row04;
  int32x4_t row05,row06,row07,row08;
  int32x4_t row09,row10,row11,row12;
  int32x4_t row13,row14,row15,row16;

  idct16_4A_neon(coeff1, coeff2, coeff3, coeff4,
                 row01, row02, row03, row04,
                 row05, row06, row07, row08,
                 row09, row10, row11, row12,
                 row13, row14, row15, row16);

  idct16_4B_neon(coeff5, coeff6, coeff7, coeff8,
                 row01, row02, row03, row04,
                 row05, row06, row07, row08,
                 row09, row10, row11, row12,
                 row13, row14, row15, row16);

  idct16_4C_neon(coeff9, coeff10, coeff11, coeff12,
                 row01, row02, row03, row04,
                 row05, row06, row07, row08,
                 row09, row10, row11, row12,
                 row13, row14, row15, row16);

  int16x4_t v_row01_16 = vqrshrn_n_s32( row01, 12 );
  int16x4_t v_row02_16 = vqrshrn_n_s32( row02, 12 );
  int16x4_t v_row03_16 = vqrshrn_n_s32( row03, 12 );
  int16x4_t v_row04_16 = vqrshrn_n_s32( row04, 12 );
  int16x4_t v_row05_16 = vqrshrn_n_s32( row05, 12 );
  int16x4_t v_row06_16 = vqrshrn_n_s32( row06, 12 );
  int16x4_t v_row07_16 = vqrshrn_n_s32( row07, 12 );
  int16x4_t v_row08_16 = vqrshrn_n_s32( row08, 12 );
  int16x4_t v_row09_16 = vqrshrn_n_s32( row09, 12 );
  int16x4_t v_row10_16 = vqrshrn_n_s32( row10, 12 );
  int16x4_t v_row11_16 = vqrshrn_n_s32( row11, 12 );
  int16x4_t v_row12_16 = vqrshrn_n_s32( row12, 12 );
  int16x4_t v_row13_16 = vqrshrn_n_s32( row13, 12 );
  int16x4_t v_row14_16 = vqrshrn_n_s32( row14, 12 );
  int16x4_t v_row15_16 = vqrshrn_n_s32( row15, 12 );
  int16x4_t v_row16_16 = vqrshrn_n_s32( row16, 12 );

  int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8, t9,t10,t11,t12,t13,t14,t15,t16;
  transpose_4x4(t1, t2, t3, t4,  v_row01_16, v_row02_16, v_row03_16, v_row04_16);
  transpose_4x4(t5, t6, t7, t8,  v_row05_16, v_row06_16, v_row07_16, v_row08_16);
  transpose_4x4(t9, t10,t11,t12, v_row09_16, v_row10_16, v_row11_16, v_row12_16);
  transpose_4x4(t13,t14,t15,t16, v_row13_16, v_row14_16, v_row15_16, v_row16_16);

  int16x8_t  out1L = vcombine_s16(t1, t5);
  int16x8_t  out2L = vcombine_s16(t2, t6);
  int16x8_t  out3L = vcombine_s16(t3, t7);
  int16x8_t  out4L = vcombine_s16(t4, t8);
  int16x8_t  out1R = vcombine_s16(t9, t13);
  int16x8_t  out2R = vcombine_s16(t10,t14);
  int16x8_t  out3R = vcombine_s16(t11,t15);
  int16x8_t  out4R = vcombine_s16(t12,t16);

  add_16x8_to_8x8_4rows_neon(out1L,out2L,out3L,out4L, dst,  stride);
  add_16x8_to_8x8_4rows_neon(out1R,out2R,out3R,out4R, dst+8,stride);
}


static inline void idct16_horiz_neon(const int16_t* coeffs, int maxRow,
                                     int32x4_t& out_row01, int32x4_t& out_row02,
                                     int32x4_t& out_row03, int32x4_t& out_row04,
                                     int32x4_t& out_row05, int32x4_t& out_row06,
                                     int32x4_t& out_row07, int32x4_t& out_row08,
                                     int32x4_t& out_row09, int32x4_t& out_row10,
                                     int32x4_t& out_row11, int32x4_t& out_row12,
                                     int32x4_t& out_row13, int32x4_t& out_row14,
                                     int32x4_t& out_row15, int32x4_t& out_row16)
{
    int16x4_t  coeff1 = vld1_s16(coeffs+0*16);
    int16x4_t  coeff2 = vld1_s16(coeffs+1*16);
    int16x4_t  coeff3 = vld1_s16(coeffs+2*16);
    int16x4_t  coeff4 = vld1_s16(coeffs+3*16);

    int32x4_t  v1_A = vshll_n_s16(coeff1, 6); // * 64
    int32x4_t  v2_90 = vmull_n_s16(coeff2, 90);
    int32x4_t  v2_87 = vmull_n_s16(coeff2, 87);
    int32x4_t  v2_80 = vmull_n_s16(coeff2, 80);
    int32x4_t  v2_70 = vmull_n_s16(coeff2, 70);
    int32x4_t  v2_57 = vmull_n_s16(coeff2, 57);
    int32x4_t  v2_43 = vmull_n_s16(coeff2, 43);
    int32x4_t  v2_25 = vmull_n_s16(coeff2, 25);
    int32x4_t  v2_09 = vmull_n_s16(coeff2,  9);
    int32x4_t  v3_89 = vmull_n_s16(coeff3, 89);
    int32x4_t  v3_75 = vmull_n_s16(coeff3, 75);
    int32x4_t  v3_50 = vmull_n_s16(coeff3, 50);
    int32x4_t  v3_18 = vmull_n_s16(coeff3, 18);
    int32x4_t  v4_87 = vmull_n_s16(coeff4, 87);
    int32x4_t  v4_57 = vmull_n_s16(coeff4, 57);
    int32x4_t  v4_09 = vmull_n_s16(coeff4,  9);
    int32x4_t  v4m43 = vmull_n_s16(coeff4,-43);
    int32x4_t  v4m80 = vmull_n_s16(coeff4,-80);
    int32x4_t  v4m90 = vmull_n_s16(coeff4,-90);
    int32x4_t  v4m70 = vmull_n_s16(coeff4,-70);
    int32x4_t  v4m25 = vmull_n_s16(coeff4,-25);

    int32x4_t  v_pAp89 = vaddq_s32(v1_A, v3_89);
    int32x4_t  v_pAp75 = vaddq_s32(v1_A, v3_75);
    int32x4_t  v_pAp50 = vaddq_s32(v1_A, v3_50);
    int32x4_t  v_pAp18 = vaddq_s32(v1_A, v3_18);
    int32x4_t  v_pAm89 = vsubq_s32(v1_A, v3_89);
    int32x4_t  v_pAm75 = vsubq_s32(v1_A, v3_75);
    int32x4_t  v_pAm50 = vsubq_s32(v1_A, v3_50);
    int32x4_t  v_pAm18 = vsubq_s32(v1_A, v3_18);

    int32x4_t  v_p90p87 = vaddq_s32(v2_90, v4_87);
    int32x4_t  v_p87p57 = vaddq_s32(v2_87, v4_57);
    int32x4_t  v_p80p09 = vaddq_s32(v2_80, v4_09);
    int32x4_t  v_p70m43 = vaddq_s32(v2_70, v4m43);
    int32x4_t  v_p57m80 = vaddq_s32(v2_57, v4m80);
    int32x4_t  v_p43m90 = vaddq_s32(v2_43, v4m90);
    int32x4_t  v_p25m70 = vaddq_s32(v2_25, v4m70);
    int32x4_t  v_p09m25 = vaddq_s32(v2_09, v4m25);

    int32x4_t  v_row01 = vaddq_s32(v_pAp89, v_p90p87);
    int32x4_t  v_row02 = vaddq_s32(v_pAp75, v_p87p57);
    int32x4_t  v_row03 = vaddq_s32(v_pAp50, v_p80p09);
    int32x4_t  v_row04 = vaddq_s32(v_pAp18, v_p70m43);
    int32x4_t  v_row05 = vaddq_s32(v_pAm18, v_p57m80);
    int32x4_t  v_row06 = vaddq_s32(v_pAm50, v_p43m90);
    int32x4_t  v_row07 = vaddq_s32(v_pAm75, v_p25m70);
    int32x4_t  v_row08 = vaddq_s32(v_pAm89, v_p09m25);

    int32x4_t  v_row09 = vsubq_s32(v_pAm89, v_p09m25);
    int32x4_t  v_row10 = vsubq_s32(v_pAm75, v_p25m70);
    int32x4_t  v_row11 = vsubq_s32(v_pAm50, v_p43m90);
    int32x4_t  v_row12 = vsubq_s32(v_pAm18, v_p57m80);
    int32x4_t  v_row13 = vsubq_s32(v_pAp18, v_p70m43);
    int32x4_t  v_row14 = vsubq_s32(v_pAp50, v_p80p09);
    int32x4_t  v_row15 = vsubq_s32(v_pAp75, v_p87p57);
    int32x4_t  v_row16 = vsubq_s32(v_pAp89, v_p90p87);

    if (maxRow >= 4) {
      int16x4_t  coeff5 = vld1_s16(coeffs+4*16);
      int16x4_t  coeff6 = vld1_s16(coeffs+5*16);
      int16x4_t  coeff7 = vld1_s16(coeffs+6*16);
      int16x4_t  coeff8 = vld1_s16(coeffs+7*16);

      int32x4_t  v5_83 = vmull_n_s16(coeff5, 83);
      int32x4_t  v5_36 = vmull_n_s16(coeff5, 36);

      int32x4_t  v7_75 = vmull_n_s16(coeff7, 75);
      int32x4_t  v7m18 = vmull_n_s16(coeff7,-18);
      int32x4_t  v7m89 = vmull_n_s16(coeff7,-89);
      int32x4_t  v7m50 = vmull_n_s16(coeff7,-50);

      int32x4_t  v_p83p75  = vaddq_s32(v5_83, v7_75);
      int32x4_t  v_p36m18  = vaddq_s32(v5_36, v7m18);
      int32x4_t  v_m36m89  = vsubq_s32(v7m89, v5_36);
      int32x4_t  v_m83m50  = vsubq_s32(v7m50, v5_83);
      int32x4_t  v_m83p50m = vaddq_s32(v5_83, v7m50);
      int32x4_t  v_m36p89m = vaddq_s32(v5_36, v7m89);
      int32x4_t  v_p36p18  = vsubq_s32( v5_36, v7m18 );
      int32x4_t  v_p83m75  = vsubq_s32( v5_83, v7_75 );

      v_row01 = vaddq_s32(v_row01, v_p83p75);
      v_row02 = vaddq_s32(v_row02, v_p36m18);
      v_row03 = vaddq_s32(v_row03, v_m36m89);
      v_row04 = vaddq_s32(v_row04, v_m83m50);
      v_row05 = vsubq_s32(v_row05, v_m83p50m);
      v_row06 = vsubq_s32(v_row06, v_m36p89m);
      v_row07 = vaddq_s32(v_row07, v_p36p18);
      v_row08 = vaddq_s32(v_row08, v_p83m75);
      v_row09 = vaddq_s32(v_row09, v_p83m75);
      v_row10 = vaddq_s32(v_row10, v_p36p18);
      v_row11 = vsubq_s32(v_row11, v_m36p89m);
      v_row12 = vsubq_s32(v_row12, v_m83p50m);
      v_row13 = vaddq_s32(v_row13, v_m83m50);
      v_row14 = vaddq_s32(v_row14, v_m36m89);
      v_row15 = vaddq_s32(v_row15, v_p36m18);
      v_row16 = vaddq_s32(v_row16, v_p83p75);

      int32x4_t v68_A = vaddq_s32( vmull_n_s16(coeff6, 80), vmull_n_s16(coeff8, 70) );
      int32x4_t v68_B = vaddq_s32( vmull_n_s16(coeff6,  9), vmull_n_s16(coeff8,-43) );
      int32x4_t v68_C = vaddq_s32( vmull_n_s16(coeff6,-70), vmull_n_s16(coeff8,-87) );
      int32x4_t v68_D = vaddq_s32( vmull_n_s16(coeff6,-87), vmull_n_s16(coeff8,  9) );
      int32x4_t v68_E = vaddq_s32( vmull_n_s16(coeff6,-25), vmull_n_s16(coeff8, 90) );
      int32x4_t v68_F = vaddq_s32( vmull_n_s16(coeff6, 57), vmull_n_s16(coeff8, 25) );
      int32x4_t v68_G = vaddq_s32( vmull_n_s16(coeff6, 90), vmull_n_s16(coeff8,-80) );
      int32x4_t v68_H = vaddq_s32( vmull_n_s16(coeff6, 43), vmull_n_s16(coeff8,-57) );

      v_row01 = vaddq_s32(v_row01, v68_A);
      v_row02 = vaddq_s32(v_row02, v68_B);
      v_row03 = vaddq_s32(v_row03, v68_C);
      v_row04 = vaddq_s32(v_row04, v68_D);
      v_row05 = vaddq_s32(v_row05, v68_E);
      v_row06 = vaddq_s32(v_row06, v68_F);
      v_row07 = vaddq_s32(v_row07, v68_G);
      v_row08 = vaddq_s32(v_row08, v68_H);
      v_row09 = vsubq_s32(v_row09, v68_H);
      v_row10 = vsubq_s32(v_row10, v68_G);
      v_row11 = vsubq_s32(v_row11, v68_F);
      v_row12 = vsubq_s32(v_row12, v68_E);
      v_row13 = vsubq_s32(v_row13, v68_D);
      v_row14 = vsubq_s32(v_row14, v68_C);
      v_row15 = vsubq_s32(v_row15, v68_B);
      v_row16 = vsubq_s32(v_row16, v68_A);


      if (maxRow >= 8) {
        int16x4_t  coeff09 = vld1_s16(coeffs+ 8*16);
        int16x4_t  coeff10 = vld1_s16(coeffs+ 9*16);
        int16x4_t  coeff11 = vld1_s16(coeffs+10*16);
        int16x4_t  coeff12 = vld1_s16(coeffs+11*16);

        int32x4_t  v09_p64 = vshll_n_s16(coeff09, 6); // * 64

        int32x4_t  v11_p50 = vmull_n_s16(coeff11, 50);
        int32x4_t  v11_m89 = vmull_n_s16(coeff11,-89);
        int32x4_t  v11_p18 = vmull_n_s16(coeff11, 18);
        int32x4_t  v11_p75 = vmull_n_s16(coeff11, 75);

        int32x4_t  v_p64p50  = vaddq_s32(v11_p50, v09_p64);
        int32x4_t  v_m64m89  = vsubq_s32(v11_m89, v09_p64);
        int32x4_t  v_m64p18  = vsubq_s32(v11_p18, v09_p64);
        int32x4_t  v_p64p75  = vaddq_s32(v11_p75, v09_p64);
        int32x4_t  v_p64m75m = vsubq_s32(v11_p75, v09_p64);
        int32x4_t  v_m64m18m = vaddq_s32(v11_p18, v09_p64);
        int32x4_t  v_m64p89m = vaddq_s32(v11_m89, v09_p64);
        int32x4_t  v_p64m50m = vsubq_s32(v11_p50, v09_p64);

        v_row01 = vaddq_s32(v_row01, v_p64p50);
        v_row02 = vaddq_s32(v_row02, v_m64m89);
        v_row03 = vaddq_s32(v_row03, v_m64p18);
        v_row04 = vaddq_s32(v_row04, v_p64p75);
        v_row05 = vsubq_s32(v_row05, v_p64m75m);
        v_row06 = vsubq_s32(v_row06, v_m64m18m);
        v_row07 = vsubq_s32(v_row07, v_m64p89m);
        v_row08 = vsubq_s32(v_row08, v_p64m50m);
        v_row09 = vsubq_s32(v_row09, v_p64m50m);
        v_row10 = vsubq_s32(v_row10, v_m64p89m);
        v_row11 = vsubq_s32(v_row11, v_m64m18m);
        v_row12 = vsubq_s32(v_row12, v_p64m75m);
        v_row13 = vaddq_s32(v_row13, v_p64p75);
        v_row14 = vaddq_s32(v_row14, v_m64p18);
        v_row15 = vaddq_s32(v_row15, v_m64m89);
        v_row16 = vaddq_s32(v_row16, v_p64p50);

        int32x4_t v1012_1 = vaddq_s32( vmull_n_s16(coeff10, 57), vmull_n_s16(coeff12, 43) );
        int32x4_t v1012_2 = vaddq_s32( vmull_n_s16(coeff10,-80), vmull_n_s16(coeff12,-90) );
        int32x4_t v1012_3 = vaddq_s32( vmull_n_s16(coeff10,-25), vmull_n_s16(coeff12, 57) );
        int32x4_t v1012_4 = vaddq_s32( vmull_n_s16(coeff10, 90), vmull_n_s16(coeff12, 25) );
        int32x4_t v1012_5 = vaddq_s32( vmull_n_s16(coeff10, -9), vmull_n_s16(coeff12,-87) );
        int32x4_t v1012_6 = vaddq_s32( vmull_n_s16(coeff10,-87), vmull_n_s16(coeff12, 70) );
        int32x4_t v1012_7 = vaddq_s32( vmull_n_s16(coeff10, 43), vmull_n_s16(coeff12,  9) );
        int32x4_t v1012_8 = vaddq_s32( vmull_n_s16(coeff10, 70), vmull_n_s16(coeff12,-80) );

        v_row01 = vaddq_s32(v_row01, v1012_1);
        v_row02 = vaddq_s32(v_row02, v1012_2);
        v_row03 = vaddq_s32(v_row03, v1012_3);
        v_row04 = vaddq_s32(v_row04, v1012_4);
        v_row05 = vaddq_s32(v_row05, v1012_5);
        v_row06 = vaddq_s32(v_row06, v1012_6);
        v_row07 = vaddq_s32(v_row07, v1012_7);
        v_row08 = vaddq_s32(v_row08, v1012_8);
        v_row09 = vsubq_s32(v_row09, v1012_8);
        v_row10 = vsubq_s32(v_row10, v1012_7);
        v_row11 = vsubq_s32(v_row11, v1012_6);
        v_row12 = vsubq_s32(v_row12, v1012_5);
        v_row13 = vsubq_s32(v_row13, v1012_4);
        v_row14 = vsubq_s32(v_row14, v1012_3);
        v_row15 = vsubq_s32(v_row15, v1012_2);
        v_row16 = vsubq_s32(v_row16, v1012_1);
      }
    }


    out_row01 = vrshrq_n_s32( v_row01, 7 );
    out_row02 = vrshrq_n_s32( v_row02, 7 );
    out_row03 = vrshrq_n_s32( v_row03, 7 );
    out_row04 = vrshrq_n_s32( v_row04, 7 );
    out_row05 = vrshrq_n_s32( v_row05, 7 );
    out_row06 = vrshrq_n_s32( v_row06, 7 );
    out_row07 = vrshrq_n_s32( v_row07, 7 );
    out_row08 = vrshrq_n_s32( v_row08, 7 );
    out_row09 = vrshrq_n_s32( v_row09, 7 );
    out_row10 = vrshrq_n_s32( v_row10, 7 );
    out_row11 = vrshrq_n_s32( v_row11, 7 );
    out_row12 = vrshrq_n_s32( v_row12, 7 );
    out_row13 = vrshrq_n_s32( v_row13, 7 );
    out_row14 = vrshrq_n_s32( v_row14, 7 );
    out_row15 = vrshrq_n_s32( v_row15, 7 );
    out_row16 = vrshrq_n_s32( v_row16, 7 );
}


void idct_16x16_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                           int maxColumn,int maxRow)
{
  tcnt16[maxColumn][maxRow]++;

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
  else if (maxColumn<=3 && maxRow<12) {
    int32x4_t  v_row01, v_row02, v_row03, v_row04, v_row05, v_row06, v_row07, v_row08;
    int32x4_t  v_row09, v_row10, v_row11, v_row12, v_row13, v_row14, v_row15, v_row16;

    idct16_horiz_neon(coeffs, maxRow,
                      v_row01, v_row02, v_row03, v_row04, v_row05, v_row06, v_row07, v_row08,
                      v_row09, v_row10, v_row11, v_row12, v_row13, v_row14, v_row15, v_row16);

    transform_16x16_add_4coeff_horiz(dst,           stride, v_row01,v_row02,v_row03,v_row04);
    transform_16x16_add_4coeff_horiz(dst+ 4*stride, stride, v_row05,v_row06,v_row07,v_row08);
    transform_16x16_add_4coeff_horiz(dst+ 8*stride, stride, v_row09,v_row10,v_row11,v_row12);
    transform_16x16_add_4coeff_horiz(dst+12*stride, stride, v_row13,v_row14,v_row15,v_row16);
  }
  else if (maxColumn<=7 && maxRow<12) {
    int32x4_t  v_row01A, v_row02A, v_row03A, v_row04A, v_row05A, v_row06A, v_row07A, v_row08A;
    int32x4_t  v_row09A, v_row10A, v_row11A, v_row12A, v_row13A, v_row14A, v_row15A, v_row16A;

    idct16_horiz_neon(coeffs, maxRow,
                      v_row01A,v_row02A,v_row03A,v_row04A,v_row05A,v_row06A,v_row07A,v_row08A,
                      v_row09A,v_row10A,v_row11A,v_row12A,v_row13A,v_row14A,v_row15A,v_row16A);

    int32x4_t  v_row01B, v_row02B, v_row03B, v_row04B, v_row05B, v_row06B, v_row07B, v_row08B;
    int32x4_t  v_row09B, v_row10B, v_row11B, v_row12B, v_row13B, v_row14B, v_row15B, v_row16B;

    idct16_horiz_neon(coeffs+4, maxRow,
                      v_row01B,v_row02B,v_row03B,v_row04B,v_row05B,v_row06B,v_row07B,v_row08B,
                      v_row09B,v_row10B,v_row11B,v_row12B,v_row13B,v_row14B,v_row15B,v_row16B);

    transform_16x16_add_8coeff_horiz(dst,           stride,
                                     v_row01A,v_row02A,v_row03A,v_row04A,
                                     v_row01B,v_row02B,v_row03B,v_row04B);
    transform_16x16_add_8coeff_horiz(dst+ 4*stride, stride,
                                     v_row05A,v_row06A,v_row07A,v_row08A,
                                     v_row05B,v_row06B,v_row07B,v_row08B);
    transform_16x16_add_8coeff_horiz(dst+ 8*stride, stride,
                                     v_row09A,v_row10A,v_row11A,v_row12A,
                                     v_row09B,v_row10B,v_row11B,v_row12B);
    transform_16x16_add_8coeff_horiz(dst+12*stride, stride,
                                     v_row13A,v_row14A,v_row15A,v_row16A,
                                     v_row13B,v_row14B,v_row15B,v_row16B);
  }
  else if (maxColumn<12 && maxRow<12) {
    int32x4_t  v_row01A, v_row02A, v_row03A, v_row04A, v_row05A, v_row06A, v_row07A, v_row08A;
    int32x4_t  v_row09A, v_row10A, v_row11A, v_row12A, v_row13A, v_row14A, v_row15A, v_row16A;

    idct16_horiz_neon(coeffs, maxRow,
                      v_row01A,v_row02A,v_row03A,v_row04A,v_row05A,v_row06A,v_row07A,v_row08A,
                      v_row09A,v_row10A,v_row11A,v_row12A,v_row13A,v_row14A,v_row15A,v_row16A);

    int32x4_t  v_row01B, v_row02B, v_row03B, v_row04B, v_row05B, v_row06B, v_row07B, v_row08B;
    int32x4_t  v_row09B, v_row10B, v_row11B, v_row12B, v_row13B, v_row14B, v_row15B, v_row16B;

    idct16_horiz_neon(coeffs+4, maxRow,
                      v_row01B,v_row02B,v_row03B,v_row04B,v_row05B,v_row06B,v_row07B,v_row08B,
                      v_row09B,v_row10B,v_row11B,v_row12B,v_row13B,v_row14B,v_row15B,v_row16B);

    int32x4_t  v_row01C, v_row02C, v_row03C, v_row04C, v_row05C, v_row06C, v_row07C, v_row08C;
    int32x4_t  v_row09C, v_row10C, v_row11C, v_row12C, v_row13C, v_row14C, v_row15C, v_row16C;

    idct16_horiz_neon(coeffs+8, maxRow,
                      v_row01C,v_row02C,v_row03C,v_row04C,v_row05C,v_row06C,v_row07C,v_row08C,
                      v_row09C,v_row10C,v_row11C,v_row12C,v_row13C,v_row14C,v_row15C,v_row16C);

    transform_16x16_add_12coeff_horiz(dst,           stride,
                                      v_row01A,v_row02A,v_row03A,v_row04A,
                                      v_row01B,v_row02B,v_row03B,v_row04B,
                                      v_row01C,v_row02C,v_row03C,v_row04C);
    transform_16x16_add_12coeff_horiz(dst+ 4*stride, stride,
                                      v_row05A,v_row06A,v_row07A,v_row08A,
                                      v_row05B,v_row06B,v_row07B,v_row08B,
                                      v_row05C,v_row06C,v_row07C,v_row08C);
    transform_16x16_add_12coeff_horiz(dst+ 8*stride, stride,
                                      v_row09A,v_row10A,v_row11A,v_row12A,
                                      v_row09B,v_row10B,v_row11B,v_row12B,
                                      v_row09C,v_row10C,v_row11C,v_row12C);
    transform_16x16_add_12coeff_horiz(dst+12*stride, stride,
                                      v_row13A,v_row14A,v_row15A,v_row16A,
                                      v_row13B,v_row14B,v_row15B,v_row16B,
                                      v_row13C,v_row14C,v_row15C,v_row16C);
  }
  else {
    transform_16x16_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}


void idct32_v(const int16_t* coeffs, int32x4_t* outrow, int maxRow)
{
  int16x4_t  coeff1 = vld1_s16(coeffs);
  int16x4_t  coeff2 = vld1_s16(coeffs+32);
  int16x4_t  coeff3 = vld1_s16(coeffs+64);
  int16x4_t  coeff4 = vld1_s16(coeffs+96);

  int32x4_t  v1_A  = vshll_n_s16(coeff1, 6); // * 64

  int32x4_t  v3_90 = vmull_n_s16(coeff3, 90);
  int32x4_t  v3_87 = vmull_n_s16(coeff3, 87);
  int32x4_t  v3_80 = vmull_n_s16(coeff3, 80);
  int32x4_t  v3_70 = vmull_n_s16(coeff3, 70);
  int32x4_t  v3_57 = vmull_n_s16(coeff3, 57);
  int32x4_t  v3_43 = vmull_n_s16(coeff3, 43);
  int32x4_t  v3_25 = vmull_n_s16(coeff3, 25);
  int32x4_t  v3_09 = vmull_n_s16(coeff3,  9);

  int32x4_t  v2_90 = vmull_n_s16(coeff2, 90);
  int32x4_t  v2_88 = vmull_n_s16(coeff2, 88);
  int32x4_t  v2_85 = vmull_n_s16(coeff2, 85);
  int32x4_t  v2_82 = vmull_n_s16(coeff2, 82);
  int32x4_t  v2_78 = vmull_n_s16(coeff2, 78);
  int32x4_t  v2_73 = vmull_n_s16(coeff2, 73);
  int32x4_t  v2_67 = vmull_n_s16(coeff2, 67);
  int32x4_t  v2_61 = vmull_n_s16(coeff2, 61);
  int32x4_t  v2_54 = vmull_n_s16(coeff2, 54);
  int32x4_t  v2_46 = vmull_n_s16(coeff2, 46);
  int32x4_t  v2_38 = vmull_n_s16(coeff2, 38);
  int32x4_t  v2_31 = vmull_n_s16(coeff2, 31);
  int32x4_t  v2_22 = vmull_n_s16(coeff2, 22);
  int32x4_t  v2_13 = vmull_n_s16(coeff2, 13);
  int32x4_t  v2_04 = vmull_n_s16(coeff2,  4);

  int32x4_t  v4_90 = vmull_n_s16(coeff4, 90);
  int32x4_t  v4_82 = vmull_n_s16(coeff4, 82);
  int32x4_t  v4_67 = vmull_n_s16(coeff4, 67);
  int32x4_t  v4_46 = vmull_n_s16(coeff4, 46);
  int32x4_t  v4_22 = vmull_n_s16(coeff4, 22);
  int32x4_t  v4m04 = vmull_n_s16(coeff4,- 4);
  int32x4_t  v4m31 = vmull_n_s16(coeff4,-31);
  int32x4_t  v4m54 = vmull_n_s16(coeff4,-54);
  int32x4_t  v4m73 = vmull_n_s16(coeff4,-73);
  int32x4_t  v4m85 = vmull_n_s16(coeff4,-85);
  //int32x4_t  v4m90 = vmull_n_s16(coeff4,-90);  we can reuse v4_90
  int32x4_t  v4m88 = vmull_n_s16(coeff4,-88);
  int32x4_t  v4m78 = vmull_n_s16(coeff4,-78);
  int32x4_t  v4m61 = vmull_n_s16(coeff4,-61);
  int32x4_t  v4m38 = vmull_n_s16(coeff4,-38);
  int32x4_t  v4m13 = vmull_n_s16(coeff4,-13);

  int32x4_t  v_p90p90 = vaddq_s32(v2_90, v4_90);
  int32x4_t  v_p90p82 = vaddq_s32(v2_90, v4_82);
  int32x4_t  v_p88p67 = vaddq_s32(v2_88, v4_67);
  int32x4_t  v_p85p46 = vaddq_s32(v2_85, v4_46);
  int32x4_t  v_p82p22 = vaddq_s32(v2_82, v4_22);
  int32x4_t  v_p78m04 = vaddq_s32(v2_78, v4m04);
  int32x4_t  v_p73m31 = vaddq_s32(v2_73, v4m31);
  int32x4_t  v_p67m54 = vaddq_s32(v2_67, v4m54);
  int32x4_t  v_p61m73 = vaddq_s32(v2_61, v4m73);
  int32x4_t  v_p54m85 = vaddq_s32(v2_54, v4m85);
  int32x4_t  v_p46m90 = vsubq_s32(v2_46, v4_90);
  int32x4_t  v_p38m88 = vaddq_s32(v2_38, v4m88);
  int32x4_t  v_p31m78 = vaddq_s32(v2_31, v4m78);
  int32x4_t  v_p22m61 = vaddq_s32(v2_22, v4m61);
  int32x4_t  v_p13m38 = vaddq_s32(v2_13, v4m38);
  int32x4_t  v_p04m13 = vaddq_s32(v2_04, v4m13);

  int32x4_t  v_pAp90 = vaddq_s32(v1_A, v3_90);
  int32x4_t  v_pAp87 = vaddq_s32(v1_A, v3_87);
  int32x4_t  v_pAp80 = vaddq_s32(v1_A, v3_80);
  int32x4_t  v_pAp70 = vaddq_s32(v1_A, v3_70);
  int32x4_t  v_pAp57 = vaddq_s32(v1_A, v3_57);
  int32x4_t  v_pAp43 = vaddq_s32(v1_A, v3_43);
  int32x4_t  v_pAp25 = vaddq_s32(v1_A, v3_25);
  int32x4_t  v_pAp09 = vaddq_s32(v1_A, v3_09);
  int32x4_t  v_pAm90 = vsubq_s32(v1_A, v3_90);
  int32x4_t  v_pAm87 = vsubq_s32(v1_A, v3_87);
  int32x4_t  v_pAm80 = vsubq_s32(v1_A, v3_80);
  int32x4_t  v_pAm70 = vsubq_s32(v1_A, v3_70);
  int32x4_t  v_pAm57 = vsubq_s32(v1_A, v3_57);
  int32x4_t  v_pAm43 = vsubq_s32(v1_A, v3_43);
  int32x4_t  v_pAm25 = vsubq_s32(v1_A, v3_25);
  int32x4_t  v_pAm09 = vsubq_s32(v1_A, v3_09);

  outrow[ 0] = vaddq_s32(v_pAp90, v_p90p90);
  outrow[ 1] = vaddq_s32(v_pAp87, v_p90p82);
  outrow[ 2] = vaddq_s32(v_pAp80, v_p88p67);
  outrow[ 3] = vaddq_s32(v_pAp70, v_p85p46);
  outrow[ 4] = vaddq_s32(v_pAp57, v_p82p22);
  outrow[ 5] = vaddq_s32(v_pAp43, v_p78m04);
  outrow[ 6] = vaddq_s32(v_pAp25, v_p73m31);
  outrow[ 7] = vaddq_s32(v_pAp09, v_p67m54);

  outrow[ 8] = vaddq_s32(v_pAm09, v_p61m73);
  outrow[ 9] = vaddq_s32(v_pAm25, v_p54m85);
  outrow[10] = vaddq_s32(v_pAm43, v_p46m90);
  outrow[11] = vaddq_s32(v_pAm57, v_p38m88);
  outrow[12] = vaddq_s32(v_pAm70, v_p31m78);
  outrow[13] = vaddq_s32(v_pAm80, v_p22m61);
  outrow[14] = vaddq_s32(v_pAm87, v_p13m38);
  outrow[15] = vaddq_s32(v_pAm90, v_p04m13);

  outrow[16] = vsubq_s32(v_pAm90, v_p04m13);
  outrow[17] = vsubq_s32(v_pAm87, v_p13m38);
  outrow[18] = vsubq_s32(v_pAm80, v_p22m61);
  outrow[19] = vsubq_s32(v_pAm70, v_p31m78);
  outrow[20] = vsubq_s32(v_pAm57, v_p38m88);
  outrow[21] = vsubq_s32(v_pAm43, v_p46m90);
  outrow[22] = vsubq_s32(v_pAm25, v_p54m85);
  outrow[23] = vsubq_s32(v_pAm09, v_p61m73);

  outrow[24] = vsubq_s32(v_pAp09, v_p67m54);
  outrow[25] = vsubq_s32(v_pAp25, v_p73m31);
  outrow[26] = vsubq_s32(v_pAp43, v_p78m04);
  outrow[27] = vsubq_s32(v_pAp57, v_p82p22);
  outrow[28] = vsubq_s32(v_pAp70, v_p85p46);
  outrow[29] = vsubq_s32(v_pAp80, v_p88p67);
  outrow[30] = vsubq_s32(v_pAp87, v_p90p82);
  outrow[31] = vsubq_s32(v_pAp90, v_p90p90);

  if (maxRow >= 4) {
    int16x4_t  coeff1b = vld1_s16(coeffs   +4*32);
    int16x4_t  coeff2b = vld1_s16(coeffs+32+4*32);
    int16x4_t  coeff3b = vld1_s16(coeffs+64+4*32);
    int16x4_t  coeff4b = vld1_s16(coeffs+96+4*32);

    int32x4_t  v1b_89 = vmull_n_s16(coeff1b, 89);
    int32x4_t  v1b_75 = vmull_n_s16(coeff1b, 75);
    int32x4_t  v1b_50 = vmull_n_s16(coeff1b, 50);
    int32x4_t  v1b_18 = vmull_n_s16(coeff1b, 18);

    int32x4_t  v3b_87 = vmull_n_s16(coeff3b, 87);
    int32x4_t  v3b_57 = vmull_n_s16(coeff3b, 57);
    int32x4_t  v3b_09 = vmull_n_s16(coeff3b,  9);
    int32x4_t  v3bm43 = vmull_n_s16(coeff3b,-43);
    int32x4_t  v3bm80 = vmull_n_s16(coeff3b,-80);
    int32x4_t  v3bm90 = vmull_n_s16(coeff3b,-90);
    int32x4_t  v3bm70 = vmull_n_s16(coeff3b,-70);
    int32x4_t  v3bm25 = vmull_n_s16(coeff3b,-25);

    int32x4_t  vb_p89p87 = vaddq_s32(v3b_87, v1b_89);
    int32x4_t  vb_p75p57 = vaddq_s32(v3b_57, v1b_75);
    int32x4_t  vb_p50p09 = vaddq_s32(v3b_09, v1b_50);
    int32x4_t  vb_p18m43 = vaddq_s32(v3bm43, v1b_18);
    int32x4_t  vb_m18m80 = vsubq_s32(v3bm80, v1b_18);
    int32x4_t  vb_m50m90 = vsubq_s32(v3bm90, v1b_50);
    int32x4_t  vb_m75m70 = vsubq_s32(v3bm70, v1b_75);
    int32x4_t  vb_m89m25 = vsubq_s32(v3bm25, v1b_89);

    int32x4_t  vb_m89p25_m = vaddq_s32(v3bm25, v1b_89);
    int32x4_t  vb_m75p70_m = vaddq_s32(v3bm70, v1b_75);
    int32x4_t  vb_m50p90_m = vaddq_s32(v3bm90, v1b_50);
    int32x4_t  vb_m18p80_m = vaddq_s32(v3bm80, v1b_18);
    int32x4_t  vb_p18p43 = vsubq_s32(v1b_18, v3bm43);
    int32x4_t  vb_p50m09 = vsubq_s32(v1b_50, v3b_09);
    int32x4_t  vb_p75m57 = vsubq_s32(v1b_75, v3b_57);
    int32x4_t  vb_p89m87 = vsubq_s32(v1b_89, v3b_87);

    outrow[ 0] = vaddq_s32(outrow[ 0], vb_p89p87);
    outrow[ 1] = vaddq_s32(outrow[ 1], vb_p75p57);
    outrow[ 2] = vaddq_s32(outrow[ 2], vb_p50p09);
    outrow[ 3] = vaddq_s32(outrow[ 3], vb_p18m43);
    outrow[ 4] = vaddq_s32(outrow[ 4], vb_m18m80);
    outrow[ 5] = vaddq_s32(outrow[ 5], vb_m50m90);
    outrow[ 6] = vaddq_s32(outrow[ 6], vb_m75m70);
    outrow[ 7] = vaddq_s32(outrow[ 7], vb_m89m25);
    outrow[ 8] = vsubq_s32(outrow[ 8], vb_m89p25_m);
    outrow[ 9] = vsubq_s32(outrow[ 9], vb_m75p70_m);
    outrow[10] = vsubq_s32(outrow[10], vb_m50p90_m);
    outrow[11] = vsubq_s32(outrow[11], vb_m18p80_m);
    outrow[12] = vaddq_s32(outrow[12], vb_p18p43);
    outrow[13] = vaddq_s32(outrow[13], vb_p50m09);
    outrow[14] = vaddq_s32(outrow[14], vb_p75m57);
    outrow[15] = vaddq_s32(outrow[15], vb_p89m87);
    outrow[16] = vaddq_s32(outrow[16], vb_p89m87);
    outrow[17] = vaddq_s32(outrow[17], vb_p75m57);
    outrow[18] = vaddq_s32(outrow[18], vb_p50m09);
    outrow[19] = vaddq_s32(outrow[19], vb_p18p43);
    outrow[20] = vsubq_s32(outrow[20], vb_m18p80_m);
    outrow[21] = vsubq_s32(outrow[21], vb_m50p90_m);
    outrow[22] = vsubq_s32(outrow[22], vb_m75p70_m);
    outrow[23] = vsubq_s32(outrow[23], vb_m89p25_m);
    outrow[24] = vaddq_s32(outrow[24], vb_m89m25);
    outrow[25] = vaddq_s32(outrow[25], vb_m75m70);
    outrow[26] = vaddq_s32(outrow[26], vb_m50m90);
    outrow[27] = vaddq_s32(outrow[27], vb_m18m80);
    outrow[28] = vaddq_s32(outrow[28], vb_p18m43);
    outrow[29] = vaddq_s32(outrow[29], vb_p50p09);
    outrow[30] = vaddq_s32(outrow[30], vb_p75p57);
    outrow[31] = vaddq_s32(outrow[31], vb_p89p87);


    int32x4_t tmp;

    int32x4_t  v2b_88 = vmull_n_s16(coeff2b, 88);
    int32x4_t  v4b_85 = vmull_n_s16(coeff4b, 85);
    tmp = vaddq_s32(v2b_88, v4b_85);
    outrow[ 0] = vaddq_s32(outrow[ 0], tmp);
    outrow[31] = vsubq_s32(outrow[31], tmp);

    int32x4_t  v2b_67 = vmull_n_s16(coeff2b, 67);
    int32x4_t  v4b_46 = vmull_n_s16(coeff4b, 46);
    tmp = vaddq_s32(v2b_67, v4b_46);
    outrow[ 1] = vaddq_s32(outrow[ 1], tmp);
    outrow[30] = vsubq_s32(outrow[30], tmp);

    int32x4_t  v2b_31 = vmull_n_s16(coeff2b, 31);
    int32x4_t  v4bm13 = vmull_n_s16(coeff4b,-13);
    tmp = vaddq_s32(v2b_31, v4bm13);
    outrow[ 2] = vaddq_s32(outrow[ 2], tmp);
    outrow[29] = vsubq_s32(outrow[29], tmp);

    int32x4_t  v2bm13 = vmull_n_s16(coeff2b,-13);
    int32x4_t  v4bm67 = vmull_n_s16(coeff4b,-67);
    tmp = vaddq_s32(v2bm13, v4bm67);
    outrow[ 3] = vaddq_s32(outrow[ 3], tmp);
    outrow[28] = vsubq_s32(outrow[28], tmp);

    int32x4_t  v2bm54 = vmull_n_s16(coeff2b,-54);
    int32x4_t  v4bm90 = vmull_n_s16(coeff4b,-90);
    tmp = vaddq_s32(v2bm54, v4bm90);
    outrow[ 4] = vaddq_s32(outrow[ 4], tmp);
    outrow[27] = vsubq_s32(outrow[27], tmp);

    int32x4_t  v2bm82 = vmull_n_s16(coeff2b,-82);
    int32x4_t  v4bm73 = vmull_n_s16(coeff4b,-73);
    tmp = vaddq_s32(v2bm82, v4bm73);
    outrow[ 5] = vaddq_s32(outrow[ 5], tmp);
    outrow[26] = vsubq_s32(outrow[26], tmp);

    int32x4_t  v2b_90 = vmull_n_s16(coeff2b, 90);
    //int32x4_t  v2bm90 = vmull_n_s16(coeff2b, -90);   can reuse v2b_90
    int32x4_t  v4bm22 = vmull_n_s16(coeff4b,-22);
    tmp = vsubq_s32(v4bm22, v2b_90);
    outrow[ 6] = vaddq_s32(outrow[ 6], tmp);
    outrow[25] = vsubq_s32(outrow[25], tmp);

    int32x4_t  v2bm78 = vmull_n_s16(coeff2b,-78);
    int32x4_t  v4b_38 = vmull_n_s16(coeff4b, 38);
    tmp = vaddq_s32(v2bm78, v4b_38);
    outrow[ 7] = vaddq_s32(outrow[ 7], tmp);
    outrow[24] = vsubq_s32(outrow[24], tmp);

    int32x4_t  v2bm46 = vmull_n_s16(coeff2b,-46);
    int32x4_t  v4b_82 = vmull_n_s16(coeff4b, 82);
    tmp = vaddq_s32(v2bm46, v4b_82);
    outrow[ 8] = vaddq_s32(outrow[ 8], tmp);
    outrow[23] = vsubq_s32(outrow[23], tmp);

    int32x4_t  v2bm04 = vmull_n_s16(coeff2b, -4);
    int32x4_t  v4b_88 = vmull_n_s16(coeff4b, 88);
    tmp = vaddq_s32(v2bm04, v4b_88);
    outrow[ 9] = vaddq_s32(outrow[ 9], tmp);
    outrow[22] = vsubq_s32(outrow[22], tmp);

    int32x4_t  v2b_38 = vmull_n_s16(coeff2b, 38);
    int32x4_t  v4b_54 = vmull_n_s16(coeff4b, 54);
    tmp = vaddq_s32(v2b_38, v4b_54);
    outrow[10] = vaddq_s32(outrow[10], tmp);
    outrow[21] = vsubq_s32(outrow[21], tmp);

    int32x4_t  v2b_73 = vmull_n_s16(coeff2b, 73);
    int32x4_t  v4bm04 = vmull_n_s16(coeff4b, -4);
    tmp = vaddq_s32(v2b_73, v4bm04);
    outrow[11] = vaddq_s32(outrow[11], tmp);
    outrow[20] = vsubq_s32(outrow[20], tmp);

    int32x4_t  v4bm61 = vmull_n_s16(coeff4b,-61);
    tmp = vaddq_s32(v2b_90, v4bm61);
    outrow[12] = vaddq_s32(outrow[12], tmp);
    outrow[19] = vsubq_s32(outrow[19], tmp);

    int32x4_t  v2b_85 = vmull_n_s16(coeff2b, 85);
    //int32x4_t  v4bm90 = vmull_n_s16(coeff4b, 85);  duplicate
    tmp = vaddq_s32(v2b_85, v4bm90);
    outrow[13] = vaddq_s32(outrow[13], tmp);
    outrow[18] = vsubq_s32(outrow[18], tmp);

    int32x4_t  v2b_61 = vmull_n_s16(coeff2b, 61);
    int32x4_t  v4bm78 = vmull_n_s16(coeff4b,-78);
    tmp = vaddq_s32(v2b_61, v4bm78);
    outrow[14] = vaddq_s32(outrow[14], tmp);
    outrow[17] = vsubq_s32(outrow[17], tmp);

    int32x4_t  v2b_22 = vmull_n_s16(coeff2b, 22);
    int32x4_t  v4bm31 = vmull_n_s16(coeff4b,-31);
    tmp = vaddq_s32(v2b_22, v4bm31);
    outrow[15] = vaddq_s32(outrow[15], tmp);
    outrow[16] = vsubq_s32(outrow[16], tmp);
  }

  if (maxRow >= 8) {
    int16x4_t  coeff09 = vld1_s16(coeffs   +8*32);
    int16x4_t  coeff10 = vld1_s16(coeffs+32+8*32);
    int16x4_t  coeff11 = vld1_s16(coeffs+64+8*32);
    int16x4_t  coeff12 = vld1_s16(coeffs+96+8*32);

    int32x4_t  v09_83 = vmull_n_s16(coeff09, 83);
    int32x4_t  v09_36 = vmull_n_s16(coeff09, 36);

    int32x4_t  v11_80 = vmull_n_s16(coeff11, 80);
    int32x4_t  v11_09 = vmull_n_s16(coeff11,  9);
    int32x4_t  v11m70 = vmull_n_s16(coeff11,-70);
    int32x4_t  v11m87 = vmull_n_s16(coeff11,-87);
    int32x4_t  v11m25 = vmull_n_s16(coeff11,-25);
    int32x4_t  v11_57 = vmull_n_s16(coeff11, 57);
    int32x4_t  v11_90 = vmull_n_s16(coeff11, 90);
    int32x4_t  v11_43 = vmull_n_s16(coeff11, 43);

    int32x4_t  v_p83p80 = vaddq_s32(v11_80, v09_83);
    int32x4_t  v_p36p09 = vaddq_s32(v11_09, v09_36);
    int32x4_t  v_m36m70 = vsubq_s32(v11m70, v09_36);
    int32x4_t  v_m83m87 = vsubq_s32(v11m87, v09_83);
    int32x4_t  v_m83m25 = vsubq_s32(v11m25, v09_83);
    int32x4_t  v_m36p57 = vsubq_s32(v11_57, v09_36);
    int32x4_t  v_p36p90 = vaddq_s32(v11_90, v09_36);
    int32x4_t  v_p83p43 = vaddq_s32(v11_43, v09_83);

    int32x4_t  v_m83p80 = vsubq_s32(v11_80, v09_83);
    int32x4_t  v_m36p09 = vsubq_s32(v11_09, v09_36);
    int32x4_t  v_p36m70 = vaddq_s32(v11m70, v09_36);
    int32x4_t  v_p83m87 = vaddq_s32(v11m87, v09_83);
    int32x4_t  v_p83m25 = vaddq_s32(v11m25, v09_83);
    int32x4_t  v_p36p57 = vaddq_s32(v11_57, v09_36);
    int32x4_t  v_m36p90 = vsubq_s32(v11_90, v09_36);
    int32x4_t  v_m83p43 = vsubq_s32(v11_43, v09_83);

    outrow[ 0] = vaddq_s32( outrow[ 0], v_p83p80 );
    outrow[ 1] = vaddq_s32( outrow[ 1], v_p36p09 );
    outrow[ 2] = vaddq_s32( outrow[ 2], v_m36m70 );
    outrow[ 3] = vaddq_s32( outrow[ 3], v_m83m87 );
    outrow[ 4] = vaddq_s32( outrow[ 4], v_m83m25 );
    outrow[ 5] = vaddq_s32( outrow[ 5], v_m36p57 );
    outrow[ 6] = vaddq_s32( outrow[ 6], v_p36p90 );
    outrow[ 7] = vaddq_s32( outrow[ 7], v_p83p43 );

    outrow[ 8] = vsubq_s32( outrow[ 8], v_m83p43 );
    outrow[ 9] = vsubq_s32( outrow[ 9], v_m36p90 );
    outrow[10] = vsubq_s32( outrow[10], v_p36p57 );
    outrow[11] = vsubq_s32( outrow[11], v_p83m25 );
    outrow[12] = vsubq_s32( outrow[12], v_p83m87 );
    outrow[13] = vsubq_s32( outrow[13], v_p36m70 );
    outrow[14] = vsubq_s32( outrow[14], v_m36p09 );
    outrow[15] = vsubq_s32( outrow[15], v_m83p80 );

    outrow[16] = vsubq_s32( outrow[16], v_m83p80 );
    outrow[17] = vsubq_s32( outrow[17], v_m36p09 );
    outrow[18] = vsubq_s32( outrow[18], v_p36m70 );
    outrow[19] = vsubq_s32( outrow[19], v_p83m87 );
    outrow[20] = vsubq_s32( outrow[20], v_p83m25 );
    outrow[21] = vsubq_s32( outrow[21], v_p36p57 );
    outrow[22] = vsubq_s32( outrow[22], v_m36p90 );
    outrow[23] = vsubq_s32( outrow[23], v_m83p43 );

    outrow[24] = vaddq_s32( outrow[24], v_p83p43 );
    outrow[25] = vaddq_s32( outrow[25], v_p36p90 );
    outrow[26] = vaddq_s32( outrow[26], v_m36p57 );
    outrow[27] = vaddq_s32( outrow[27], v_m83m25 );
    outrow[28] = vaddq_s32( outrow[28], v_m83m87 );
    outrow[29] = vaddq_s32( outrow[29], v_m36m70 );
    outrow[30] = vaddq_s32( outrow[30], v_p36p09 );
    outrow[31] = vaddq_s32( outrow[31], v_p83p80 );

    int32x4_t tmp;
    tmp = vaddq_s32( vmull_n_s16(coeff10, 82), vmull_n_s16(coeff12, 78) );
    outrow[ 0] = vaddq_s32(outrow[ 0], tmp);
    outrow[31] = vsubq_s32(outrow[31], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 22), vmull_n_s16(coeff12, -4) );
    outrow[ 1] = vaddq_s32(outrow[ 1], tmp);
    outrow[30] = vsubq_s32(outrow[30], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10,-54), vmull_n_s16(coeff12,-82) );
    outrow[ 2] = vaddq_s32(outrow[ 2], tmp);
    outrow[29] = vsubq_s32(outrow[29], tmp);

    int32x4_t v10_m90 = vmull_n_s16(coeff10,-90);
    tmp = vaddq_s32( v10_m90,                  vmull_n_s16(coeff12,-73) );
    outrow[ 3] = vaddq_s32(outrow[ 3], tmp);
    outrow[28] = vsubq_s32(outrow[28], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10,-61), vmull_n_s16(coeff12, 13) );
    outrow[ 4] = vaddq_s32(outrow[ 4], tmp);
    outrow[27] = vsubq_s32(outrow[27], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 13), vmull_n_s16(coeff12, 85) );
    outrow[ 5] = vaddq_s32(outrow[ 5], tmp);
    outrow[26] = vsubq_s32(outrow[26], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 78), vmull_n_s16(coeff12, 67) );
    outrow[ 6] = vaddq_s32(outrow[ 6], tmp);
    outrow[25] = vsubq_s32(outrow[25], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 85), vmull_n_s16(coeff12,-22) );
    outrow[ 7] = vaddq_s32(outrow[ 7], tmp);
    outrow[24] = vsubq_s32(outrow[24], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 31), vmull_n_s16(coeff12,-88) );
    outrow[ 8] = vaddq_s32(outrow[ 8], tmp);
    outrow[23] = vsubq_s32(outrow[23], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10,-46), vmull_n_s16(coeff12,-61) );
    outrow[ 9] = vaddq_s32(outrow[ 9], tmp);
    outrow[22] = vsubq_s32(outrow[22], tmp);

    tmp = vaddq_s32( v10_m90,                  vmull_n_s16(coeff12, 31) );
    outrow[10] = vaddq_s32(outrow[10], tmp);
    outrow[21] = vsubq_s32(outrow[21], tmp);

    int32x4_t v12_90 = vmull_n_s16(coeff12,90);
    tmp = vaddq_s32( vmull_n_s16(coeff10,-67), v12_90 );
    outrow[11] = vaddq_s32(outrow[11], tmp);
    outrow[20] = vsubq_s32(outrow[20], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10,  4), vmull_n_s16(coeff12, 54) );
    outrow[12] = vaddq_s32(outrow[12], tmp);
    outrow[19] = vsubq_s32(outrow[19], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 73), vmull_n_s16(coeff12,-38) );
    outrow[13] = vaddq_s32(outrow[13], tmp);
    outrow[18] = vsubq_s32(outrow[18], tmp);

    tmp = vsubq_s32( vmull_n_s16(coeff10, 88), v12_90 );
    outrow[14] = vaddq_s32(outrow[14], tmp);
    outrow[17] = vsubq_s32(outrow[17], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff10, 38), vmull_n_s16(coeff12,-46) );
    outrow[15] = vaddq_s32(outrow[15], tmp);
    outrow[16] = vsubq_s32(outrow[16], tmp);
  }

  if (maxRow >= 12) {
    int16x4_t  coeff13 = vld1_s16(coeffs   +12*32);
    int16x4_t  coeff14 = vld1_s16(coeffs+32+12*32);
    int16x4_t  coeff15 = vld1_s16(coeffs+64+12*32);
    int16x4_t  coeff16 = vld1_s16(coeffs+96+12*32);

    int32x4_t  v13_75 = vmull_n_s16(coeff13, 75);
    int32x4_t  v13m18 = vmull_n_s16(coeff13,-18);
    int32x4_t  v13m89 = vmull_n_s16(coeff13,-89);
    int32x4_t  v13m50 = vmull_n_s16(coeff13,-50);

    int32x4_t  v15_70 = vmull_n_s16(coeff15, 70);
    int32x4_t  v15m43 = vmull_n_s16(coeff15,-43);
    int32x4_t  v15m87 = vmull_n_s16(coeff15,-87);
    int32x4_t  v15_09 = vmull_n_s16(coeff15,  9);
    int32x4_t  v15_90 = vmull_n_s16(coeff15, 90);
    int32x4_t  v15_25 = vmull_n_s16(coeff15, 25);
    int32x4_t  v15m80 = vmull_n_s16(coeff15,-80);
    int32x4_t  v15m57 = vmull_n_s16(coeff15,-57);

    int32x4_t  v_p75p70 = vaddq_s32(v15_70, v13_75);
    int32x4_t  v_m18m43 = vaddq_s32(v15m43, v13m18);
    int32x4_t  v_m89m87 = vaddq_s32(v15m87, v13m89);
    int32x4_t  v_m50p09 = vaddq_s32(v15_09, v13m50);
    int32x4_t  v_p50p90 = vsubq_s32(v15_90, v13m50);
    int32x4_t  v_p89p25 = vsubq_s32(v15_25, v13m89);
    int32x4_t  v_p18m80 = vsubq_s32(v15m80, v13m18);
    int32x4_t  v_m75m57 = vsubq_s32(v15m57, v13_75);

    int32x4_t  v_p75m57 = vaddq_s32(v15m57, v13_75);
    int32x4_t  v_m18m80 = vaddq_s32(v15m80, v13m18);
    int32x4_t  v_m89p25 = vaddq_s32(v15_25, v13m89);
    int32x4_t  v_m50p90 = vaddq_s32(v15_90, v13m50);
    int32x4_t  v_p50p09 = vsubq_s32(v15_09, v13m50);
    int32x4_t  v_p89m87 = vsubq_s32(v15m87, v13m89);
    int32x4_t  v_p18m43 = vsubq_s32(v15m43, v13m18);
    int32x4_t  v_m75p70 = vsubq_s32(v15_70, v13_75);

    outrow[ 0] = vaddq_s32( outrow[ 0], v_p75p70 );
    outrow[ 1] = vaddq_s32( outrow[ 1], v_m18m43 );
    outrow[ 2] = vaddq_s32( outrow[ 2], v_m89m87 );
    outrow[ 3] = vaddq_s32( outrow[ 3], v_m50p09 );
    outrow[ 4] = vaddq_s32( outrow[ 4], v_p50p90 );
    outrow[ 5] = vaddq_s32( outrow[ 5], v_p89p25 );
    outrow[ 6] = vaddq_s32( outrow[ 6], v_p18m80 );
    outrow[ 7] = vaddq_s32( outrow[ 7], v_m75m57 );

    outrow[ 8] = vsubq_s32( outrow[ 8], v_p75m57 );
    outrow[ 9] = vsubq_s32( outrow[ 9], v_m18m80 );
    outrow[10] = vsubq_s32( outrow[10], v_m89p25 );
    outrow[11] = vsubq_s32( outrow[11], v_m50p90 );
    outrow[12] = vsubq_s32( outrow[12], v_p50p09 );
    outrow[13] = vsubq_s32( outrow[13], v_p89m87 );
    outrow[14] = vsubq_s32( outrow[14], v_p18m43 );
    outrow[15] = vsubq_s32( outrow[15], v_m75p70 );

    outrow[16] = vsubq_s32( outrow[16], v_m75p70 );
    outrow[17] = vsubq_s32( outrow[17], v_p18m43 );
    outrow[18] = vsubq_s32( outrow[18], v_p89m87 );
    outrow[19] = vsubq_s32( outrow[19], v_p50p09 );
    outrow[20] = vsubq_s32( outrow[20], v_m50p90 );
    outrow[21] = vsubq_s32( outrow[21], v_m89p25 );
    outrow[22] = vsubq_s32( outrow[22], v_m18m80 );
    outrow[23] = vsubq_s32( outrow[23], v_p75m57 );

    outrow[24] = vaddq_s32( outrow[24], v_m75m57 );
    outrow[25] = vaddq_s32( outrow[25], v_p18m80 );
    outrow[26] = vaddq_s32( outrow[26], v_p89p25 );
    outrow[27] = vaddq_s32( outrow[27], v_p50p90 );
    outrow[28] = vaddq_s32( outrow[28], v_m50p09 );
    outrow[29] = vaddq_s32( outrow[29], v_m89m87 );
    outrow[30] = vaddq_s32( outrow[30], v_m18m43 );
    outrow[31] = vaddq_s32( outrow[31], v_p75p70 );

    int32x4_t tmp;
    int32x4_t v14_m90 = vmull_n_s16(coeff14,-90);
    int32x4_t v16_p90 = vmull_n_s16(coeff16, 90);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 73), vmull_n_s16(coeff16, 67) );
    outrow[ 0] = vaddq_s32(outrow[ 0], tmp);
    outrow[31] = vsubq_s32(outrow[31], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14,-31), vmull_n_s16(coeff16,-54) );
    outrow[ 1] = vaddq_s32(outrow[ 1], tmp);
    outrow[30] = vsubq_s32(outrow[30], tmp);

    tmp = vaddq_s32( v14_m90,                  vmull_n_s16(coeff16,-78) );
    outrow[ 2] = vaddq_s32(outrow[ 2], tmp);
    outrow[29] = vsubq_s32(outrow[29], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14,-22), vmull_n_s16(coeff16, 38) );
    outrow[ 3] = vaddq_s32(outrow[ 3], tmp);
    outrow[28] = vsubq_s32(outrow[28], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 78), vmull_n_s16(coeff16, 85) );
    outrow[ 4] = vaddq_s32(outrow[ 4], tmp);
    outrow[27] = vsubq_s32(outrow[27], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 67), vmull_n_s16(coeff16,-22) );
    outrow[ 5] = vaddq_s32(outrow[ 5], tmp);
    outrow[26] = vsubq_s32(outrow[26], tmp);

    tmp = vsubq_s32( vmull_n_s16(coeff14,-38), v16_p90 );
    outrow[ 6] = vaddq_s32(outrow[ 6], tmp);
    outrow[25] = vsubq_s32(outrow[25], tmp);

    tmp = vaddq_s32( v14_m90,                  vmull_n_s16(coeff16,  4) );
    outrow[ 7] = vaddq_s32(outrow[ 7], tmp);
    outrow[24] = vsubq_s32(outrow[24], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14,-13), v16_p90 );
    outrow[ 8] = vaddq_s32(outrow[ 8], tmp);
    outrow[23] = vsubq_s32(outrow[23], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 82), vmull_n_s16(coeff16, 13) );
    outrow[ 9] = vaddq_s32(outrow[ 9], tmp);
    outrow[22] = vsubq_s32(outrow[22], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 61), vmull_n_s16(coeff16,-88) );
    outrow[10] = vaddq_s32(outrow[10], tmp);
    outrow[21] = vsubq_s32(outrow[21], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14,-46), vmull_n_s16(coeff16,-31) );
    outrow[11] = vaddq_s32(outrow[11], tmp);
    outrow[20] = vsubq_s32(outrow[20], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14,-88), vmull_n_s16(coeff16, 82) );
    outrow[12] = vaddq_s32(outrow[12], tmp);
    outrow[19] = vsubq_s32(outrow[19], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, -4), vmull_n_s16(coeff16, 46) );
    outrow[13] = vaddq_s32(outrow[13], tmp);
    outrow[18] = vsubq_s32(outrow[18], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 85), vmull_n_s16(coeff16,-73) );
    outrow[14] = vaddq_s32(outrow[14], tmp);
    outrow[17] = vsubq_s32(outrow[17], tmp);

    tmp = vaddq_s32( vmull_n_s16(coeff14, 54), vmull_n_s16(coeff16,-61) );
    outrow[15] = vaddq_s32(outrow[15], tmp);
    outrow[16] = vsubq_s32(outrow[16], tmp);
  }

  outrow[ 0] = vrshrq_n_s32( outrow[ 0], 7 );
  outrow[ 1] = vrshrq_n_s32( outrow[ 1], 7 );
  outrow[ 2] = vrshrq_n_s32( outrow[ 2], 7 );
  outrow[ 3] = vrshrq_n_s32( outrow[ 3], 7 );
  outrow[ 4] = vrshrq_n_s32( outrow[ 4], 7 );
  outrow[ 5] = vrshrq_n_s32( outrow[ 5], 7 );
  outrow[ 6] = vrshrq_n_s32( outrow[ 6], 7 );
  outrow[ 7] = vrshrq_n_s32( outrow[ 7], 7 );
  outrow[ 8] = vrshrq_n_s32( outrow[ 8], 7 );
  outrow[ 9] = vrshrq_n_s32( outrow[ 9], 7 );
  outrow[10] = vrshrq_n_s32( outrow[10], 7 );
  outrow[11] = vrshrq_n_s32( outrow[11], 7 );
  outrow[12] = vrshrq_n_s32( outrow[12], 7 );
  outrow[13] = vrshrq_n_s32( outrow[13], 7 );
  outrow[14] = vrshrq_n_s32( outrow[14], 7 );
  outrow[15] = vrshrq_n_s32( outrow[15], 7 );

  outrow[16] = vrshrq_n_s32( outrow[16], 7 );
  outrow[17] = vrshrq_n_s32( outrow[17], 7 );
  outrow[18] = vrshrq_n_s32( outrow[18], 7 );
  outrow[19] = vrshrq_n_s32( outrow[19], 7 );
  outrow[20] = vrshrq_n_s32( outrow[20], 7 );
  outrow[21] = vrshrq_n_s32( outrow[21], 7 );
  outrow[22] = vrshrq_n_s32( outrow[22], 7 );
  outrow[23] = vrshrq_n_s32( outrow[23], 7 );
  outrow[24] = vrshrq_n_s32( outrow[24], 7 );
  outrow[25] = vrshrq_n_s32( outrow[25], 7 );
  outrow[26] = vrshrq_n_s32( outrow[26], 7 );
  outrow[27] = vrshrq_n_s32( outrow[27], 7 );
  outrow[28] = vrshrq_n_s32( outrow[28], 7 );
  outrow[29] = vrshrq_n_s32( outrow[29], 7 );
  outrow[30] = vrshrq_n_s32( outrow[30], 7 );
  outrow[31] = vrshrq_n_s32( outrow[31], 7 );
}


void transform_32x32_add_horiz(uint8_t* dst, ptrdiff_t stride,
                               int maxColumn,
                               const int32x4_t* block1,
                               const int32x4_t* block2,
                               const int32x4_t* block3,
                               const int32x4_t* block4)
{
  int32x4_t coeff1,coeff2,coeff3,coeff4;
  transpose_4x4(coeff1,coeff2,coeff3,coeff4, block1[0],block1[1],block1[2],block1[3]);

  int32x4_t  v1_A  = vshlq_n_s32(coeff1, 6); // * 64

  int32x4_t  v3_90 = vmulq_n_s32(coeff3, 90);
  int32x4_t  v3_87 = vmulq_n_s32(coeff3, 87);
  int32x4_t  v3_80 = vmulq_n_s32(coeff3, 80);
  int32x4_t  v3_70 = vmulq_n_s32(coeff3, 70);
  int32x4_t  v3_57 = vmulq_n_s32(coeff3, 57);
  int32x4_t  v3_43 = vmulq_n_s32(coeff3, 43);
  int32x4_t  v3_25 = vmulq_n_s32(coeff3, 25);
  int32x4_t  v3_09 = vmulq_n_s32(coeff3,  9);

  int32x4_t  v2_90 = vmulq_n_s32(coeff2, 90);
  int32x4_t  v2_88 = vmulq_n_s32(coeff2, 88);
  int32x4_t  v2_85 = vmulq_n_s32(coeff2, 85);
  int32x4_t  v2_82 = vmulq_n_s32(coeff2, 82);
  int32x4_t  v2_78 = vmulq_n_s32(coeff2, 78);
  int32x4_t  v2_73 = vmulq_n_s32(coeff2, 73);
  int32x4_t  v2_67 = vmulq_n_s32(coeff2, 67);
  int32x4_t  v2_61 = vmulq_n_s32(coeff2, 61);
  int32x4_t  v2_54 = vmulq_n_s32(coeff2, 54);
  int32x4_t  v2_46 = vmulq_n_s32(coeff2, 46);
  int32x4_t  v2_38 = vmulq_n_s32(coeff2, 38);
  int32x4_t  v2_31 = vmulq_n_s32(coeff2, 31);
  int32x4_t  v2_22 = vmulq_n_s32(coeff2, 22);
  int32x4_t  v2_13 = vmulq_n_s32(coeff2, 13);
  int32x4_t  v2_04 = vmulq_n_s32(coeff2,  4);

  int32x4_t  v4_90 = vmulq_n_s32(coeff4, 90);
  int32x4_t  v4_82 = vmulq_n_s32(coeff4, 82);
  int32x4_t  v4_67 = vmulq_n_s32(coeff4, 67);
  int32x4_t  v4_46 = vmulq_n_s32(coeff4, 46);
  int32x4_t  v4_22 = vmulq_n_s32(coeff4, 22);
  int32x4_t  v4m04 = vmulq_n_s32(coeff4,- 4);
  int32x4_t  v4m31 = vmulq_n_s32(coeff4,-31);
  int32x4_t  v4m54 = vmulq_n_s32(coeff4,-54);
  int32x4_t  v4m73 = vmulq_n_s32(coeff4,-73);
  int32x4_t  v4m85 = vmulq_n_s32(coeff4,-85);
  //int32x4_t  v4m90 = vmulq_n_s32(coeff4,-90);  we can reuse v4_90
  int32x4_t  v4m88 = vmulq_n_s32(coeff4,-88);
  int32x4_t  v4m78 = vmulq_n_s32(coeff4,-78);
  int32x4_t  v4m61 = vmulq_n_s32(coeff4,-61);
  int32x4_t  v4m38 = vmulq_n_s32(coeff4,-38);
  int32x4_t  v4m13 = vmulq_n_s32(coeff4,-13);

  int32x4_t  v_p90p90 = vaddq_s32(v2_90, v4_90);
  int32x4_t  v_p90p82 = vaddq_s32(v2_90, v4_82);
  int32x4_t  v_p88p67 = vaddq_s32(v2_88, v4_67);
  int32x4_t  v_p85p46 = vaddq_s32(v2_85, v4_46);
  int32x4_t  v_p82p22 = vaddq_s32(v2_82, v4_22);
  int32x4_t  v_p78m04 = vaddq_s32(v2_78, v4m04);
  int32x4_t  v_p73m31 = vaddq_s32(v2_73, v4m31);
  int32x4_t  v_p67m54 = vaddq_s32(v2_67, v4m54);
  int32x4_t  v_p61m73 = vaddq_s32(v2_61, v4m73);
  int32x4_t  v_p54m85 = vaddq_s32(v2_54, v4m85);
  int32x4_t  v_p46m90 = vsubq_s32(v2_46, v4_90);
  int32x4_t  v_p38m88 = vaddq_s32(v2_38, v4m88);
  int32x4_t  v_p31m78 = vaddq_s32(v2_31, v4m78);
  int32x4_t  v_p22m61 = vaddq_s32(v2_22, v4m61);
  int32x4_t  v_p13m38 = vaddq_s32(v2_13, v4m38);
  int32x4_t  v_p04m13 = vaddq_s32(v2_04, v4m13);

  int32x4_t  v_pAp90 = vaddq_s32(v1_A, v3_90);
  int32x4_t  v_pAp87 = vaddq_s32(v1_A, v3_87);
  int32x4_t  v_pAp80 = vaddq_s32(v1_A, v3_80);
  int32x4_t  v_pAp70 = vaddq_s32(v1_A, v3_70);
  int32x4_t  v_pAp57 = vaddq_s32(v1_A, v3_57);
  int32x4_t  v_pAp43 = vaddq_s32(v1_A, v3_43);
  int32x4_t  v_pAp25 = vaddq_s32(v1_A, v3_25);
  int32x4_t  v_pAp09 = vaddq_s32(v1_A, v3_09);
  int32x4_t  v_pAm90 = vsubq_s32(v1_A, v3_90);
  int32x4_t  v_pAm87 = vsubq_s32(v1_A, v3_87);
  int32x4_t  v_pAm80 = vsubq_s32(v1_A, v3_80);
  int32x4_t  v_pAm70 = vsubq_s32(v1_A, v3_70);
  int32x4_t  v_pAm57 = vsubq_s32(v1_A, v3_57);
  int32x4_t  v_pAm43 = vsubq_s32(v1_A, v3_43);
  int32x4_t  v_pAm25 = vsubq_s32(v1_A, v3_25);
  int32x4_t  v_pAm09 = vsubq_s32(v1_A, v3_09);

  int32x4_t  outrow[32];

  outrow[ 0] = vaddq_s32(v_pAp90, v_p90p90);
  outrow[ 1] = vaddq_s32(v_pAp87, v_p90p82);
  outrow[ 2] = vaddq_s32(v_pAp80, v_p88p67);
  outrow[ 3] = vaddq_s32(v_pAp70, v_p85p46);
  outrow[ 4] = vaddq_s32(v_pAp57, v_p82p22);
  outrow[ 5] = vaddq_s32(v_pAp43, v_p78m04);
  outrow[ 6] = vaddq_s32(v_pAp25, v_p73m31);
  outrow[ 7] = vaddq_s32(v_pAp09, v_p67m54);

  outrow[ 8] = vaddq_s32(v_pAm09, v_p61m73);
  outrow[ 9] = vaddq_s32(v_pAm25, v_p54m85);
  outrow[10] = vaddq_s32(v_pAm43, v_p46m90);
  outrow[11] = vaddq_s32(v_pAm57, v_p38m88);
  outrow[12] = vaddq_s32(v_pAm70, v_p31m78);
  outrow[13] = vaddq_s32(v_pAm80, v_p22m61);
  outrow[14] = vaddq_s32(v_pAm87, v_p13m38);
  outrow[15] = vaddq_s32(v_pAm90, v_p04m13);

  outrow[16] = vsubq_s32(v_pAm90, v_p04m13);
  outrow[17] = vsubq_s32(v_pAm87, v_p13m38);
  outrow[18] = vsubq_s32(v_pAm80, v_p22m61);
  outrow[19] = vsubq_s32(v_pAm70, v_p31m78);
  outrow[20] = vsubq_s32(v_pAm57, v_p38m88);
  outrow[21] = vsubq_s32(v_pAm43, v_p46m90);
  outrow[22] = vsubq_s32(v_pAm25, v_p54m85);
  outrow[23] = vsubq_s32(v_pAm09, v_p61m73);

  outrow[24] = vsubq_s32(v_pAp09, v_p67m54);
  outrow[25] = vsubq_s32(v_pAp25, v_p73m31);
  outrow[26] = vsubq_s32(v_pAp43, v_p78m04);
  outrow[27] = vsubq_s32(v_pAp57, v_p82p22);
  outrow[28] = vsubq_s32(v_pAp70, v_p85p46);
  outrow[29] = vsubq_s32(v_pAp80, v_p88p67);
  outrow[30] = vsubq_s32(v_pAp87, v_p90p82);
  outrow[31] = vsubq_s32(v_pAp90, v_p90p90);


  if (maxColumn >= 4) {
    int32x4_t coeff5,coeff6,coeff7,coeff8;
    transpose_4x4(coeff5,coeff6,coeff7,coeff8, block2[0],block2[1],block2[2],block2[3]);

    int32x4_t  v1b_89 = vmulq_n_s32(coeff5, 89);
    int32x4_t  v1b_75 = vmulq_n_s32(coeff5, 75);
    int32x4_t  v1b_50 = vmulq_n_s32(coeff5, 50);
    int32x4_t  v1b_18 = vmulq_n_s32(coeff5, 18);

    int32x4_t  v2b_88 = vmulq_n_s32(coeff6, 88);
    int32x4_t  v2b_67 = vmulq_n_s32(coeff6, 67);
    int32x4_t  v2b_31 = vmulq_n_s32(coeff6, 31);
    int32x4_t  v2bm13 = vmulq_n_s32(coeff6,-13);
    int32x4_t  v2bm54 = vmulq_n_s32(coeff6,-54);
    int32x4_t  v2bm82 = vmulq_n_s32(coeff6,-82);
    //int32x4_t  v2bm90 = vmulq_n_s32(coeff6, -90);   can reuse v2b_90
    int32x4_t  v2bm78 = vmulq_n_s32(coeff6,-78);
    int32x4_t  v2bm46 = vmulq_n_s32(coeff6,-46);
    int32x4_t  v2bm04 = vmulq_n_s32(coeff6, -4);
    int32x4_t  v2b_38 = vmulq_n_s32(coeff6, 38);
    int32x4_t  v2b_73 = vmulq_n_s32(coeff6, 73);
    int32x4_t  v2b_90 = vmulq_n_s32(coeff6, 90);
    int32x4_t  v2b_85 = vmulq_n_s32(coeff6, 85);
    int32x4_t  v2b_61 = vmulq_n_s32(coeff6, 61);
    int32x4_t  v2b_22 = vmulq_n_s32(coeff6, 22);

    int32x4_t  v3b_87 = vmulq_n_s32(coeff7, 87);
    int32x4_t  v3b_57 = vmulq_n_s32(coeff7, 57);
    int32x4_t  v3b_09 = vmulq_n_s32(coeff7,  9);
    int32x4_t  v3bm43 = vmulq_n_s32(coeff7,-43);
    int32x4_t  v3bm80 = vmulq_n_s32(coeff7,-80);
    int32x4_t  v3bm90 = vmulq_n_s32(coeff7,-90);
    int32x4_t  v3bm70 = vmulq_n_s32(coeff7,-70);
    int32x4_t  v3bm25 = vmulq_n_s32(coeff7,-25);

    int32x4_t  v4b_85 = vmulq_n_s32(coeff8, 85);
    int32x4_t  v4b_46 = vmulq_n_s32(coeff8, 46);
    int32x4_t  v4bm13 = vmulq_n_s32(coeff8,-13);
    int32x4_t  v4bm67 = vmulq_n_s32(coeff8,-67);
    int32x4_t  v4bm90 = vmulq_n_s32(coeff8,-90);
    int32x4_t  v4bm73 = vmulq_n_s32(coeff8,-73);
    int32x4_t  v4bm22 = vmulq_n_s32(coeff8,-22);
    int32x4_t  v4b_38 = vmulq_n_s32(coeff8, 38);
    int32x4_t  v4b_82 = vmulq_n_s32(coeff8, 82);
    int32x4_t  v4b_88 = vmulq_n_s32(coeff8, 88);
    int32x4_t  v4b_54 = vmulq_n_s32(coeff8, 54);
    int32x4_t  v4bm04 = vmulq_n_s32(coeff8, -4);
    int32x4_t  v4bm61 = vmulq_n_s32(coeff8,-61);
    //int32x4_t  v4bm90 = vmulq_n_s32(coeff8, 85);  duplicate
    int32x4_t  v4bm78 = vmulq_n_s32(coeff8,-78);
    int32x4_t  v4bm31 = vmulq_n_s32(coeff8,-31);


    int32x4_t  vb_p89p87 = vaddq_s32(v3b_87, v1b_89);
    int32x4_t  vb_p75p57 = vaddq_s32(v3b_57, v1b_75);
    int32x4_t  vb_p50p09 = vaddq_s32(v3b_09, v1b_50);
    int32x4_t  vb_p18m43 = vaddq_s32(v3bm43, v1b_18);
    int32x4_t  vb_m18m80 = vsubq_s32(v3bm80, v1b_18);
    int32x4_t  vb_m50m90 = vsubq_s32(v3bm90, v1b_50);
    int32x4_t  vb_m75m70 = vsubq_s32(v3bm70, v1b_75);
    int32x4_t  vb_m89m25 = vsubq_s32(v3bm25, v1b_89);

    int32x4_t  vb_m89p25_m = vaddq_s32(v3bm25, v1b_89);
    int32x4_t  vb_m75p70_m = vaddq_s32(v3bm70, v1b_75);
    int32x4_t  vb_m50p90_m = vaddq_s32(v3bm90, v1b_50);
    int32x4_t  vb_m18p80_m = vaddq_s32(v3bm80, v1b_18);
    int32x4_t  vb_p18p43 = vsubq_s32(v1b_18, v3bm43);
    int32x4_t  vb_p50m09 = vsubq_s32(v1b_50, v3b_09);
    int32x4_t  vb_p75m57 = vsubq_s32(v1b_75, v3b_57);
    int32x4_t  vb_p89m87 = vsubq_s32(v1b_89, v3b_87);

    outrow[ 0] = vaddq_s32(outrow[ 0], vb_p89p87);
    outrow[ 1] = vaddq_s32(outrow[ 1], vb_p75p57);
    outrow[ 2] = vaddq_s32(outrow[ 2], vb_p50p09);
    outrow[ 3] = vaddq_s32(outrow[ 3], vb_p18m43);
    outrow[ 4] = vaddq_s32(outrow[ 4], vb_m18m80);
    outrow[ 5] = vaddq_s32(outrow[ 5], vb_m50m90);
    outrow[ 6] = vaddq_s32(outrow[ 6], vb_m75m70);
    outrow[ 7] = vaddq_s32(outrow[ 7], vb_m89m25);
    outrow[ 8] = vsubq_s32(outrow[ 8], vb_m89p25_m);
    outrow[ 9] = vsubq_s32(outrow[ 9], vb_m75p70_m);
    outrow[10] = vsubq_s32(outrow[10], vb_m50p90_m);
    outrow[11] = vsubq_s32(outrow[11], vb_m18p80_m);
    outrow[12] = vaddq_s32(outrow[12], vb_p18p43);
    outrow[13] = vaddq_s32(outrow[13], vb_p50m09);
    outrow[14] = vaddq_s32(outrow[14], vb_p75m57);
    outrow[15] = vaddq_s32(outrow[15], vb_p89m87);
    outrow[16] = vaddq_s32(outrow[16], vb_p89m87);
    outrow[17] = vaddq_s32(outrow[17], vb_p75m57);
    outrow[18] = vaddq_s32(outrow[18], vb_p50m09);
    outrow[19] = vaddq_s32(outrow[19], vb_p18p43);
    outrow[20] = vsubq_s32(outrow[20], vb_m18p80_m);
    outrow[21] = vsubq_s32(outrow[21], vb_m50p90_m);
    outrow[22] = vsubq_s32(outrow[22], vb_m75p70_m);
    outrow[23] = vsubq_s32(outrow[23], vb_m89p25_m);
    outrow[24] = vaddq_s32(outrow[24], vb_m89m25);
    outrow[25] = vaddq_s32(outrow[25], vb_m75m70);
    outrow[26] = vaddq_s32(outrow[26], vb_m50m90);
    outrow[27] = vaddq_s32(outrow[27], vb_m18m80);
    outrow[28] = vaddq_s32(outrow[28], vb_p18m43);
    outrow[29] = vaddq_s32(outrow[29], vb_p50p09);
    outrow[30] = vaddq_s32(outrow[30], vb_p75p57);
    outrow[31] = vaddq_s32(outrow[31], vb_p89p87);

    int32x4_t tmp;
    tmp = vaddq_s32(v2b_88, v4b_85);
    outrow[ 0] = vaddq_s32(outrow[ 0], tmp);
    outrow[31] = vsubq_s32(outrow[31], tmp);

    tmp = vaddq_s32(v2b_67, v4b_46);
    outrow[ 1] = vaddq_s32(outrow[ 1], tmp);
    outrow[30] = vsubq_s32(outrow[30], tmp);

    tmp = vaddq_s32(v2b_31, v4bm13);
    outrow[ 2] = vaddq_s32(outrow[ 2], tmp);
    outrow[29] = vsubq_s32(outrow[29], tmp);

    tmp = vaddq_s32(v2bm13, v4bm67);
    outrow[ 3] = vaddq_s32(outrow[ 3], tmp);
    outrow[28] = vsubq_s32(outrow[28], tmp);

    tmp = vaddq_s32(v2bm54, v4bm90);
    outrow[ 4] = vaddq_s32(outrow[ 4], tmp);
    outrow[27] = vsubq_s32(outrow[27], tmp);

    tmp = vaddq_s32(v2bm82, v4bm73);
    outrow[ 5] = vaddq_s32(outrow[ 5], tmp);
    outrow[26] = vsubq_s32(outrow[26], tmp);

    tmp = vsubq_s32(v4bm22, v2b_90);
    outrow[ 6] = vaddq_s32(outrow[ 6], tmp);
    outrow[25] = vsubq_s32(outrow[25], tmp);

    tmp = vaddq_s32(v2bm78, v4b_38);
    outrow[ 7] = vaddq_s32(outrow[ 7], tmp);
    outrow[24] = vsubq_s32(outrow[24], tmp);

    tmp = vaddq_s32(v2bm46, v4b_82);
    outrow[ 8] = vaddq_s32(outrow[ 8], tmp);
    outrow[23] = vsubq_s32(outrow[23], tmp);

    tmp = vaddq_s32(v2bm04, v4b_88);
    outrow[ 9] = vaddq_s32(outrow[ 9], tmp);
    outrow[22] = vsubq_s32(outrow[22], tmp);

    tmp = vaddq_s32(v2b_38, v4b_54);
    outrow[10] = vaddq_s32(outrow[10], tmp);
    outrow[21] = vsubq_s32(outrow[21], tmp);

    tmp = vaddq_s32(v2b_73, v4bm04);
    outrow[11] = vaddq_s32(outrow[11], tmp);
    outrow[20] = vsubq_s32(outrow[20], tmp);

    tmp = vaddq_s32(v2b_90, v4bm61);
    outrow[12] = vaddq_s32(outrow[12], tmp);
    outrow[19] = vsubq_s32(outrow[19], tmp);

    tmp = vaddq_s32(v2b_85, v4bm90);
    outrow[13] = vaddq_s32(outrow[13], tmp);
    outrow[18] = vsubq_s32(outrow[18], tmp);

    tmp = vaddq_s32(v2b_61, v4bm78);
    outrow[14] = vaddq_s32(outrow[14], tmp);
    outrow[17] = vsubq_s32(outrow[17], tmp);

    tmp = vaddq_s32(v2b_22, v4bm31);
    outrow[15] = vaddq_s32(outrow[15], tmp);
    outrow[16] = vsubq_s32(outrow[16], tmp);
  }

  if (maxColumn >= 8) {
    int32x4_t coeff09,coeff10,coeff11,coeff12;
    transpose_4x4(coeff09,coeff10,coeff11,coeff12, block3[0],block3[1],block3[2],block3[3]);

    int32x4_t  v09_83 = vmulq_n_s32(coeff09, 83);
    int32x4_t  v09_36 = vmulq_n_s32(coeff09, 36);

    int32x4_t  v11_80 = vmulq_n_s32(coeff11, 80);
    int32x4_t  v11_09 = vmulq_n_s32(coeff11,  9);
    int32x4_t  v11m70 = vmulq_n_s32(coeff11,-70);
    int32x4_t  v11m87 = vmulq_n_s32(coeff11,-87);
    int32x4_t  v11m25 = vmulq_n_s32(coeff11,-25);
    int32x4_t  v11_57 = vmulq_n_s32(coeff11, 57);
    int32x4_t  v11_90 = vmulq_n_s32(coeff11, 90);
    int32x4_t  v11_43 = vmulq_n_s32(coeff11, 43);

    int32x4_t  v_p83p80 = vaddq_s32(v11_80, v09_83);
    int32x4_t  v_p36p09 = vaddq_s32(v11_09, v09_36);
    int32x4_t  v_m36m70 = vsubq_s32(v11m70, v09_36);
    int32x4_t  v_m83m87 = vsubq_s32(v11m87, v09_83);
    int32x4_t  v_m83m25 = vsubq_s32(v11m25, v09_83);
    int32x4_t  v_m36p57 = vsubq_s32(v11_57, v09_36);
    int32x4_t  v_p36p90 = vaddq_s32(v11_90, v09_36);
    int32x4_t  v_p83p43 = vaddq_s32(v11_43, v09_83);

    int32x4_t  v_m83p80 = vsubq_s32(v11_80, v09_83);
    int32x4_t  v_m36p09 = vsubq_s32(v11_09, v09_36);
    int32x4_t  v_p36m70 = vaddq_s32(v11m70, v09_36);
    int32x4_t  v_p83m87 = vaddq_s32(v11m87, v09_83);
    int32x4_t  v_p83m25 = vaddq_s32(v11m25, v09_83);
    int32x4_t  v_p36p57 = vaddq_s32(v11_57, v09_36);
    int32x4_t  v_m36p90 = vsubq_s32(v11_90, v09_36);
    int32x4_t  v_m83p43 = vsubq_s32(v11_43, v09_83);

    outrow[ 0] = vaddq_s32( outrow[ 0], v_p83p80 );
    outrow[ 1] = vaddq_s32( outrow[ 1], v_p36p09 );
    outrow[ 2] = vaddq_s32( outrow[ 2], v_m36m70 );
    outrow[ 3] = vaddq_s32( outrow[ 3], v_m83m87 );
    outrow[ 4] = vaddq_s32( outrow[ 4], v_m83m25 );
    outrow[ 5] = vaddq_s32( outrow[ 5], v_m36p57 );
    outrow[ 6] = vaddq_s32( outrow[ 6], v_p36p90 );
    outrow[ 7] = vaddq_s32( outrow[ 7], v_p83p43 );

    outrow[ 8] = vsubq_s32( outrow[ 8], v_m83p43 );
    outrow[ 9] = vsubq_s32( outrow[ 9], v_m36p90 );
    outrow[10] = vsubq_s32( outrow[10], v_p36p57 );
    outrow[11] = vsubq_s32( outrow[11], v_p83m25 );
    outrow[12] = vsubq_s32( outrow[12], v_p83m87 );
    outrow[13] = vsubq_s32( outrow[13], v_p36m70 );
    outrow[14] = vsubq_s32( outrow[14], v_m36p09 );
    outrow[15] = vsubq_s32( outrow[15], v_m83p80 );

    outrow[16] = vsubq_s32( outrow[16], v_m83p80 );
    outrow[17] = vsubq_s32( outrow[17], v_m36p09 );
    outrow[18] = vsubq_s32( outrow[18], v_p36m70 );
    outrow[19] = vsubq_s32( outrow[19], v_p83m87 );
    outrow[20] = vsubq_s32( outrow[20], v_p83m25 );
    outrow[21] = vsubq_s32( outrow[21], v_p36p57 );
    outrow[22] = vsubq_s32( outrow[22], v_m36p90 );
    outrow[23] = vsubq_s32( outrow[23], v_m83p43 );

    outrow[24] = vaddq_s32( outrow[24], v_p83p43 );
    outrow[25] = vaddq_s32( outrow[25], v_p36p90 );
    outrow[26] = vaddq_s32( outrow[26], v_m36p57 );
    outrow[27] = vaddq_s32( outrow[27], v_m83m25 );
    outrow[28] = vaddq_s32( outrow[28], v_m83m87 );
    outrow[29] = vaddq_s32( outrow[29], v_m36m70 );
    outrow[30] = vaddq_s32( outrow[30], v_p36p09 );
    outrow[31] = vaddq_s32( outrow[31], v_p83p80 );

    int32x4_t tmp;
    tmp = vaddq_s32( vmulq_n_s32(coeff10, 82), vmulq_n_s32(coeff12, 78) );
    outrow[ 0] = vaddq_s32(outrow[ 0], tmp);
    outrow[31] = vsubq_s32(outrow[31], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 22), vmulq_n_s32(coeff12, -4) );
    outrow[ 1] = vaddq_s32(outrow[ 1], tmp);
    outrow[30] = vsubq_s32(outrow[30], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10,-54), vmulq_n_s32(coeff12,-82) );
    outrow[ 2] = vaddq_s32(outrow[ 2], tmp);
    outrow[29] = vsubq_s32(outrow[29], tmp);

    int32x4_t v10_m90 = vmulq_n_s32(coeff10,-90);
    tmp = vaddq_s32( v10_m90,                  vmulq_n_s32(coeff12,-73) );
    outrow[ 3] = vaddq_s32(outrow[ 3], tmp);
    outrow[28] = vsubq_s32(outrow[28], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10,-61), vmulq_n_s32(coeff12, 13) );
    outrow[ 4] = vaddq_s32(outrow[ 4], tmp);
    outrow[27] = vsubq_s32(outrow[27], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 13), vmulq_n_s32(coeff12, 85) );
    outrow[ 5] = vaddq_s32(outrow[ 5], tmp);
    outrow[26] = vsubq_s32(outrow[26], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 78), vmulq_n_s32(coeff12, 67) );
    outrow[ 6] = vaddq_s32(outrow[ 6], tmp);
    outrow[25] = vsubq_s32(outrow[25], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 85), vmulq_n_s32(coeff12,-22) );
    outrow[ 7] = vaddq_s32(outrow[ 7], tmp);
    outrow[24] = vsubq_s32(outrow[24], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 31), vmulq_n_s32(coeff12,-88) );
    outrow[ 8] = vaddq_s32(outrow[ 8], tmp);
    outrow[23] = vsubq_s32(outrow[23], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10,-46), vmulq_n_s32(coeff12,-61) );
    outrow[ 9] = vaddq_s32(outrow[ 9], tmp);
    outrow[22] = vsubq_s32(outrow[22], tmp);

    tmp = vaddq_s32( v10_m90,                  vmulq_n_s32(coeff12, 31) );
    outrow[10] = vaddq_s32(outrow[10], tmp);
    outrow[21] = vsubq_s32(outrow[21], tmp);

    int32x4_t v12_90 = vmulq_n_s32(coeff12,90);
    tmp = vaddq_s32( vmulq_n_s32(coeff10,-67), v12_90 );
    outrow[11] = vaddq_s32(outrow[11], tmp);
    outrow[20] = vsubq_s32(outrow[20], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10,  4), vmulq_n_s32(coeff12, 54) );
    outrow[12] = vaddq_s32(outrow[12], tmp);
    outrow[19] = vsubq_s32(outrow[19], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 73), vmulq_n_s32(coeff12,-38) );
    outrow[13] = vaddq_s32(outrow[13], tmp);
    outrow[18] = vsubq_s32(outrow[18], tmp);

    tmp = vsubq_s32( vmulq_n_s32(coeff10, 88), v12_90 );
    outrow[14] = vaddq_s32(outrow[14], tmp);
    outrow[17] = vsubq_s32(outrow[17], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff10, 38), vmulq_n_s32(coeff12,-46) );
    outrow[15] = vaddq_s32(outrow[15], tmp);
    outrow[16] = vsubq_s32(outrow[16], tmp);
  }



  if (maxColumn >= 12) {
    int32x4_t coeff13,coeff14,coeff15,coeff16;
    transpose_4x4(coeff13,coeff14,coeff15,coeff16, block4[0],block4[1],block4[2],block4[3]);

    int32x4_t  v13_75 = vmulq_n_s32(coeff13, 75);
    int32x4_t  v13m18 = vmulq_n_s32(coeff13,-18);
    int32x4_t  v13m89 = vmulq_n_s32(coeff13,-89);
    int32x4_t  v13m50 = vmulq_n_s32(coeff13,-50);

    int32x4_t  v15_70 = vmulq_n_s32(coeff15, 70);
    int32x4_t  v15m43 = vmulq_n_s32(coeff15,-43);
    int32x4_t  v15m87 = vmulq_n_s32(coeff15,-87);
    int32x4_t  v15_09 = vmulq_n_s32(coeff15,  9);
    int32x4_t  v15_90 = vmulq_n_s32(coeff15, 90);
    int32x4_t  v15_25 = vmulq_n_s32(coeff15, 25);
    int32x4_t  v15m80 = vmulq_n_s32(coeff15,-80);
    int32x4_t  v15m57 = vmulq_n_s32(coeff15,-57);

    int32x4_t  v_p75p70 = vaddq_s32(v15_70, v13_75);
    int32x4_t  v_m18m43 = vaddq_s32(v15m43, v13m18);
    int32x4_t  v_m89m87 = vaddq_s32(v15m87, v13m89);
    int32x4_t  v_m50p09 = vaddq_s32(v15_09, v13m50);
    int32x4_t  v_p50p90 = vsubq_s32(v15_90, v13m50);
    int32x4_t  v_p89p25 = vsubq_s32(v15_25, v13m89);
    int32x4_t  v_p18m80 = vsubq_s32(v15m80, v13m18);
    int32x4_t  v_m75m57 = vsubq_s32(v15m57, v13_75);

    int32x4_t  v_p75m57 = vaddq_s32(v15m57, v13_75);
    int32x4_t  v_m18m80 = vaddq_s32(v15m80, v13m18);
    int32x4_t  v_m89p25 = vaddq_s32(v15_25, v13m89);
    int32x4_t  v_m50p90 = vaddq_s32(v15_90, v13m50);
    int32x4_t  v_p50p09 = vsubq_s32(v15_09, v13m50);
    int32x4_t  v_p89m87 = vsubq_s32(v15m87, v13m89);
    int32x4_t  v_p18m43 = vsubq_s32(v15m43, v13m18);
    int32x4_t  v_m75p70 = vsubq_s32(v15_70, v13_75);

    outrow[ 0] = vaddq_s32( outrow[ 0], v_p75p70 );
    outrow[ 1] = vaddq_s32( outrow[ 1], v_m18m43 );
    outrow[ 2] = vaddq_s32( outrow[ 2], v_m89m87 );
    outrow[ 3] = vaddq_s32( outrow[ 3], v_m50p09 );
    outrow[ 4] = vaddq_s32( outrow[ 4], v_p50p90 );
    outrow[ 5] = vaddq_s32( outrow[ 5], v_p89p25 );
    outrow[ 6] = vaddq_s32( outrow[ 6], v_p18m80 );
    outrow[ 7] = vaddq_s32( outrow[ 7], v_m75m57 );

    outrow[ 8] = vsubq_s32( outrow[ 8], v_p75m57 );
    outrow[ 9] = vsubq_s32( outrow[ 9], v_m18m80 );
    outrow[10] = vsubq_s32( outrow[10], v_m89p25 );
    outrow[11] = vsubq_s32( outrow[11], v_m50p90 );
    outrow[12] = vsubq_s32( outrow[12], v_p50p09 );
    outrow[13] = vsubq_s32( outrow[13], v_p89m87 );
    outrow[14] = vsubq_s32( outrow[14], v_p18m43 );
    outrow[15] = vsubq_s32( outrow[15], v_m75p70 );

    outrow[16] = vsubq_s32( outrow[16], v_m75p70 );
    outrow[17] = vsubq_s32( outrow[17], v_p18m43 );
    outrow[18] = vsubq_s32( outrow[18], v_p89m87 );
    outrow[19] = vsubq_s32( outrow[19], v_p50p09 );
    outrow[20] = vsubq_s32( outrow[20], v_m50p90 );
    outrow[21] = vsubq_s32( outrow[21], v_m89p25 );
    outrow[22] = vsubq_s32( outrow[22], v_m18m80 );
    outrow[23] = vsubq_s32( outrow[23], v_p75m57 );

    outrow[24] = vaddq_s32( outrow[24], v_m75m57 );
    outrow[25] = vaddq_s32( outrow[25], v_p18m80 );
    outrow[26] = vaddq_s32( outrow[26], v_p89p25 );
    outrow[27] = vaddq_s32( outrow[27], v_p50p90 );
    outrow[28] = vaddq_s32( outrow[28], v_m50p09 );
    outrow[29] = vaddq_s32( outrow[29], v_m89m87 );
    outrow[30] = vaddq_s32( outrow[30], v_m18m43 );
    outrow[31] = vaddq_s32( outrow[31], v_p75p70 );

    int32x4_t tmp;
    int32x4_t v14_m90 = vmulq_n_s32(coeff14,-90);
    int32x4_t v16_p90 = vmulq_n_s32(coeff16, 90);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 73), vmulq_n_s32(coeff16, 67) );
    outrow[ 0] = vaddq_s32(outrow[ 0], tmp);
    outrow[31] = vsubq_s32(outrow[31], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14,-31), vmulq_n_s32(coeff16,-54) );
    outrow[ 1] = vaddq_s32(outrow[ 1], tmp);
    outrow[30] = vsubq_s32(outrow[30], tmp);

    tmp = vaddq_s32( v14_m90,                  vmulq_n_s32(coeff16,-78) );
    outrow[ 2] = vaddq_s32(outrow[ 2], tmp);
    outrow[29] = vsubq_s32(outrow[29], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14,-22), vmulq_n_s32(coeff16, 38) );
    outrow[ 3] = vaddq_s32(outrow[ 3], tmp);
    outrow[28] = vsubq_s32(outrow[28], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 78), vmulq_n_s32(coeff16, 85) );
    outrow[ 4] = vaddq_s32(outrow[ 4], tmp);
    outrow[27] = vsubq_s32(outrow[27], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 67), vmulq_n_s32(coeff16,-22) );
    outrow[ 5] = vaddq_s32(outrow[ 5], tmp);
    outrow[26] = vsubq_s32(outrow[26], tmp);

    tmp = vsubq_s32( vmulq_n_s32(coeff14,-38), v16_p90 );
    outrow[ 6] = vaddq_s32(outrow[ 6], tmp);
    outrow[25] = vsubq_s32(outrow[25], tmp);

    tmp = vaddq_s32( v14_m90,                  vmulq_n_s32(coeff16,  4) );
    outrow[ 7] = vaddq_s32(outrow[ 7], tmp);
    outrow[24] = vsubq_s32(outrow[24], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14,-13), v16_p90 );
    outrow[ 8] = vaddq_s32(outrow[ 8], tmp);
    outrow[23] = vsubq_s32(outrow[23], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 82), vmulq_n_s32(coeff16, 13) );
    outrow[ 9] = vaddq_s32(outrow[ 9], tmp);
    outrow[22] = vsubq_s32(outrow[22], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 61), vmulq_n_s32(coeff16,-88) );
    outrow[10] = vaddq_s32(outrow[10], tmp);
    outrow[21] = vsubq_s32(outrow[21], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14,-46), vmulq_n_s32(coeff16,-31) );
    outrow[11] = vaddq_s32(outrow[11], tmp);
    outrow[20] = vsubq_s32(outrow[20], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14,-88), vmulq_n_s32(coeff16, 82) );
    outrow[12] = vaddq_s32(outrow[12], tmp);
    outrow[19] = vsubq_s32(outrow[19], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, -4), vmulq_n_s32(coeff16, 46) );
    outrow[13] = vaddq_s32(outrow[13], tmp);
    outrow[18] = vsubq_s32(outrow[18], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 85), vmulq_n_s32(coeff16,-73) );
    outrow[14] = vaddq_s32(outrow[14], tmp);
    outrow[17] = vsubq_s32(outrow[17], tmp);

    tmp = vaddq_s32( vmulq_n_s32(coeff14, 54), vmulq_n_s32(coeff16,-61) );
    outrow[15] = vaddq_s32(outrow[15], tmp);
    outrow[16] = vsubq_s32(outrow[16], tmp);
  }


  int16x4_t v_row01_16 = vqrshrn_n_s32( outrow[ 0], 12 );
  int16x4_t v_row02_16 = vqrshrn_n_s32( outrow[ 1], 12 );
  int16x4_t v_row03_16 = vqrshrn_n_s32( outrow[ 2], 12 );
  int16x4_t v_row04_16 = vqrshrn_n_s32( outrow[ 3], 12 );
  int16x4_t v_row05_16 = vqrshrn_n_s32( outrow[ 4], 12 );
  int16x4_t v_row06_16 = vqrshrn_n_s32( outrow[ 5], 12 );
  int16x4_t v_row07_16 = vqrshrn_n_s32( outrow[ 6], 12 );
  int16x4_t v_row08_16 = vqrshrn_n_s32( outrow[ 7], 12 );
  int16x4_t v_row09_16 = vqrshrn_n_s32( outrow[ 8], 12 );
  int16x4_t v_row10_16 = vqrshrn_n_s32( outrow[ 9], 12 );
  int16x4_t v_row11_16 = vqrshrn_n_s32( outrow[10], 12 );
  int16x4_t v_row12_16 = vqrshrn_n_s32( outrow[11], 12 );
  int16x4_t v_row13_16 = vqrshrn_n_s32( outrow[12], 12 );
  int16x4_t v_row14_16 = vqrshrn_n_s32( outrow[13], 12 );
  int16x4_t v_row15_16 = vqrshrn_n_s32( outrow[14], 12 );
  int16x4_t v_row16_16 = vqrshrn_n_s32( outrow[15], 12 );
  int16x4_t v_row17_16 = vqrshrn_n_s32( outrow[16], 12 );
  int16x4_t v_row18_16 = vqrshrn_n_s32( outrow[17], 12 );
  int16x4_t v_row19_16 = vqrshrn_n_s32( outrow[18], 12 );
  int16x4_t v_row20_16 = vqrshrn_n_s32( outrow[19], 12 );
  int16x4_t v_row21_16 = vqrshrn_n_s32( outrow[20], 12 );
  int16x4_t v_row22_16 = vqrshrn_n_s32( outrow[21], 12 );
  int16x4_t v_row23_16 = vqrshrn_n_s32( outrow[22], 12 );
  int16x4_t v_row24_16 = vqrshrn_n_s32( outrow[23], 12 );
  int16x4_t v_row25_16 = vqrshrn_n_s32( outrow[24], 12 );
  int16x4_t v_row26_16 = vqrshrn_n_s32( outrow[25], 12 );
  int16x4_t v_row27_16 = vqrshrn_n_s32( outrow[26], 12 );
  int16x4_t v_row28_16 = vqrshrn_n_s32( outrow[27], 12 );
  int16x4_t v_row29_16 = vqrshrn_n_s32( outrow[28], 12 );
  int16x4_t v_row30_16 = vqrshrn_n_s32( outrow[29], 12 );
  int16x4_t v_row31_16 = vqrshrn_n_s32( outrow[30], 12 );
  int16x4_t v_row32_16 = vqrshrn_n_s32( outrow[31], 12 );

  {
    int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8, t9,t10,t11,t12,t13,t14,t15,t16;
    transpose_4x4(t1, t2, t3, t4,  v_row01_16, v_row02_16, v_row03_16, v_row04_16);
    transpose_4x4(t5, t6, t7, t8,  v_row05_16, v_row06_16, v_row07_16, v_row08_16);
    transpose_4x4(t9, t10,t11,t12, v_row09_16, v_row10_16, v_row11_16, v_row12_16);
    transpose_4x4(t13,t14,t15,t16, v_row13_16, v_row14_16, v_row15_16, v_row16_16);

    int16x8_t  out1L = vcombine_s16(t1, t5);
    int16x8_t  out2L = vcombine_s16(t2, t6);
    int16x8_t  out3L = vcombine_s16(t3, t7);
    int16x8_t  out4L = vcombine_s16(t4, t8);
    int16x8_t  out1R = vcombine_s16(t9, t13);
    int16x8_t  out2R = vcombine_s16(t10,t14);
    int16x8_t  out3R = vcombine_s16(t11,t15);
    int16x8_t  out4R = vcombine_s16(t12,t16);

    add_16x8_to_8x8_4rows_neon(out1L,out2L,out3L,out4L, dst,  stride);
    add_16x8_to_8x8_4rows_neon(out1R,out2R,out3R,out4R, dst+8,stride);
  }

  {
    int16x4_t  t1,t2,t3,t4,t5,t6,t7,t8, t9,t10,t11,t12,t13,t14,t15,t16;
    transpose_4x4(t1, t2, t3, t4,  v_row17_16, v_row18_16, v_row19_16, v_row20_16);
    transpose_4x4(t5, t6, t7, t8,  v_row21_16, v_row22_16, v_row23_16, v_row24_16);
    transpose_4x4(t9, t10,t11,t12, v_row25_16, v_row26_16, v_row27_16, v_row28_16);
    transpose_4x4(t13,t14,t15,t16, v_row29_16, v_row30_16, v_row31_16, v_row32_16);

    int16x8_t  out1L = vcombine_s16(t1, t5);
    int16x8_t  out2L = vcombine_s16(t2, t6);
    int16x8_t  out3L = vcombine_s16(t3, t7);
    int16x8_t  out4L = vcombine_s16(t4, t8);
    int16x8_t  out1R = vcombine_s16(t9, t13);
    int16x8_t  out2R = vcombine_s16(t10,t14);
    int16x8_t  out3R = vcombine_s16(t11,t15);
    int16x8_t  out4R = vcombine_s16(t12,t16);

    add_16x8_to_8x8_4rows_neon(out1L,out2L,out3L,out4L, dst+16,stride);
    add_16x8_to_8x8_4rows_neon(out1R,out2R,out3R,out4R, dst+24,stride);
  }
}


void idct_32x32_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,
                           int maxColumn,int maxRow)
{
  tcnt32[maxColumn][maxRow]++;

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
  else if (maxColumn<16 && maxRow<16) {
    int32x4_t row[4][32];

    idct32_v(coeffs, row[0], maxRow);

    if (maxColumn>=4) {
      idct32_v(coeffs+4, row[1], maxRow);
    }

    if (maxColumn>=8) {
      idct32_v(coeffs+8, row[2], maxRow);
    }

    if (maxColumn>=12) {
      idct32_v(coeffs+12, row[3], maxRow);
    }

    transform_32x32_add_horiz(dst,           stride, maxColumn, &row[0][ 0],&row[1][ 0],&row[2][ 0],&row[3][ 0]);
    transform_32x32_add_horiz(dst+ 4*stride, stride, maxColumn, &row[0][ 4],&row[1][ 4],&row[2][ 4],&row[3][ 4]);
    transform_32x32_add_horiz(dst+ 8*stride, stride, maxColumn, &row[0][ 8],&row[1][ 8],&row[2][ 8],&row[3][ 8]);
    transform_32x32_add_horiz(dst+12*stride, stride, maxColumn, &row[0][12],&row[1][12],&row[2][12],&row[3][12]);
    transform_32x32_add_horiz(dst+16*stride, stride, maxColumn, &row[0][16],&row[1][16],&row[2][16],&row[3][16]);
    transform_32x32_add_horiz(dst+20*stride, stride, maxColumn, &row[0][20],&row[1][20],&row[2][20],&row[3][20]);
    transform_32x32_add_horiz(dst+24*stride, stride, maxColumn, &row[0][24],&row[1][24],&row[2][24],&row[3][24]);
    transform_32x32_add_horiz(dst+28*stride, stride, maxColumn, &row[0][28],&row[1][28],&row[2][28],&row[3][28]);
  }
  else {
    transform_32x32_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}


#include <iostream>
template <int n> void dump_matrix(int m[n][n])
{
  for (int y=0;y<n;y++) {
    for(int x=0;x<n;x++) {
      std::cout << x << " " << y << " " << m[x][y] << "   #s" << n << "\n";
    }
    std::cout << "#s" << n << "\n";
  }
}


void debug_dump_dct_sizes()
{
#if 0
  dump_matrix<4>(tcnt4);
  dump_matrix<8>(tcnt8);
  dump_matrix<16>(tcnt16);
  dump_matrix<32>(tcnt32);
#endif
}
