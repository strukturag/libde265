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



void transform_16x16_add_4coeff_horiz(uint8_t* dst, ptrdiff_t stride,
                                      int32x4_t row1, int32x4_t row2, int32x4_t row3, int32x4_t row4)
{
  int32x4_t coeff1,coeff2,coeff3,coeff4;
  transpose_4x4(coeff1,coeff2,coeff3,coeff4, row1,row2,row3,row4);

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

  int16x4_t v_row01_16 = vqrshrn_n_s32( v_row01, 12 );
  int16x4_t v_row02_16 = vqrshrn_n_s32( v_row02, 12 );
  int16x4_t v_row03_16 = vqrshrn_n_s32( v_row03, 12 );
  int16x4_t v_row04_16 = vqrshrn_n_s32( v_row04, 12 );
  int16x4_t v_row05_16 = vqrshrn_n_s32( v_row05, 12 );
  int16x4_t v_row06_16 = vqrshrn_n_s32( v_row06, 12 );
  int16x4_t v_row07_16 = vqrshrn_n_s32( v_row07, 12 );
  int16x4_t v_row08_16 = vqrshrn_n_s32( v_row08, 12 );
  int16x4_t v_row09_16 = vqrshrn_n_s32( v_row09, 12 );
  int16x4_t v_row10_16 = vqrshrn_n_s32( v_row10, 12 );
  int16x4_t v_row11_16 = vqrshrn_n_s32( v_row11, 12 );
  int16x4_t v_row12_16 = vqrshrn_n_s32( v_row12, 12 );
  int16x4_t v_row13_16 = vqrshrn_n_s32( v_row13, 12 );
  int16x4_t v_row14_16 = vqrshrn_n_s32( v_row14, 12 );
  int16x4_t v_row15_16 = vqrshrn_n_s32( v_row15, 12 );
  int16x4_t v_row16_16 = vqrshrn_n_s32( v_row16, 12 );

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
  else if (maxColumn<=3 && maxRow<=3) {
    int16x4_t  coeff1 = vld1_s16(coeffs);
    int16x4_t  coeff2 = vld1_s16(coeffs+16);
    int16x4_t  coeff3 = vld1_s16(coeffs+32);
    int16x4_t  coeff4 = vld1_s16(coeffs+48);

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

    v_row01 = vrshrq_n_s32( v_row01, 7 );
    v_row02 = vrshrq_n_s32( v_row02, 7 );
    v_row03 = vrshrq_n_s32( v_row03, 7 );
    v_row04 = vrshrq_n_s32( v_row04, 7 );
    v_row05 = vrshrq_n_s32( v_row05, 7 );
    v_row06 = vrshrq_n_s32( v_row06, 7 );
    v_row07 = vrshrq_n_s32( v_row07, 7 );
    v_row08 = vrshrq_n_s32( v_row08, 7 );
    v_row09 = vrshrq_n_s32( v_row09, 7 );
    v_row10 = vrshrq_n_s32( v_row10, 7 );
    v_row11 = vrshrq_n_s32( v_row11, 7 );
    v_row12 = vrshrq_n_s32( v_row12, 7 );
    v_row13 = vrshrq_n_s32( v_row13, 7 );
    v_row14 = vrshrq_n_s32( v_row14, 7 );
    v_row15 = vrshrq_n_s32( v_row15, 7 );
    v_row16 = vrshrq_n_s32( v_row16, 7 );

    Deb(v_row01);
    Deb(v_row02);
    Deb(v_row03);
    Deb(v_row04);
    Deb(v_row05);
    Deb(v_row06);
    Deb(v_row07);
    Deb(v_row08);
    Deb(v_row09);
    Deb(v_row10);
    Deb(v_row11);
    Deb(v_row12);
    Deb(v_row13);
    Deb(v_row14);
    Deb(v_row15);
    Deb(v_row16);

    transform_16x16_add_4coeff_horiz(dst,           stride, v_row01,v_row02,v_row03,v_row04);
    transform_16x16_add_4coeff_horiz(dst+ 4*stride, stride, v_row05,v_row06,v_row07,v_row08);
    transform_16x16_add_4coeff_horiz(dst+ 8*stride, stride, v_row09,v_row10,v_row11,v_row12);
    transform_16x16_add_4coeff_horiz(dst+12*stride, stride, v_row13,v_row14,v_row15,v_row16);
  }
  else {
    transform_16x16_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
  }
}


void transform_32x32_add_4coeff_horiz(uint8_t* dst, ptrdiff_t stride,
                                      int32x4_t row1, int32x4_t row2, int32x4_t row3, int32x4_t row4)
{
  int32x4_t coeff1,coeff2,coeff3,coeff4;
  transpose_4x4(coeff1,coeff2,coeff3,coeff4, row1,row2,row3,row4);

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

  int32x4_t  v_row01 = vaddq_s32(v_pAp90, v_p90p90);
  int32x4_t  v_row02 = vaddq_s32(v_pAp87, v_p90p82);
  int32x4_t  v_row03 = vaddq_s32(v_pAp80, v_p88p67);
  int32x4_t  v_row04 = vaddq_s32(v_pAp70, v_p85p46);
  int32x4_t  v_row05 = vaddq_s32(v_pAp57, v_p82p22);
  int32x4_t  v_row06 = vaddq_s32(v_pAp43, v_p78m04);
  int32x4_t  v_row07 = vaddq_s32(v_pAp25, v_p73m31);
  int32x4_t  v_row08 = vaddq_s32(v_pAp09, v_p67m54);

  int32x4_t  v_row09 = vaddq_s32(v_pAm09, v_p61m73);
  int32x4_t  v_row10 = vaddq_s32(v_pAm25, v_p54m85);
  int32x4_t  v_row11 = vaddq_s32(v_pAm43, v_p46m90);
  int32x4_t  v_row12 = vaddq_s32(v_pAm57, v_p38m88);
  int32x4_t  v_row13 = vaddq_s32(v_pAm70, v_p31m78);
  int32x4_t  v_row14 = vaddq_s32(v_pAm80, v_p22m61);
  int32x4_t  v_row15 = vaddq_s32(v_pAm87, v_p13m38);
  int32x4_t  v_row16 = vaddq_s32(v_pAm90, v_p04m13);

  int32x4_t  v_row17 = vsubq_s32(v_pAm90, v_p04m13);
  int32x4_t  v_row18 = vsubq_s32(v_pAm87, v_p13m38);
  int32x4_t  v_row19 = vsubq_s32(v_pAm80, v_p22m61);
  int32x4_t  v_row20 = vsubq_s32(v_pAm70, v_p31m78);
  int32x4_t  v_row21 = vsubq_s32(v_pAm57, v_p38m88);
  int32x4_t  v_row22 = vsubq_s32(v_pAm43, v_p46m90);
  int32x4_t  v_row23 = vsubq_s32(v_pAm25, v_p54m85);
  int32x4_t  v_row24 = vsubq_s32(v_pAm09, v_p61m73);

  int32x4_t  v_row25 = vsubq_s32(v_pAp09, v_p67m54);
  int32x4_t  v_row26 = vsubq_s32(v_pAp25, v_p73m31);
  int32x4_t  v_row27 = vsubq_s32(v_pAp43, v_p78m04);
  int32x4_t  v_row28 = vsubq_s32(v_pAp57, v_p82p22);
  int32x4_t  v_row29 = vsubq_s32(v_pAp70, v_p85p46);
  int32x4_t  v_row30 = vsubq_s32(v_pAp80, v_p88p67);
  int32x4_t  v_row31 = vsubq_s32(v_pAp87, v_p90p82);
  int32x4_t  v_row32 = vsubq_s32(v_pAp90, v_p90p90);

  int16x4_t v_row01_16 = vqrshrn_n_s32( v_row01, 12 );
  int16x4_t v_row02_16 = vqrshrn_n_s32( v_row02, 12 );
  int16x4_t v_row03_16 = vqrshrn_n_s32( v_row03, 12 );
  int16x4_t v_row04_16 = vqrshrn_n_s32( v_row04, 12 );
  int16x4_t v_row05_16 = vqrshrn_n_s32( v_row05, 12 );
  int16x4_t v_row06_16 = vqrshrn_n_s32( v_row06, 12 );
  int16x4_t v_row07_16 = vqrshrn_n_s32( v_row07, 12 );
  int16x4_t v_row08_16 = vqrshrn_n_s32( v_row08, 12 );
  int16x4_t v_row09_16 = vqrshrn_n_s32( v_row09, 12 );
  int16x4_t v_row10_16 = vqrshrn_n_s32( v_row10, 12 );
  int16x4_t v_row11_16 = vqrshrn_n_s32( v_row11, 12 );
  int16x4_t v_row12_16 = vqrshrn_n_s32( v_row12, 12 );
  int16x4_t v_row13_16 = vqrshrn_n_s32( v_row13, 12 );
  int16x4_t v_row14_16 = vqrshrn_n_s32( v_row14, 12 );
  int16x4_t v_row15_16 = vqrshrn_n_s32( v_row15, 12 );
  int16x4_t v_row16_16 = vqrshrn_n_s32( v_row16, 12 );
  int16x4_t v_row17_16 = vqrshrn_n_s32( v_row17, 12 );
  int16x4_t v_row18_16 = vqrshrn_n_s32( v_row18, 12 );
  int16x4_t v_row19_16 = vqrshrn_n_s32( v_row19, 12 );
  int16x4_t v_row20_16 = vqrshrn_n_s32( v_row20, 12 );
  int16x4_t v_row21_16 = vqrshrn_n_s32( v_row21, 12 );
  int16x4_t v_row22_16 = vqrshrn_n_s32( v_row22, 12 );
  int16x4_t v_row23_16 = vqrshrn_n_s32( v_row23, 12 );
  int16x4_t v_row24_16 = vqrshrn_n_s32( v_row24, 12 );
  int16x4_t v_row25_16 = vqrshrn_n_s32( v_row25, 12 );
  int16x4_t v_row26_16 = vqrshrn_n_s32( v_row26, 12 );
  int16x4_t v_row27_16 = vqrshrn_n_s32( v_row27, 12 );
  int16x4_t v_row28_16 = vqrshrn_n_s32( v_row28, 12 );
  int16x4_t v_row29_16 = vqrshrn_n_s32( v_row29, 12 );
  int16x4_t v_row30_16 = vqrshrn_n_s32( v_row30, 12 );
  int16x4_t v_row31_16 = vqrshrn_n_s32( v_row31, 12 );
  int16x4_t v_row32_16 = vqrshrn_n_s32( v_row32, 12 );

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
  else if (maxColumn<=4-1 && maxRow<=8-1) {
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

    int32x4_t  v_row01 = vaddq_s32(v_pAp90, v_p90p90);
    int32x4_t  v_row02 = vaddq_s32(v_pAp87, v_p90p82);
    int32x4_t  v_row03 = vaddq_s32(v_pAp80, v_p88p67);
    int32x4_t  v_row04 = vaddq_s32(v_pAp70, v_p85p46);
    int32x4_t  v_row05 = vaddq_s32(v_pAp57, v_p82p22);
    int32x4_t  v_row06 = vaddq_s32(v_pAp43, v_p78m04);
    int32x4_t  v_row07 = vaddq_s32(v_pAp25, v_p73m31);
    int32x4_t  v_row08 = vaddq_s32(v_pAp09, v_p67m54);

    int32x4_t  v_row09 = vaddq_s32(v_pAm09, v_p61m73);
    int32x4_t  v_row10 = vaddq_s32(v_pAm25, v_p54m85);
    int32x4_t  v_row11 = vaddq_s32(v_pAm43, v_p46m90);
    int32x4_t  v_row12 = vaddq_s32(v_pAm57, v_p38m88);
    int32x4_t  v_row13 = vaddq_s32(v_pAm70, v_p31m78);
    int32x4_t  v_row14 = vaddq_s32(v_pAm80, v_p22m61);
    int32x4_t  v_row15 = vaddq_s32(v_pAm87, v_p13m38);
    int32x4_t  v_row16 = vaddq_s32(v_pAm90, v_p04m13);

    int32x4_t  v_row17 = vsubq_s32(v_pAm90, v_p04m13);
    int32x4_t  v_row18 = vsubq_s32(v_pAm87, v_p13m38);
    int32x4_t  v_row19 = vsubq_s32(v_pAm80, v_p22m61);
    int32x4_t  v_row20 = vsubq_s32(v_pAm70, v_p31m78);
    int32x4_t  v_row21 = vsubq_s32(v_pAm57, v_p38m88);
    int32x4_t  v_row22 = vsubq_s32(v_pAm43, v_p46m90);
    int32x4_t  v_row23 = vsubq_s32(v_pAm25, v_p54m85);
    int32x4_t  v_row24 = vsubq_s32(v_pAm09, v_p61m73);

    int32x4_t  v_row25 = vsubq_s32(v_pAp09, v_p67m54);
    int32x4_t  v_row26 = vsubq_s32(v_pAp25, v_p73m31);
    int32x4_t  v_row27 = vsubq_s32(v_pAp43, v_p78m04);
    int32x4_t  v_row28 = vsubq_s32(v_pAp57, v_p82p22);
    int32x4_t  v_row29 = vsubq_s32(v_pAp70, v_p85p46);
    int32x4_t  v_row30 = vsubq_s32(v_pAp80, v_p88p67);
    int32x4_t  v_row31 = vsubq_s32(v_pAp87, v_p90p82);
    int32x4_t  v_row32 = vsubq_s32(v_pAp90, v_p90p90);

    if (maxRow > 4-1) {
      int16x4_t  coeff1b = vld1_s16(coeffs   +4*32);
      int16x4_t  coeff2b = vld1_s16(coeffs+32+4*32);
      int16x4_t  coeff3b = vld1_s16(coeffs+64+4*32);
      int16x4_t  coeff4b = vld1_s16(coeffs+96+4*32);

      int32x4_t  v1b_89 = vmull_n_s16(coeff1b, 89);
      int32x4_t  v1b_75 = vmull_n_s16(coeff1b, 75);
      int32x4_t  v1b_50 = vmull_n_s16(coeff1b, 50);
      int32x4_t  v1b_18 = vmull_n_s16(coeff1b, 18);

      int32x4_t  v2b_88 = vmull_n_s16(coeff2b, 88);
      int32x4_t  v2b_67 = vmull_n_s16(coeff2b, 67);
      int32x4_t  v2b_31 = vmull_n_s16(coeff2b, 31);
      int32x4_t  v2bm13 = vmull_n_s16(coeff2b,-13);
      int32x4_t  v2bm54 = vmull_n_s16(coeff2b,-54);
      int32x4_t  v2bm82 = vmull_n_s16(coeff2b,-82);
      //int32x4_t  v2bm90 = vmull_n_s16(coeff2b, -90);   can reuse v2b_90
      int32x4_t  v2bm78 = vmull_n_s16(coeff2b,-78);
      int32x4_t  v2bm46 = vmull_n_s16(coeff2b,-46);
      int32x4_t  v2bm04 = vmull_n_s16(coeff2b, -4);
      int32x4_t  v2b_38 = vmull_n_s16(coeff2b, 38);
      int32x4_t  v2b_73 = vmull_n_s16(coeff2b, 73);
      int32x4_t  v2b_90 = vmull_n_s16(coeff2b, 90);
      int32x4_t  v2b_85 = vmull_n_s16(coeff2b, 85);
      int32x4_t  v2b_61 = vmull_n_s16(coeff2b, 61);
      int32x4_t  v2b_22 = vmull_n_s16(coeff2b, 22);

      int32x4_t  v3b_87 = vmull_n_s16(coeff3b, 87);
      int32x4_t  v3b_57 = vmull_n_s16(coeff3b, 57);
      int32x4_t  v3b_09 = vmull_n_s16(coeff3b,  9);
      int32x4_t  v3bm43 = vmull_n_s16(coeff3b,-43);
      int32x4_t  v3bm80 = vmull_n_s16(coeff3b,-80);
      int32x4_t  v3bm90 = vmull_n_s16(coeff3b,-90);
      int32x4_t  v3bm70 = vmull_n_s16(coeff3b,-70);
      int32x4_t  v3bm25 = vmull_n_s16(coeff3b,-25);

      int32x4_t  v4b_85 = vmull_n_s16(coeff4b, 85);
      int32x4_t  v4b_46 = vmull_n_s16(coeff4b, 46);
      int32x4_t  v4bm13 = vmull_n_s16(coeff4b,-13);
      int32x4_t  v4bm67 = vmull_n_s16(coeff4b,-67);
      int32x4_t  v4bm90 = vmull_n_s16(coeff4b,-90);
      int32x4_t  v4bm73 = vmull_n_s16(coeff4b,-73);
      int32x4_t  v4bm22 = vmull_n_s16(coeff4b,-22);
      int32x4_t  v4b_38 = vmull_n_s16(coeff4b, 38);
      int32x4_t  v4b_82 = vmull_n_s16(coeff4b, 82);
      int32x4_t  v4b_88 = vmull_n_s16(coeff4b, 88);
      int32x4_t  v4b_54 = vmull_n_s16(coeff4b, 54);
      int32x4_t  v4bm04 = vmull_n_s16(coeff4b, -4);
      int32x4_t  v4bm61 = vmull_n_s16(coeff4b,-61);
      //int32x4_t  v4bm90 = vmull_n_s16(coeff4b, 85);  duplicate
      int32x4_t  v4bm78 = vmull_n_s16(coeff4b,-78);
      int32x4_t  v4bm31 = vmull_n_s16(coeff4b,-31);


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

      v_row01 = vaddq_s32(v_row01, vb_p89p87);
      v_row02 = vaddq_s32(v_row02, vb_p75p57);
      v_row03 = vaddq_s32(v_row03, vb_p50p09);
      v_row04 = vaddq_s32(v_row04, vb_p18m43);
      v_row05 = vaddq_s32(v_row05, vb_m18m80);
      v_row06 = vaddq_s32(v_row06, vb_m50m90);
      v_row07 = vaddq_s32(v_row07, vb_m75m70);
      v_row08 = vaddq_s32(v_row08, vb_m89m25);
      v_row09 = vsubq_s32(v_row09, vb_m89p25_m);
      v_row10 = vsubq_s32(v_row10, vb_m75p70_m);
      v_row11 = vsubq_s32(v_row11, vb_m50p90_m);
      v_row12 = vsubq_s32(v_row12, vb_m18p80_m);
      v_row13 = vaddq_s32(v_row13, vb_p18p43);
      v_row14 = vaddq_s32(v_row14, vb_p50m09);
      v_row15 = vaddq_s32(v_row15, vb_p75m57);
      v_row16 = vaddq_s32(v_row16, vb_p89m87);
      v_row17 = vaddq_s32(v_row17, vb_p89m87);
      v_row18 = vaddq_s32(v_row18, vb_p75m57);
      v_row19 = vaddq_s32(v_row19, vb_p50m09);
      v_row20 = vaddq_s32(v_row20, vb_p18p43);
      v_row21 = vsubq_s32(v_row21, vb_m18p80_m);
      v_row22 = vsubq_s32(v_row22, vb_m50p90_m);
      v_row23 = vsubq_s32(v_row23, vb_m75p70_m);
      v_row24 = vsubq_s32(v_row24, vb_m89p25_m);
      v_row25 = vaddq_s32(v_row25, vb_m89m25);
      v_row26 = vaddq_s32(v_row26, vb_m75m70);
      v_row27 = vaddq_s32(v_row27, vb_m50m90);
      v_row28 = vaddq_s32(v_row28, vb_m18m80);
      v_row29 = vaddq_s32(v_row29, vb_p18m43);
      v_row30 = vaddq_s32(v_row30, vb_p50p09);
      v_row31 = vaddq_s32(v_row31, vb_p75p57);
      v_row32 = vaddq_s32(v_row32, vb_p89p87);

      int32x4_t tmp;
      tmp = vaddq_s32(v2b_88, v4b_85);
      v_row01 = vaddq_s32(v_row01, tmp);
      v_row32 = vsubq_s32(v_row32, tmp);

      tmp = vaddq_s32(v2b_67, v4b_46);
      v_row02 = vaddq_s32(v_row02, tmp);
      v_row31 = vsubq_s32(v_row31, tmp);

      tmp = vaddq_s32(v2b_31, v4bm13);
      v_row03 = vaddq_s32(v_row03, tmp);
      v_row30 = vsubq_s32(v_row30, tmp);

      tmp = vaddq_s32(v2bm13, v4bm67);
      v_row04 = vaddq_s32(v_row04, tmp);
      v_row29 = vsubq_s32(v_row29, tmp);

      tmp = vaddq_s32(v2bm54, v4bm90);
      v_row05 = vaddq_s32(v_row05, tmp);
      v_row28 = vsubq_s32(v_row28, tmp);

      tmp = vaddq_s32(v2bm82, v4bm73);
      v_row06 = vaddq_s32(v_row06, tmp);
      v_row27 = vsubq_s32(v_row27, tmp);

      tmp = vsubq_s32(v4bm22, v2b_90);
      v_row07 = vaddq_s32(v_row07, tmp);
      v_row26 = vsubq_s32(v_row26, tmp);

      tmp = vaddq_s32(v2bm78, v4b_38);
      v_row08 = vaddq_s32(v_row08, tmp);
      v_row25 = vsubq_s32(v_row25, tmp);

      tmp = vaddq_s32(v2bm46, v4b_82);
      v_row09 = vaddq_s32(v_row09, tmp);
      v_row24 = vsubq_s32(v_row24, tmp);

      tmp = vaddq_s32(v2bm04, v4b_88);
      v_row10 = vaddq_s32(v_row10, tmp);
      v_row23 = vsubq_s32(v_row23, tmp);

      tmp = vaddq_s32(v2b_38, v4b_54);
      v_row11 = vaddq_s32(v_row11, tmp);
      v_row22 = vsubq_s32(v_row22, tmp);

      tmp = vaddq_s32(v2b_73, v4bm04);
      v_row12 = vaddq_s32(v_row12, tmp);
      v_row21 = vsubq_s32(v_row21, tmp);

      tmp = vaddq_s32(v2b_90, v4bm61);
      v_row13 = vaddq_s32(v_row13, tmp);
      v_row20 = vsubq_s32(v_row20, tmp);

      tmp = vaddq_s32(v2b_85, v4bm90);
      v_row14 = vaddq_s32(v_row14, tmp);
      v_row19 = vsubq_s32(v_row19, tmp);

      tmp = vaddq_s32(v2b_61, v4bm78);
      v_row15 = vaddq_s32(v_row15, tmp);
      v_row18 = vsubq_s32(v_row18, tmp);

      tmp = vaddq_s32(v2b_22, v4bm31);
      v_row16 = vaddq_s32(v_row16, tmp);
      v_row17 = vsubq_s32(v_row17, tmp);
    }


    v_row01 = vrshrq_n_s32( v_row01, 7 );
    v_row02 = vrshrq_n_s32( v_row02, 7 );
    v_row03 = vrshrq_n_s32( v_row03, 7 );
    v_row04 = vrshrq_n_s32( v_row04, 7 );
    v_row05 = vrshrq_n_s32( v_row05, 7 );
    v_row06 = vrshrq_n_s32( v_row06, 7 );
    v_row07 = vrshrq_n_s32( v_row07, 7 );
    v_row08 = vrshrq_n_s32( v_row08, 7 );
    v_row09 = vrshrq_n_s32( v_row09, 7 );
    v_row10 = vrshrq_n_s32( v_row10, 7 );
    v_row11 = vrshrq_n_s32( v_row11, 7 );
    v_row12 = vrshrq_n_s32( v_row12, 7 );
    v_row13 = vrshrq_n_s32( v_row13, 7 );
    v_row14 = vrshrq_n_s32( v_row14, 7 );
    v_row15 = vrshrq_n_s32( v_row15, 7 );
    v_row16 = vrshrq_n_s32( v_row16, 7 );

    v_row17 = vrshrq_n_s32( v_row17, 7 );
    v_row18 = vrshrq_n_s32( v_row18, 7 );
    v_row19 = vrshrq_n_s32( v_row19, 7 );
    v_row20 = vrshrq_n_s32( v_row20, 7 );
    v_row21 = vrshrq_n_s32( v_row21, 7 );
    v_row22 = vrshrq_n_s32( v_row22, 7 );
    v_row23 = vrshrq_n_s32( v_row23, 7 );
    v_row24 = vrshrq_n_s32( v_row24, 7 );
    v_row25 = vrshrq_n_s32( v_row25, 7 );
    v_row26 = vrshrq_n_s32( v_row26, 7 );
    v_row27 = vrshrq_n_s32( v_row27, 7 );
    v_row28 = vrshrq_n_s32( v_row28, 7 );
    v_row29 = vrshrq_n_s32( v_row29, 7 );
    v_row30 = vrshrq_n_s32( v_row30, 7 );
    v_row31 = vrshrq_n_s32( v_row31, 7 );
    v_row32 = vrshrq_n_s32( v_row32, 7 );

    transform_32x32_add_4coeff_horiz(dst,           stride, v_row01,v_row02,v_row03,v_row04);
    transform_32x32_add_4coeff_horiz(dst+ 4*stride, stride, v_row05,v_row06,v_row07,v_row08);
    transform_32x32_add_4coeff_horiz(dst+ 8*stride, stride, v_row09,v_row10,v_row11,v_row12);
    transform_32x32_add_4coeff_horiz(dst+12*stride, stride, v_row13,v_row14,v_row15,v_row16);
    transform_32x32_add_4coeff_horiz(dst+16*stride, stride, v_row17,v_row18,v_row19,v_row20);
    transform_32x32_add_4coeff_horiz(dst+20*stride, stride, v_row21,v_row22,v_row23,v_row24);
    transform_32x32_add_4coeff_horiz(dst+24*stride, stride, v_row25,v_row26,v_row27,v_row28);
    transform_32x32_add_4coeff_horiz(dst+28*stride, stride, v_row29,v_row30,v_row31,v_row32);

    //transform_32x32_add_8_fallback(dst,coeffs,stride,maxColumn,maxRow);
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
