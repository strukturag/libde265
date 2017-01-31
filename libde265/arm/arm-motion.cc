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


#define D 1

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


template <bool chroma, bool exact>
inline void put_pred_8_neon_intern(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                                   const int16_t __restrict__ *src, ptrdiff_t srcstride,
                                   int width, int height)
{
  //printf("PUT-PRED %d %d\n",width,height);

  int16x8_t  zero16 = vdupq_n_s16(0);

  while (width >= 16) {
    //uint16x8_t rnd = vmovq_n_u16(32);

    for (int y=0;y<height;y++) {
      int16x8_t input1 = vld1q_s16(src+y*srcstride);
      int16x8_t input2 = vld1q_s16(src+y*srcstride + 8);

      if (exact) {
        input1 = vmaxq_s16(input1, zero16);
        input2 = vmaxq_s16(input2, zero16);
      }

      uint8x8_t  output1 = vqrshrn_n_u16( vreinterpretq_u16_s16(input1), 6);
      uint8x8_t  output2 = vqrshrn_n_u16( vreinterpretq_u16_s16(input2), 6);

      uint8x16_t combined_output = vcombine_u8(output1, output2);

      vst1q_u8((uint8_t*)(dst+dststride*y), combined_output);
    }

    width -= 16;
    src += 16;
    dst += 16;
  }

  if (width >= 8) {
    for (int y=0;y<height;y++) {
      int16x8_t input_signed = vld1q_s16(src+y*srcstride);
      if (exact) { input_signed = vmaxq_s16(input_signed, zero16); }
      uint16x8_t input = vreinterpretq_u16_s16( input_signed );
      uint8x8_t output = vqrshrn_n_u16(input, 6);

      vst1_u8((uint8_t*)(dst+dststride*y), output);
    }

    width -= 8;
    src += 8;
    dst += 8;
  }

  if (width >= 4) {
    for (int y=0;y<height;y++) {
      int16x4_t input = vld1_s16(src+y*srcstride);
      int16x8_t extended_input = vcombine_s16(input, input); // extend to 128 bit
      // clipping to zero could also be done on 64bit, but we do not have a 64bit zero constant available
      if (exact) { extended_input = vmaxq_s16(extended_input, zero16); }
      uint8x8_t  output = vqrshrn_n_u16( vreinterpretq_u16_s16(extended_input), 6);

      uint32x2_t output32 = vreinterpret_u32_u8(output);
      vst1_lane_u32((uint32_t*)(dst+dststride*y), output32, 0);
    }

    width -= 4;
    src += 4;
    dst += 4;
  }


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


void put_pred_8_neon(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                     const int16_t __restrict__ *src, ptrdiff_t srcstride,
                     int width, int height)
{
  put_pred_8_neon_intern<true,true>(dst,dststride, src,srcstride, width,height);
}


void put_bipred_8_neon(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                       const int16_t __restrict__ *src1,
                       const int16_t __restrict__ *src2, ptrdiff_t srcstride,
                       int width, int height)
{
  const bool exact = true;
  //printf("PUT-BI-PRED %d %d\n",width,height);

  int16x8_t  zero16 = vdupq_n_s16(0);

#if 1
  while (width >= 16) {
    for (int y=0;y<height;y++) {
      int16x8_t input1a = vld1q_s16(src1+y*srcstride);
      int16x8_t input2a = vld1q_s16(src2+y*srcstride);
      int16x8_t input1b = vld1q_s16(src1+y*srcstride+8);
      int16x8_t input2b = vld1q_s16(src2+y*srcstride+8);

      int16x8_t suma    = vqaddq_s16(input1a, input2a);
      int16x8_t sumb    = vqaddq_s16(input1b, input2b);
      if (exact) {
        suma = vmaxq_s16(suma, zero16);
        sumb = vmaxq_s16(sumb, zero16);
      }
      uint8x8_t  outputa = vqrshrn_n_u16( vreinterpretq_u16_s16(suma), 7);
      uint8x8_t  outputb = vqrshrn_n_u16( vreinterpretq_u16_s16(sumb), 7);

      uint8x16_t combined_output = vcombine_u8(outputa, outputb);

      vst1q_u8(dst+dststride*y, combined_output);
    }

    width -= 16;
    src1 += 16;
    src2 += 16;
    dst += 16;
  }
#endif

#if 1
  if (width >= 8) {
    for (int y=0;y<height;y++) {
      int16x8_t input1 = vld1q_s16(src1+y*srcstride);
      int16x8_t input2 = vld1q_s16(src2+y*srcstride);
      int16x8_t sum    = vqaddq_s16(input1, input2);
      if (exact) { sum = vmaxq_s16(sum, zero16); }
      uint8x8_t output = vqrshrn_n_u16(vreinterpretq_u16_s16(sum), 7);
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
    for (int y=0;y<height;y++) {
      int16x4_t input1 = vld1_s16(src1+y*srcstride);
      int16x4_t input2 = vld1_s16(src2+y*srcstride);
      int16x4_t sum    = vqadd_s16(input1, input2);
      int16x8_t extended_sum = vcombine_s16(sum,sum); // extend to 128 bit
      if (exact) { extended_sum = vmaxq_s16(extended_sum, zero16); }
      uint8x8_t  output = vqrshrn_n_u16( vreinterpretq_u16_s16(extended_sum), 7);

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
void mc_noshift_8_neon(int16_t *dst, ptrdiff_t dststride,
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


void mc_noshift_8_luma_neon(int16_t *dst, ptrdiff_t dststride,
                                const uint8_t *src, ptrdiff_t srcstride,
                                int width, int height,
                                int16_t* mcbuffer)
{
  mc_noshift_8_neon<false>(dst,dststride, src,srcstride, width,height);
}


void mc_noshift_8_chroma_neon(int16_t *dst, ptrdiff_t dststride,
                                  const uint8_t *src, ptrdiff_t srcstride,
                                  int width, int height, int mx,
                                  int my, int16_t* mcbuffer)
{
  mc_noshift_8_neon<true>(dst,dststride, src,srcstride, width,height);
}


static uint8_t qpel_filter[3][16] = {
  // 1/4 pel shift
  { 0,4, 0,58,17,0,1,0,   // plus
    1,0,10, 0, 0,5,0,0 }, // minus
  // 2/4 pel shift
  { 0,4, 0,40,40, 0,4,0,   // plus
    1,0,11, 0, 0,11,0,1 }, // minus
  // 3/4 pel shift
  { 0,1,0,17,58, 0,4,0,   // plus
    0,0,5, 0, 0,10,0,1 }  // minus
};

template<int filterIdx>
void mc_qpel_h_8_neon(int16_t *dst, ptrdiff_t dststride,
                      const uint8_t *src, ptrdiff_t srcstride,
                      int width, int height,
                      int16_t* mcbuffer)
{
  while (width>=4) {
    uint8x8_t filter_plus  = vld1_u8(qpel_filter[filterIdx-1]);
    uint8x8_t filter_minus = vld1_u8(qpel_filter[filterIdx-1]+8);

    for (int y=0;y<height;y++) {
      uint8x8_t input_left  = vld1_u8(src+y*srcstride -3);
      uint8x8_t input_right = vld1_u8(src+y*srcstride -3+8);

      uint16x8_t acc0p = vmull_u8(       input_left, filter_plus);
      uint16x8_t acc0_ = vmlsl_u8(acc0p, input_left, filter_minus);
      int16x8_t  acc0  = vreinterpretq_s16_u16(acc0_);
      int16x4_t  sum0 = vpadd_s16(vget_low_s16(acc0), vget_high_s16(acc0));

      uint8x8_t input1 = vext_u8(input_left, input_right, 1);
      uint16x8_t acc1p = vmull_u8(       input1, filter_plus);
      uint16x8_t acc1_ = vmlsl_u8(acc1p, input1, filter_minus);
      int16x8_t  acc1  = vreinterpretq_s16_u16(acc1_);
      int16x4_t  sum1 = vpadd_s16(vget_low_s16(acc1), vget_high_s16(acc1));

      uint8x8_t input2 = vext_u8(input_left, input_right, 2);
      uint16x8_t acc2p = vmull_u8(       input2, filter_plus);
      uint16x8_t acc2_ = vmlsl_u8(acc2p, input2, filter_minus);
      int16x8_t  acc2  = vreinterpretq_s16_u16(acc2_);
      int16x4_t  sum2 = vpadd_s16(vget_low_s16(acc2), vget_high_s16(acc2));

      uint8x8_t input3 = vext_u8(input_left, input_right, 3);
      uint16x8_t acc3p = vmull_u8(       input3, filter_plus);
      uint16x8_t acc3_ = vmlsl_u8(acc3p, input3, filter_minus);
      int16x8_t  acc3  = vreinterpretq_s16_u16(acc3_);
      int16x4_t  sum3 = vpadd_s16(vget_low_s16(acc3), vget_high_s16(acc3));

      int16x4_t  sum01 = vpadd_s16(sum0,sum1);
      int16x4_t  sum23 = vpadd_s16(sum2,sum3);
      int16x4_t  sum0123 = vpadd_s16(sum01,sum23);

      vst1_s16((dst+dststride*y), sum0123);
    }

    width-=4;
    src+=4;
    dst+=4;
  }

  assert(width==0);
}


void mc_qpel_h1_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer)
{
  mc_qpel_h_8_neon<1>(dst,dststride, src,srcstride, width,height,mcbuffer);
}

void mc_qpel_h2_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer)
{
  mc_qpel_h_8_neon<2>(dst,dststride, src,srcstride, width,height,mcbuffer);
}

void mc_qpel_h3_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer)
{
  mc_qpel_h_8_neon<3>(dst,dststride, src,srcstride, width,height,mcbuffer);
}




#include "fallback-motion.h"

static int8_t qpel_v_filter[3][8] = {
  // 1/4 pel shift
  { -1,4,-10,58,17,-5,1,0 },

  // 2/4 pel shift
  { -1,4,-11,40,40,-11,4,-1 },

  // 3/4 pel shift
  { 0,1,-5,17,58,-10,4,-1 }
};

template<int filterIdx>
void mc_qpel_v_8_neon(int16_t *dst, ptrdiff_t dststride,
                      const uint8_t *src, ptrdiff_t srcstride,
                      int width, int height,
                      int16_t* mcbuffer)
{
  //printf("%d x %d [Vshift=%d]\n",width,height,filterIdx);

  if (width & 7) {
    switch (filterIdx) {
    case 1: put_qpel_0_1_fallback(dst,dststride, src,srcstride, width,height, mcbuffer); break;
    case 2: put_qpel_0_2_fallback(dst,dststride, src,srcstride, width,height, mcbuffer); break;
    case 3: put_qpel_0_3_fallback(dst,dststride, src,srcstride, width,height, mcbuffer); break;
    }

    if (width & 7) return;
  }

  while (width>=8) {

    for (int y=0;y<height;y++) {
      uint8x8_t  row_m3     = vld1_u8(src+(y-3)*srcstride);
      uint16x8_t row_m3_16  = vmovl_u8(row_m3);
      int16x8_t  row_m3_s16 = vreinterpretq_s16_u16(row_m3_16);

      row_m3_s16 = vmulq_n_s16(row_m3_s16, qpel_v_filter[filterIdx-1][0]);

      uint8x8_t  row_m2     = vld1_u8(src+(y-2)*srcstride);
      uint16x8_t row_m2_16  = vmovl_u8(row_m2);
      int16x8_t  row_m2_s16 = vreinterpretq_s16_u16(row_m2_16);

      row_m2_s16 = vmulq_n_s16(row_m2_s16, qpel_v_filter[filterIdx-1][1]);

      uint8x8_t  row_m1     = vld1_u8(src+(y-1)*srcstride);
      uint16x8_t row_m1_16  = vmovl_u8(row_m1);
      int16x8_t  row_m1_s16 = vreinterpretq_s16_u16(row_m1_16);

      row_m1_s16 = vmulq_n_s16(row_m1_s16, qpel_v_filter[filterIdx-1][2]);

      uint8x8_t  row_p0     = vld1_u8(src+(y-0)*srcstride);
      uint16x8_t row_p0_16  = vmovl_u8(row_p0);
      int16x8_t  row_p0_s16 = vreinterpretq_s16_u16(row_p0_16);

      row_p0_s16 = vmulq_n_s16(row_p0_s16, qpel_v_filter[filterIdx-1][3]);

      uint8x8_t  row_p1     = vld1_u8(src+(y+1)*srcstride);
      uint16x8_t row_p1_16  = vmovl_u8(row_p1);
      int16x8_t  row_p1_s16 = vreinterpretq_s16_u16(row_p1_16);

      row_p1_s16 = vmulq_n_s16(row_p1_s16, qpel_v_filter[filterIdx-1][4]);

      uint8x8_t  row_p2     = vld1_u8(src+(y+2)*srcstride);
      uint16x8_t row_p2_16  = vmovl_u8(row_p2);
      int16x8_t  row_p2_s16 = vreinterpretq_s16_u16(row_p2_16);

      row_p2_s16 = vmulq_n_s16(row_p2_s16, qpel_v_filter[filterIdx-1][5]);

      uint8x8_t  row_p3     = vld1_u8(src+(y+3)*srcstride);
      uint16x8_t row_p3_16  = vmovl_u8(row_p3);
      int16x8_t  row_p3_s16 = vreinterpretq_s16_u16(row_p3_16);

      row_p3_s16 = vmulq_n_s16(row_p3_s16, qpel_v_filter[filterIdx-1][6]);

      uint8x8_t  row_p4     = vld1_u8(src+(y+4)*srcstride);
      uint16x8_t row_p4_16  = vmovl_u8(row_p4);
      int16x8_t  row_p4_s16 = vreinterpretq_s16_u16(row_p4_16);

      row_p4_s16 = vmulq_n_s16(row_p4_s16, qpel_v_filter[filterIdx-1][7]);

      int16x8_t rowsum = vaddq_s16(row_m3_s16, row_m2_s16);
      rowsum = vaddq_s16(rowsum, row_m1_s16);
      rowsum = vaddq_s16(rowsum, row_p0_s16);
      rowsum = vaddq_s16(rowsum, row_p1_s16);
      rowsum = vaddq_s16(rowsum, row_p2_s16);
      rowsum = vaddq_s16(rowsum, row_p3_s16);
      rowsum = vaddq_s16(rowsum, row_p4_s16);

      /*
      Deb(rowsum);
      int16x8_t ref = vld1q_s16(dst+y*dststride);
      Deb(ref);
      */

      vst1q_s16(dst+y*dststride, rowsum);
    }

    width-=8;
    src+=8;
    dst+=8;
  }

  //assert(width==0);
}


void mc_qpel_v1_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer)
{
  mc_qpel_v_8_neon<1>(dst,dststride, src,srcstride, width,height,mcbuffer);
}

void mc_qpel_v2_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer)
{
  mc_qpel_v_8_neon<2>(dst,dststride, src,srcstride, width,height,mcbuffer);
}

void mc_qpel_v3_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer)
{
  mc_qpel_v_8_neon<3>(dst,dststride, src,srcstride, width,height,mcbuffer);
}
