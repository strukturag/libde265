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

#include "fallback-motion.h"


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

// applies the filter to the input and returns a 16x4, all 4 values should still be added
inline int16x4_t hfilter_qpel_sum(uint8x8_t filter_plus,
                                  uint8x8_t filter_minus,
                                  uint8x8_t input)
{
  uint16x8_t acc0p = vmull_u8(       input, filter_plus);
  uint16x8_t acc0_ = vmlsl_u8(acc0p, input, filter_minus);
  int16x8_t  acc0  = vreinterpretq_s16_u16(acc0_);
  return vpadd_s16(vget_low_s16(acc0), vget_high_s16(acc0));
}

inline int16x4_t hfilter_qpel_4x(uint8x8_t filter_plus,
                                 uint8x8_t filter_minus,
                                 uint8x8_t input_left,
                                 uint8x8_t input_right)
{
  int16x4_t  sum0 = hfilter_qpel_sum(filter_plus,filter_minus,  input_left);

  uint8x8_t input1 = vext_u8(input_left, input_right, 1);
  int16x4_t  sum1 = hfilter_qpel_sum(filter_plus,filter_minus,  input1);

  uint8x8_t input2 = vext_u8(input_left, input_right, 2);
  int16x4_t  sum2 = hfilter_qpel_sum(filter_plus,filter_minus,  input2);

  uint8x8_t input3 = vext_u8(input_left, input_right, 3);
  int16x4_t  sum3 = hfilter_qpel_sum(filter_plus,filter_minus,  input3);

  int16x4_t  sum01 = vpadd_s16(sum0,sum1);
  int16x4_t  sum23 = vpadd_s16(sum2,sum3);
  int16x4_t  sum0123 = vpadd_s16(sum01,sum23);

  return sum0123;
}


inline int16x8_t hfilter_qpel_8x(uint8x8_t filter_plus,
                                 uint8x8_t filter_minus,
                                 uint8x8_t input_left,
                                 uint8x8_t input_right)
{
  int16x4_t  sum0 = hfilter_qpel_sum(filter_plus,filter_minus,  input_left);

  uint8x8_t input1 = vext_u8(input_left, input_right, 1);
  int16x4_t  sum1 = hfilter_qpel_sum(filter_plus,filter_minus,  input1);

  uint8x8_t input2 = vext_u8(input_left, input_right, 2);
  int16x4_t  sum2 = hfilter_qpel_sum(filter_plus,filter_minus,  input2);

  uint8x8_t input3 = vext_u8(input_left, input_right, 3);
  int16x4_t  sum3 = hfilter_qpel_sum(filter_plus,filter_minus,  input3);

  uint8x8_t input4 = vext_u8(input_left, input_right, 4);
  int16x4_t  sum4 = hfilter_qpel_sum(filter_plus,filter_minus,  input4);

  uint8x8_t input5 = vext_u8(input_left, input_right, 5);
  int16x4_t  sum5 = hfilter_qpel_sum(filter_plus,filter_minus,  input5);

  uint8x8_t input6 = vext_u8(input_left, input_right, 6);
  int16x4_t  sum6 = hfilter_qpel_sum(filter_plus,filter_minus,  input6);

  uint8x8_t input7 = vext_u8(input_left, input_right, 7);
  int16x4_t  sum7 = hfilter_qpel_sum(filter_plus,filter_minus,  input7);

  int16x4_t  sum01 = vpadd_s16(sum0,sum1);
  int16x4_t  sum23 = vpadd_s16(sum2,sum3);
  int16x4_t  sum0123 = vpadd_s16(sum01,sum23);

  int16x4_t  sum45 = vpadd_s16(sum4,sum5);
  int16x4_t  sum67 = vpadd_s16(sum6,sum7);
  int16x4_t  sum4567 = vpadd_s16(sum45,sum67);

  int16x8_t  sum01234567 = vcombine_s16(sum0123, sum4567);

  return sum01234567;
}


template<int filterIdx>
void mc_qpel_h_8_neon(int16_t *dst, ptrdiff_t dststride,
                      const uint8_t *src, ptrdiff_t srcstride,
                      int width, int height,
                      int16_t* mcbuffer)
{
  uint8x8_t filter_plus  = vld1_u8(qpel_filter[filterIdx-1]);
  uint8x8_t filter_minus = vld1_u8(qpel_filter[filterIdx-1]+8);

  while (width>=8) {
    for (int y=0;y<height;y++) {
      uint8x8_t input_left  = vld1_u8(src+y*srcstride -3);
      uint8x8_t input_right = vld1_u8(src+y*srcstride -3+8);

      int16x8_t result = hfilter_qpel_8x(filter_plus,filter_minus, input_left,input_right);

      vst1q_s16((dst+dststride*y), result);
    }

    width-=8;
    src+=8;
    dst+=8;
  }


  if (width>=4) {
    for (int y=0;y<height;y++) {
      uint8x8_t input_left  = vld1_u8(src+y*srcstride -3);
      uint8x8_t input_right = vld1_u8(src+y*srcstride -3+8);

      int16x4_t result = hfilter_qpel_4x(filter_plus,filter_minus, input_left,input_right);

      vst1_s16((dst+dststride*y), result);
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




inline int16x8_t load_u8_to_s16(const uint8_t* src)
{
  uint8x8_t  in8 = vld1_u8(src);
  uint16x8_t in16= vmovl_u8(in8);
  return vreinterpretq_s16_u16(in16);
}


// { -1,4,-10,58,17,-5,1,0 }
inline int16x8_t vfilter_1qpel_8x(int16x8_t row0, int16x8_t row1, int16x8_t row2, int16x8_t row3,
                                  int16x8_t row4, int16x8_t row5, int16x8_t row6, int16x8_t row7)
{
  int16x8_t m1,m2,m3,m4,m5;

  m1 = vshlq_n_s16(row1, 2); // *4
  m2 = vmulq_n_s16(row2, -10);
  m3 = vmulq_n_s16(row3, 58);
  m4 = vmulq_n_s16(row4, 17);
  m5 = vmulq_n_s16(row5, -5);

  int16x8_t t1 = vsubq_s16(m1,row0);
  int16x8_t t2 = vaddq_s16(m2,m3);
  int16x8_t t3 = vaddq_s16(m4,m5);
  int16x8_t t4 = row6;

  int16x8_t u1 = vaddq_s16(t1,t2);
  int16x8_t u2 = vaddq_s16(t3,t4);

  return vaddq_s16(u1,u2);
}


//  { -1,4,-11,40,40,-11,4,-1 }
inline int16x8_t vfilter_2qpel_8x(int16x8_t row0, int16x8_t row1, int16x8_t row2, int16x8_t row3,
                                  int16x8_t row4, int16x8_t row5, int16x8_t row6, int16x8_t row7)
{
  int16x8_t m0,m1,m2,m3;

  m0 = vaddq_s16(row0,row7);
  m1 = vaddq_s16(row1,row6);
  m2 = vaddq_s16(row2,row5);
  m3 = vaddq_s16(row3,row4);

  m1 = vshlq_n_s16(m1, 2); // *4
  m2 = vmulq_n_s16(m2, 11);
  m3 = vmulq_n_s16(m3, 40);

  m0 = vaddq_s16(m0,m2);
  m1 = vaddq_s16(m1,m3);
  return vsubq_s16(m1,m0);
}



// { -1,4,-10,58,17,-5,1,0 }
inline int16x8_t vfilter_1qpel_8x_32bit(int16x8_t row0,int16x8_t row1,int16x8_t row2,int16x8_t row3,
                                        int16x8_t row4,int16x8_t row5,int16x8_t row6,int16x8_t row7)
{
  int16x8_t m1,m2,m3,m4,m5;

  int32x4_t sumlow, sumhigh;

  sumlow  = vmovl_s16(vget_low_s16(row6));
  sumhigh = vmovl_s16(vget_high_s16(row6));

  sumlow  = vmlal_n_s16(sumlow,  vget_low_s16 (row0), -1);
  sumhigh = vmlal_n_s16(sumhigh, vget_high_s16(row0), -1);

  sumlow  = vmlal_n_s16(sumlow,  vget_low_s16 (row1), 4);
  sumhigh = vmlal_n_s16(sumhigh, vget_high_s16(row1), 4);

  sumlow  = vmlal_n_s16(sumlow,  vget_low_s16 (row2), -10);
  sumhigh = vmlal_n_s16(sumhigh, vget_high_s16(row2), -10);

  sumlow  = vmlal_n_s16(sumlow,  vget_low_s16 (row3), 58);
  sumhigh = vmlal_n_s16(sumhigh, vget_high_s16(row3), 58);

  sumlow  = vmlal_n_s16(sumlow,  vget_low_s16 (row4), 17);
  sumhigh = vmlal_n_s16(sumhigh, vget_high_s16(row4), 17);

  sumlow  = vmlal_n_s16(sumlow,  vget_low_s16 (row5), -5);
  sumhigh = vmlal_n_s16(sumhigh, vget_high_s16(row5), -5);

  int16x4_t sum16low  = vshrn_n_s32(sumlow,  6);
  int16x4_t sum16high = vshrn_n_s32(sumhigh, 6);

  return vcombine_s16(sum16low, sum16high);
}


//  { -1,4,-11,40,40,-11,4,-1 }
inline int16x8_t vfilter_2qpel_8x_32bit(int16x8_t row0,int16x8_t row1,int16x8_t row2,int16x8_t row3,
                                        int16x8_t row4,int16x8_t row5,int16x8_t row6,int16x8_t row7)
{
  // we have to add inputs with the same coefficients in 32bit because it may not fit into s16

  int32x4_t m0low  = vaddl_s16(vget_low_s16(row0),  vget_low_s16(row7));
  int32x4_t m0high = vaddl_s16(vget_high_s16(row0), vget_high_s16(row7));

  int32x4_t m1low  = vaddl_s16(vget_low_s16(row1),  vget_low_s16(row6));
  int32x4_t m1high = vaddl_s16(vget_high_s16(row1), vget_high_s16(row6));

  int32x4_t m2low  = vaddl_s16(vget_low_s16(row2),  vget_low_s16(row5));
  int32x4_t m2high = vaddl_s16(vget_high_s16(row2), vget_high_s16(row5));

  int32x4_t m3low  = vaddl_s16(vget_low_s16(row3),  vget_low_s16(row4));
  int32x4_t m3high = vaddl_s16(vget_high_s16(row3), vget_high_s16(row4));


  int32x4_t sumlow, sumhigh;

  sumlow  = vshlq_n_s32(m1low,  2);
  sumhigh = vshlq_n_s32(m1high, 2);

  sumlow  = vsubq_s32(sumlow,  m0low);
  sumhigh = vsubq_s32(sumhigh, m0high);

  sumlow  = vmlaq_n_s32(sumlow,  m2low,  -11);
  sumhigh = vmlaq_n_s32(sumhigh, m2high, -11);

  sumlow  = vmlaq_n_s32(sumlow,  m3low,  40);
  sumhigh = vmlaq_n_s32(sumhigh, m3high, 40);


  int16x4_t sum16low  = vshrn_n_s32(sumlow,  6);
  int16x4_t sum16high = vshrn_n_s32(sumhigh, 6);

  return vcombine_s16(sum16low, sum16high);
}


template<int filterIdx>
inline void mc_qpel_v_8_neon(int16_t *dst, ptrdiff_t dststride,
                             const uint8_t *src, ptrdiff_t srcstride,
                             int width, int height,
                             int16_t* mcbuffer)
{
  //printf("%d x %d [Vshift=%d]\n",width,height,filterIdx);

  while (width>=8) {

    int16x8_t  row0, row1, row2, row3, row4, row5, row6, row7;

    if (filterIdx!=3) {
      row0= load_u8_to_s16(src+(-3)*srcstride);
    }
    row1= load_u8_to_s16(src+(-2)*srcstride);
    row2= load_u8_to_s16(src+(-1)*srcstride);
    row3= load_u8_to_s16(src+( 0)*srcstride);
    row4= load_u8_to_s16(src+(+1)*srcstride);
    row5= load_u8_to_s16(src+(+2)*srcstride);
    if (filterIdx!=1) {
      row6= load_u8_to_s16(src+(+3)*srcstride);
    }

    for (int y=0;y<height;y++) {
      if (filterIdx==1) { row6= load_u8_to_s16(src+(y+3)*srcstride); }
      else              { row7= load_u8_to_s16(src+(y+4)*srcstride); }

      int16x8_t rowsum;
      if (filterIdx==1) rowsum = vfilter_1qpel_8x(row0,row1,row2,row3,row4,row5,row6,row7);
      if (filterIdx==2) rowsum = vfilter_2qpel_8x(row0,row1,row2,row3,row4,row5,row6,row7);
      if (filterIdx==3) rowsum = vfilter_1qpel_8x(row7,row6,row5,row4,row3,row2,row1,row0);

      vst1q_s16(dst+y*dststride, rowsum);

      if (filterIdx!=3) {
        row0=row1;
      }
      row1=row2;
      row2=row3;
      row3=row4;
      row4=row5;
      row5=row6;
      if (filterIdx!=1) {
        row6=row7;
      }
    }

    width-=8;
    src+=8;
    dst+=8;
  }

  if (width > 0) {
    switch (filterIdx) {
    case 1: put_qpel_0_1_fallback(dst,dststride, src,srcstride, width,height, mcbuffer); break;
    case 2: put_qpel_0_2_fallback(dst,dststride, src,srcstride, width,height, mcbuffer); break;
    case 3: put_qpel_0_3_fallback(dst,dststride, src,srcstride, width,height, mcbuffer); break;
    }
  }
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




template<int dh,int dv>
inline void mc_qpel_hv_8_neon(int16_t *dst, ptrdiff_t dststride,
                              const uint8_t *src, ptrdiff_t srcstride,
                              int width, int height,
                              int16_t* mcbuffer)
{
  //printf("%d x %d [Vshift=%d]\n",width,height,filterIdx);

  uint8x8_t hfilter_plus  = vld1_u8(qpel_filter[dh-1]);
  uint8x8_t hfilter_minus = vld1_u8(qpel_filter[dh-1]+8);

  while (width>=8) {
    uint8x8_t input_left;
    uint8x8_t input_right;

    int16x8_t  row0, row1, row2, row3, row4, row5, row6, row7;

    input_left  = vld1_u8(src+(-3)*srcstride -3);
    input_right = vld1_u8(src+(-3)*srcstride -3+8);
    row0 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

    input_left  = vld1_u8(src+(-2)*srcstride -3);
    input_right = vld1_u8(src+(-2)*srcstride -3+8);
    row1 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

    input_left  = vld1_u8(src+(-1)*srcstride -3);
    input_right = vld1_u8(src+(-1)*srcstride -3+8);
    row2 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

    input_left  = vld1_u8(src+(0 )*srcstride -3);
    input_right = vld1_u8(src+(0 )*srcstride -3+8);
    row3 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

    input_left  = vld1_u8(src+(+1)*srcstride -3);
    input_right = vld1_u8(src+(+1)*srcstride -3+8);
    row4 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

    input_left  = vld1_u8(src+(+2)*srcstride -3);
    input_right = vld1_u8(src+(+2)*srcstride -3+8);
    row5 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

    input_left  = vld1_u8(src+(+3)*srcstride -3);
    input_right = vld1_u8(src+(+3)*srcstride -3+8);
    row6 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);


    for (int y=0;y<height;y++) {
      input_left  = vld1_u8(src+(y+4)*srcstride -3);
      input_right = vld1_u8(src+(y+4)*srcstride -3+8);
      row7 = hfilter_qpel_8x(hfilter_plus,hfilter_minus, input_left,input_right);

      int16x8_t rowsum;
      if (dv==1) rowsum = vfilter_1qpel_8x_32bit(row0,row1,row2,row3,row4,row5,row6,row7);
      if (dv==2) rowsum = vfilter_2qpel_8x_32bit(row0,row1,row2,row3,row4,row5,row6,row7);
      if (dv==3) rowsum = vfilter_1qpel_8x_32bit(row7,row6,row5,row4,row3,row2,row1,row0);

      vst1q_s16(dst+y*dststride, rowsum);

      row0=row1;
      row1=row2;
      row2=row3;
      row3=row4;
      row4=row5;
      row5=row6;
      row6=row7;
    }

    width-=8;
    src+=8;
    dst+=8;
  }

  if (width > 0) {
    if (dh==1 && dv==1) put_qpel_1_1_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==2 && dv==1) put_qpel_2_1_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==3 && dv==1) put_qpel_3_1_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==1 && dv==2) put_qpel_1_2_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==2 && dv==2) put_qpel_2_2_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==3 && dv==2) put_qpel_3_2_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==1 && dv==3) put_qpel_1_3_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==2 && dv==3) put_qpel_2_3_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
    if (dh==3 && dv==3) put_qpel_3_3_fallback(dst,dststride, src,srcstride, width,height, mcbuffer);
  }
}


void mc_qpel_h1v1_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<1,1>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h2v1_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<2,1>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h3v1_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<3,1>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h1v2_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<1,2>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h2v2_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<2,2>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h3v2_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<3,2>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h1v3_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<1,3>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h2v3_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<2,3>(dst,dststride, src,srcstride, width,height,mcbuffer); }
void mc_qpel_h3v3_8_neon(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride,
                         int width, int height, int16_t* mcbuffer)
{ mc_qpel_hv_8_neon<3,3>(dst,dststride, src,srcstride, width,height,mcbuffer); }
