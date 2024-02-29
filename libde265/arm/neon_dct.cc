#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <iostream>

#include <cassert>
#include <cstring>
#include <stddef.h>
#include <arm_neon.h>
#include "./libde265/util.h"
#include "neon_dct.h"
#include "neon_common.h"

#include <sys/time.h>

LIBDE265_INLINE void Transpose4x4(const int16x8_t in[4],
                                        int16x8_t out[4]) {
  // Swap 16 bit elements. Goes from:
  // a0: 00 01 02 03
  // a1: 10 11 12 13
  // a2: 20 21 22 23
  // a3: 30 31 32 33
  // to:
  // b0.val[0]: 00 10 02 12
  // b0.val[1]: 01 11 03 13
  // b1.val[0]: 20 30 22 32
  // b1.val[1]: 21 31 23 33
  const int16x4_t a0 = vget_low_s16(in[0]);
  const int16x4_t a1 = vget_low_s16(in[1]);
  const int16x4_t a2 = vget_low_s16(in[2]);
  const int16x4_t a3 = vget_low_s16(in[3]);

  const int16x4x2_t b0 = vtrn_s16(a0, a1);
  const int16x4x2_t b1 = vtrn_s16(a2, a3);

  // Swap 32 bit elements resulting in:
  // c0.val[0]: 00 10 20 30 04 14 24 34
  // c0.val[1]: 02 12 22 32 06 16 26 36
  // c1.val[0]: 01 11 21 31 05 15 25 35
  // c1.val[1]: 03 13 23 33 07 17 27 37
  const int32x2x2_t c0 = vtrn_s32(vreinterpret_s32_s16(b0.val[0]),
                                  vreinterpret_s32_s16(b1.val[0]));
  const int32x2x2_t c1 = vtrn_s32(vreinterpret_s32_s16(b0.val[1]),
                                  vreinterpret_s32_s16(b1.val[1]));

  const int16x4_t d0 = vreinterpret_s16_s32(c0.val[0]);
  const int16x4_t d1 = vreinterpret_s16_s32(c1.val[0]);
  const int16x4_t d2 = vreinterpret_s16_s32(c0.val[1]);
  const int16x4_t d3 = vreinterpret_s16_s32(c1.val[1]);

  out[0] = vcombine_s16(d0, d0);
  out[1] = vcombine_s16(d1, d1);
  out[2] = vcombine_s16(d2, d2);
  out[3] = vcombine_s16(d3, d3);
}

template <int tx_width>
LIBDE265_INLINE void RowShift(int16x8_t* source, int32x4_t s_l[][2], int num_rows,
                                    bool row_shift) {

  if (tx_width == 4 && row_shift) {
    for (int i = 0; i < num_rows; i++)
    {
      source[i] = vcombine_s16(vqrshrn_n_s32(s_l[i][0], 7), vqrshrn_n_s32(s_l[i][1], 7));
    }
  } else if (tx_width == 4){
    for (int i = 0; i < num_rows; i++)
    {
      source[i] = vcombine_s16(vqrshrn_n_s32(s_l[i][0], 12), vqrshrn_n_s32(s_l[i][1], 12));
    }
  }
  else
  {
/*
    int i = 0;
    do {
      for (int j = 0; j < tx_width; j += 8) {
        const int16x8_t residual = vld1q_s16(&source[i * tx_width + j]);
        const int16x8_t residual_shifted =
            vqrshlq_s16(residual, vdupq_n_s16(row_shift));
        vst1q_s16(&source[i * tx_width + j], residual_shifted);
      }
    } while (++i < num_rows);
*/
  }

}

template <int store_width, int store_count>
LIBDE265_INLINE void StoreDst(int16_t* LIBDE265_RESTRICT dst,
                                    int32_t stride, int32_t idx,
                                    const int16x8_t* const s) {
  assert(store_count % 4 == 0);
  assert(store_width == 8 || store_width == 16);
  // NOTE: It is expected that the compiler will unroll these loops.
  if (store_width == 16) {
    for (int i = 0; i < store_count; i += 4) {
      vst1q_s16(&dst[i * stride + idx], (s[i]));
      vst1q_s16(&dst[(i + 1) * stride + idx], (s[i + 1]));
      vst1q_s16(&dst[(i + 2) * stride + idx], (s[i + 2]));
      vst1q_s16(&dst[(i + 3) * stride + idx], (s[i + 3]));
    }
  } else {
    // store_width == 8
    for (int i = 0; i < store_count; i += 4) {
      vst1_s16(&dst[i * stride + idx], vget_low_s16(s[i]));
      vst1_s16(&dst[(i + 1) * stride + idx], vget_low_s16(s[i + 1]));
      vst1_s16(&dst[(i + 2) * stride + idx], vget_low_s16(s[i + 2]));
      vst1_s16(&dst[(i + 3) * stride + idx], vget_low_s16(s[i + 3]));
    }
  }
}
                                      
template <int load_width, int load_count>
LIBDE265_INLINE void LoadSrc(const int16_t* LIBDE265_RESTRICT src,
                                   ptrdiff_t stride, int32_t idx, int16x8_t* x) {
  assert(load_count % 4 == 0);
  assert(load_width == 8 || load_width == 16);
  // NOTE: It is expected that the compiler will unroll these loops.
  if (load_width == 16) {
    for (int i = 0; i < load_count; i += 4) {
      x[i] = vld1q_s16(&src[i * stride + idx]);
      x[i + 1] = vld1q_s16(&src[(i + 1) * stride + idx]);
      x[i + 2] = vld1q_s16(&src[(i + 2) * stride + idx]);
      x[i + 3] = vld1q_s16(&src[(i + 3) * stride + idx]);
    }
  } else {
    // load_width == 8
    const int64x2_t zero = vdupq_n_s64(0);
    for (int i = 0; i < load_count; i += 4) {
      // The src buffer is aligned to 32 bytes.  Each load will always be 8
      // byte aligned.
      x[i] = vreinterpretq_s16_s64(vld1q_lane_s64(
          reinterpret_cast<const int64_t*>(&src[i * stride + idx]), zero, 0));
      x[i + 1] = vreinterpretq_s16_s64(vld1q_lane_s64(
          reinterpret_cast<const int64_t*>(&src[(i + 1) * stride + idx]), zero,
          0));
      x[i + 2] = vreinterpretq_s16_s64(vld1q_lane_s64(
          reinterpret_cast<const int64_t*>(&src[(i + 2) * stride + idx]), zero,
          0));
      x[i + 3] = vreinterpretq_s16_s64(vld1q_lane_s64(
          reinterpret_cast<const int64_t*>(&src[(i + 3) * stride + idx]), zero,
          0));
    }
  }
}

// // Load 4 uint8_t values into 4 lanes staring with |lane| * 4.
// template <int lane>
// inline uint8x8_t Load4(const void* const buf, uint8x8_t val) {
//   uint32_t temp;
//   std::memcpy(&temp, buf, 4);
//   return vreinterpret_u8_u32(
//       vld1_lane_u32(&temp, vreinterpret_u32_u8(val), lane));
// }
// template <typename T>
// inline void ValueToMem(void* const buf, T val) {
//   memcpy(buf, &val, sizeof(val));
// }

// Store 4 int8_t values from the low half of an int8x8_t register.
inline void StoreLo4(void* const buf, const int8x8_t val) {
  ValueToMem<int32_t>(buf, vget_lane_s32(vreinterpret_s32_s8(val), 0));
}

// Store 4 uint8_t values from the low half of a uint8x8_t register.
inline void StoreLo4(void* const buf, const uint8x8_t val) {
  ValueToMem<uint32_t>(buf, vget_lane_u32(vreinterpret_u32_u8(val), 0));
}

template <int tx_width>
LIBDE265_INLINE void StoreToFrameWithRound( uint8_t* LIBDE265_RESTRICT dst, const ptrdiff_t stride, const int16x8_t* LIBDE265_RESTRICT source) 
{
  if (tx_width == 4) {
    uint8x8_t frame_data = vdup_n_u8(0);
    for (int i = 0; i < tx_width; ++i) {
      const int16x4_t residual = vreinterpret_s16_s64(vget_low_s64(vreinterpretq_s64_s16(source[i])));
      frame_data = Load4<0>(dst, frame_data);
      const uint16x8_t b =
          vaddw_u8(vreinterpretq_u16_s16(vcombine_s16(residual, residual)), frame_data);
      const uint8x8_t d = vqmovun_s16(vreinterpretq_s16_u16(b));
      StoreLo4(dst, d);
      dst += stride;
    }
  } else if (tx_width == 8) {
    for (int i = 0; i < tx_width; ++i) {
      const int16x8_t residual = source[i];
      const uint8x8_t frame_data = vld1_u8(dst);
      const uint16x8_t b = vaddw_u8(vreinterpretq_u16_s16(residual), frame_data);
      const uint8x8_t d = vqmovun_s16(vreinterpretq_s16_u16(b));
      vst1_u8(dst, d);
      dst += stride;
    }
  } 
  else {
    int idx = 0;
    for (int i = 0; i < tx_width; ++i) {
      int j = 0;
      do {
        const int16x8_t residual = source[idx];
        const int16x8_t residual_hi = source[++idx];
        const uint8x16_t frame_data = vld1q_u8(dst + j);
        const uint16x8_t b =
            vaddw_u8(vreinterpretq_u16_s16(residual), vget_low_u8(frame_data));
        const uint16x8_t b_hi =
            vaddw_u8(vreinterpretq_u16_s16(residual_hi), vget_high_u8(frame_data));
        vst1q_u8(dst + j,
                 vcombine_u8(vqmovun_s16(vreinterpretq_s16_u16(b)),
                             vqmovun_s16(vreinterpretq_s16_u16(b_hi))));
        j += 16;
        ++idx;
      } while (j < tx_width);
      dst += stride;
    }
  }
}

template <int tx_width>
LIBDE265_INLINE void StoreToFrameWithRound(uint8_t* LIBDE265_RESTRICT dst, const ptrdiff_t stride,  const int16_t* LIBDE265_RESTRICT source) {
  if (tx_width == 4) {
    uint8x8_t frame_data = vdup_n_u8(0);
    for (int i = 0; i < tx_width; ++i) {
      const int16x4_t residual = vld1_s16(&source[i * 4]);
      frame_data = Load4<0>(dst, frame_data);
      const uint16x8_t b =
          vaddw_u8(vreinterpretq_u16_s16(vcombine_s16(residual, residual)), frame_data);
      const uint8x8_t d = vqmovun_s16(vreinterpretq_s16_u16(b));
      StoreLo4(dst, d);
      dst += stride;
    }
  } else if (tx_width == 8) {
    for (int i = 0; i < tx_width; ++i) {
      const int16x8_t residual = vld1q_s16(&source[i * 8]);
      const uint8x8_t frame_data = vld1_u8(dst);
      const uint16x8_t b = vaddw_u8(vreinterpretq_u16_s16(residual), frame_data);
      const uint8x8_t d = vqmovun_s16(vreinterpretq_s16_u16(b));
      vst1_u8(dst, d);
      dst += stride;
    }
  }   
  else {
  for (int i = 0; i < tx_width; ++i) {
      const int row = i * tx_width;
      int j = 0;
      do {
        const int16x8_t residual = vld1q_s16(&source[row + j]);
        const int16x8_t residual_hi = vld1q_s16(&source[row + j + 8]);
        const uint8x16_t frame_data = vld1q_u8(dst + j);
        const uint16x8_t b =
            vaddw_u8(vreinterpretq_u16_s16(residual), vget_low_u8(frame_data));
        const uint16x8_t b_hi =
            vaddw_u8(vreinterpretq_u16_s16(residual_hi), vget_high_u8(frame_data));
        vst1q_u8(dst + j,
                 vcombine_u8(vqmovun_s16(vreinterpretq_s16_u16(b)),
                             vqmovun_s16(vreinterpretq_s16_u16(b_hi))));
        j += 16;
      } while (j < tx_width);
      dst += stride;
    }
  }
}

void ff_hevc_transform_4x4_dc_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit)
{
  (void)col_limit;
  int16x8_t residual[4];
  int16_t dc = (((coeffs[0]+1)>>1)+32)>>6;
  for (size_t i = 0; i < 4; i++)
      residual[i] = vdupq_n_s16(dc);
  StoreToFrameWithRound<4>(dst, stride, residual);
  /*
  for (size_t j = 0; j < 4; j++)
  {
      for (size_t i = 0; i < 4; i++)
      {
          std::cout << (int)dst[j*stride+i];
          //dst[j*stride+i] = Clip_BitDepth(dst[j*stride+i] + dc, 8);
      }     
      std::cout << std::endl;
  }
  */
}

void ff_hevc_transform_8x8_dc_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit)
{
  (void)col_limit;
  int16x8_t residual[8];
  int16_t dc = (((coeffs[0]+1)>>1)+32)>>6;
  for (size_t i = 0; i < 8; i++)
      residual[i] = vdupq_n_s16(dc);

  StoreToFrameWithRound<8>(dst, stride, residual);
}

void ff_hevc_transform_16x16_dc_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit)
{
  (void)col_limit;
  int16x8_t residual[2*16];
  int16_t dc = (((coeffs[0]+1)>>1)+32)>>6;
  for (size_t i = 0; i < 2*16; i++)
      residual[i] = vdupq_n_s16(dc);
  StoreToFrameWithRound<16>(dst, stride, residual);
}

void ff_hevc_transform_32x32_dc_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit)
{
  (void)col_limit;
  int16x8_t residual[4*32];
  int16_t dc = (((coeffs[0]+1)>>1)+32)>>6;
  for (size_t i = 0; i < 4*32; i++)
      residual[i] = vdupq_n_s16(dc);
  StoreToFrameWithRound<32>(dst, stride, residual);
}

#define tr4_luma_shift_transpose(r0, r1, r2, r3, shift)   \
  do {                                                    \
      int32x4_t q5 = vaddl_s16(r0, r2);                   \
      int32x4_t q2 = vaddl_s16(r2, r3);                   \
      int32x4_t q4 = vsubl_s16(r0, r3);                   \
      int32x4_t q6 = vmull_n_s16(r1, d00);                \
                                                          \
      int32x4_t q7 = vaddl_s16(r0, r3);                   \
      q7 = vsubw_s16(q7, r2);                             \
      q7 = vmulq_n_s32(q7,d00);                           \
                                                          \
      int32x4_t q8 = vmulq_n_s32(q5,d01);                 \
      int32x4_t q9 = vmulq_n_s32(q2,d10);                 \
      q8 = vaddq_s32(q8, q9);                             \
      q8 = vaddq_s32(q8, q6);                             \
                                                          \
      q2 = vmulq_n_s32(q2,d01);                           \
      q9 = vmulq_n_s32(q4,d10);                           \
      q9 = vsubq_s32(q9, q2);                             \
      q9 = vaddq_s32(q9, q6);                             \
                                                          \
      q5 = vmulq_n_s32(q5,d10);                           \
      q4 = vmulq_n_s32(q4,d01);                           \
      q5 = vaddq_s32(q5, q4);                             \
      q5 = vsubq_s32(q5, q6);                             \
                                                          \
      r0 = vqrshrn_n_s32(q8, shift);                      \
      r1 = vqrshrn_n_s32(q9, shift);                      \
      r2 = vqrshrn_n_s32(q7, shift);                      \
      r3 = vqrshrn_n_s32(q5, shift);                      \
                                                          \
      const int16x4x2_t b0 = vtrn_s16(r0, r1);            \
      const int16x4x2_t b1 = vtrn_s16(r2, r3);            \
      const int32x2x2_t c0 = vtrn_s32(                    \
          vreinterpret_s32_s16(b0.val[0]),                \
          vreinterpret_s32_s16(b1.val[0]));               \
      const int32x2x2_t c1 = vtrn_s32(                    \
          vreinterpret_s32_s16(b0.val[1]),                \
          vreinterpret_s32_s16(b1.val[1]));               \
      r0 = vreinterpret_s16_s32(c0.val[0]);               \
      r1 = vreinterpret_s16_s32(c1.val[0]);               \
      r2 = vreinterpret_s16_s32(c0.val[1]);               \
      r3 = vreinterpret_s16_s32(c1.val[1]);               \
  } while(0)                                              

void ff_hevc_transform_4x4_luma_add_8_neon(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth){
  
  int16x8_t q14 = vld1q_s16(coeffs);
  int16x8_t q15 = vld1q_s16(coeffs+8);

  int32_t d00 = 0x4a;  //74
  int32_t d01 = 0x1d;  //29
  int32_t d10 = 0x37;  //55

  int16x4_t r[4];

  r[0] = vreinterpret_s16_s64(vget_low_s64(vreinterpretq_s64_s16(q14)));
  r[1] = vreinterpret_s16_s64(vget_high_s64(vreinterpretq_s64_s16(q14)));
  r[2] = vreinterpret_s16_s64(vget_low_s64(vreinterpretq_s64_s16(q15)));
  r[3] = vreinterpret_s16_s64(vget_high_s64(vreinterpretq_s64_s16(q15)));

  tr4_luma_shift_transpose (r[0], r[1], r[2], r[3], 7);
  tr4_luma_shift_transpose (r[0], r[1], r[2], r[3], 12);

  int16x8_t residual[4];
  for (size_t i = 0; i < 4; i++)
      residual[i] = vcombine_s16(r[i], r[i]);
  
  StoreToFrameWithRound<4>(dst, stride, residual);
}

LIBDE265_INLINE void HadamardRotation(int16x8_t* a, int16x8_t* b,
                                            bool flip) {
  int16x8_t x, y;
  if (flip) {
    y = vqaddq_s16(*b, *a);
    x = vqsubq_s16(*b, *a);
  } else {
    x = vqaddq_s16(*a, *b);
    y = vqsubq_s16(*a, *b);
  }
  *a = x;
  *b = y;
}

LIBDE265_INLINE void HadamardRotation4(int32x4_t *E, int32x4_t *O, bool flip) {
/*
  int16x8_t x, y;
  if (flip) {
    y = vqaddq_s16(*b, *a);
    x = vqsubq_s16(*b, *a);
  } else {
    x = vqaddq_s16(*a, *b);
    y = vqsubq_s16(*a, *b);
  }
  *a = x;
  *b = y;
*/

  int32x4_t outA, outB;

  outA = vqaddq_s32(E[0], O[0]);
  outB = vqsubq_s32(E[0], O[0]);

  E[0] = outA;
  O[0] = outB;
}

/*
DCT4[4][4] = 
{ 
  { 64,  64,  64,  64},
  { 83,  36, -36, -83}, 
  { 64, -64, -64,  64}, 
  { 36, -83,  83, -36}  
}

DCT[8][8] =  
{ 
  { 64,  64,  64,  64,  64,  64,  64,  a}, 
  { 89,  75,  50,  18, -18, -50, -75, -d}, 
  { 83,  36, -36, -83, -83, -36,  36,  b}, 
  { 75, -18, -89, -50,  50,  89,  18, -e}, 
  { 64, -64, -64,  64,  64, -64, -64,  a}, 
  { 50, -89,  18,  75, -75, -18,  89, -f}, 
  { 36, -83,  83, -36, -36,  83, -83,  c}, 
  { 18, -50,  75, -89,  89, -75,  50, -g}  
}

int DCT16[16][16] =
{ 
  { 64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64}, 
  { 90,  87,  80,  70,  57,  43,  25,   9,  -9, -25, -43, -57, -70, -80, -87, -90}, 
  { 89,  75,  50,  18, -18, -50, -75, -89, -89, -75, -50, -18,  18,  50,  75,  89}, 
  { 87,  57,   9, -43, -80, -90, -70, -25,  25,  70,  90,  80,  43,  -9, -57, -87}, 
  { 83,  36, -36, -83, -83, -36,  36,  83,  83,  36, -36, -83, -83, -36,  36,  83}, 
  { 80,   9, -70, -87, -25,  57,  90,  43, -43, -90, -57,  25,  87,  70,  -9, -80}, 
  { 75, -18, -89, -50,  50,  89,  18, -75, -75,  18,  89,  50, -50, -89, -18,  75}, 
  { 70, -43, -87,   9,  90,  25, -80, -57,  57,  80, -25, -90,  -9,  87,  43, -70}, 
  { 64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64}, 
  { 57, -80, -25,  90,  -9, -87,  43,  70, -70, -43,  87,   9, -90,  25,  80, -57}, 
  { 50, -89,  18,  75, -75, -18,  89, -50, -50,  89, -18, -75,  75,  18, -89,  50}, 
  { 43, -90,  57,  25, -87,  70,   9, -80,  80,  -9, -70,  87, -25, -57,  90, -43}, 
  { 36, -83,  83, -36, -36,  83, -83,  36,  36, -83,  83, -36, -36,  83, -83,  36}, 
  { 25, -70,  90, -80,  43,   9, -57,  87, -87,  57,  -9, -43,  80, -90,  70, -25}, 
  { 18, -50,  75, -89,  89, -75,  50, -18, -18,  50, -75,  89, -89,  75, -50,  18}, 
  {  9, -25,  43, -57,  70, -80,  87, -90,  90, -87,  80, -70,  57, -43,  25,  -9}  
};

*/

// Butterfly rotate 4 values.
LIBDE265_INLINE void ButterflyRotation_4(int16x8_t* a, int16x8_t* b, int32x4_t *outA, int32x4_t *outB, const int16_t coefA, const int16_t coefB, bool flip) { 

  const int32x4_t acc_x = vmull_n_s16(vget_low_s16(*a), coefA);
  const int32x4_t acc_y = vmull_n_s16(vget_low_s16(*a), coefB);
  const int32x4_t x = vmlal_n_s16(acc_x, vget_low_s16(*b), coefB);
  const int32x4_t y = vmlsl_n_s16(acc_y, vget_low_s16(*b), coefA);
  if(flip){
    outA[0] = y;
    outB[0] = x;
  }else{
    outA[0] = x;
    outB[0] = y;
  }
}

using ButterflyRotationFunc = void (*)(int16x8_t* a, int16x8_t* b, int32x4_t *outA, int32x4_t *outB, const int16_t coefA, const int16_t coefB,  bool flip);

template <ButterflyRotationFunc butterfly_rotation, bool is_fast_butterfly = false>
LIBDE265_INLINE void Dct4Stages(int16x8_t* s, int32x4_t s_l[][2]) {
  if (is_fast_butterfly) {
    //ButterflyRotation_SecondIsZero(&s[0], &s[1], 32, true);
    //ButterflyRotation_SecondIsZero(&s[2], &s[3], 48, false);
  } else {
    butterfly_rotation(&s[0], &s[1], s_l[0], s_l[1], 64, 64, false);
    butterfly_rotation(&s[2], &s[3], s_l[3], s_l[2], 83, 36, false);
  }

  // stage 17.
  HadamardRotation4(s_l[0], s_l[3], false);
  HadamardRotation4(s_l[1], s_l[2], false);
}

/*
DT*DATA*D =
DT*(DT*DATAT)T=
(DT*(DT*DATA)T)T
*/

template <ButterflyRotationFunc butterfly_rotation>
LIBDE265_INLINE void DCT4_NEON(void* dest, int32_t step, bool transpose, bool is_row){
  auto* const dst = static_cast<int16_t*>(dest);
  int16x8_t s[4], x[4];
  int32x4_t s_l[4][2];

  LoadSrc<16, 4> (dst, step, 0, x);
  //if(transpose)
    //Transpose4x4(x, x);

  s[0] = x[0];
  s[1] = x[2];
  s[2] = x[1];
  s[3] = x[3];
  
  Dct4Stages<butterfly_rotation>(s, s_l);

  RowShift<4>(s, s_l, 4, is_row);

  if (transpose) {
      Transpose4x4(s, s);
  }
  
  StoreDst<8, 4>(dst, step, 0, s);
}



void ff_hevc_transform_4x4_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit){
  auto* src = static_cast<int16_t*>(coeffs);
  DCT4_NEON<ButterflyRotation_4>(src, 4, true, true);
  DCT4_NEON<ButterflyRotation_4>(src, 4, true, false);
  StoreToFrameWithRound<4>(dst, stride, src);
}

LIBDE265_INLINE void swp_element(int16x4_t* r0, int16x4_t* r1){
  int16x4_t tmp = *r0;
  *r0 = *r1;
  *r1 = tmp;
}
LIBDE265_INLINE void transpose_4x4(int16x4_t* r0, int16x4_t* r1, int16x4_t* r2, int16x4_t* r3){
  int32x2x2_t tmp0 = vtrn_s32(vreinterpret_s32_s16(*r0), vreinterpret_s32_s16(*r2));
  *r0 = vreinterpret_s16_s32(tmp0.val[0]);
  *r2 = vreinterpret_s16_s32(tmp0.val[1]);
  tmp0 = vtrn_s32(vreinterpret_s32_s16(*r1), vreinterpret_s32_s16(*r3));
  *r1 = vreinterpret_s16_s32(tmp0.val[0]);
  *r3 = vreinterpret_s16_s32(tmp0.val[1]);
  int16x4x2_t tmp1 = vtrn_s16(*r0, *r1);
  *r0 = tmp1.val[0];
  *r1 = tmp1.val[1];
  tmp1 = vtrn_s16(*r2, *r3);
  *r2 = tmp1.val[0];
  *r3 = tmp1.val[1];
}

LIBDE265_INLINE void trans4(int32x4_t* S, int16x4_t r0, int16x4_t r1, int16x4_t r2, int16x4_t r3,
                            const int16_t d0, const int16_t d1, const int16_t d2, const int16_t d3){
  int32x4_t E[2], O[2];

  S[0] = vshll_n_s16(r0, 6);
  S[1] = vshll_n_s16(r2, 6);

  S[2] = vmull_n_s16(r1, d2);
  S[3] = vmull_n_s16(r1, d3);

  E[0] = vaddq_s32(S[0], S[1]);
  E[1] = vsubq_s32(S[0], S[1]);

  O[0] = vmlal_n_s16(S[2], r3, d3);
  O[1] = vmlsl_n_s16(S[3], r3, d2);

  S[0] = vaddq_s32(E[0], O[0]);
  S[1] = vaddq_s32(E[1], O[1]);
  S[2] = vsubq_s32(E[1], O[1]);
  S[3] = vsubq_s32(E[0], O[0]);
}
LIBDE265_INLINE void trans8(int32x4_t* O, int16x4_t r0, int16x4_t r1, int16x4_t r2, int16x4_t r3,
                            const int16_t d0, const int16_t d1, const int16_t d2, const int16_t d3){
  O[0] = vmull_n_s16(r0, d0);
  O[1] = vmull_n_s16(r0, d1);
  O[2] = vmull_n_s16(r0, d2);
  O[3] = vmull_n_s16(r0, d3);

  O[0] = vmlal_n_s16(O[0], r1, d1);
  O[1] = vmlsl_n_s16(O[1], r1, d3);
  O[2] = vmlsl_n_s16(O[2], r1, d0);
  O[3] = vmlsl_n_s16(O[3], r1, d2);

  O[0] = vmlal_n_s16(O[0], r2, d2);
  O[1] = vmlsl_n_s16(O[1], r2, d0);
  O[2] = vmlal_n_s16(O[2], r2, d3);
  O[3] = vmlal_n_s16(O[3], r2, d1);

  O[0] = vmlal_n_s16(O[0], r3, d3);
  O[1] = vmlsl_n_s16(O[1], r3, d2);
  O[2] = vmlal_n_s16(O[2], r3, d1);
  O[3] = vmlsl_n_s16(O[3], r3, d0);
}

LIBDE265_INLINE void trans8_end(int16x4_t* s, int32x4_t* O, int32x4_t* E, bool is_row){
  int32x4_t dst[8];

  for (int i = 0; i < 4; i++)
  {
    dst[i] = vaddq_s32(E[i], O[i]);
    dst[i+4] = vsubq_s32(E[3-i], O[3-i]);
  }
  if(is_row){
    for (int i = 0; i < 8; i++)
      s[i] = vqrshrn_n_s32(dst[i], 7);
  }
  else{
    for (int i = 0; i < 8; i++)
      s[i] = vqrshrn_n_s32(dst[i], 12);    
  }
}

void ff_hevc_transform_8x8_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit)
{
  // std::cout << "original coeff " << std::endl;
  // for (size_t j = 0; j < 8; j++)
  // {
  //   for (size_t i = 0; i < 8; i++)
  //   {
  //     std::cout << (int) coeffs[j*8+i] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // int16_t g[32*32];  // actually, only [nT*nT] used
  // int16_t gg[32*32];
  // {
  //   int nT = 8;
  //   int postShift = 20-8;
  //   int rnd1 = 1<<(7-1);
  //   int rnd2 = 1<<(postShift-1);
  //   int fact = (1<<(5-Log2(nT)));

  //   for (int c=0;c<nT;c++) {
  //     int lastCol = nT-1;
  //     for (;lastCol>=0;lastCol--) {
  //       if (coeffs[c+lastCol*nT]) { break; }
  //     }

  //     for (int i=0;i<nT;i++) {
  //       int sum=0;
  //       for (int j=0;j<=lastCol /*nT*/;j++) {
  //         sum += mat_dct[fact*j][i] * coeffs[c+j*nT];
  //       }
  //       g[c+i*nT] = Clip3(-32768,32767, (sum+rnd1)>>7);
  //     }
  //   }

  //   for (int y=0;y<nT;y++) {
  //     int lastCol = nT-1;
  //     for (;lastCol>=0;lastCol--) {
  //       if (g[y*nT+lastCol]) { break; }
  //     }
  //     for (int i=0;i<nT;i++) {
  //       int sum=0;
  //       for (int j=0;j<=lastCol /*nT*/;j++) {
  //         sum += mat_dct[fact*j][i] * g[y*nT+j];
  //       }
  //       int out = (sum+rnd2)>>postShift;
  //     gg[y*nT+i] = out;
  //     }
  //   }
  // }

  auto* src = static_cast<int16_t*>(coeffs);

  int16x4_t s[8];
  int32x4_t O[4];
  int32x4_t E[4];

  //left half
  for (int i = 0; i < 8; i++)
    s[i] = vld1_s16(&src[i*8]);
  trans8(O, s[1], s[3], s[5], s[7], 89, 75, 50, 18);
  trans4(E, s[0], s[2], s[4], s[6], 64, 64, 83, 36);
  trans8_end(s, O, E, true);
  for (int i = 0; i < 8; i++)
    vst1_s16(&src[i*8], s[i]);

  //right half
  if(col_limit>1)
  {
    for (int i = 0; i < 8; i++)
      s[i] = vld1_s16(&src[i*8+4]);
    trans8(O, s[1], s[3], s[5], s[7], 89, 75, 50, 18);
    trans4(E, s[0], s[2], s[4], s[6], 64, 64, 83, 36);
    trans8_end(s, O, E, true);
    for (int i = 0; i < 8; i++)
      vst1_s16(&src[i*8+4], s[i]);
  }

  //top half
  for (int i = 0; i < 4; i++)
  {
    int16x8_t tmp = vcombine_s16(s[i*2], s[i*2+1]);
    tmp = vld1q_s16(&src[i*8]);
    s[i*2] = vget_low_s16(tmp);
    s[i*2+1] = vget_high_s16(tmp);
  }

  transpose_4x4(&s[0], &s[2], &s[4], &s[6]);
  transpose_4x4(&s[1], &s[3], &s[5], &s[7]);
  trans8(O, s[2], s[6], s[3], s[7], 89, 75, 50, 18);
  trans4(E, s[0], s[4], s[1], s[5], 64, 64, 83, 36);
  trans8_end(s, O, E, false);
  transpose_4x4(&s[0], &s[1], &s[2], &s[3]);
  transpose_4x4(&s[4], &s[5], &s[6], &s[7]);
  swp_element(&s[5], &s[3]);
  swp_element(&s[5], &s[6]);
  swp_element(&s[1], &s[4]);
  swp_element(&s[4], &s[2]);
  for (int i = 0; i < 4; i++)
    vst1q_s16(&src[i*8], vcombine_s16(s[i*2], s[i*2+1]));

  //bottom half
  for (int i = 0; i < 4; i++)
  {
    int16x8_t tmp = vcombine_s16(s[i*2], s[i*2+1]);
    tmp = vld1q_s16(&src[32+i*8]);
    s[i*2] = vget_low_s16(tmp);
    s[i*2+1] = vget_high_s16(tmp);
  }
  transpose_4x4(&s[0], &s[2], &s[4], &s[6]);
  transpose_4x4(&s[1], &s[3], &s[5], &s[7]);
  trans8(O, s[2], s[6], s[3], s[7], 89, 75, 50, 18);
  trans4(E, s[0], s[4], s[1], s[5], 64, 64, 83, 36);
  trans8_end(s, O, E, false);
  transpose_4x4(&s[0], &s[1], &s[2], &s[3]);
  transpose_4x4(&s[4], &s[5], &s[6], &s[7]);
  swp_element(&s[5], &s[3]);
  swp_element(&s[5], &s[6]);
  swp_element(&s[1], &s[4]);
  swp_element(&s[4], &s[2]);
  for (int i = 0; i < 4; i++)
    vst1q_s16(&src[(i+4)*8], vcombine_s16(s[i*2], s[i*2+1]));

  StoreToFrameWithRound<8>(dst, stride,  coeffs);
}

LIBDE265_INLINE void trans16( int32x4_t* O, 
                              int16x4_t s1, int16x4_t s3, int16x4_t s5, int16x4_t s7, 
                              int16x4_t s9, int16x4_t s11, int16x4_t s13, int16x4_t s15, 
                              const int16_t d0, const int16_t d1, const int16_t d2, const int16_t d3, 
                              const int16_t d4, const int16_t d5, const int16_t d6, const int16_t d7){
  //d0 90, d1 87, d2 80, d3 70, d4 57, d5 43, d6 25, d7 9
  O[0] = vmull_n_s16(s1, d0);
  O[0] = vmlal_n_s16(O[0], s3, d1);
  O[0] = vmlal_n_s16(O[0], s5, d2);
  O[0] = vmlal_n_s16(O[0], s7, d3);
  O[0] = vmlal_n_s16(O[0], s9, d4);
  O[0] = vmlal_n_s16(O[0], s11, d5);
  O[0] = vmlal_n_s16(O[0], s13, d6);
  O[0] = vmlal_n_s16(O[0], s15, d7);

  O[1] = vmull_n_s16(s1, d1);
  O[1] = vmlal_n_s16(O[1], s3, d4);
  O[1] = vmlal_n_s16(O[1], s5, d7);
  O[1] = vmlsl_n_s16(O[1], s7, d5);
  O[1] = vmlsl_n_s16(O[1], s9, d2);
  O[1] = vmlsl_n_s16(O[1], s11, d0);
  O[1] = vmlsl_n_s16(O[1], s13, d3);
  O[1] = vmlsl_n_s16(O[1], s15, d6);

  O[2] = vmull_n_s16(s1, d2);
  O[2] = vmlal_n_s16(O[2], s3, d7);
  O[2] = vmlsl_n_s16(O[2], s5, d3);
  O[2] = vmlsl_n_s16(O[2], s7, d1);
  O[2] = vmlsl_n_s16(O[2], s9, d6);
  O[2] = vmlal_n_s16(O[2], s11, d4);
  O[2] = vmlal_n_s16(O[2], s13, d0);
  O[2] = vmlal_n_s16(O[2], s15, d5);

  O[3] = vmull_n_s16(s1, d3);
  O[3] = vmlsl_n_s16(O[3], s3, d5);
  O[3] = vmlsl_n_s16(O[3], s5, d1);
  O[3] = vmlal_n_s16(O[3], s7, d7);
  O[3] = vmlal_n_s16(O[3], s9, d0);
  O[3] = vmlal_n_s16(O[3], s11, d6);
  O[3] = vmlsl_n_s16(O[3], s13, d2);
  O[3] = vmlsl_n_s16(O[3], s15, d4);

  O[4] = vmull_n_s16(s1, d4);
  O[4] = vmlsl_n_s16(O[4], s3, d2);
  O[4] = vmlsl_n_s16(O[4], s5, d6);
  O[4] = vmlal_n_s16(O[4], s7, d0);
  O[4] = vmlsl_n_s16(O[4], s9, d7);
  O[4] = vmlsl_n_s16(O[4], s11, d1);
  O[4] = vmlal_n_s16(O[4], s13, d5);
  O[4] = vmlal_n_s16(O[4], s15, d3);

  O[5] = vmull_n_s16(s1, d5);
  O[5] = vmlsl_n_s16(O[5], s3, d0);
  O[5] = vmlal_n_s16(O[5], s5, d4);
  O[5] = vmlal_n_s16(O[5], s7, d6);
  O[5] = vmlsl_n_s16(O[5], s9, d1);
  O[5] = vmlal_n_s16(O[5], s11, d3);
  O[5] = vmlal_n_s16(O[5], s13, d7);
  O[5] = vmlsl_n_s16(O[5], s15, d2);

  O[6] = vmull_n_s16(s1, d6);
  O[6] = vmlsl_n_s16(O[6], s3, d3);
  O[6] = vmlal_n_s16(O[6], s5, d0);
  O[6] = vmlsl_n_s16(O[6], s7, d2);
  O[6] = vmlal_n_s16(O[6], s9, d5);
  O[6] = vmlal_n_s16(O[6], s11, d7);
  O[6] = vmlsl_n_s16(O[6], s13, d4);
  O[6] = vmlal_n_s16(O[6], s15, d1);

  O[7] = vmull_n_s16(s1, d7);
  O[7] = vmlsl_n_s16(O[7], s3, d6);
  O[7] = vmlal_n_s16(O[7], s5, d5);
  O[7] = vmlsl_n_s16(O[7], s7, d4);
  O[7] = vmlal_n_s16(O[7], s9, d3);
  O[7] = vmlsl_n_s16(O[7], s11, d2);
  O[7] = vmlal_n_s16(O[7], s13, d1);
  O[7] = vmlsl_n_s16(O[7], s15, d0);
}

LIBDE265_INLINE void trans8_end2(int32x4_t* E, int32x4_t* EO, int32x4_t* EE){
  E[0] = vaddq_s32(EE[0], EO[0]);
  E[7] = vsubq_s32(EE[0], EO[0]);
  E[1] = vaddq_s32(EE[1], EO[1]);
  E[6] = vsubq_s32(EE[1], EO[1]);
  E[2] = vaddq_s32(EE[2], EO[2]);
  E[5] = vsubq_s32(EE[2], EO[2]);
  E[3] = vaddq_s32(EE[3], EO[3]);
  E[4] = vsubq_s32(EE[3], EO[3]); 
}

LIBDE265_INLINE void trains16_end(int16x4_t* s, int32x4_t* O, int32x4_t* E, bool is_row, int16_t offset){
  for (int i = 0; i < 8; i++)
  {
    int32x4_t tmp = vaddq_s32(E[i], O[i]);
    s[i] = vqrshrn_n_s32(tmp, 7);
  }
  
}

LIBDE265_INLINE void transpose8x4to4x8(const int16x8_t in[4], int16x4_t out[8]){
  const int16x8x2_t b0 = vtrnq_s16(in[0], in[1]);
  const int16x8x2_t b1 = vtrnq_s16(in[2], in[3]);
  const int32x4x2_t c0 = vtrnq_s32(vreinterpretq_s32_s16(b0.val[0]),
                                   vreinterpretq_s32_s16(b1.val[0]));
  const int32x4x2_t c1 = vtrnq_s32(vreinterpretq_s32_s16(b0.val[1]),
                                   vreinterpretq_s32_s16(b1.val[1]));
  out[0] = vreinterpret_s16_s32(vget_low_s32(c0.val[0]));
  out[4] = vreinterpret_s16_s32(vget_high_s32(c0.val[0]));

  out[2] = vreinterpret_s16_s32(vget_low_s32(c0.val[1]));
  out[6] = vreinterpret_s16_s32(vget_high_s32(c0.val[1]));

  out[1] = vreinterpret_s16_s32(vget_low_s32(c1.val[0]));
  out[5] = vreinterpret_s16_s32(vget_high_s32(c1.val[0]));

  out[3] = vreinterpret_s16_s32(vget_low_s32(c1.val[1]));
  out[7] = vreinterpret_s16_s32(vget_high_s32(c1.val[1]));
}

LIBDE265_INLINE void transpose4x8to8x4(int16x4_t in[8], int16x8_t out[4]){
  int16x4x2_t b0 = vtrn_s16(in[0], in[1]);
  int16x4x2_t b1 = vtrn_s16(in[2], in[3]);
  int16x4x2_t b2 = vtrn_s16(in[4], in[5]);
  int16x4x2_t b3 = vtrn_s16(in[6], in[7]);
  int32x2x2_t c0 = vtrn_s32(vreinterpret_s32_s16(b0.val[0]),
                             vreinterpret_s32_s16(b1.val[0]));
  int32x2x2_t c1 = vtrn_s32(vreinterpret_s32_s16(b0.val[1]),
                             vreinterpret_s32_s16(b1.val[1]));
  int32x2x2_t c2 = vtrn_s32(vreinterpret_s32_s16(b2.val[0]),
                             vreinterpret_s32_s16(b3.val[0]));
  int32x2x2_t c3 = vtrn_s32(vreinterpret_s32_s16(b2.val[1]),
                             vreinterpret_s32_s16(b3.val[1]));
  out[0] = vcombine_s16(vreinterpret_s16_s32(c0.val[0]),
                        vreinterpret_s16_s32(c2.val[0]));
  out[1] = vcombine_s16(vreinterpret_s16_s32(c1.val[0]),
                        vreinterpret_s16_s32(c3.val[0]));
  out[2] = vcombine_s16(vreinterpret_s16_s32(c0.val[1]),
                        vreinterpret_s16_s32(c2.val[1]));
  out[3] = vcombine_s16(vreinterpret_s16_s32(c1.val[1]),
                        vreinterpret_s16_s32(c3.val[1]));
}

void ff_hevc_transform_16x16_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit){

  auto* src = static_cast<int16_t*>(coeffs);
  int16x4_t s[16];
  int32x4_t O[8];
  int32x4_t E[8];
  int32x4_t EO[4];
  int32x4_t EE[4];

  for (int idx = 0; idx < col_limit; idx++)
  {
    for (int i = 0; i < 16; i++)
       s[i] = vld1_s16(&src[i*16]);
    // for (int i = 0; i < 8; i++)
    //   s[i] = vld1_s16(&src[16+i*32]);
    trans16(O, s[1], s[3], s[5], s[7], s[9], s[11], s[13], s[15], 90, 87, 80, 70, 57, 43, 25, 9);
    // for (int i = 0; i < 8; i++)
    //   s[i] = vld1_s16(&src[i*32]);
    trans8(EO, s[2], s[6], s[10], s[14], 89, 75, 50, 18);
    trans4(EE, s[0], s[4], s[8], s[12], 64, 64, 83, 36);
    trans8_end2(E, EO, EE);
    for (int i = 0; i < 8; i++)
    {
      int32x4_t tmp0;
      int16x4_t tmp1;

      tmp0 = vaddq_s32(E[i], O[i]);
      tmp1 = vqrshrn_n_s32(tmp0, 7);
      vst1_s16(&src[i*16], tmp1);

      tmp0 = vsubq_s32(E[7-i], O[7-i]);
      tmp1 = vqrshrn_n_s32(tmp0, 7);
      vst1_s16(&src[128+i*16], tmp1);
    }
    src+=4;
  }

  for (int idx = 0; idx < 4; idx++)
  {
    src = static_cast<int16_t*>(coeffs)+ idx*64;
    int16x8_t tmp[8];
    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < 4; i++)
       tmp[i] = vld1q_s16(&src[i*16+j*8]);
      transpose8x4to4x8(tmp, &s[j*8]);
    }
    trans16(O, s[1], s[3], s[5], s[7], s[9], s[11], s[13], s[15], 90, 87, 80, 70, 57, 43, 25, 9);
    trans8(EO, s[2], s[6], s[10], s[14], 89, 75, 50, 18);
    trans4(EE, s[0], s[4], s[8], s[12], 64, 64, 83, 36);
    trans8_end2(E, EO, EE);
    for (int i = 0; i < 8; i++)
    {
      int32x4_t tmp0;
      tmp0 = vaddq_s32(E[i], O[i]);
      s[i] = vqrshrn_n_s32(tmp0, 12);
      tmp0 = vsubq_s32(E[7-i], O[7-i]);
      s[8+i] = vqrshrn_n_s32(tmp0, 12);
    }
    for (int j = 0; j < 2; j++)
    {
      transpose4x8to8x4(&s[j*8], &tmp[j*4]);
      auto* dst = src + j*8;
      for (int i = 0; i < 4; i++)
      {
        vst1q_s16(dst+i*16, tmp[j*4+i]);
      }
    }
  }
  StoreToFrameWithRound<16>(dst, stride,  coeffs);
}
/*
int DCT32[32][32] =
{ 
  { 64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64,  64}, 
  { 90,  90,  88,  85,  82,  78,  73,  67,  61,  54,  46,  38,  31,  22,  13,   4,  -4, -13, -22, -31, -38, -46, -54, -61, -67, -73, -78, -82, -85, -88, -90, -90}, 
  { 90,  87,  80,  70,  57,  43,  25,   9,  -9, -25, -43, -57, -70, -80, -87, -90, -90, -87, -80, -70, -57, -43, -25,  -9,   9,  25,  43,  57,  70,  80,  87,  90}, 
  { 90,  82,  67,  46,  22,  -4, -31, -54, -73, -85, -90, -88, -78, -61, -38, -13,  13,  38,  61,  78,  88,  90,  85,  73,  54,  31,   4, -22, -46, -67, -82, -90}, 
  { 89,  75,  50,  18, -18, -50, -75, -89, -89, -75, -50, -18,  18,  50,  75,  89,  89,  75,  50,  18, -18, -50, -75, -89, -89, -75, -50, -18,  18,  50,  75,  89}, 
  { 88,  67,  31, -13, -54, -82, -90, -78, -46,  -4,  38,  73,  90,  85,  61,  22, -22, -61, -85, -90, -73, -38,   4,  46,  78,  90,  82,  54,  13, -31, -67, -88}, 
  { 87,  57,   9, -43, -80, -90, -70, -25,  25,  70,  90,  80,  43,  -9, -57, -87, -87, -57,  -9,  43,  80,  90,  70,  25, -25, -70, -90, -80, -43,   9,  57,  87}, 
  { 85,  46, -13, -67, -90, -73, -22,  38,  82,  88,  54,  -4, -61, -90, -78, -31,  31,  78,  90,  61,   4, -54, -88, -82, -38,  22,  73,  90,  67,  13, -46, -85}, 
  { 83,  36, -36, -83, -83, -36,  36,  83,  83,  36, -36, -83, -83, -36,  36,  83,  83,  36, -36, -83, -83, -36,  36,  83,  83,  36, -36, -83, -83, -36,  36,  83}, 
  { 82,  22, -54, -90, -61,  13,  78,  85,  31, -46, -90, -67,   4,  73,  88,  38, -38, -88, -73,  -4,  67,  90,  46, -31, -85, -78, -13,  61,  90,  54, -22, -82}, 
  { 80,   9, -70, -87, -25,  57,  90,  43, -43, -90, -57,  25,  87,  70,  -9, -80, -80,  -9,  70,  87,  25, -57, -90, -43,  43,  90,  57, -25, -87, -70,   9,  80}, 
  { 78,  -4, -82, -73,  13,  85,  67, -22, -88, -61,  31,  90,  54, -38, -90, -46,  46,  90,  38, -54, -90, -31,  61,  88,  22, -67, -85, -13,  73,  82,   4, -78}, 
  { 75, -18, -89, -50,  50,  89,  18, -75, -75,  18,  89,  50, -50, -89, -18,  75,  75, -18, -89, -50,  50,  89,  18, -75, -75,  18,  89,  50, -50, -89, -18,  75}, 
  { 73, -31, -90, -22,  78,  67, -38, -90, -13,  82,  61, -46, -88,  -4,  85,  54, -54, -85,   4,  88,  46, -61, -82,  13,  90,  38, -67, -78,  22,  90,  31, -73}, 
  { 70, -43, -87,   9,  90,  25, -80, -57,  57,  80, -25, -90,  -9,  87,  43, -70, -70,  43,  87,  -9, -90, -25,  80,  57, -57, -80,  25,  90,   9, -87, -43,  70}, 
  { 67, -54, -78,  38,  85, -22, -90,   4,  90,  13, -88, -31,  82,  46, -73, -61,  61,  73, -46, -82,  31,  88, -13, -90,  -4,  90,  22, -85, -38,  78,  54, -67}, 
  { 64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64,  64, -64, -64,  64}, 
  { 61, -73, -46,  82,  31, -88, -13,  90,  -4, -90,  22,  85, -38, -78,  54,  67, -67, -54,  78,  38, -85, -22,  90,   4, -90,  13,  88, -31, -82,  46,  73, -61}, 
  { 57, -80, -25,  90,  -9, -87,  43,  70, -70, -43,  87,   9, -90,  25,  80, -57, -57,  80,  25, -90,   9,  87, -43, -70,  70,  43, -87,  -9,  90, -25, -80,  57}, 
  { 54, -85,  -4,  88, -46, -61,  82,  13, -90,  38,  67, -78, -22,  90, -31, -73,  73,  31, -90,  22,  78, -67, -38,  90, -13, -82,  61,  46, -88,   4,  85, -54}, 
  { 50, -89,  18,  75, -75, -18,  89, -50, -50,  89, -18, -75,  75,  18, -89,  50,  50, -89,  18,  75, -75, -18,  89, -50, -50,  89, -18, -75,  75,  18, -89,  50}, 
  { 46, -90,  38,  54, -90,  31,  61, -88,  22,  67, -85,  13,  73, -82,   4,  78, -78,  -4,  82, -73, -13,  85, -67, -22,  88, -61, -31,  90, -54, -38,  90, -46}, 
  { 43, -90,  57,  25, -87,  70,   9, -80,  80,  -9, -70,  87, -25, -57,  90, -43, -43,  90, -57, -25,  87, -70,  -9,  80, -80,   9,  70, -87,  25,  57, -90,  43}, 
  { 38, -88,  73,  -4, -67,  90, -46, -31,  85, -78,  13,  61, -90,  54,  22, -82,  82, -22, -54,  90, -61, -13,  78, -85,  31,  46, -90,  67,   4, -73,  88, -38}, 
  { 36, -83,  83, -36, -36,  83, -83,  36,  36, -83,  83, -36, -36,  83, -83,  36,  36, -83,  83, -36, -36,  83, -83,  36,  36, -83,  83, -36, -36,  83, -83,  36}, 
  { 31, -78,  90, -61,   4,  54, -88,  82, -38, -22,  73, -90,  67, -13, -46,  85, -85,  46,  13, -67,  90, -73,  22,  38, -82,  88, -54,  -4,  61, -90,  78, -31}, 
  { 25, -70,  90, -80,  43,   9, -57,  87, -87,  57,  -9, -43,  80, -90,  70, -25, -25,  70, -90,  80, -43,  -9,  57, -87,  87, -57,   9,  43, -80,  90, -70,  25}, 
  { 22, -61,  85, -90,  73, -38,  -4,  46, -78,  90, -82,  54, -13, -31,  67, -88,  88, -67,  31,  13, -54,  82, -90,  78, -46,   4,  38, -73,  90, -85,  61, -22}, 
  { 18, -50,  75, -89,  89, -75,  50, -18, -18,  50, -75,  89, -89,  75, -50,  18,  18, -50,  75, -89,  89, -75,  50, -18, -18,  50, -75,  89, -89,  75, -50,  18}, 
  { 13, -38,  61, -78,  88, -90,  85, -73,  54, -31,   4,  22, -46,  67, -82,  90, -90,  82, -67,  46, -22,  -4,  31, -54,  73, -85,  90, -88,  78, -61,  38, -13}, 
  {  9, -25,  43, -57,  70, -80,  87, -90,  90, -87,  80, -70,  57, -43,  25,  -9,  -9,  25, -43,  57, -70,  80, -87,  90, -90,  87, -80,  70, -57,  43, -25,   9}, 
  {  4, -13,  22, -31,  38, -46,  54, -61,  67, -73,  78, -82,  85, -88,  90, -90,  90, -90,  88, -85,  82, -78,  73, -67,  61, -54,  46, -38,  31, -22,  13,  -4}  
}
*/

LIBDE265_INLINE void  trans32(int32x4_t* O, int16x4_t* s){

  O[0] = vmull_n_s16(s[1], 90);/*90*/
  O[0] = vmlal_n_s16(O[0], s[3], 90);/*90*/
  O[0] = vmlal_n_s16(O[0], s[5], 88);/*88*/
  O[0] = vmlal_n_s16(O[0], s[7], 85);/*85*/
  O[0] = vmlal_n_s16(O[0], s[9], 82);/*82*/
  O[0] = vmlal_n_s16(O[0], s[11], 78);/*78*/
  O[0] = vmlal_n_s16(O[0], s[13], 73);/*73*/
  O[0] = vmlal_n_s16(O[0], s[15], 67);/*67*/
  O[0] = vmlal_n_s16(O[0], s[17], 61);/*61*/
  O[0] = vmlal_n_s16(O[0], s[19], 54);/*54*/
  O[0] = vmlal_n_s16(O[0], s[21], 46);/*46*/
  O[0] = vmlal_n_s16(O[0], s[23], 38);/*38*/
  O[0] = vmlal_n_s16(O[0], s[25], 31);/*31*/
  O[0] = vmlal_n_s16(O[0], s[27], 22);/*22*/
  O[0] = vmlal_n_s16(O[0], s[29], 13);/*13*/
  O[0] = vmlal_n_s16(O[0], s[31], 4);/*4*/

  O[1] = vmull_n_s16(s[1], 90);/*90*/
  O[1] = vmlal_n_s16(O[1], s[3], 82);/*82*/
  O[1] = vmlal_n_s16(O[1], s[5], 67);/*67*/
  O[1] = vmlal_n_s16(O[1], s[7], 46);/*46*/
  O[1] = vmlal_n_s16(O[1], s[9], 22);/*22*/
  O[1] = vmlal_n_s16(O[1], s[11], -4);/*-4*/
  O[1] = vmlal_n_s16(O[1], s[13], -31);/*-31*/
  O[1] = vmlal_n_s16(O[1], s[15], -54);/*-54*/
  O[1] = vmlal_n_s16(O[1], s[17], -73);/*-73*/
  O[1] = vmlal_n_s16(O[1], s[19], -85);/*-85*/
  O[1] = vmlal_n_s16(O[1], s[21], -90);/*-90*/
  O[1] = vmlal_n_s16(O[1], s[23], -88);/*-88*/
  O[1] = vmlal_n_s16(O[1], s[25], -78);/*-78*/
  O[1] = vmlal_n_s16(O[1], s[27], -61);/*-61*/
  O[1] = vmlal_n_s16(O[1], s[29], -38);/*-38*/
  O[1] = vmlal_n_s16(O[1], s[31], -13);/*-13*/

  O[2] = vmull_n_s16(s[1], 88);/*88*/
  O[2] = vmlal_n_s16(O[2], s[3], 67);/*67*/
  O[2] = vmlal_n_s16(O[2], s[5], 31);/*31*/
  O[2] = vmlal_n_s16(O[2], s[7], -13);/*-13*/
  O[2] = vmlal_n_s16(O[2], s[9], -54);/*-54*/
  O[2] = vmlal_n_s16(O[2], s[11], -82);/*-82*/
  O[2] = vmlal_n_s16(O[2], s[13], -90);/*-90*/
  O[2] = vmlal_n_s16(O[2], s[15], -78);/*-78*/
  O[2] = vmlal_n_s16(O[2], s[17], -46);/*-46*/
  O[2] = vmlal_n_s16(O[2], s[19], -4);/*-4*/
  O[2] = vmlal_n_s16(O[2], s[21], 38);/*38*/
  O[2] = vmlal_n_s16(O[2], s[23], 73);/*73*/
  O[2] = vmlal_n_s16(O[2], s[25], 90);/*90*/
  O[2] = vmlal_n_s16(O[2], s[27], 85);/*85*/
  O[2] = vmlal_n_s16(O[2], s[29], 61);/*61*/
  O[2] = vmlal_n_s16(O[2], s[31], 22);/*22*/

  O[3] = vmull_n_s16(s[1], 85);/*85*/
  O[3] = vmlal_n_s16(O[3], s[3], 46);/*46*/
  O[3] = vmlal_n_s16(O[3], s[5], -13);/*-13*/
  O[3] = vmlal_n_s16(O[3], s[7], -67);/*-67*/
  O[3] = vmlal_n_s16(O[3], s[9], -90);/*-90*/
  O[3] = vmlal_n_s16(O[3], s[11], -73);/*-73*/
  O[3] = vmlal_n_s16(O[3], s[13], -22);/*-22*/
  O[3] = vmlal_n_s16(O[3], s[15], 38);/*38*/
  O[3] = vmlal_n_s16(O[3], s[17], 82);/*82*/
  O[3] = vmlal_n_s16(O[3], s[19], 88);/*88*/
  O[3] = vmlal_n_s16(O[3], s[21], 54);/*54*/
  O[3] = vmlal_n_s16(O[3], s[23], -4);/*-4*/
  O[3] = vmlal_n_s16(O[3], s[25], -61);/*-61*/
  O[3] = vmlal_n_s16(O[3], s[27], -90);/*-90*/
  O[3] = vmlal_n_s16(O[3], s[29], -78);/*-78*/
  O[3] = vmlal_n_s16(O[3], s[31], -31);/*-31*/

  O[4] = vmull_n_s16(s[1], 82);/*82*/
  O[4] = vmlal_n_s16(O[4], s[3], 22);/*22*/
  O[4] = vmlal_n_s16(O[4], s[5], -54);/*-54*/
  O[4] = vmlal_n_s16(O[4], s[7], -90);/*-90*/
  O[4] = vmlal_n_s16(O[4], s[9], -61);/*-61*/
  O[4] = vmlal_n_s16(O[4], s[11], 13);/*13*/
  O[4] = vmlal_n_s16(O[4], s[13], 78);/*78*/
  O[4] = vmlal_n_s16(O[4], s[15], 85);/*85*/
  O[4] = vmlal_n_s16(O[4], s[17], 31);/*31*/
  O[4] = vmlal_n_s16(O[4], s[19], -46);/*-46*/
  O[4] = vmlal_n_s16(O[4], s[21], -90);/*-90*/
  O[4] = vmlal_n_s16(O[4], s[23], -67);/*-67*/
  O[4] = vmlal_n_s16(O[4], s[25], 4);/*4*/
  O[4] = vmlal_n_s16(O[4], s[27], 73);/*73*/
  O[4] = vmlal_n_s16(O[4], s[29], 88);/*88*/
  O[4] = vmlal_n_s16(O[4], s[31], 38);/*38*/

  O[5] = vmull_n_s16(s[1], 78);/*78*/
  O[5] = vmlal_n_s16(O[5], s[3], -4);/*-4*/
  O[5] = vmlal_n_s16(O[5], s[5], -82);/*-82*/
  O[5] = vmlal_n_s16(O[5], s[7], -73);/*-73*/
  O[5] = vmlal_n_s16(O[5], s[9], 13);/*13*/
  O[5] = vmlal_n_s16(O[5], s[11], 85);/*85*/
  O[5] = vmlal_n_s16(O[5], s[13], 67);/*67*/
  O[5] = vmlal_n_s16(O[5], s[15], -22);/*-22*/
  O[5] = vmlal_n_s16(O[5], s[17], -88);/*-88*/
  O[5] = vmlal_n_s16(O[5], s[19], -61);/*-61*/
  O[5] = vmlal_n_s16(O[5], s[21], 31);/*31*/
  O[5] = vmlal_n_s16(O[5], s[23], 90);/*90*/
  O[5] = vmlal_n_s16(O[5], s[25], 54);/*54*/
  O[5] = vmlal_n_s16(O[5], s[27], -38);/*-38*/
  O[5] = vmlal_n_s16(O[5], s[29], -90);/*-90*/
  O[5] = vmlal_n_s16(O[5], s[31], -46);/*-46*/

  O[6] = vmull_n_s16(s[1], 73);/*73*/
  O[6] = vmlal_n_s16(O[6], s[3], -31);/*-31*/
  O[6] = vmlal_n_s16(O[6], s[5], -90);/*-90*/
  O[6] = vmlal_n_s16(O[6], s[7], -22);/*-22*/
  O[6] = vmlal_n_s16(O[6], s[9], 78);/*78*/
  O[6] = vmlal_n_s16(O[6], s[11], 67);/*67*/
  O[6] = vmlal_n_s16(O[6], s[13], -38);/*-38*/
  O[6] = vmlal_n_s16(O[6], s[15], -90);/*-90*/
  O[6] = vmlal_n_s16(O[6], s[17], -13);/*-13*/
  O[6] = vmlal_n_s16(O[6], s[19], 82);/*82*/
  O[6] = vmlal_n_s16(O[6], s[21], 61);/*61*/
  O[6] = vmlal_n_s16(O[6], s[23], -46);/*-46*/
  O[6] = vmlal_n_s16(O[6], s[25], -88);/*-88*/
  O[6] = vmlal_n_s16(O[6], s[27], -4);/*-4*/
  O[6] = vmlal_n_s16(O[6], s[29], 85);/*85*/
  O[6] = vmlal_n_s16(O[6], s[31], 54);/*54*/

  O[7] = vmull_n_s16(s[1], 67);/*67*/
  O[7] = vmlal_n_s16(O[7], s[3], -54);/*-54*/
  O[7] = vmlal_n_s16(O[7], s[5], -78);/*-78*/
  O[7] = vmlal_n_s16(O[7], s[7], 38);/*38*/
  O[7] = vmlal_n_s16(O[7], s[9], 85);/*85*/
  O[7] = vmlal_n_s16(O[7], s[11], -22);/*-22*/
  O[7] = vmlal_n_s16(O[7], s[13], -90);/*-90*/
  O[7] = vmlal_n_s16(O[7], s[15], 4);/*4*/
  O[7] = vmlal_n_s16(O[7], s[17], 90);/*90*/
  O[7] = vmlal_n_s16(O[7], s[19], 13);/*13*/
  O[7] = vmlal_n_s16(O[7], s[21], -88);/*-88*/
  O[7] = vmlal_n_s16(O[7], s[23], -31);/*-31*/
  O[7] = vmlal_n_s16(O[7], s[25], 82);/*82*/
  O[7] = vmlal_n_s16(O[7], s[27], 46);/*46*/
  O[7] = vmlal_n_s16(O[7], s[29], -73);/*-73*/
  O[7] = vmlal_n_s16(O[7], s[31], -61);/*-61*/

  O[8] = vmull_n_s16(s[1], 61);/*61*/
  O[8] = vmlal_n_s16(O[8], s[3], -73);/*-73*/
  O[8] = vmlal_n_s16(O[8], s[5], -46);/*-46*/
  O[8] = vmlal_n_s16(O[8], s[7], 82);/*82*/
  O[8] = vmlal_n_s16(O[8], s[9], 31);/*31*/
  O[8] = vmlal_n_s16(O[8], s[11], -88);/*-88*/
  O[8] = vmlal_n_s16(O[8], s[13], -13);/*-13*/
  O[8] = vmlal_n_s16(O[8], s[15], 90);/*90*/
  O[8] = vmlal_n_s16(O[8], s[17], -4);/*-4*/
  O[8] = vmlal_n_s16(O[8], s[19], -90);/*-90*/
  O[8] = vmlal_n_s16(O[8], s[21], 22);/*22*/
  O[8] = vmlal_n_s16(O[8], s[23], 85);/*85*/
  O[8] = vmlal_n_s16(O[8], s[25], -38);/*-38*/
  O[8] = vmlal_n_s16(O[8], s[27], -78);/*-78*/
  O[8] = vmlal_n_s16(O[8], s[29], 54);/*54*/
  O[8] = vmlal_n_s16(O[8], s[31], 67);/*67*/

  O[9] = vmull_n_s16(s[1], 54);/*54*/
  O[9] = vmlal_n_s16(O[9], s[3], -85);/*-85*/
  O[9] = vmlal_n_s16(O[9], s[5], -4);/*-4*/
  O[9] = vmlal_n_s16(O[9], s[7], 88);/*88*/
  O[9] = vmlal_n_s16(O[9], s[9], -46);/*-46*/
  O[9] = vmlal_n_s16(O[9], s[11], -61);/*-61*/
  O[9] = vmlal_n_s16(O[9], s[13], 82);/*82*/
  O[9] = vmlal_n_s16(O[9], s[15], 13);/*13*/
  O[9] = vmlal_n_s16(O[9], s[17], -90);/*-90*/
  O[9] = vmlal_n_s16(O[9], s[19], 38);/*38*/
  O[9] = vmlal_n_s16(O[9], s[21], 67);/*67*/
  O[9] = vmlal_n_s16(O[9], s[23], -78);/*-78*/
  O[9] = vmlal_n_s16(O[9], s[25], -22);/*-22*/
  O[9] = vmlal_n_s16(O[9], s[27], 90);/*90*/
  O[9] = vmlal_n_s16(O[9], s[29], -31);/*-31*/
  O[9] = vmlal_n_s16(O[9], s[31], -73);/*-73*/

  O[10] = vmull_n_s16(s[1], 46);/*46*/
  O[10] = vmlal_n_s16(O[10], s[3], -90);/*-90*/
  O[10] = vmlal_n_s16(O[10], s[5], 38);/*38*/
  O[10] = vmlal_n_s16(O[10], s[7], 54);/*54*/
  O[10] = vmlal_n_s16(O[10], s[9], -90);/*-90*/
  O[10] = vmlal_n_s16(O[10], s[11], 31);/*31*/
  O[10] = vmlal_n_s16(O[10], s[13], 61);/*61*/
  O[10] = vmlal_n_s16(O[10], s[15], -88);/*-88*/
  O[10] = vmlal_n_s16(O[10], s[17], 22);/*22*/
  O[10] = vmlal_n_s16(O[10], s[19], 67);/*67*/
  O[10] = vmlal_n_s16(O[10], s[21], -85);/*-85*/
  O[10] = vmlal_n_s16(O[10], s[23], 13);/*13*/
  O[10] = vmlal_n_s16(O[10], s[25], 73);/*73*/
  O[10] = vmlal_n_s16(O[10], s[27], -82);/*-82*/
  O[10] = vmlal_n_s16(O[10], s[29], 4);/*4*/
  O[10] = vmlal_n_s16(O[10], s[31], 78);/*78*/

  O[11] = vmull_n_s16(s[1], 38);/*38*/
  O[11] = vmlal_n_s16(O[11], s[3], -88);/*-88*/
  O[11] = vmlal_n_s16(O[11], s[5], 73);/*73*/
  O[11] = vmlal_n_s16(O[11], s[7], -4);/*-4*/
  O[11] = vmlal_n_s16(O[11], s[9], -67);/*-67*/
  O[11] = vmlal_n_s16(O[11], s[11], 90);/*90*/
  O[11] = vmlal_n_s16(O[11], s[13], -46);/*-46*/
  O[11] = vmlal_n_s16(O[11], s[15], -31);/*-31*/
  O[11] = vmlal_n_s16(O[11], s[17], 85);/*85*/
  O[11] = vmlal_n_s16(O[11], s[19], -78);/*-78*/
  O[11] = vmlal_n_s16(O[11], s[21], 13);/*13*/
  O[11] = vmlal_n_s16(O[11], s[23], 61);/*61*/
  O[11] = vmlal_n_s16(O[11], s[25], -90);/*-90*/
  O[11] = vmlal_n_s16(O[11], s[27], 54);/*54*/
  O[11] = vmlal_n_s16(O[11], s[29], 22);/*22*/
  O[11] = vmlal_n_s16(O[11], s[31], -82);/*-82*/

  O[12] = vmull_n_s16(s[1], 31);/*31*/
  O[12] = vmlal_n_s16(O[12], s[3], -78);/*-78*/
  O[12] = vmlal_n_s16(O[12], s[5], 90);/*90*/
  O[12] = vmlal_n_s16(O[12], s[7], -61);/*-61*/
  O[12] = vmlal_n_s16(O[12], s[9], 4);/*4*/
  O[12] = vmlal_n_s16(O[12], s[11], 54);/*54*/
  O[12] = vmlal_n_s16(O[12], s[13], -88);/*-88*/
  O[12] = vmlal_n_s16(O[12], s[15], 82);/*82*/
  O[12] = vmlal_n_s16(O[12], s[17], -38);/*-38*/
  O[12] = vmlal_n_s16(O[12], s[19], -22);/*-22*/
  O[12] = vmlal_n_s16(O[12], s[21], 73);/*73*/
  O[12] = vmlal_n_s16(O[12], s[23], -90);/*-90*/
  O[12] = vmlal_n_s16(O[12], s[25], 67);/*67*/
  O[12] = vmlal_n_s16(O[12], s[27], -13);/*-13*/
  O[12] = vmlal_n_s16(O[12], s[29], -46);/*-46*/
  O[12] = vmlal_n_s16(O[12], s[31], 85);/*85*/

  O[13] = vmull_n_s16(s[1], 22);/*22*/
  O[13] = vmlal_n_s16(O[13], s[3], -61);/*-61*/
  O[13] = vmlal_n_s16(O[13], s[5], 85);/*85*/
  O[13] = vmlal_n_s16(O[13], s[7], -90);/*-90*/
  O[13] = vmlal_n_s16(O[13], s[9], 73);/*73*/
  O[13] = vmlal_n_s16(O[13], s[11], -38);/*-38*/
  O[13] = vmlal_n_s16(O[13], s[13], -4);/*-4*/
  O[13] = vmlal_n_s16(O[13], s[15], 46);/*46*/
  O[13] = vmlal_n_s16(O[13], s[17], -78);/*-78*/
  O[13] = vmlal_n_s16(O[13], s[19], 90);/*90*/
  O[13] = vmlal_n_s16(O[13], s[21], -82);/*-82*/
  O[13] = vmlal_n_s16(O[13], s[23], 54);/*54*/
  O[13] = vmlal_n_s16(O[13], s[25], -13);/*-13*/
  O[13] = vmlal_n_s16(O[13], s[27], -31);/*-31*/
  O[13] = vmlal_n_s16(O[13], s[29], 67);/*67*/
  O[13] = vmlal_n_s16(O[13], s[31], -88);/*-88*/

  O[14] = vmull_n_s16(s[1], 13);/*13*/
  O[14] = vmlal_n_s16(O[14], s[3], -38);/*-38*/
  O[14] = vmlal_n_s16(O[14], s[5], 61);/*61*/
  O[14] = vmlal_n_s16(O[14], s[7], -78);/*-78*/
  O[14] = vmlal_n_s16(O[14], s[9], 88);/*88*/
  O[14] = vmlal_n_s16(O[14], s[11], -90);/*-90*/
  O[14] = vmlal_n_s16(O[14], s[13], 85);/*85*/
  O[14] = vmlal_n_s16(O[14], s[15], -73);/*-73*/
  O[14] = vmlal_n_s16(O[14], s[17], 54);/*54*/
  O[14] = vmlal_n_s16(O[14], s[19], -31);/*-31*/
  O[14] = vmlal_n_s16(O[14], s[21], 4);/*4*/
  O[14] = vmlal_n_s16(O[14], s[23], 22);/*22*/
  O[14] = vmlal_n_s16(O[14], s[25], -46);/*-46*/
  O[14] = vmlal_n_s16(O[14], s[27], 67);/*67*/
  O[14] = vmlal_n_s16(O[14], s[29], -82);/*-82*/
  O[14] = vmlal_n_s16(O[14], s[31], 90);/*90*/

  O[15] = vmull_n_s16(s[1], 4);/*4*/
  O[15] = vmlal_n_s16(O[15], s[3], -13);/*-13*/
  O[15] = vmlal_n_s16(O[15], s[5], 22);/*22*/
  O[15] = vmlal_n_s16(O[15], s[7], -31);/*-31*/
  O[15] = vmlal_n_s16(O[15], s[9], 38);/*38*/
  O[15] = vmlal_n_s16(O[15], s[11], -46);/*-46*/
  O[15] = vmlal_n_s16(O[15], s[13], 54);/*54*/
  O[15] = vmlal_n_s16(O[15], s[15], -61);/*-61*/
  O[15] = vmlal_n_s16(O[15], s[17], 67);/*67*/
  O[15] = vmlal_n_s16(O[15], s[19], -73);/*-73*/
  O[15] = vmlal_n_s16(O[15], s[21], 78);/*78*/
  O[15] = vmlal_n_s16(O[15], s[23], -82);/*-82*/
  O[15] = vmlal_n_s16(O[15], s[25], 85);/*85*/
  O[15] = vmlal_n_s16(O[15], s[27], -88);/*-88*/
  O[15] = vmlal_n_s16(O[15], s[29], 90);/*90*/
  O[15] = vmlal_n_s16(O[15], s[31], -90);/*-90*/
}

LIBDE265_INLINE void trans16_end(int32x4_t* E, int32x4_t* EO, int32x4_t* EE){
  E[ 0] = vaddq_s32(EE[0], EO[0]);
  E[15] = vsubq_s32(EE[0], EO[0]);
  E[ 1] = vaddq_s32(EE[1], EO[1]);
  E[14] = vsubq_s32(EE[1], EO[1]);
  E[ 2] = vaddq_s32(EE[2], EO[2]);
  E[13] = vsubq_s32(EE[2], EO[2]);
  E[ 3] = vaddq_s32(EE[3], EO[3]);
  E[12] = vsubq_s32(EE[3], EO[3]);
  E[ 4] = vaddq_s32(EE[4], EO[4]);
  E[11] = vsubq_s32(EE[4], EO[4]);
  E[ 5] = vaddq_s32(EE[5], EO[5]);
  E[10] = vsubq_s32(EE[5], EO[5]);
  E[ 6] = vaddq_s32(EE[6], EO[6]);
  E[ 9] = vsubq_s32(EE[6], EO[6]);
  E[ 7] = vaddq_s32(EE[7], EO[7]);
  E[ 8] = vsubq_s32(EE[7], EO[7]); 
}

void ff_hevc_transform_32x32_add_8_neon(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride, int16_t col_limit){
  auto* src = static_cast<int16_t*>(coeffs);
  int16x4_t s[32];
  int32x4_t O[16];
  int32x4_t E[16];
  int32x4_t EE[8];
  int32x4_t EO[8];
  int32x4_t EEO[4];
  int32x4_t EEE[4];

  for (int idx = 0; idx < col_limit; idx++)
  {
    for (int i = 0; i < 32; i++)
       s[i] = vld1_s16(&src[i*32]);

    trans32(O, s);
    trans16(EO, s[2], s[6], s[10], s[14], s[18], s[22], s[26], s[30], 90, 87, 80, 70, 57, 43, 25, 9);
    trans8(EEO, s[4], s[12], s[20], s[28], 89, 75, 50, 18);
    trans4(EEE, s[0], s[8], s[16], s[24], 64, 64, 83, 36);
    trans8_end2(EE, EEO, EEE);
    trans16_end(E, EO, EE);
    for (int i = 0; i < 16; i++)
    {
      int32x4_t tmp0;
      int16x4_t tmp1;

      tmp0 = vaddq_s32(E[i], O[i]);
      tmp1 = vqrshrn_n_s32(tmp0, 7);
      vst1_s16(&src[i*32], tmp1);

      tmp0 = vsubq_s32(E[15-i], O[15-i]);
      tmp1 = vqrshrn_n_s32(tmp0, 7);
      vst1_s16(&src[512+i*32], tmp1);
    }
    src+=4;
  }

  for (int idx = 0; idx < 8; idx++)
  {
    src = static_cast<int16_t*>(coeffs)+ idx*128;
    int16x8_t tmp[4];
    for (int j = 0; j < 4; j++)
    {
      for (int i = 0; i < 4; i++)
       tmp[i] = vld1q_s16(&src[i*32+j*8]);
      transpose8x4to4x8(tmp, &s[j*8]);
    }
    trans32(O, s);
    trans16(EO, s[2], s[6], s[10], s[14], s[18], s[22], s[26], s[30], 90, 87, 80, 70, 57, 43, 25, 9);
    trans8(EEO, s[4], s[12], s[20], s[28], 89, 75, 50, 18);
    trans4(EEE, s[0], s[8], s[16], s[24], 64, 64, 83, 36);
    trans8_end2(EE, EEO, EEE);
    trans16_end(E, EO, EE);
    for (int i = 0; i < 16; i++)
    {
      int32x4_t tmp0;
      tmp0 = vaddq_s32(E[i], O[i]);
      s[i] = vqrshrn_n_s32(tmp0, 12);
      tmp0 = vsubq_s32(E[15-i], O[15-i]);
      s[16+i] = vqrshrn_n_s32(tmp0, 12);
    }
    for (int j = 0; j < 4; j++)
    {
      transpose4x8to8x4(&s[j*8], tmp);
      auto* dst = src + j*8;
      for (int i = 0; i < 4; i++)
      {
        vst1q_s16(dst+i*32, tmp[i]);
      }
    }
  }

  StoreToFrameWithRound<32>(dst, stride,  coeffs);

}

void ff_hevc_residual16_add_8_neon(uint8_t* _dst, ptrdiff_t _stride, const int16_t* coeffs, int nT, int bit_depth){
  uint8_t* dst = _dst;
  const int16_t* coef = coeffs;
  for (int i = 0; i < 4; i++)
  {
    uint8x8_t pred = vld1_u8(dst);
    int16x4_t residual = vld1_s16(coef);
    const uint16x8_t b =
          vaddw_u8(vreinterpretq_u16_s16(vcombine_s16(residual, residual)), pred);
    const uint8x8_t d = vqmovun_s16(vreinterpretq_s16_u16(b));
    StoreLo4(dst, d);
    coef +=4;
    dst += _stride;
  }
}

void ff_hevc_transform_skip_residual16 (int16_t *residual, const int16_t *coeffs, int nT, int tsShift,int bdShift){
  int shift = tsShift - bdShift;
  const int16_t* coef = coeffs;
  int16_t* resi = residual;
  for (int i = 0; i < 2; i++)
  {
    int16x8_t residual = vld1q_s16(coef);
    int16x8_t shifts = vdupq_n_s16(shift);
    residual = vqrshlq_s16(residual, shifts);
    vst1q_s16(resi, residual);
    coef += 8;
    resi += 8;
  }
}