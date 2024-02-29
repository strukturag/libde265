#ifndef LIBDE265_NEON_COMMON_H
#define LIBDE265_NEON_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include <cassert>
#include <cstring>
#include <arm_neon.h>

template <typename T>
inline void ValueToMem(void* const buf, T val) {
  memcpy(buf, &val, sizeof(val));
}

// Load 4 uint8_t values into 4 lanes staring with |lane| * 4. 
template <int lane>
inline uint8x8_t Load4(const void* const buf, uint8x8_t val) {
  uint32_t temp;
  memcpy(&temp, buf, 4);
  return vreinterpret_u8_u32(
      vld1_lane_u32(&temp, vreinterpret_u32_u8(val), lane));
}

template <int lane>
inline uint8x16_t Load8(const void* const buf, uint8x16_t val) {
  uint64_t temp ;
  memcpy(&temp, buf, 8);
  return vreinterpretq_u8_u64(
    vld1q_lane_u64(&temp,vreinterpretq_u64_u8(val), lane));
}

// Blend two values based on weights that sum to 32.
inline uint8x8_t WeightedBlend(const uint8x8_t a, const uint8x8_t b,
                               const uint8x8_t a_weight,
                               const uint8x8_t b_weight) {
  const uint16x8_t a_product = vmull_u8(a, a_weight);
  const uint16x8_t sum = vmlal_u8(a_product, b, b_weight);

  return vrshrn_n_u16(sum, 5 /*log2(32)*/);
}

// For vertical operations the weights are one constant value.
inline uint8x8_t WeightedBlend(const uint8x8_t a, const uint8x8_t b,
                               const uint8_t weight) {
  return WeightedBlend(a, b, vdup_n_u8(32 - weight), vdup_n_u8(weight));
}


// Store val(uint32x2) low 8 bit value to * dest
template <int lane>
inline void intra_pred_DcStore_neon(void* const dest, int dstStride, int nT, const uint32x2_t val) {

  const uint8x16_t dc_dup = vdupq_lane_u8(vreinterpret_u8_u32(val),lane);
  auto *dst = static_cast<uint8_t *>(dest);

  int i = nT ;
  if(nT == 4) {
    do {
      ValueToMem<int32_t>(dst, vget_lane_u32(vreinterpret_u32_u8(vget_low_u8(dc_dup)),0));
      if(nT != 1) dst +=dstStride ;
    } while (--i != 0);
  }
  else if(nT == 8) {
    do {
      vst1_u8(dst, vget_low_u8(dc_dup));
      if(nT != 1) dst +=dstStride ;
    } while (--i != 0);
  }
  else if(nT == 16) {
    do {
      vst1q_u8(dst, dc_dup);
      if(nT != 1) dst +=dstStride ;
    } while (--i != 0);
  }
  else {
    assert(nT == 32);
    do {
      vst1q_u8(dst, dc_dup);
      vst1q_u8(dst + 16, dc_dup);
      if(nT != 1) dst +=dstStride ;
    } while (--i != 0);
    assert(nT == 32);
  }

}

#endif