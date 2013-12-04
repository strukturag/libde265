
#ifndef SSE_DCT_H
#define SSE_DCT_H

#include <stddef.h>
#include <stdint.h>

void ff_hevc_transform_skip_8_sse(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride);
void ff_hevc_transform_4x4_luma_add_8_sse4(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void ff_hevc_transform_4x4_add_8_sse4(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void ff_hevc_transform_8x8_add_8_sse4(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void ff_hevc_transform_16x16_add_8_sse4(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void ff_hevc_transform_32x32_add_8_sse4(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);

#endif
