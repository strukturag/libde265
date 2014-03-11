
#ifndef FALLBACK_DCT_H
#define FALLBACK_DCT_H

#include <stddef.h>
#include <stdint.h>


void transform_skip_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void transform_bypass_8_fallback(uint8_t *dst, int16_t *coeffs, int nT, ptrdiff_t stride);

void transform_4x4_luma_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void transform_4x4_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void transform_8x8_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void transform_16x16_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);
void transform_32x32_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);

#endif
