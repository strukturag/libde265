
#ifndef FALLBACK_DCT_H
#define FALLBACK_DCT_H

#include <stddef.h>
#include <stdint.h>


void transform_4x4_luma_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride);

#endif
