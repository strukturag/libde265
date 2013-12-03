
#ifndef SSE_DCT_H
#define SSE_DCT_H

#include <stddef.h>
#include <stdint.h>

void ff_hevc_transform_4x4_luma_add_8_sse4(uint8_t *_dst, int16_t *coeffs,
                                           ptrdiff_t _stride);

#endif
