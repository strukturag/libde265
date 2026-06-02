/*
 * H.265 video codec.
 * Copyright (c) 2026 Dirk Farin <dirk.farin@gmail.com>
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

#ifndef SSE_TRANSFORM_AVX512_H
#define SSE_TRANSFORM_AVX512_H

#include <stddef.h>
#include <stdint.h>

// AVX-512 (F+BW) inverse DCT + add. Bit-identical to the SSE4.1 / AVX2 / scalar
// versions (verified by dev-tools/test-transform). Same signature as
// transform_add_8[].
void transform_32x32_add_8_avx512(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);

#endif
