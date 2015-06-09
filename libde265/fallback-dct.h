/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
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

#ifndef FALLBACK_DCT_H
#define FALLBACK_DCT_H

#include <stddef.h>
#include <stdint.h>


// --- decoding ---

void transform_skip_8_fallback(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);
void transform_bypass_8_fallback(uint8_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride);

void transform_skip_rdpcm_v_8_fallback(uint8_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride);
void transform_skip_rdpcm_h_8_fallback(uint8_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride);
void transform_bypass_rdpcm_v_8_fallback(uint8_t *dst, const int16_t *coeffs,int nT,ptrdiff_t stride);
void transform_bypass_rdpcm_h_8_fallback(uint8_t *dst, const int16_t *coeffs,int nT,ptrdiff_t stride);

void transform_4x4_luma_add_8_fallback(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);
void transform_4x4_add_8_fallback(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);
void transform_8x8_add_8_fallback(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);
void transform_16x16_add_8_fallback(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);
void transform_32x32_add_8_fallback(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride);


void transform_skip_16_fallback(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
void transform_bypass_16_fallback(uint16_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride, int bit_depth);

void transform_4x4_luma_add_16_fallback(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
void transform_4x4_add_16_fallback(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
void transform_8x8_add_16_fallback(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
void transform_16x16_add_16_fallback(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
void transform_32x32_add_16_fallback(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);

void rotate_coefficients_fallback(int16_t *coeff, int nT);


// --- encoding ---

void fdst_4x4_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void fdct_4x4_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void fdct_8x8_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void fdct_16x16_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void fdct_32x32_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);

void hadamard_4x4_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void hadamard_8x8_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void hadamard_16x16_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);
void hadamard_32x32_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride);

#endif
