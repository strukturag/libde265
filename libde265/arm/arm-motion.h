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

#ifndef ARM_MOTION_H
#define ARM_MOTION_H

#include <stddef.h>
#include <stdint.h>

void put_pred_8_neon(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                     const int16_t __restrict__ *src, ptrdiff_t srcstride,
                     int width, int height);

void put_bipred_8_neon(uint8_t __restrict__ *dst, ptrdiff_t dststride,
                       const int16_t __restrict__ *src1,
                       const int16_t __restrict__ *src2, ptrdiff_t srcstride,
                       int width, int height);

// TODO: for noshift-variant, we do not need mx/my/mcbuffer
void mc_noshift_8_luma_neon(int16_t *dst, ptrdiff_t dststride,
                            const uint8_t *src, ptrdiff_t srcstride,
                            int width, int height,
                            int16_t* mcbuffer);

void mc_noshift_8_chroma_neon(int16_t *dst, ptrdiff_t dststride,
                              const uint8_t *src, ptrdiff_t srcstride,
                              int width, int height, int mx,
                              int my, int16_t* mcbuffer);

void mc_qpel_h1_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer);
void mc_qpel_h2_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer);
void mc_qpel_h3_8_neon(int16_t *dst, ptrdiff_t dststride,
                       const uint8_t *src, ptrdiff_t srcstride,
                       int width, int height,
                       int16_t* mcbuffer);

#endif
