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

#ifndef SSE_MOTION_NEW_H
#define SSE_MOTION_NEW_H

#include <stddef.h>
#include <stdint.h>


void put_weighted_pred_8_sse(uint8_t *dst, ptrdiff_t dststride,
                             const int16_t *src, ptrdiff_t srcstride,
                             int width, int height,
                             int w,int o,int log2WD);

void put_weighted_bipred_8_sse(uint8_t *dst, ptrdiff_t dststride,
                               const int16_t *src1, const int16_t *src2, ptrdiff_t srcstride,
                               int width, int height,
                               int w1,int o1, int w2,int o2, int log2WD);

void put_hevc_luma_direct_8_sse(int16_t *dst, ptrdiff_t dststride,
                                const uint8_t *_src, ptrdiff_t _srcstride,
                                int width, int height,
                                int16_t* mcbuffer);

void put_hevc_chroma_direct_8_sse(int16_t *dst, ptrdiff_t dststride,
                                  const uint8_t *src, ptrdiff_t srcstride,
                                  int width, int height, int mx,
                                  int my, int16_t* mcbuffer);

#endif
