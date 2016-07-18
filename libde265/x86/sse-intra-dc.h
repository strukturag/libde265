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

#ifndef SSE_INTRA_DC_H
#define SSE_INTRA_DC_H

#include <stddef.h>
#include <stdint.h>


void intra_dc_noavg_8_4x4_sse4(uint8_t* dst,int dstStride, uint8_t* border);
void intra_dc_avg_8_4x4_sse4(uint8_t* dst,int dstStride, uint8_t* border);

void intra_dc_noavg_8_8x8_sse4(uint8_t* dst,int dstStride, uint8_t* border);
void intra_dc_avg_8_8x8_sse4(uint8_t* dst,int dstStride, uint8_t* border);

void intra_dc_noavg_8_16x16_sse4(uint8_t* dst,int dstStride, uint8_t* border);
void intra_dc_avg_8_16x16_sse4(uint8_t* dst,int dstStride, uint8_t* border);

void intra_dc_noavg_8_32x32_sse4(uint8_t* dst,int dstStride, uint8_t* border);
void intra_dc_avg_8_32x32_sse4(uint8_t* dst,int dstStride, uint8_t* border);

#endif
