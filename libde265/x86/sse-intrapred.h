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

#ifndef SSE_INTRAPRED_H
#define SSE_INTRAPRED_H

#include <stddef.h>
#include <stdint.h>

// SSE4.1 accelerated 8-bit intra prediction. All three are bit-identical to the
// scalar kernels in intrapred.h (verified by dev-tools/test-intrapred.cc).

void intra_pred_dc_8_sse4    (uint8_t* dst, ptrdiff_t stride, int nT, int cIdx, const uint8_t* border);
void intra_pred_planar_8_sse4(uint8_t* dst, ptrdiff_t stride, int nT, int cIdx, const uint8_t* border);
void intra_pred_angular_8_sse4(uint8_t* dst, ptrdiff_t stride, int bit_depth, int disableBoundaryFilter,
                               int xB0, int yB0, int mode, int nT, int cIdx, const uint8_t* border);

#endif
