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

#ifndef DE265_FALLBACK_INTRAPRED_H
#define DE265_FALLBACK_INTRAPRED_H

#include <stddef.h>
#include <stdint.h>

// Scalar fallback wrappers around the intra-prediction kernels in intrapred.h.
// They have plain (non-templated mode/flag) signatures so they can be stored in
// the acceleration_functions function-pointer table.

template <class pixel_t>
void intra_pred_dc_fallback(pixel_t* dst, ptrdiff_t stride, int nT, int cIdx, const pixel_t* border);

template <class pixel_t>
void intra_pred_planar_fallback(pixel_t* dst, ptrdiff_t stride, int nT, int cIdx, const pixel_t* border);

template <class pixel_t>
void intra_pred_angular_fallback(pixel_t* dst, ptrdiff_t stride, int bit_depth, int disableBoundaryFilter,
                                 int xB0, int yB0, int mode, int nT, int cIdx, const pixel_t* border);

#endif
