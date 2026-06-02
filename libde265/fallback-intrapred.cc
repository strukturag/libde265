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

#include "fallback-intrapred.h"
#include "intrapred.h"


template <class pixel_t>
void intra_pred_dc_fallback(pixel_t* dst, ptrdiff_t stride, int nT, int cIdx, const pixel_t* border)
{
  intra_prediction_DC<pixel_t>(dst, (int)stride, nT, cIdx, const_cast<pixel_t*>(border));
}

template <class pixel_t>
void intra_pred_planar_fallback(pixel_t* dst, ptrdiff_t stride, int nT, int cIdx, const pixel_t* border)
{
  intra_prediction_planar<pixel_t>(dst, (int)stride, nT, cIdx, const_cast<pixel_t*>(border));
}

template <class pixel_t>
void intra_pred_angular_fallback(pixel_t* dst, ptrdiff_t stride, int bit_depth, int disableBoundaryFilter,
                                 int xB0, int yB0, int mode, int nT, int cIdx, const pixel_t* border)
{
  intra_prediction_angular<pixel_t>(dst, (int)stride, bit_depth, (bool)disableBoundaryFilter,
                                    xB0, yB0, (enum IntraPredMode)mode, nT, cIdx,
                                    const_cast<pixel_t*>(border));
}


// explicit instantiations so the symbols can be installed into the acceleration table

template void intra_pred_dc_fallback<uint8_t> (uint8_t*,  ptrdiff_t, int, int, const uint8_t*);
template void intra_pred_dc_fallback<uint16_t>(uint16_t*, ptrdiff_t, int, int, const uint16_t*);
template void intra_pred_planar_fallback<uint8_t> (uint8_t*,  ptrdiff_t, int, int, const uint8_t*);
template void intra_pred_planar_fallback<uint16_t>(uint16_t*, ptrdiff_t, int, int, const uint16_t*);
template void intra_pred_angular_fallback<uint8_t> (uint8_t*,  ptrdiff_t, int, int, int, int, int, int, int, const uint8_t*);
template void intra_pred_angular_fallback<uint16_t>(uint16_t*, ptrdiff_t, int, int, int, int, int, int, int, const uint16_t*);
