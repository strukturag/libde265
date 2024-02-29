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

#include "fallback-intrapred.h"
#include "intrapred.h"

#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#elif defined(HAVE_ALLOCA_H)
# include <alloca.h>
#endif

#include <assert.h>
#include <algorithm>

void intra_pred_8_fallback(uint8_t *dst, int dstStride, int nT,int cIdx, uint8_t *border)
{
  intra_prediction_DC<uint8_t>(dst, dstStride, nT,cIdx, border);
}

void intra_pred_16_fallback(uint16_t *dst, int dstStride, int nT,int cIdx, uint16_t *border)
{
  intra_prediction_DC<uint16_t>(dst, dstStride, nT,cIdx, border);
}

void intra_prediction_angular_8_fallback(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border)
{
  intra_prediction_angular<uint8_t>(dst,dstStride, bit_depth,disableIntraBoundaryFilter, xB0,yB0,intraPredMode,nT,cIdx, border);
}

void intra_prediction_angular_16_fallback(uint16_t* dst, int dstStride,
                                          int bit_depth, bool disableIntraBoundaryFilter,
                                          int xB0,int yB0,
                                          enum IntraPredMode intraPredMode,
                                          int nT,int cIdx,
                                          uint16_t* border)
{
  intra_prediction_angular<uint16_t>(dst,dstStride, bit_depth,disableIntraBoundaryFilter, xB0,yB0,intraPredMode,nT,cIdx, border);
}

void intra_prediction_sample_filtering_8_fallback(const seq_parameter_set& sps,
                                                  uint8_t *p,
                                                  int nT, int cIdx,
                                                  enum IntraPredMode intraPredMode)
{
  intra_prediction_sample_filtering<uint8_t>(sps, p, nT, cIdx, intraPredMode); 
}

void intra_prediction_sample_filtering_16_fallback(const seq_parameter_set& sps,
                                                   uint16_t *p,
                                                   int nT, int cIdx,
                                                   enum IntraPredMode intraPredMode)
{
  intra_prediction_sample_filtering<uint16_t>(sps, p, nT, cIdx, intraPredMode); 
}

void intra_prediction_planar_8_fallback(uint8_t *dst, int dstStride, int nT,int cIdx, uint8_t *border)
{
  intra_prediction_planar<uint8_t>(dst, dstStride, nT, cIdx, border);
}

void intra_prediction_planar_16_fallback(uint16_t *dst, int dstStride, int nT,int cIdx, uint16_t *border)
{
  intra_prediction_planar<uint16_t>(dst, dstStride, nT, cIdx, border);
}

