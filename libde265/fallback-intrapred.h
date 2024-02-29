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

#ifndef FALLBACK_INTRAPRED_H
#define FALLBACK_INTRAPRED_H

#include <stddef.h>
#include <stdint.h>

#include "intrapred.h"
#include "util.h"

// --- encoding ---
void intra_pred_8_fallback(uint8_t *dst, int dstStride, int nT,int cIdx, uint8_t *border);
void intra_pred_16_fallback(uint16_t *dst, int dstStride, int nT,int cIdx, uint16_t *border);

void intra_prediction_angular_8_fallback(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border);

void intra_prediction_angular_16_fallback(uint16_t* dst, int dstStride,
                                          int bit_depth, bool disableIntraBoundaryFilter,
                                          int xB0,int yB0,
                                          enum IntraPredMode intraPredMode,
                                          int nT,int cIdx,
                                          uint16_t* border);

void intra_prediction_sample_filtering_8_fallback(const seq_parameter_set& sps,
                                                  uint8_t *p,
                                                  int nT, int cIdx,
                                                  enum IntraPredMode intraPredMode);

void intra_prediction_sample_filtering_16_fallback(const seq_parameter_set& sps,
                                                   uint16_t *p,
                                                   int nT, int cIdx,
                                                   enum IntraPredMode intraPredMode);

void intra_prediction_planar_8_fallback(uint8_t *dst, int dstStride, int nT,int cIdx, uint8_t *border);
void intra_prediction_planar_16_fallback(uint16_t *dst, int dstStride, int nT,int cIdx, uint16_t *border);

#endif
