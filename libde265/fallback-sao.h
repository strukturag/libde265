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

#ifndef FALLBACK_SAO_H
#define FALLBACK_SAO_H

#include <stddef.h>
#include <stdint.h>

// --- SAO ---

void sao_band_fallback_8bit(uint8_t* dst,int dststride, const uint8_t* src,int srcstride,
                            int width, int height,
                            int baseBand, int offset0, int offset1, int offset2, int offset3);

void sao_band_fallback_hibit(uint8_t* dst,int dststride, const uint8_t* src,int srcstride,
                             int bitdepth,
                             int width, int height,
                             int baseBand, int offset0, int offset1, int offset2, int offset3);

#endif
