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

#include "fallback-deblk.h"

void deblock_luma_8_fallback(uint8_t* ptr, ptrdiff_t stride, int vertical,
                             int dE, int dEp, int dEq, int tc, int filterP, int filterQ)
{
  deblock_luma_kernel<uint8_t>(ptr, stride, vertical!=0, dE, dEp, dEq, tc,
                               filterP!=0, filterQ!=0, 8);
}

void deblock_chroma_8_fallback(uint8_t* ptr, ptrdiff_t stride, int vertical,
                               int tc, int filterP, int filterQ)
{
  deblock_chroma_kernel<uint8_t>(ptr, stride, vertical!=0, tc, filterP!=0, filterQ!=0, 8);
}
