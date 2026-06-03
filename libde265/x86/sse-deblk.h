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

#ifndef SSE_DEBLK_H
#define SSE_DEBLK_H

#include <stddef.h>
#include <stdint.h>

// SSE4.1 8-bit deblocking of one 4-line edge segment. Bit-identical to the
// scalar kernels in fallback-deblk.h (verified by dev-tools/test-deblk).
void deblock_luma_8_sse4(uint8_t* ptr, ptrdiff_t stride, int vertical,
                         int dE, int dEp, int dEq, int tc, int filterP, int filterQ);
// (chroma deblock stays scalar — SSE measured slower; see sse-deblk.cc)

#endif
