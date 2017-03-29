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

#include "libde265/fallback-pixelformat.h"

#include <assert.h>


void pixel_format_interleaved_to_planes_32bit_fallback(const uint8_t* __restrict__ input,
                                                       int bytes_per_line,
                                                       uint8_t* __restrict__ plane0, int stride0,
                                                       uint8_t* __restrict__ plane1, int stride1,
                                                       uint8_t* __restrict__ plane2, int stride2,
                                                       uint8_t* __restrict__ plane3, int stride3,
                                                       int width, int height)
{
  if (plane3 == nullptr) {
    for (int y=0;y<height;y++) {
      for (int x=0;x<width;x++) {
        plane0[x+y*stride0] = input[y*bytes_per_line+4*x+0];
        plane1[x+y*stride1] = input[y*bytes_per_line+4*x+1];
        plane2[x+y*stride2] = input[y*bytes_per_line+4*x+2];
      }
    }
  }
  else {
    assert(false);
  }
}
