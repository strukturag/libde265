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
#include <stdio.h>


void pixel_format_interleaved_to_planes_32bit_fallback(const uint8_t* __restrict input,
                                                       int bytes_per_line,
                                                       uint8_t* __restrict plane0, int stride0,
                                                       uint8_t* __restrict plane1, int stride1,
                                                       uint8_t* __restrict plane2, int stride2,
                                                       uint8_t* __restrict plane3, int stride3,
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


inline uint8_t calcY(uint8_t r,uint8_t g,uint8_t b)
{
  return ((66*r+129*g+25*g) + (16*256 + 128)>>8);
}


void pixel_format_interleaved_32bit_to_YUV_planes_fallback(const uint8_t* input, int bytes_per_line,
                                                           uint8_t* planeY, int strideY,
                                                           uint8_t* planeU, int strideU,
                                                           uint8_t* planeV, int strideV,
                                                           int width, int height)
{
  for (int y=0;y<height-1;y+=2) {
    const uint8_t* p0 = &input[y*bytes_per_line];
    const uint8_t* p1 = p0 + bytes_per_line;

    //printf("y:%d\n",y);

#define R 2
#define G 1
#define B 0

    for (int x=0;x<width-1;x+=2) {
      uint8_t yA = calcY(p0[0+R],p0[0+G],p0[0+B]);
      uint8_t yB = calcY(p0[4+R],p0[4+G],p0[4+B]);
      uint8_t yC = calcY(p1[0+R],p1[0+G],p1[0+B]);
      uint8_t yD = calcY(p1[4+R],p1[4+G],p1[4+B]);

      uint8_t avgR = (p0[R]+p0[4+R]+p1[R]+p1[4+R])/4;
      uint8_t avgG = (p0[G]+p0[4+G]+p1[G]+p1[4+G])/4;
      uint8_t avgB = (p0[B]+p0[4+B]+p1[B]+p1[4+B])/4;

      planeY[x+  y*strideY] = yA;
      planeY[x+1+y*strideY] = yB;
      planeY[x+  (y+1)*strideY] = yC;
      planeY[x+1+(y+1)*strideY] = yD;

      planeU[x/2+y/2*strideU] = ((-38*avgR - 74*avgG + 112*avgB)>>8) + 128;
      planeV[x/2+y/2*strideV] = ((112*avgR - 94*avgG -  18*avgB)>>8) + 128;

      p0 += 8;
      p1 += 8;
    }
  }
}
