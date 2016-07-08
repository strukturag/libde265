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

#include "fallback-sao.h"
#include "util.h"

#include <assert.h>
#include <string.h>


template <class pixel_t>
void sao_band_fallback(pixel_t* dst,int dststride, const pixel_t* src,int srcstride,
                       int width, int height,
                       int baseBand, int offset0, int offset1, int offset2, int offset3,
                       int bitDepth)
{
  const int maxPixelValue = (1<<bitDepth)-1;
  const int bandShift = bitDepth-5;

  char bandOffset[32];
  memset(bandOffset, 0, sizeof(char)*32);

  bandOffset[ (baseBand  )    ] = offset0;
  bandOffset[ (baseBand+1)&31 ] = offset1;
  bandOffset[ (baseBand+2)&31 ] = offset2;
  bandOffset[ (baseBand+3)&31 ] = offset3;

#if 0
  for (int y=0;y<height;y++)
    for (int x=0;x<width;x++) {

      int band = src[x+y*srcstride]>>bandShift;
      int offset = bandOffset[band];

      if (offset) {
        dst[x+dststride*y] = Clip3(0,maxPixelValue, src[x+y*srcstride] + offset);
        //dst[x+dststride*y] = (src[x+y*srcstride] + offset);
      }
    }
#endif

#if 1
  for (int y=0;y<height;y++)
    for (int x=0;x<width;x+=4) {

      uint8_t dsttmp[4];

      *(uint32_t*)dsttmp = *(uint32_t*)(src+x+srcstride*y);

      int offset1,offset2,offset3,offset4;
      offset1 = bandOffset[ dsttmp[0] >>bandShift ];
      offset2 = bandOffset[ dsttmp[1] >>bandShift ];
      offset3 = bandOffset[ dsttmp[2] >>bandShift ];
      offset4 = bandOffset[ dsttmp[3] >>bandShift ];

      if (offset1) {
        dsttmp[0] = Clip3(0,maxPixelValue, dsttmp[0] + offset1);
      }

      if (offset2) {
        dsttmp[1] = Clip3(0,maxPixelValue, dsttmp[1] + offset2);
      }

      if (offset3) {
        dsttmp[2] = Clip3(0,maxPixelValue, dsttmp[2] + offset3);
      }

      if (offset4) {
        dsttmp[3] = Clip3(0,maxPixelValue, dsttmp[3] + offset4);
      }

      *(uint32_t*)(dst+x+dststride*y) = *(uint32_t*)dsttmp;
    }
#endif
}


void sao_band_fallback_8bit(uint8_t* dst,int dststride, const uint8_t* src,int srcstride,
                            int width, int height,
                            int baseBand, int offset0, int offset1, int offset2, int offset3)
{
  sao_band_fallback(dst,dststride, src,srcstride,
                    width,height, baseBand,
                    offset0,offset1,offset2,offset3, 8);
}


void sao_band_fallback_hibit(uint8_t* dst,int dststride, const uint8_t* src,int srcstride,
                             int bitdepth,
                             int width, int height,
                             int baseBand, int offset0, int offset1, int offset2, int offset3)
{
  assert(false);
}
