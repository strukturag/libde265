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

#include "quality.h"
#include <math.h>


double MSE(const uint8_t* img, int imgStride,
           const uint8_t* ref, int refStride,
           int width, int height)
{
  double sum=0.0;

  const uint8_t* iPtr = img;
  const uint8_t* rPtr = ref;

  for (int y=0;y<height;y++) {
    uint32_t lineSum=0;

    for (int x=0;x<width;x++) {
      int diff = iPtr[x] - rPtr[x];
      lineSum += diff*diff;
    }

    sum += ((double)lineSum)/width;

    iPtr += imgStride;
    rPtr += refStride;
  }

  return sum/height;
}


double PSNR(double mse)
{
  if (mse==0) { return 99.99999; }

  return 10*log10(255.0*255.0/mse);
}
