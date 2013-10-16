/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#ifndef DE265_MOTION_H
#define DE265_MOTION_H

#include <stdint.h>


typedef struct
{
  int16_t x,y;
} MotionVector;


typedef struct
{
   int8_t refIdx[2];
  uint8_t predFlag[2];
  MotionVector mv[2];
} PredVectorInfo;


typedef struct
{
  PredVectorInfo lum;
  MotionVector mvC[2];
} VectorInfo;


#endif
