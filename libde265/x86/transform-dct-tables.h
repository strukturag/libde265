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

// Coefficient tables for the AVX2 / AVX-512 inverse transforms. Each inner row
// holds one madd coefficient pattern (a,b,a,b,...) and is broadcast to all
// 128-bit lanes of the SIMD register. Values match transform16x16_{1,2,3} /
// transform32x32 in sse-dct.cc. Included by transform-avx2.cc and
// transform-avx512.cc (static -> a private copy per translation unit).

#ifndef X86_TRANSFORM_DCT_TABLES_H
#define X86_TRANSFORM_DCT_TABLES_H

#include <stdint.h>

#ifndef ALIGNED_64
#define ALIGNED_64(decl) decl __attribute__((aligned(64)))
#endif

ALIGNED_64(static const int16_t) idct_T16_1[4][8][8] = {
  {{ 90, 87, 90, 87, 90, 87, 90, 87},{ 87, 57, 87, 57, 87, 57, 87, 57},
   { 80,  9, 80,  9, 80,  9, 80,  9},{ 70,-43, 70,-43, 70,-43, 70,-43},
   { 57,-80, 57,-80, 57,-80, 57,-80},{ 43,-90, 43,-90, 43,-90, 43,-90},
   { 25,-70, 25,-70, 25,-70, 25,-70},{  9,-25,  9,-25,  9,-25,  9,-25}},
  {{ 80, 70, 80, 70, 80, 70, 80, 70},{  9,-43,  9,-43,  9,-43,  9,-43},
   {-70,-87,-70,-87,-70,-87,-70,-87},{-87,  9,-87,  9,-87,  9,-87,  9},
   {-25, 90,-25, 90,-25, 90,-25, 90},{ 57, 25, 57, 25, 57, 25, 57, 25},
   { 90,-80, 90,-80, 90,-80, 90,-80},{ 43,-57, 43,-57, 43,-57, 43,-57}},
  {{ 57, 43, 57, 43, 57, 43, 57, 43},{-80,-90,-80,-90,-80,-90,-80,-90},
   {-25, 57,-25, 57,-25, 57,-25, 57},{ 90, 25, 90, 25, 90, 25, 90, 25},
   { -9,-87, -9,-87, -9,-87, -9,-87},{-87, 70,-87, 70,-87, 70,-87, 70},
   { 43,  9, 43,  9, 43,  9, 43,  9},{ 70,-80, 70,-80, 70,-80, 70,-80}},
  {{ 25,  9, 25,  9, 25,  9, 25,  9},{-70,-25,-70,-25,-70,-25,-70,-25},
   { 90, 43, 90, 43, 90, 43, 90, 43},{-80,-57,-80,-57,-80,-57,-80,-57},
   { 43, 70, 43, 70, 43, 70, 43, 70},{  9,-80,  9,-80,  9,-80,  9,-80},
   {-57, 87,-57, 87,-57, 87,-57, 87},{ 87,-90, 87,-90, 87,-90, 87,-90}}
};

ALIGNED_64(static const int16_t) idct_T16_2[2][4][8] = {
  {{ 89, 75, 89, 75, 89, 75, 89, 75},{ 75,-18, 75,-18, 75,-18, 75,-18},
   { 50,-89, 50,-89, 50,-89, 50,-89},{ 18,-50, 18,-50, 18,-50, 18,-50}},
  {{ 50, 18, 50, 18, 50, 18, 50, 18},{-89,-50,-89,-50,-89,-50,-89,-50},
   { 18, 75, 18, 75, 18, 75, 18, 75},{ 75,-89, 75,-89, 75,-89, 75,-89}}
};

ALIGNED_64(static const int16_t) idct_T16_3[2][2][8] = {
  {{ 83, 36, 83, 36, 83, 36, 83, 36},{ 36,-83, 36,-83, 36,-83, 36,-83}},
  {{ 64, 64, 64, 64, 64, 64, 64, 64},{ 64,-64, 64,-64, 64,-64, 64,-64}}
};

#define IDCT_R8(a,b) { a,b,a,b,a,b,a,b }
ALIGNED_64(static const int16_t) idct_T32[8][16][8] = {
  { IDCT_R8(90,90),IDCT_R8(90,82),IDCT_R8(88,67),IDCT_R8(85,46),IDCT_R8(82,22),IDCT_R8(78,-4),IDCT_R8(73,-31),IDCT_R8(67,-54),
    IDCT_R8(61,-73),IDCT_R8(54,-85),IDCT_R8(46,-90),IDCT_R8(38,-88),IDCT_R8(31,-78),IDCT_R8(22,-61),IDCT_R8(13,-38),IDCT_R8(4,-13) },
  { IDCT_R8(88,85),IDCT_R8(67,46),IDCT_R8(31,-13),IDCT_R8(-13,-67),IDCT_R8(-54,-90),IDCT_R8(-82,-73),IDCT_R8(-90,-22),IDCT_R8(-78,38),
    IDCT_R8(-46,82),IDCT_R8(-4,88),IDCT_R8(38,54),IDCT_R8(73,-4),IDCT_R8(90,-61),IDCT_R8(85,-90),IDCT_R8(61,-78),IDCT_R8(22,-31) },
  { IDCT_R8(82,78),IDCT_R8(22,-4),IDCT_R8(-54,-82),IDCT_R8(-90,-73),IDCT_R8(-61,13),IDCT_R8(13,85),IDCT_R8(78,67),IDCT_R8(85,-22),
    IDCT_R8(31,-88),IDCT_R8(-46,-61),IDCT_R8(-90,31),IDCT_R8(-67,90),IDCT_R8(4,54),IDCT_R8(73,-38),IDCT_R8(88,-90),IDCT_R8(38,-46) },
  { IDCT_R8(73,67),IDCT_R8(-31,-54),IDCT_R8(-90,-78),IDCT_R8(-22,38),IDCT_R8(78,85),IDCT_R8(67,-22),IDCT_R8(-38,-90),IDCT_R8(-90,4),
    IDCT_R8(-13,90),IDCT_R8(82,13),IDCT_R8(61,-88),IDCT_R8(-46,-31),IDCT_R8(-88,82),IDCT_R8(-4,46),IDCT_R8(85,-73),IDCT_R8(54,-61) },
  { IDCT_R8(61,54),IDCT_R8(-73,-85),IDCT_R8(-46,-4),IDCT_R8(82,88),IDCT_R8(31,-46),IDCT_R8(-88,-61),IDCT_R8(-13,82),IDCT_R8(90,13),
    IDCT_R8(-4,-90),IDCT_R8(-90,38),IDCT_R8(22,67),IDCT_R8(85,-78),IDCT_R8(-38,-22),IDCT_R8(-78,90),IDCT_R8(54,-31),IDCT_R8(67,-73) },
  { IDCT_R8(46,38),IDCT_R8(-90,-88),IDCT_R8(38,73),IDCT_R8(54,-4),IDCT_R8(-90,-67),IDCT_R8(31,90),IDCT_R8(61,-46),IDCT_R8(-88,-31),
    IDCT_R8(22,85),IDCT_R8(67,-78),IDCT_R8(-85,13),IDCT_R8(13,61),IDCT_R8(73,-90),IDCT_R8(-82,54),IDCT_R8(4,22),IDCT_R8(78,-82) },
  { IDCT_R8(31,22),IDCT_R8(-78,-61),IDCT_R8(90,85),IDCT_R8(-61,-90),IDCT_R8(4,73),IDCT_R8(54,-38),IDCT_R8(-88,-4),IDCT_R8(82,46),
    IDCT_R8(-38,-78),IDCT_R8(-22,90),IDCT_R8(73,-82),IDCT_R8(-90,54),IDCT_R8(67,-13),IDCT_R8(-13,-31),IDCT_R8(-46,67),IDCT_R8(85,-88) },
  { IDCT_R8(13,4),IDCT_R8(-38,-13),IDCT_R8(61,22),IDCT_R8(-78,-31),IDCT_R8(88,38),IDCT_R8(-90,-46),IDCT_R8(85,54),IDCT_R8(-73,-61),
    IDCT_R8(54,67),IDCT_R8(-31,-73),IDCT_R8(4,78),IDCT_R8(22,-82),IDCT_R8(-46,85),IDCT_R8(67,-88),IDCT_R8(-82,90),IDCT_R8(90,-90) }
};
#undef IDCT_R8

#endif
