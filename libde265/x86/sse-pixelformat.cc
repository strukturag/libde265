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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <emmintrin.h> // SSE2

#if HAVE_SSE4_1
#include <tmmintrin.h> // SSSE3
#include <smmintrin.h> // SSE4.1
#endif


#include <assert.h>
#include <stdio.h>


static int8_t interleaved_to_planar_mask[16] = { 0,4,8,12, 1,5,9,13, 2,6,10,14, 3,7,11,15 };

#ifdef _MSC_VER
#define __restrict__
#endif

void pixel_format_interleaved_to_planes_32bit_sse(const uint8_t* __restrict__ input,
                                                  int bytes_per_line,
                                                  uint8_t* __restrict__ plane0, int stride0,
                                                  uint8_t* __restrict__ plane1, int stride1,
                                                  uint8_t* __restrict__ plane2, int stride2,
                                                  uint8_t* __restrict__ plane3, int stride3,
                                                  int width, int height)
{
  if (plane3 == nullptr) {

    for (int y=0;y<height;y++) {
      //const uint8_t* in_end = input[y*bytes_per_lines + 4*width];
      int n = width;

      uint8_t* p0 = &plane0[y*stride0];
      uint8_t* p1 = &plane1[y*stride1];
      uint8_t* p2 = &plane2[y*stride2];

      const uint8_t* in = &input[y*bytes_per_line];

      __m128i mask = _mm_load_si128((__m128i*)interleaved_to_planar_mask);

      while (n >= 16) {
        __m128i inA = _mm_load_si128((__m128i*)&in[0 ]);
        __m128i inB = _mm_load_si128((__m128i*)&in[16]);
        __m128i inC = _mm_load_si128((__m128i*)&in[32]);
        __m128i inD = _mm_load_si128((__m128i*)&in[48]);

        inA = _mm_shuffle_epi8(inA, mask);
        inB = _mm_shuffle_epi8(inB, mask);
        inC = _mm_shuffle_epi8(inC, mask);
        inD = _mm_shuffle_epi8(inD, mask);

        __m128i ABl = _mm_unpacklo_epi32(inA,inB);
        __m128i ABh = _mm_unpackhi_epi32(inA,inB);
        __m128i CDl = _mm_unpacklo_epi32(inC,inD);
        __m128i CDh = _mm_unpackhi_epi32(inC,inD);

        _mm_store_si128( (__m128i*)p0, _mm_unpacklo_epi64(ABl, CDl) );
        _mm_store_si128( (__m128i*)p1, _mm_unpackhi_epi64(ABl, CDl) );
        _mm_store_si128( (__m128i*)p2, _mm_unpacklo_epi64(ABh, CDh) );
        //_mm_store_si128( (__m128i*)p3, _mm_unpackhi_epi64(ABh, CDh) );

        in += 4*16;
        p0 += 16;
        p1 += 16;
        p2 += 16;
        n  -= 16;
      }

      while (n) {
        *p0++ = *in++;
        *p1++ = *in++;
        *p2++ = *in++;
        in++;
        n--;
      }
    }
  }
  else {
    assert(false);
  }
}
