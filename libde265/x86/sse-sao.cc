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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "x86/sse-sao.h"

#include <emmintrin.h>
#if HAVE_SSE4_1
#include <tmmintrin.h>
#include <smmintrin.h>
#endif
#include <stdio.h>
#include <assert.h>

#ifndef _MSC_VER
#include <inttypes.h>
#endif

#include "fallback-sao.h"

#define D 0

static void print128(__m128i m)
{
  for (int i=0;i<16;i++) {
    uint8_t v = ((uint8_t*)&m)[15-i];
    printf("%02x",v);
  }
}


void sao_band_8bit_sse2(uint8_t* dst,int dststride, const uint8_t* src,int srcstride,
                        int width, int height,
                        int baseBand, int offset0, int offset1, int offset2, int offset3)
{
  const char bitDepth = 8;
  const char shift = bitDepth-5;

  __m128i mask = _mm_set1_epi8((0xFF<<shift)&0xFF);
  __m128i band0 = _mm_set1_epi8(  baseBand       <<shift);
  __m128i band1 = _mm_set1_epi8(((baseBand+1)%32)<<shift);
  __m128i band2 = _mm_set1_epi8(((baseBand+2)%32)<<shift);
  __m128i band3 = _mm_set1_epi8(((baseBand+3)%32)<<shift);

  if (D) { print128(mask);  printf(" mask\n"); }
  if (D) { print128(band0); printf(" band0\n"); }
  if (D) { print128(band1); printf(" band1\n"); }
  if (D) { print128(band2); printf(" band2\n"); }
  if (D) { print128(band3); printf(" band3\n"); }
  if (D) { printf("\n"); }

  int x=0;

  for (int y=0;y<height;y++) {
    for (x=0;x+15<width;x+=16) {
      const __m128i* in = (const __m128i*)(src+x+srcstride*y);

      __m128i masked_input = _mm_load_si128(in);
      masked_input = _mm_and_si128(masked_input, mask);
      __m128i cmpband0 = _mm_cmpeq_epi8(masked_input, band0); // SSE2
      __m128i cmpband1 = _mm_cmpeq_epi8(masked_input, band1);
      __m128i cmpband2 = _mm_cmpeq_epi8(masked_input, band2);
      __m128i cmpband3 = _mm_cmpeq_epi8(masked_input, band3);

      if (D) { print128(masked_input);  printf(" masked_input\n"); }
      if (D) { print128(cmpband0);  printf(" cmpband0\n"); }
      if (D) { print128(cmpband1);  printf(" cmpband1\n"); }
      if (D) { print128(cmpband2);  printf(" cmpband2\n"); }
      if (D) { print128(cmpband3);  printf(" cmpband3\n"); }

      __m128i offsetband0 = _mm_and_si128(cmpband0, _mm_set1_epi8(offset0)); // SSE2
      __m128i offsetband1 = _mm_and_si128(cmpband1, _mm_set1_epi8(offset1));
      __m128i offsetband2 = _mm_and_si128(cmpband2, _mm_set1_epi8(offset2));
      __m128i offsetband3 = _mm_and_si128(cmpband3, _mm_set1_epi8(offset3));

      __m128i offset = _mm_or_si128( _mm_or_si128(offsetband0, offsetband1), // SSE2
                                     _mm_or_si128(offsetband2, offsetband3) );

      if (D) { print128(offset);  printf(" offset\n"); }

      __m128i zero = _mm_setzero_si128();
      __m128i input_low  = _mm_unpacklo_epi8(*in,zero);
      __m128i input_high = _mm_unpackhi_epi8(*in,zero);

      if (D) { print128(input_low);  printf(" input_low\n"); }
      if (D) { print128(input_high); printf(" input_high\n"); }

      __m128i undef = zero; // any value will do
      __m128i offset_low  = _mm_unpacklo_epi8(undef,offset);
      __m128i offset_high = _mm_unpackhi_epi8(undef,offset);

      offset_low  = _mm_srai_epi16(offset_low, 8);
      offset_high = _mm_srai_epi16(offset_high,8);

      if (D) { print128(offset_low);  printf(" offset_low\n"); }
      if (D) { print128(offset_high); printf(" offset_high\n"); }

      __m128i sum_low  = _mm_add_epi16(input_low,  offset_low);
      __m128i sum_high = _mm_add_epi16(input_high, offset_high);

      if (D) { print128(sum_low);  printf(" sum_low\n"); }
      if (D) { print128(sum_high); printf(" sum_high\n"); }

      __m128i output = _mm_packus_epi16(sum_low,sum_high);

      if (D) { print128(output); printf(" output\n"); }

      _mm_store_si128 ((__m128i*)(dst+x+y*dststride), output);
      if (D) { printf("\n"); }
    }

    if (x+8<=width) {
      const __m128i* in = (const __m128i*)(src+x+srcstride*y);

      __m128i masked_input = _mm_loadl_epi64(in);
      masked_input = _mm_and_si128(masked_input, mask);
      __m128i cmpband0 = _mm_cmpeq_epi8(masked_input, band0);
      __m128i cmpband1 = _mm_cmpeq_epi8(masked_input, band1);
      __m128i cmpband2 = _mm_cmpeq_epi8(masked_input, band2);
      __m128i cmpband3 = _mm_cmpeq_epi8(masked_input, band3);

      if (D) { print128(masked_input);  printf(" masked_input\n"); }
      if (D) { print128(cmpband0);  printf(" cmpband0\n"); }
      if (D) { print128(cmpband1);  printf(" cmpband1\n"); }
      if (D) { print128(cmpband2);  printf(" cmpband2\n"); }
      if (D) { print128(cmpband3);  printf(" cmpband3\n"); }

      __m128i offsetband0 = _mm_and_si128(cmpband0, _mm_set1_epi8(offset0));
      __m128i offsetband1 = _mm_and_si128(cmpband1, _mm_set1_epi8(offset1));
      __m128i offsetband2 = _mm_and_si128(cmpband2, _mm_set1_epi8(offset2));
      __m128i offsetband3 = _mm_and_si128(cmpband3, _mm_set1_epi8(offset3));

      __m128i offset = _mm_or_si128( _mm_or_si128(offsetband0, offsetband1),
                                     _mm_or_si128(offsetband2, offsetband3) );

      if (D) { print128(offset);  printf(" offset\n"); }

      __m128i zero = _mm_setzero_si128();
      __m128i input= _mm_loadl_epi64(in);
      __m128i input_low  = _mm_unpacklo_epi8(input,zero);

      if (D) { print128(input_low);  printf(" input_low\n"); }
      //print128(input_high); printf(" input_high\n");

      __m128i undef = zero; // any value will do
      __m128i offset_low  = _mm_unpacklo_epi8(undef,offset);
      //__m128i offset_high = _mm_unpackhi_epi8(undef,offset);

      offset_low  = _mm_srai_epi16(offset_low, 8);
      //offset_high = _mm_srai_epi16(offset_high,8);

      if (D) { print128(offset_low);  printf(" offset_low\n"); }
      //print128(offset_high); printf(" offset_high\n");

      __m128i sum_low  = _mm_add_epi16(input_low,  offset_low);
      //__m128i sum_high = _mm_add_epi16(input_high, offset_high);

      if (D) { print128(sum_low);  printf(" sum_low\n"); }
      //print128(sum_high); printf(" sum_high\n");

      __m128i output = _mm_packus_epi16(sum_low, zero);

      if (D) { print128(output); printf(" output\n"); }

      _mm_storel_epi64 ((__m128i*)(dst+x+y*dststride), output);
      if (D) { printf("\n"); }

      x+=8;
    }
  }

  if (width & 0x04) {
    sao_band_fallback_8bit(dst+width-4,dststride, src+width-4,srcstride, 4,height,
                           baseBand, offset0,offset1,offset2,offset3);
  }
}
