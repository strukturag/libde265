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

#ifndef DE265_BITSTREAM_H
#define DE265_BITSTREAM_H

#include <cstdint>


// HEVC (ITU-T H.265, E.3.3) allows ue(v) values up to 2^32-2 (e.g. bit_rate_value_minus1),
// which requires 31 leading zeros in the exp-Golomb code. 32 leading zeros would give a
// minimum codeNum of 2^32-1, which exceeds every syntax element's valid range.
constexpr int MAX_UVLC_LEADING_ZEROS = 31;
constexpr uint32_t UVLC_ERROR = UINT32_MAX;
constexpr int32_t SVLC_ERROR = INT32_MIN;


struct bitreader {
  uint8_t* data;
  int bytes_remaining;

  uint64_t nextbits; // left-aligned bits
  int nextbits_cnt;
};

void bitreader_init(bitreader*, unsigned char* buffer, int len);
void bitreader_refill(bitreader*); // refill to at least 56+1 bits
int  next_bit(bitreader*);
int  next_bit_norefill(bitreader*);
uint32_t get_bits(bitreader*, int n); // n in [0;32]
uint32_t get_bits_fast(bitreader*, int n); // n in [0;32]
uint32_t peek_bits(bitreader*, int n);
void skip_bits(bitreader*, int n);
void skip_bits_fast(bitreader*, int n);
void skip_to_byte_boundary(bitreader*);
void prepare_for_CABAC(bitreader*);
uint32_t get_uvlc(bitreader*);  // may return UVLC_ERROR
int32_t  get_svlc(bitreader*);  // may return SVLC_ERROR

bool check_rbsp_trailing_bits(bitreader*); // return true if remaining filler bits are all zero

#endif
