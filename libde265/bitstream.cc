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

#include "bitstream.h"
#include "de265.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>



bitreader::bitreader(unsigned char* buffer, int len)
{
  data = buffer;
  bytes_remaining = len;
}

void bitreader::refill()
{
  int shift = 64-nextbits_cnt;

  while (shift >= 8 && bytes_remaining) {
    uint64_t newval = *data++;
    bytes_remaining--;

    shift -= 8;
    newval <<= shift;
    nextbits |= newval;
  }

  nextbits_cnt = 64-shift;
}

uint32_t bitreader::get_bits(int n)
{
  if (n == 0) return 0;
  assert(n<=32);

  if (nextbits_cnt < n) {
    refill();
  }

  uint64_t val = nextbits;
  val >>= 64-n;

  nextbits <<= n;
  nextbits_cnt -= n;

  return val;
}

uint32_t bitreader::get_bits_fast(int n)
{
  if (n == 0) return 0;
  assert(n<=32);

  assert(nextbits_cnt >= n);

  uint64_t val = nextbits;
  val >>= 64-n;

  nextbits <<= n;
  nextbits_cnt -= n;

  return val;
}

uint32_t bitreader::peek_bits(int n)
{
  if (n == 0) return 0;
  assert(n<=32);

  if (nextbits_cnt < n) {
    refill();
  }

  uint64_t val = nextbits;
  val >>= 64-n;

  return val;
}

void bitreader::skip_bits(int n)
{
  if (nextbits_cnt < n) {
    refill();
  }

  nextbits <<= n;
  nextbits_cnt -= n;
}

void bitreader::skip_bits_fast(int n)
{
  nextbits <<= n;
  nextbits_cnt -= n;
}

void bitreader::skip_to_byte_boundary()
{
  int nskip = (nextbits_cnt & 7);

  nextbits <<= nskip;
  nextbits_cnt -= nskip;
}

void bitreader::prepare_for_CABAC()
{
  skip_to_byte_boundary();

  int rewind = nextbits_cnt/8;
  data -= rewind;
  bytes_remaining += rewind;
  nextbits = 0;
  nextbits_cnt = 0;
}

uint32_t bitreader::get_uvlc()
{
  int num_zeros=0;

  while (get_bits(1)==0) {
    num_zeros++;

    if (num_zeros > MAX_UVLC_LEADING_ZEROS) { return UVLC_ERROR; }
  }

  if (num_zeros != 0) {
    uint32_t offset = get_bits(num_zeros);
    uint32_t value = offset + (static_cast<uint32_t>(1)<<num_zeros)-1;
    assert(value>0);
    return value;
  } else {
    return 0;
  }
}

int32_t bitreader::get_svlc()
{
  uint32_t v = get_uvlc();
  if (v==0) return 0;
  if (v==UVLC_ERROR) return SVLC_ERROR;

  bool negative = ((v&1)==0);
  return negative ? -static_cast<int32_t>(v/2) : static_cast<int32_t>((v+1)/2);
}

bool bitreader::check_rbsp_trailing_bits()
{
  int stop_bit = get_bits(1);
  assert(stop_bit==1);
  (void)stop_bit;

  while (nextbits_cnt>0 || bytes_remaining>0) {
    int filler = get_bits(1);
    if (filler!=0) {
      return false;
    }
  }

  return true;
}
