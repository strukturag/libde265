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

#include "cabac.h"
#include "util.h"

#include <stdint.h>
#include <stdio.h>
#include <assert.h>


static const uint8_t LPS_table[64][4] =
  {
    { 128, 176, 208, 240},
    { 128, 167, 197, 227},
    { 128, 158, 187, 216},
    { 123, 150, 178, 205},
    { 116, 142, 169, 195},
    { 111, 135, 160, 185},
    { 105, 128, 152, 175},
    { 100, 122, 144, 166},
    {  95, 116, 137, 158},
    {  90, 110, 130, 150},
    {  85, 104, 123, 142},
    {  81,  99, 117, 135},
    {  77,  94, 111, 128},
    {  73,  89, 105, 122},
    {  69,  85, 100, 116},
    {  66,  80,  95, 110},
    {  62,  76,  90, 104},
    {  59,  72,  86,  99},
    {  56,  69,  81,  94},
    {  53,  65,  77,  89},
    {  51,  62,  73,  85},
    {  48,  59,  69,  80},
    {  46,  56,  66,  76},
    {  43,  53,  63,  72},
    {  41,  50,  59,  69},
    {  39,  48,  56,  65},
    {  37,  45,  54,  62},
    {  35,  43,  51,  59},
    {  33,  41,  48,  56},
    {  32,  39,  46,  53},
    {  30,  37,  43,  50},
    {  29,  35,  41,  48},
    {  27,  33,  39,  45},
    {  26,  31,  37,  43},
    {  24,  30,  35,  41},
    {  23,  28,  33,  39},
    {  22,  27,  32,  37},
    {  21,  26,  30,  35},
    {  20,  24,  29,  33},
    {  19,  23,  27,  31},
    {  18,  22,  26,  30},
    {  17,  21,  25,  28},
    {  16,  20,  23,  27},
    {  15,  19,  22,  25},
    {  14,  18,  21,  24},
    {  14,  17,  20,  23},
    {  13,  16,  19,  22},
    {  12,  15,  18,  21},
    {  12,  14,  17,  20},
    {  11,  14,  16,  19},
    {  11,  13,  15,  18},
    {  10,  12,  15,  17},
    {  10,  12,  14,  16},
    {   9,  11,  13,  15},
    {   9,  11,  12,  14},
    {   8,  10,  12,  14},
    {   8,   9,  11,  13},
    {   7,   9,  11,  12},
    {   7,   9,  10,  12},
    {   7,   8,  10,  11},
    {   6,   8,   9,  11},
    {   6,   7,   9,  10},
    {   6,   7,   8,   9},
    {   2,   2,   2,   2}
  };

static const uint8_t renorm_table[32] =
  {
    6,  5,  4,  4,
    3,  3,  3,  3,
    2,  2,  2,  2,
    2,  2,  2,  2,
    1,  1,  1,  1,
    1,  1,  1,  1,
    1,  1,  1,  1,
    1,  1,  1,  1
  };

static const uint8_t next_state_MPS[64] =
  {
    1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
    17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
    33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,
    49,50,51,52,53,54,55,56,57,58,59,60,61,62,62,63
  };

static const uint8_t next_state_LPS[64] =
  {
    0,0,1,2,2,4,4,5,6,7,8,9,9,11,11,12,
    13,13,15,15,16,16,18,18,19,19,21,21,22,22,23,24,
    24,25,26,26,27,27,28,29,29,30,30,30,31,32,32,33,
    33,33,34,34,35,35,35,36,36,36,37,37,37,38,38,63
  };





#ifdef DE265_LOG_TRACE
int logcnt=1;
#endif

void init_CABAC_decoder(CABAC_decoder* decoder, uint8_t* bitstream, int length)
{
  assert(length >= 0);

  decoder->bitstream_start = bitstream;
  decoder->bitstream_curr  = bitstream;
  decoder->bitstream_end   = bitstream+length;
}

void init_CABAC_decoder_2(CABAC_decoder* decoder)
{
  int length = decoder->bitstream_end - decoder->bitstream_curr;

  decoder->range = 510;
  decoder->bits_needed = 8;

  decoder->value = 0;

  if (length>0) { decoder->value  = (*decoder->bitstream_curr++) << 8;  decoder->bits_needed-=8; }
  if (length>1) { decoder->value |= (*decoder->bitstream_curr++);       decoder->bits_needed-=8; }

  logtrace(LogCABAC,"[%3d] init_CABAC_decode_2 r:%x v:%x\n", logcnt, decoder->range, decoder->value);
}


//#include <sys/types.h>
//#include <signal.h>

int  decode_CABAC_bit(CABAC_decoder* decoder, context_model* model)
{
  //if (logcnt >= 1100000) { enablelog(); }

  // if (logcnt==400068770) { raise(SIGINT); }

  logtrace(LogCABAC,"[%3d] decodeBin r:%x v:%x state:%d\n",logcnt,decoder->range, decoder->value, model->state);

  //assert(decoder->range>=0x100);

  int decoded_bit;
  int LPS = LPS_table[model->state][ ( decoder->range >> 6 ) - 4 ];
  decoder->range -= LPS;

  uint32_t scaled_range = decoder->range << 7;

  logtrace(LogCABAC,"[%3d] sr:%x v:%x\n",logcnt,scaled_range, decoder->value);

  if (decoder->value < scaled_range)
    {
      logtrace(LogCABAC,"[%3d] MPS\n",logcnt);

      // MPS path

      decoded_bit = model->MPSbit;
      model->state = next_state_MPS[model->state];

      if (scaled_range < ( 256 << 7 ) )
        {
          // scaled range, highest bit (15) not set

          decoder->range = scaled_range >> 6; // shift range by one bit
          decoder->value <<= 1;               // shift value by one bit
          decoder->bits_needed++;

          if (decoder->bits_needed == 0)
            {
              decoder->bits_needed = -8;
              if (decoder->bitstream_curr < decoder->bitstream_end)
                { decoder->value |= *decoder->bitstream_curr++; }
            }
        }
    }
  else
    {
      logtrace(LogCABAC,"[%3d] LPS\n",logcnt);

      // LPS path

      int num_bits = renorm_table[ LPS >> 3 ];
      decoder->value = (decoder->value - scaled_range);

      decoder->value <<= num_bits;
      decoder->range   = LPS << num_bits;  /* this is always >= 0x100 except for state 63,
                                              but state 63 is never used */
      decoded_bit      = 1 - model->MPSbit;

      if (model->state==0) { model->MPSbit = 1-model->MPSbit; }
      model->state = next_state_LPS[model->state];

      decoder->bits_needed += num_bits;

      if (decoder->bits_needed >= 0)
        {
          logtrace(LogCABAC,"bits_needed: %d\n", decoder->bits_needed);
          if (decoder->bitstream_curr < decoder->bitstream_end)
            { decoder->value |= (*decoder->bitstream_curr++) << decoder->bits_needed; }

          decoder->bits_needed -= 8;
        }
    }

  logtrace(LogCABAC,"[%3d] -> bit %d  r:%x v:%x\n", logcnt, decoded_bit, decoder->range, decoder->value);
#ifdef DE265_LOG_TRACE
  logcnt++;
#endif

  //assert(decoder->range>=0x100);


  return decoded_bit;
}

int  decode_CABAC_term_bit(CABAC_decoder* decoder)
{
  decoder->range -= 2;
  uint32_t scaledRange = decoder->range << 7;

  if (decoder->value >= scaledRange)
    {
      return 1;
    }
  else
    {
      // there is a while loop in the standard, but it will always be executed only once

      if (scaledRange < (256<<7))
        {
          decoder->range = scaledRange >> 6;
          decoder->value *= 2;

          decoder->bits_needed++;
          if (decoder->bits_needed==0)
            {
              decoder->bits_needed = -8;

              if (decoder->bitstream_curr < decoder->bitstream_end) {
                decoder->value += (*decoder->bitstream_curr++);
              }
            }
        }

      return 0;
    }
}



int  decode_CABAC_bypass(CABAC_decoder* decoder)
{
  logtrace(LogCABAC,"[%3d] bypass r:%x v:%x\n",logcnt,decoder->range, decoder->value);

  //assert(decoder->range>=0x100);

  decoder->value <<= 1;
  decoder->bits_needed++;

  if (decoder->bits_needed >= 0)
    {
      //assert(decoder->bits_needed==0);

      if (decoder->bitstream_end > decoder->bitstream_curr) {
        decoder->bits_needed = -8;
        decoder->value |= *decoder->bitstream_curr++;
      }
    }

  int bit;
  uint32_t scaled_range = decoder->range << 7;
  if (decoder->value >= scaled_range)
    {
      decoder->value -= scaled_range;
      bit=1;
    }
  else
    {
      bit=0;
    }

  logtrace(LogCABAC,"[%3d] -> bit %d  r:%x v:%x\n", logcnt, bit, decoder->range, decoder->value);
#ifdef DE265_LOG_TRACE
  logcnt++;
#endif

  //assert(decoder->range>=0x100);

  return bit;
}


int  decode_CABAC_TU_bypass(CABAC_decoder* decoder, int cMax)
{
  for (int i=0;i<cMax;i++)
    {
      int bit = decode_CABAC_bypass(decoder);
      if (bit==0)
        return i;
    }

  return cMax;
}

int  decode_CABAC_TU(CABAC_decoder* decoder, int cMax, context_model* model)
{
  for (int i=0;i<cMax;i++)
    {
      int bit = decode_CABAC_bit(decoder,model);
      if (bit==0)
        return i;
    }

  return cMax;
}


int  decode_CABAC_FL_bypass_parallel(CABAC_decoder* decoder, int nBits)
{
  logtrace(LogCABAC,"[%3d] bypass group r:%x v:%x\n",logcnt,decoder->range, decoder->value);

  decoder->value <<= nBits;
  decoder->bits_needed+=nBits;

  if (decoder->bits_needed >= 0)
    {
      if (decoder->bitstream_end > decoder->bitstream_curr) {
        int input = *decoder->bitstream_curr++;
        input <<= decoder->bits_needed;

        decoder->bits_needed -= 8;
        decoder->value |= input;
      }
    }

  uint32_t scaled_range = decoder->range << 7;
  int value = decoder->value / scaled_range;
  if (unlikely(value>=(1<<nBits))) { value=(1<<nBits)-1; } // may happen with broken bitstreams
  decoder->value -= value * scaled_range;

  logtrace(LogCABAC,"[%3d] -> value %d  r:%x v:%x\n", logcnt+nBits-1,
           value, decoder->range, decoder->value);
#ifdef DE265_LOG_TRACE
  logcnt+=nBits;
#endif

  //assert(decoder->range>=0x100);

  return value;
}


int  decode_CABAC_FL_bypass(CABAC_decoder* decoder, int nBits)
{
  int value=0;


  if (likely(nBits<=8)) {
    if (nBits==0) {
      return 0;
    }
    // we could use decode_CABAC_bypass() for a single bit, but this seems to be slower
#if 0
    else if (nBits==1) {
      value = decode_CABAC_bypass(decoder);
    }
#endif
    else {
      value = decode_CABAC_FL_bypass_parallel(decoder,nBits);
    }
  }
  else {
    value = decode_CABAC_FL_bypass_parallel(decoder,8);
    nBits-=8;

    while (nBits--) {
      value <<= 1;
      value |= decode_CABAC_bypass(decoder);
    }
  }

  logtrace(LogCABAC,"      -> FL: %d\n", value);

  return value;
}

int  decode_CABAC_TR_bypass(CABAC_decoder* decoder, int cRiceParam, int cTRMax)
{
  int prefix = decode_CABAC_TU_bypass(decoder, cTRMax>>cRiceParam);
  if (prefix==4) { // TODO check: constant 4 only works for coefficient decoding
    return cTRMax;
  }

  int suffix = decode_CABAC_FL_bypass(decoder, cRiceParam);

  return (prefix << cRiceParam) | suffix;
}

int  decode_CABAC_EGk_bypass(CABAC_decoder* decoder, int k)
{
  int base=0;
  int n=k;

  for (;;)
    {
      int bit = decode_CABAC_bypass(decoder);
      if (bit==0)
        break;
      else {
        base += 1<<n;
        n++;
      }
    }

  int suffix = decode_CABAC_FL_bypass(decoder, n);
  return base + suffix;
}
