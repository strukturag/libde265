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

static const uint8_t next_state_MPS_HM[128] =
  {
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
    34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
    66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81,
    82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
    98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
    114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 124, 125, 126, 127
  };

static const uint8_t next_state_LPS_HM[128] =
  {
    1, 0, 0, 1, 2, 3, 4, 5, 4, 5, 8, 9, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 18, 19, 22, 23, 22, 23, 24, 25,
    26, 27, 26, 27, 30, 31, 30, 31, 32, 33, 32, 33, 36, 37, 36, 37,
    38, 39, 38, 39, 42, 43, 42, 43, 44, 45, 44, 45, 46, 47, 48, 49,
    48, 49, 50, 51, 52, 53, 52, 53, 54, 55, 54, 55, 56, 57, 58, 59,
    58, 59, 60, 61, 60, 61, 60, 61, 62, 63, 64, 65, 64, 65, 66, 67,
    66, 67, 66, 67, 68, 69, 68, 69, 70, 71, 70, 71, 70, 71, 72, 73,
    72, 73, 72, 73, 74, 75, 74, 75, 74, 75, 76, 77, 76, 77, 126, 127
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





int logcnt=1;

void init_CABAC_decoder(CABAC_decoder* decoder, uint8_t* bitstream, int length)
{
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
  //if (logcnt >= 400000000) { enablelog(); }

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
          decoder->range = scaled_range >> 6;
          decoder->value <<= 1;
          decoder->bits_needed++;

          if (decoder->bits_needed == 0)
            {
              decoder->bits_needed = -8;
              if (decoder->bitstream_curr != decoder->bitstream_end)
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
      decoder->range   = LPS << num_bits;
      decoded_bit      = 1 - model->MPSbit;

      if (model->state==0) { model->MPSbit = 1-model->MPSbit; }
      model->state = next_state_LPS[model->state];

      decoder->bits_needed += num_bits;

      if (decoder->bits_needed >= 0)
        {
          logtrace(LogCABAC,"bits_needed: %d\n", decoder->bits_needed);
          if (decoder->bitstream_curr != decoder->bitstream_end)
            { decoder->value |= (*decoder->bitstream_curr++) << decoder->bits_needed; }

          decoder->bits_needed -= 8;
        }
    }

  logtrace(LogCABAC,"[%3d] -> bit %d  r:%x v:%x\n", logcnt, decoded_bit, decoder->range, decoder->value);
  logcnt++;

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

              if (decoder->bitstream_curr != decoder->bitstream_end) {
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

      decoder->bits_needed = -8;
      decoder->value |= *decoder->bitstream_curr++;
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
  logcnt++;

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

int  decode_CABAC_FL_bypass(CABAC_decoder* decoder, int nBits)
{
  // assert(nBits<8); // TODO: HM has fast code for reading 8 bypass bits at once. But does this ever occur?

  int value=0;
  while (nBits--) {
    value <<= 1;
    value |= decode_CABAC_bypass(decoder);
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

