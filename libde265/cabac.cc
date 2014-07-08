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
#include <stdlib.h>
#include <assert.h>

#define INITIAL_CABAC_BUFFER_CAPACITY 4096


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
  logtrace(LogCABAC,"[%3d] decodeBin r:%x v:%x state:%d\n",logcnt,decoder->range, decoder->value, model->state);

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
              if (decoder->bitstream_curr != decoder->bitstream_end)
                { decoder->value |= *decoder->bitstream_curr++; }
            }
        }
    }
  else
    {
      logtrace(LogCABAC,"[%3d] LPS\n",logcnt);

      // LPS path

      decoder->value = (decoder->value - scaled_range);

      int num_bits = renorm_table[ LPS >> 3 ];
      decoder->value <<= num_bits;
      decoder->range   = LPS << num_bits;  /* this is always >= 0x100 except for state 63,
                                              but state 63 is never used */

      int num_bitsTab = renorm_table[ LPS >> 3 ];

      assert(num_bits == num_bitsTab);

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
#ifdef DE265_LOG_TRACE
  logcnt++;
#endif

  return decoded_bit;
}

int  decode_CABAC_term_bit(CABAC_decoder* decoder)
{
  logtrace(LogCABAC,"CABAC term: range=%x\n", decoder->range);

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

  decoder->value <<= 1;
  decoder->bits_needed++;

  if (decoder->bits_needed >= 0)
    {
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
#ifdef DE265_LOG_TRACE
  logcnt++;
#endif

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
      int input = *decoder->bitstream_curr++;
      input <<= decoder->bits_needed;

      decoder->bits_needed -= 8;
      decoder->value |= input;
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


// ---------------------------------------------------------------------------

CABAC_encoder_bitstream::CABAC_encoder_bitstream()
{
  data_mem = NULL;
  data_capacity = 0;
  data_size = 0;
  state = 0;

  vlc_buffer_len = 0;

  init_CABAC();
}

CABAC_encoder_bitstream::~CABAC_encoder_bitstream()
{
  delete[] data_mem;
}


void CABAC_encoder_bitstream::write_bits(uint32_t bits,int n)
{
  vlc_buffer <<= n;
  vlc_buffer |= bits;
  vlc_buffer_len += n;

  while (vlc_buffer_len>=8) {
    append_byte((vlc_buffer >> (vlc_buffer_len-8)) & 0xFF);
    vlc_buffer_len -= 8;
  }
}

void CABAC_encoder::write_uvlc(int value)
{
  assert(value>=0);

  int nLeadingZeros=0;
  int base=0;
  int range=1;

  while (value>=base+range) {
    base += range;
    range <<= 1;
    nLeadingZeros++;
  }

  write_bits((1<<nLeadingZeros) | (value-base),2*nLeadingZeros+1);
}

void CABAC_encoder::write_svlc(int value)
{
  if      (value==0) write_bits(1,1);
  else if (value>0)  write_uvlc(2*value-1);
  else               write_uvlc(-2*value);
}

void CABAC_encoder_bitstream::flush_VLC()
{
  while (vlc_buffer_len>=8) {
    append_byte((vlc_buffer >> (vlc_buffer_len-8)) & 0xFF);
    vlc_buffer_len -= 8;
  }

  if (vlc_buffer_len>0) {
    append_byte(vlc_buffer << (8-vlc_buffer_len));
    vlc_buffer_len = 0;
  }
}

void CABAC_encoder_bitstream::skip_bits(int nBits)
{
  while (nBits>=8) {
    write_bits(0,8);
    nBits-=8;
  }

  if (nBits>0) {
    write_bits(0,nBits);
  }
}


void CABAC_encoder_bitstream::check_size_and_resize(int nBytes)
{
  if (data_size+nBytes > data_capacity) { // 1 extra byte for stuffing
    if (data_capacity==0) {
      data_capacity = INITIAL_CABAC_BUFFER_CAPACITY;
    } else {
      data_capacity *= 2;
    }

    data_mem = (uint8_t*)realloc(data_mem,data_capacity);
  }
}


void CABAC_encoder_bitstream::append_byte(int byte)
{
  check_size_and_resize(2);

  // --- emulation prevention ---

  /* These byte sequences may never occur in the bitstream:
     0x000000 / 0x000001 / 0x000002

     Hence, we have to add a 0x03 before the third byte.
     We also have to add a 0x03 for this sequence: 0x000003, because
     the escape byte itself also has to be escaped.
  */

  // S0 --(0)--> S1 --(0)--> S2 --(0,1,2,3)--> add stuffing

  if (byte<=3) {
    /**/ if (state< 2 && byte==0) { state++; }
    else if (state==2 && byte<=3) {
      data_mem[ data_size++ ] = 3;

      if (byte==0) state=1;
      else         state=0;
    }
    else { state=0; }
  }
  else { state=0; }


  // write actual data byte

  data_mem[ data_size++ ] = byte;
}


void CABAC_encoder_bitstream::write_startcode()
{
  check_size_and_resize(3);

  data_mem[ data_size+0 ] = 0;
  data_mem[ data_size+1 ] = 0;
  data_mem[ data_size+2 ] = 1;
  data_size+=3;
}

void CABAC_encoder_bitstream::init_CABAC()
{
  range = 510;
  low = 0;

  bits_left = 23;
  buffered_byte = 0xFF;
  num_buffered_bytes = 0;
}

void CABAC_encoder_bitstream::flush_CABAC()
{
  if (low >> (32 - bits_left))
    {
      append_byte(buffered_byte + 1);
      while (num_buffered_bytes > 1)
        {
          append_byte(0x00);
          num_buffered_bytes--;
        }

      low -= 1 << (32 - bits_left);
    }
  else
    {
      if (num_buffered_bytes > 0)
        {
          append_byte(buffered_byte);
        }

      while (num_buffered_bytes > 1)
        {
          append_byte(0xff);
          num_buffered_bytes--;
        }    
    }

  //fprintf(stderr,"low: %08x nbits left:%d\n",low,bits_left);

  int n = 32-bits_left;
  int val = (low);

  // make sure we output full bytes

  while (n%8) {
    val<<=1;
    n++;
  }

  while (n>0) {
    append_byte( (val>>(n-8)) & 0xFF );
    n-=8;
  }
}


void CABAC_encoder_bitstream::write_out()
{
  logtrace(LogCABAC,"low = %08x (bits_left=%d)\n",low,bits_left);
  int leadByte = low >> (24 - bits_left);
  bits_left += 8;
  low &= 0xffffffffu >> bits_left;

  logtrace(LogCABAC,"write byte %02x\n",leadByte);
  logtrace(LogCABAC,"-> low = %08x\n",low);
  
  if (leadByte == 0xff)
    {
      num_buffered_bytes++;
    }
  else
    {
      if (num_buffered_bytes > 0)
        {
          int carry = leadByte >> 8;
          int byte = buffered_byte + carry;
          buffered_byte = leadByte & 0xff;
          append_byte(byte);
      
          byte = ( 0xff + carry ) & 0xff;
          while ( num_buffered_bytes > 1 )
            {
              append_byte(byte);
              num_buffered_bytes--;
            }
        }
      else
        {
          num_buffered_bytes = 1;
          buffered_byte = leadByte;
        }      
    }    
}

void CABAC_encoder_bitstream::testAndWriteOut()
{
  logtrace(LogCABAC,"bits_left = %d\n",bits_left);

  if (bits_left < 12)
    {
      write_out();
    }
}


#ifdef DE265_LOG_TRACE
static int encBinCnt=1;
#endif

void CABAC_encoder_bitstream::write_CABAC_bit(context_model* model, int bin)
{
  //m_uiBinsCoded += m_binCountIncrement;
  //rcCtxModel.setBinsCoded( 1 );

  logtrace(LogCABAC,"[%d] range=%x low=%x state=%d, bin=%d\n",encBinCnt, range,low, model->state,bin);
#ifdef DE265_LOG_TRACE
  encBinCnt++;
#endif

  uint32_t LPS = LPS_table[model->state][ ( range >> 6 ) - 4 ];
  range -= LPS;
  
  if (bin != model->MPSbit)
    {
      logtrace(LogCABAC,"LPS\n");

      int num_bits = renorm_table[ LPS >> 3 ];
      low = (low + range) << num_bits;
      range   = LPS << num_bits;

      if (model->state==0) { model->MPSbit = 1-model->MPSbit; }

      model->state = next_state_LPS[model->state];
  
      bits_left -= num_bits;
    }
  else
    {
      logtrace(LogCABAC,"MPS\n");

      model->state = next_state_MPS[model->state];


      // renorm

      if (range >= 256) { return; }

      low <<= 1;
      range <<= 1;
      bits_left--;
    }
  
  testAndWriteOut();
}

void CABAC_encoder_bitstream::write_CABAC_bypass(int bin)
{
  logtrace(LogCABAC,"[%d] bypass = %d, range=%x\n",encBinCnt,bin,range);
#ifdef DE265_LOG_TRACE
  encBinCnt++;
#endif

  // BinsCoded += m_binCountIncrement;
  low <<= 1;

  if (bin)
    {
      low += range;
    }
  bits_left--;
  
  testAndWriteOut();
}

void CABAC_encoder::write_CABAC_TU_bypass(int value, int cMax)
{
  for (int i=0;i<value;i++) {
    write_CABAC_bypass(1);
  }

  if (value<cMax) {
    write_CABAC_bypass(0);
  }
}

void CABAC_encoder::write_CABAC_FL_bypass(int value, int n)
{
  while (n>0) {
    n--;
    write_CABAC_bypass(value & (1<<n));
  }
}

void CABAC_encoder_bitstream::write_CABAC_term_bit(int bit)
{
  logtrace(LogCABAC,"CABAC term: range=%x\n", range);

  range -= 2;

  if (bit) {
    low += range;

    low <<= 7;
    range = 2 << 7;
    bits_left -= 7;
  }
  else if (range >= 256)
    {
      return;
    }
  else
    {
      low   <<= 1;
      range <<= 1;
      bits_left--;
    }
  
  testAndWriteOut();
}




static const uint32_t entropy_table[128] = {
  /* state= 0 */  0x07d13 /* 0.977160 */,  0x08254 /* 1.018216 */,
  /* state= 1 */  0x07736 /* 0.931361 */,  0x086ef /* 1.054174 */,
  /* state= 2 */  0x0702b /* 0.876331 */,  0x0935a /* 1.151209 */,
  /* state= 3 */  0x069e5 /* 0.827318 */,  0x09c7f /* 1.222631 */,
  /* state= 4 */  0x062e9 /* 0.772748 */,  0x0a2c7 /* 1.271703 */,
  /* state= 5 */  0x05c16 /* 0.719426 */,  0x0ae26 /* 1.360548 */,
  /* state= 6 */  0x05633 /* 0.673441 */,  0x0b723 /* 1.430773 */,
  /* state= 7 */  0x05145 /* 0.634942 */,  0x0c05e /* 1.502881 */,
  /* state= 8 */  0x04be0 /* 0.592785 */,  0x0ccf3 /* 1.601185 */,
  /* state= 9 */  0x0478c /* 0.558987 */,  0x0d57b /* 1.667846 */,
  /* state=10 */  0x042ab /* 0.520853 */,  0x0de7f /* 1.738254 */,
  /* state=11 */  0x03f4f /* 0.494604 */,  0x0e4b5 /* 1.786779 */,
  /* state=12 */  0x03a9d /* 0.457936 */,  0x0f472 /* 1.909753 */,
  /* state=13 */  0x037d9 /* 0.436326 */,  0x0fc56 /* 1.971385 */,
  /* state=14 */  0x034c0 /* 0.412130 */,  0x10235 /* 2.017265 */,
  /* state=15 */  0x031a6 /* 0.387879 */,  0x10d5c /* 2.104398 */,
  /* state=16 */  0x02e63 /* 0.362418 */,  0x11b34 /* 2.212536 */,
  /* state=17 */  0x02c21 /* 0.344764 */,  0x120b6 /* 2.255556 */,
  /* state=18 */  0x029ba /* 0.326012 */,  0x1294c /* 2.322657 */,
  /* state=19 */  0x02792 /* 0.309167 */,  0x135e2 /* 2.420985 */,
  /* state=20 */  0x02563 /* 0.292097 */,  0x13e3a /* 2.486146 */,
  /* state=21 */  0x0230a /* 0.273749 */,  0x144fa /* 2.538881 */,
  /* state=22 */  0x02192 /* 0.262279 */,  0x150ca /* 2.631186 */,
  /* state=23 */  0x01f5d /* 0.245047 */,  0x15c9f /* 2.723632 */,
  /* state=24 */  0x01de5 /* 0.233559 */,  0x162fb /* 2.773286 */,
  /* state=25 */  0x01c2c /* 0.220110 */,  0x16d99 /* 2.856250 */,
  /* state=26 */  0x01a8d /* 0.207458 */,  0x17a93 /* 2.957619 */,
  /* state=27 */  0x0195a /* 0.198065 */,  0x18052 /* 3.002508 */,
  /* state=28 */  0x01807 /* 0.187719 */,  0x18763 /* 3.057729 */,
  /* state=29 */  0x0164c /* 0.174217 */,  0x19462 /* 3.159266 */,
  /* state=30 */  0x01538 /* 0.165798 */,  0x19f20 /* 3.243194 */,
  /* state=31 */  0x01450 /* 0.158718 */,  0x1a466 /* 3.284374 */,
  /* state=32 */  0x0133a /* 0.150237 */,  0x1b423 /* 3.407334 */,
  /* state=33 */  0x0120b /* 0.140978 */,  0x1bce4 /* 3.475737 */,
  /* state=34 */  0x01110 /* 0.133321 */,  0x1c393 /* 3.527933 */,
  /* state=35 */  0x0104b /* 0.127316 */,  0x1d057 /* 3.627676 */,
  /* state=36 */  0x00f8d /* 0.121502 */,  0x1d749 /* 3.681944 */,
  /* state=37 */  0x00ef3 /* 0.116795 */,  0x1dfce /* 3.748483 */,
  /* state=38 */  0x00e0f /* 0.109853 */,  0x1e6d3 /* 3.803340 */,
  /* state=39 */  0x00d40 /* 0.103540 */,  0x1f925 /* 3.946453 */,
  /* state=40 */  0x00cc4 /* 0.099758 */,  0x1fda7 /* 3.981663 */,
  /* state=41 */  0x00c42 /* 0.095779 */,  0x203f7 /* 4.030997 */,
  /* state=42 */  0x00b79 /* 0.089639 */,  0x20f7d /* 4.121029 */,
  /* state=43 */  0x00afc /* 0.085841 */,  0x21dd8 /* 4.233161 */,
  /* state=44 */  0x00a5d /* 0.080974 */,  0x22417 /* 4.281976 */,
  /* state=45 */  0x00a1a /* 0.078921 */,  0x22a5b /* 4.330930 */,
  /* state=46 */  0x00988 /* 0.074490 */,  0x23755 /* 4.432283 */,
  /* state=47 */  0x0091a /* 0.071133 */,  0x24225 /* 4.516768 */,
  /* state=48 */  0x008cf /* 0.068839 */,  0x24719 /* 4.555457 */,
  /* state=49 */  0x0085a /* 0.065247 */,  0x25313 /* 4.649039 */,
  /* state=50 */  0x00814 /* 0.063139 */,  0x25d66 /* 4.729690 */,
  /* state=51 */  0x007b6 /* 0.060267 */,  0x2651e /* 4.789990 */,
  /* state=52 */  0x0076e /* 0.058074 */,  0x2687d /* 4.816323 */,
  /* state=53 */  0x00708 /* 0.054942 */,  0x27da5 /* 4.981623 */,
  /* state=54 */  0x006d5 /* 0.053382 */,  0x28172 /* 5.011293 */,
  /* state=55 */  0x00659 /* 0.049602 */,  0x28947 /* 5.072501 */,
  /* state=56 */  0x00617 /* 0.047602 */,  0x297c5 /* 5.185716 */,
  /* state=57 */  0x005dc /* 0.045783 */,  0x2a2db /* 5.272339 */,
  /* state=58 */  0x005c1 /* 0.044971 */,  0x2a582 /* 5.293050 */,
  /* state=59 */  0x00574 /* 0.042610 */,  0x2ad5a /* 5.354314 */,
  /* state=60 */  0x0053b /* 0.040866 */,  0x2bba7 /* 5.466035 */,
  /* state=61 */  0x0050d /* 0.039476 */,  0x2c599 /* 5.543748 */,
  /* state=62 */  0x004eb /* 0.038429 */,  0x2cd88 /* 5.605742 */,
  0x004eb ,  0x2cd88 /* should never be used */
};


void CABAC_encoder_estim::write_CABAC_bit(context_model* model, int bit)
{
  int idx = model->state<<1;
  if (bit!=model->MPSbit) idx++;

  mFracBits += entropy_table[idx];
}

