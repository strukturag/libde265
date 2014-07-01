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

#ifndef DE265_CABAC_H
#define DE265_CABAC_H

#include <stdint.h>


typedef struct {
  uint8_t* bitstream_start;
  uint8_t* bitstream_curr;
  uint8_t* bitstream_end;

  uint32_t range;
  uint32_t value;
  int16_t  bits_needed;
} CABAC_decoder;


typedef struct {
  uint8_t MPSbit : 1;
  uint8_t state  : 7;
} context_model;


void init_CABAC_decoder(CABAC_decoder* decoder, uint8_t* bitstream, int length);
void init_CABAC_decoder_2(CABAC_decoder* decoder);
int  decode_CABAC_bit(CABAC_decoder* decoder, context_model* model);
int  decode_CABAC_TU(CABAC_decoder* decoder, int cMax, context_model* model);
int  decode_CABAC_term_bit(CABAC_decoder* decoder);

int  decode_CABAC_bypass(CABAC_decoder* decoder);
int  decode_CABAC_TU_bypass(CABAC_decoder* decoder, int cMax);
int  decode_CABAC_FL_bypass(CABAC_decoder* decoder, int nBits);
int  decode_CABAC_TR_bypass(CABAC_decoder* decoder, int cRiceParam, int cTRMax);
int  decode_CABAC_EGk_bypass(CABAC_decoder* decoder, int k);


// ---------------------------------------------------------------------------

class CABAC_encoder
{
public:
  CABAC_encoder();
  ~CABAC_encoder();

  int size() const { return data_size; }
  uint8_t* data() const { return data_mem; }

  // --- VLC ---

  void write_bits(uint32_t bits,int n);
  void write_bit(int bit) { write_bits(bit,1); }
  void write_uvlc(int value);
  void write_svlc(int value);
  void write_startcode();
  void skip_bits(int nBits);

  // output all remaining bits and fill with zeros to next byte boundary
  void flush_VLC();


  // --- CABAC ---

  void write_CABAC_bit(context_model* model, int bit);
  void write_CABAC_bypass(int bit);
  void write_CABAC_TU_bypass(int value, int cMax);
  void write_CABAC_FL_bypass(int value, int nBits);
  void write_CABAC_term_bit(int bit);
  void flush_CABAC();


private:
  // data buffer

  uint8_t* data_mem;
  uint32_t data_capacity;
  uint32_t data_size;
  char     state; // for inserting emulation-prevention bytes

  // VLC

  uint32_t vlc_buffer;
  uint32_t vlc_buffer_len;


  // CABAC

  uint32_t range;
  uint32_t low;
  int8_t   bits_left;
  uint8_t  buffered_byte;
  uint16_t num_buffered_bytes;


  void check_size_and_resize(int nBytes);
  void testAndWriteOut();
  void write_out();
  void append_byte(int byte);
};

#endif
