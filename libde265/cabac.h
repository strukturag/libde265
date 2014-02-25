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

typedef struct {
  uint8_t* data;
  uint32_t data_capacity;
  uint32_t data_size;

  uint32_t range;
  uint32_t low;
  int8_t   bits_left;
  uint8_t  buffered_byte;
  uint16_t num_buffered_bytes;
} CABAC_encoder;

void init_CABAC_encoder(CABAC_encoder* encoder);
void encode_CABAC_bit(CABAC_encoder* encoder,context_model* model, int bit);
void encode_CABAC_bypass(CABAC_encoder* encoder, int bit);

#endif
