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

#ifndef DE265_SEI_H
#define DE265_SEI_H

#include "libde265/bitstream.h"
#include "libde265/decctx.h"


enum sei_payload_type {
  sei_payload_type_decoded_picture_hash = 132
};


enum sei_decoded_picture_hash_type {
  sei_decoded_picture_hash_type_MD5 = 0,
  sei_decoded_picture_hash_type_CRC = 1,
  sei_decoded_picture_hash_type_checksum = 2
};


typedef struct {
  enum sei_decoded_picture_hash_type hash_type;
  uint8_t md5[3][16];
  uint16_t crc[3];
  uint32_t checksum[3];
} sei_decoded_picture_hash;


typedef struct {
  enum sei_payload_type payload_type;
  int payload_size;

  union {
    sei_decoded_picture_hash decoded_picture_hash;
  } data;
} sei_message;



void read_sei(bitreader* reader, sei_message*, bool suffix, const decoder_context* ctx);
void dump_sei(const sei_message*, const decoder_context* ctx);
de265_error process_sei(const sei_message*, decoder_context* ctx);

#endif
