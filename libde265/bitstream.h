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

#ifndef DE265_BITSTREAM_H
#define DE265_BITSTREAM_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


typedef struct {
  unsigned char* data;
  int size;
  int capacity;
} rbsp_buffer;

void rbsp_buffer_init(rbsp_buffer* buffer);
void rbsp_buffer_resize(rbsp_buffer* buffer, int new_size);
void rbsp_buffer_free(rbsp_buffer* buffer);
void rbsp_buffer_append(rbsp_buffer* buffer, const unsigned char* data, int n);
void rbsp_buffer_pop(rbsp_buffer* buffer, int n);

typedef struct input_context {
  rbsp_buffer input_buffer;

  bool (*refill_buffer)(struct input_context*);
} input_context;


typedef struct {
  input_context ctx;
  FILE* input_file;
} input_context_FILE;


int init_file_context(input_context_FILE* ctx, const char* filename);
int read_nal_unit(input_context* ctx, rbsp_buffer* buffer);



typedef struct {
  uint8_t* data;
  int bytes_remaining;

  uint64_t nextbits; // left-aligned bits
  int nextbits_cnt;
} bitreader;

void bitreader_init(bitreader*, rbsp_buffer*);
void bitreader_refill(bitreader*); // refill to at least 56+1 bits
int  next_bit(bitreader*);
int  next_bit_norefill(bitreader*);
int  get_bits(bitreader*, int n);
int  get_bits_fast(bitreader*, int n);
int  peek_bits(bitreader*, int n);
void skip_bits(bitreader*, int n);
void skip_bits_fast(bitreader*, int n);
void skip_to_byte_boundary(bitreader*);
void prepare_for_CABAC(bitreader*);
int  get_uvlc(bitreader*);
int  get_svlc(bitreader*);

void check_rbsp_trailing_bits(bitreader*);

#endif
