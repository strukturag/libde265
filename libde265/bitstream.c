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

#include "bitstream.h"
#include "de265.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>


void rbsp_buffer_init(rbsp_buffer* buffer)
{
  buffer->data = NULL;
  buffer->size = 0;
  buffer->capacity = 0;
}

static void init_context(input_context* ctx)
{
  rbsp_buffer_init(&ctx->input_buffer);
}


void rbsp_buffer_resize(rbsp_buffer* buffer, int new_size)
{
  if (buffer->capacity < new_size) {
    unsigned char* newbuffer = (unsigned char*)malloc(new_size);

    if (buffer->data != NULL) {
      memcpy(newbuffer, buffer->data, buffer->size);
      free(buffer->data);
    }

    buffer->data = newbuffer;
    buffer->capacity = new_size;
  }
}


void rbsp_buffer_free(rbsp_buffer* buffer)
{
  if (buffer->data != NULL) {
    free(buffer->data);

    buffer->data = NULL;
    buffer->size = 0;
    buffer->capacity = 0;
  }
}


void rbsp_buffer_append(rbsp_buffer* buffer, const unsigned char* data, int n)
{
  rbsp_buffer_resize(buffer, buffer->size + n);
  memcpy(buffer->data + buffer->size, data, n);
  buffer->size += n;
}


void rbsp_buffer_pop(rbsp_buffer* buffer, int n)
{
  memmove(buffer->data,
          buffer->data + n,
          buffer->size - n);
  buffer->size -= n;
}


#define READ_CHUNK_SIZE 1024

static bool input_context_FILE_read(input_context* ctx)
{
  input_context_FILE* filectx = (input_context_FILE*)ctx;

  int free_space = ctx->input_buffer.capacity - ctx->input_buffer.size;
  int read_chunk_size;

  if (free_space < READ_CHUNK_SIZE) {
    int new_size = ctx->input_buffer.capacity + READ_CHUNK_SIZE;
    rbsp_buffer_resize(&ctx->input_buffer, new_size);
    read_chunk_size = ctx->input_buffer.capacity - ctx->input_buffer.size;
  } else {
    read_chunk_size = free_space;
  }

  int nread = fread(&ctx->input_buffer.data[ctx->input_buffer.size],
                    1,read_chunk_size,
                    filectx->input_file);

  if (nread==0) {
    return false;
  } else {
    ctx->input_buffer.size += nread;
    return true;
  }
}


int  init_file_context(input_context_FILE* ctx, const char* filename)
{
  ctx->input_file = fopen(filename,"rb");
  if (ctx->input_file==0) {
    return DE265_ERROR_NO_SUCH_FILE;
  }

  init_context(&ctx->ctx);
  ctx->ctx.refill_buffer = input_context_FILE_read;

  return DE265_OK;
}


int read_nal_unit(input_context* ctx, rbsp_buffer* buffer)
{
  buffer->size=0;

  // if there is not even a NAL header remaining in the input, we are at EOF

  if (ctx->input_buffer.size < 4+2) {
    bool more_data = ctx->refill_buffer(ctx);
    if (!more_data || ctx->input_buffer.size < 4+2) {
      return DE265_ERROR_EOF;
    }
  }

  // reserve enough space so that complete input would fit into output buffer
  rbsp_buffer_resize(buffer, ctx->input_buffer.size);


  unsigned char* out = buffer->data;
  int in_idx =0;
  int out_idx=0;

  // check for start-code at the beginning

  unsigned char* in = ctx->input_buffer.data;
  while (in[in_idx+0]==0 && in[in_idx+1]==0 && in[in_idx+2]==0) {
    in_idx++;
    if (ctx->input_buffer.size < in_idx+4+2) {
      return DE265_ERROR_EOF;
    }
  }

  if (in[in_idx+0] != 0 || in[in_idx+1] != 0 || in[in_idx+2] != 1) {
    return DE265_ERROR_NO_STARTCODE;
  }

  in_idx+=3;


  // copy NAL header
  out[out_idx++] = in[in_idx++];
  out[out_idx++] = in[in_idx++];


  // copy until next start-code (removing start-code emulation prevention bytes)

  for (;;) {
    // when we approach the end of the input buffer, resize (both) buffers

    if (ctx->input_buffer.size < in_idx+4) {
      bool more_data = ctx->refill_buffer(ctx);
      in = ctx->input_buffer.data;

      if (!more_data) {
        assert(ctx->input_buffer.size - in_idx < 4);
        buffer->size = out_idx;
        rbsp_buffer_resize(buffer, buffer->size+4);  // space for up to 4 more bytes
        out = buffer->data;

        // copy remaining data to output

        while (in_idx < ctx->input_buffer.size) {
          out[out_idx++] = in[in_idx++];
        }

        ctx->input_buffer.size = 0; // we read all data
        buffer->size = out_idx;

        return DE265_OK;
      }

      // make space such that all input data would possibly fit into the output

      int remaining_input_data = ctx->input_buffer.size - in_idx;
      buffer->size = out_idx;
      rbsp_buffer_resize(buffer, buffer->size + remaining_input_data);
      out = buffer->data;
    }

    // check for next start-code
    if (in[in_idx+0]==0 && in[in_idx+1]==0 && in[in_idx+2]==1) {

      // remove read data from input buffer

      memmove(ctx->input_buffer.data, ctx->input_buffer.data + in_idx,
              ctx->input_buffer.size - in_idx);

      ctx->input_buffer.size -= in_idx;
      buffer->size = out_idx;

      return DE265_OK;
    }

    // check for start-code emulation
    if (in[in_idx+0]==0 && in[in_idx+1]==0 && in[in_idx+2]==3) {
      assert(out_idx+2 <= buffer->capacity);
      out[out_idx++] = in[in_idx++];
      out[out_idx++] = in[in_idx++];
      in_idx++; // skip start-code emulation byte
    } else {
      assert(out_idx+1 <= buffer->capacity);
      out[out_idx++] = in[in_idx++];
    }
  }
}




void bitreader_init(bitreader* br, rbsp_buffer* buffer)
{
  br->data = buffer->data;
  br->bytes_remaining = buffer->size;

  br->nextbits=0;
  br->nextbits_cnt=0;

  bitreader_refill(br);
}

void bitreader_refill(bitreader* br)
{
  int shift = 64-br->nextbits_cnt;

  while (shift >= 8 && br->bytes_remaining) {
    uint64_t newval = *br->data++;
    br->bytes_remaining--;

    shift -= 8;
    newval <<= shift;
    br->nextbits |= newval;
  }

  br->nextbits_cnt = 64-shift;
}

int  get_bits(bitreader* br, int n)
{
  if (br->nextbits_cnt < n) {
    bitreader_refill(br);
  }

  uint64_t val = br->nextbits;
  val >>= 64-n;

  br->nextbits <<= n;
  br->nextbits_cnt -= n;

  return val;
}

int  get_bits_fast(bitreader* br, int n)
{
  assert(br->nextbits_cnt >= n);

  uint64_t val = br->nextbits;
  val >>= 64-n;

  br->nextbits <<= n;
  br->nextbits_cnt -= n;

  return val;
}

int  peek_bits(bitreader* br, int n)
{
  if (br->nextbits_cnt < n) {
    bitreader_refill(br);
  }

  uint64_t val = br->nextbits;
  val >>= 64-n;

  return val;
}

void skip_bits(bitreader* br, int n)
{
  if (br->nextbits_cnt < n) {
    bitreader_refill(br);
  }

  br->nextbits <<= n;
  br->nextbits_cnt -= n;
}

void skip_bits_fast(bitreader* br, int n)
{
  br->nextbits <<= n;
  br->nextbits_cnt -= n;
}

void skip_to_byte_boundary(bitreader* br)
{
  int nskip = (br->nextbits_cnt & 7);

  br->nextbits <<= nskip;
  br->nextbits_cnt -= nskip;
}

void prepare_for_CABAC(bitreader* br)
{
  skip_to_byte_boundary(br);

  int rewind = br->nextbits_cnt/8;
  br->data -= rewind;
  br->bytes_remaining += rewind;
  br->nextbits = 0;
  br->nextbits_cnt = 0;
}

int  get_uvlc(bitreader* br)
{
  int num_zeros=0;

  while (get_bits(br,1)==0) {
    num_zeros++;

    if (num_zeros > MAX_UVLC_LEADING_ZEROS) { return UVLC_ERROR; }
  }

  int offset = 0;
  if (num_zeros != 0) {
    offset = get_bits(br, num_zeros);
    return offset + (1<<num_zeros)-1;
  } else {
    return 0;
  }
}

int  get_svlc(bitreader* br)
{
  int v = get_uvlc(br);
  if (v==0) return v;
  if (v==UVLC_ERROR) return UVLC_ERROR;

  bool negative = ((v&1)==0);
  return negative ? -v/2 : (v+1)/2;
}

void check_rbsp_trailing_bits(bitreader* br)
{
  int stop_bit = get_bits(br,1);
  assert(stop_bit==1);

  while (br->nextbits_cnt>0 || br->bytes_remaining>0) {
    int filler = get_bits(br,1);
    assert(filler==0);
  }
}
