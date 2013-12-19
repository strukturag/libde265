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

#include "sei.h"
#include "util.h"
#include "md5.h"

#include <assert.h>


static void read_sei_decoded_picture_hash(bitreader* reader, sei_message* sei,
                                          const decoder_context* ctx)
{
  sei_decoded_picture_hash* seihash = &sei->data.decoded_picture_hash;

  seihash->hash_type = (enum sei_decoded_picture_hash_type)get_bits(reader,8);

  int nHashes = ctx->current_sps->chroma_format_idc==0 ? 1 : 3;
  for (int i=0;i<nHashes;i++) {
    switch (seihash->hash_type) {
    case sei_decoded_picture_hash_type_MD5:
      for (int b=0;b<16;b++) { seihash->md5[i][b] = get_bits(reader,8); }
      break;

    case sei_decoded_picture_hash_type_CRC:
      seihash->crc[i] = get_bits(reader,16);
      break;

    case sei_decoded_picture_hash_type_checksum:
      seihash->checksum[i]  = get_bits(reader,32);
      break;
    }
  }
}


static void dump_sei_decoded_picture_hash(const sei_message* sei,
                                          const decoder_context* ctx)
{
  const sei_decoded_picture_hash* seihash = &sei->data.decoded_picture_hash;

  logdebug(LogSEI,"  hash_type: ");
  switch (seihash->hash_type) {
  case sei_decoded_picture_hash_type_MD5: logdebug(LogSEI,"MD5\n"); break;
  case sei_decoded_picture_hash_type_CRC: logdebug(LogSEI,"CRC\n"); break;
  case sei_decoded_picture_hash_type_checksum: logdebug(LogSEI,"checksum\n"); break;
  }

  int nHashes = ctx->current_sps->chroma_format_idc==0 ? 1 : 3;
  for (int i=0;i<nHashes;i++) {
    switch (seihash->hash_type) {
    case sei_decoded_picture_hash_type_MD5:
      logdebug(LogSEI,"  MD5[%d]: %02x", i,seihash->md5[i][0]);
      for (int b=1;b<16;b++) {
        logdebug(LogSEI,"*:%02x", seihash->md5[i][b]);
      }
      logdebug(LogSEI,"*\n");
      break;

    case sei_decoded_picture_hash_type_CRC:
      logdebug(LogSEI,"  CRC[%d]: %02x\n", i,seihash->crc[i]);
      break;

    case sei_decoded_picture_hash_type_checksum:
      logdebug(LogSEI,"  checksum[%d]: %04x\n", i,seihash->checksum[i]);
      break;
    }
  }
}


static uint32_t compute_checksum_8bit(uint8_t* data,int w,int h,int stride)
{
  uint32_t sum = 0;
  for (int y=0; y<h; y++)
    for(int x=0; x<w; x++) {
      uint8_t xorMask = ( x & 0xFF ) ^ ( y & 0xFF ) ^ ( x  >>  8 ) ^ ( y  >>  8 );
      sum += data[y*stride + x] ^ xorMask;

      /*
      if (compDepth[cIdx] > 8 )
        sum = ( sum + ( ( component[cIdx][y * compWidth[cIdx] + x]  >>  8 ) ^ xorMask ) ) &
          0xFFFFFFFF
          }
      */
    }

  return sum & 0xFFFFFFFF;
}

static uint32_t crc_process_byte(uint32_t crc, uint8_t byte)
{
  for (int bit=0;bit<8;bit++) {
    int bitVal = (byte >> (7-bit)) & 1;

    int crcMsb = (crc>>15) & 1;
    crc = (((crc<<1) + bitVal) & 0xFFFF) ^ (crcMsb * 0x1021);
  }

  return crc;
}

static uint32_t compute_CRC_8bit(uint8_t* data,int w,int h,int stride)
{
  uint32_t crc = 0xFFFF;
  for (int y=0; y<h; y++)
    for(int x=0; x<w; x++) {
      crc = crc_process_byte(crc, data[y*stride+x]);
    }

  crc = crc_process_byte(crc, 0);
  crc = crc_process_byte(crc, 0);

  return crc;
}

static void compute_MD5_8bit(uint8_t* data,int w,int h,int stride, uint8_t* result)
{
  MD5_CTX md5;
  MD5_Init(&md5);

  for (int y=0; y<h; y++) {
    MD5_Update(&md5, &data[y*stride], w);
  }

  MD5_Final(result, &md5);
}


static de265_error process_sei_decoded_picture_hash(const sei_message* sei, decoder_context* ctx)
{
  const sei_decoded_picture_hash* seihash = &sei->data.decoded_picture_hash;

  de265_image* img = ctx->last_decoded_image;
  assert(img != NULL);

  //write_picture(img);

  int nHashes = ctx->current_sps->chroma_format_idc==0 ? 1 : 3;
  for (int i=0;i<nHashes;i++) {
    uint8_t* data;
    int w,h,stride;

    switch (i) {
    case 0:
      w = img->width;
      h = img->height;
      stride = img->stride;
      break;

    case 1:
    case 2:
      w = img->chroma_width;
      h = img->chroma_height;
      stride = img->chroma_stride;
      break;
    }

    /**/ if (i==0) data = img->y;
    else if (i==1) data = img->cb;
    else           data = img->cr;

    switch (seihash->hash_type) {
    case sei_decoded_picture_hash_type_MD5:
      {
        uint8_t md5[16];
        compute_MD5_8bit(data,w,h,stride,md5);

/*
        fprintf(stderr,"computed MD5: ");
        for (int b=0;b<16;b++) {
          fprintf(stderr,"%02x", md5[b]);
        }
        fprintf(stderr,"\n");
*/

        for (int b=0;b<16;b++) {
          if (md5[b] != seihash->md5[i][b]) {
            fprintf(stderr,"SEI decoded picture MD5 mismatch (POC=%d)\n", img->PicOrderCntVal);
            return DE265_ERROR_CHECKSUM_MISMATCH;
          }
        }
      }
      break;

    case sei_decoded_picture_hash_type_CRC:
      {
        uint32_t crc = compute_CRC_8bit(data,w,h,stride);

        logtrace(LogSEI,"SEI decoded picture hash: %04x <-[%d]-> decoded picture: %04x\n",
                 seihash->crc[i], i, crc);

        if (crc != seihash->crc[i]) {
          fprintf(stderr,"SEI decoded picture hash: %04x, decoded picture: %04x\n",
                  seihash->crc[i], crc);
          return DE265_ERROR_CHECKSUM_MISMATCH;
        }
      }
      break;

    case sei_decoded_picture_hash_type_checksum:
      {
        uint32_t chksum = compute_checksum_8bit(data,w,h,stride);

        if (chksum != seihash->checksum[i]) {
          fprintf(stderr,"SEI decoded picture hash: %04x, decoded picture: %04x\n",
                  seihash->checksum[i], chksum);
          return DE265_ERROR_CHECKSUM_MISMATCH;
        }
      }
      break;
    }
  }

  logdebug(LogSEI,"decoded picture hash checked: OK\n");

  return DE265_OK;
}


void read_sei(bitreader* reader, sei_message* sei, bool suffix, const decoder_context* ctx)
{
  int payload_type = 0;
  for (;;)
    {
      int byte = get_bits(reader,8);
      payload_type += byte;
      if (byte != 0xFF) { break; }
    }

  int payload_size = 0;
  for (;;)
    {
      int byte = get_bits(reader,8);
      payload_size += byte;
      if (byte != 0xFF) { break; }
    }

  sei->payload_type = (enum sei_payload_type)payload_type;
  sei->payload_size = payload_size;


  // --- sei message dispatch

  switch (sei->payload_type) {
  case sei_payload_type_decoded_picture_hash:
    read_sei_decoded_picture_hash(reader,sei,ctx);
    break;
  }
}

void dump_sei(const sei_message* sei, const decoder_context* ctx)
{
  switch (sei->payload_type) {
  case sei_payload_type_decoded_picture_hash:
    dump_sei_decoded_picture_hash(sei, ctx);
    break;
  }
}


de265_error process_sei(const sei_message* sei, decoder_context* ctx)
{
  de265_error err = DE265_OK;

  switch (sei->payload_type) {
  case sei_payload_type_decoded_picture_hash:
    if (ctx->param_sei_check_hash) {
      err = process_sei_decoded_picture_hash(sei, ctx);
    }

    break;
  }

  return err;
}


