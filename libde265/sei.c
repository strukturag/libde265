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

#include "sei.h"
#include "util.h"
#include "md5.h"

#include <assert.h>


static bool read_sei_decoded_picture_hash(bitreader* reader, sei_message* sei,
                                          const decoder_context* ctx)
{
  // cannot read hash SEI, because SPS is not defined
  if (ctx->current_sps == NULL) {
    return false;
  }

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

  return true;
}


static void dump_sei_decoded_picture_hash(const sei_message* sei,
                                          const decoder_context* ctx)
{
  const sei_decoded_picture_hash* seihash = &sei->data.decoded_picture_hash;

  loginfo(LogSEI,"  hash_type: ");
  switch (seihash->hash_type) {
  case sei_decoded_picture_hash_type_MD5: loginfo(LogSEI,"MD5\n"); break;
  case sei_decoded_picture_hash_type_CRC: loginfo(LogSEI,"CRC\n"); break;
  case sei_decoded_picture_hash_type_checksum: loginfo(LogSEI,"checksum\n"); break;
  }

  int nHashes = ctx->current_sps->chroma_format_idc==0 ? 1 : 3;
  for (int i=0;i<nHashes;i++) {
    switch (seihash->hash_type) {
    case sei_decoded_picture_hash_type_MD5:
      loginfo(LogSEI,"  MD5[%d]: %02x", i,seihash->md5[i][0]);
      for (int b=1;b<16;b++) {
        loginfo(LogSEI,"*:%02x", seihash->md5[i][b]);
      }
      loginfo(LogSEI,"*\n");
      break;

    case sei_decoded_picture_hash_type_CRC:
      loginfo(LogSEI,"  CRC[%d]: %02x\n", i,seihash->crc[i]);
      break;

    case sei_decoded_picture_hash_type_checksum:
      loginfo(LogSEI,"  checksum[%d]: %04x\n", i,seihash->checksum[i]);
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

static inline uint16_t crc_process_byte(uint16_t crc, uint8_t byte)
{
  for (int bit=0;bit<8;bit++) {
    int bitVal = (byte >> (7-bit)) & 1;

    int crcMsb = (crc>>15) & 1;
    crc = (((crc<<1) + bitVal) & 0xFFFF);

    if (crcMsb) { crc ^=  0x1021; }
  }

  return crc;
}

/*
static uint16_t compute_CRC_8bit_old(const uint8_t* data,int w,int h,int stride)
{
  uint16_t crc = 0xFFFF;

  for (int y=0; y<h; y++)
    for(int x=0; x<w; x++) {
      crc = crc_process_byte(crc, data[y*stride+x]);
    }

  crc = crc_process_byte(crc, 0);
  crc = crc_process_byte(crc, 0);

  return crc;
}
*/

static inline uint16_t crc_process_byte_parallel(uint16_t crc, uint8_t byte)
{
  uint16_t s = byte ^ (crc >> 8);
  uint16_t t = s ^ (s >> 4);

  return  ((crc << 8) ^
	   t ^
	   (t <<  5) ^
	   (t << 12)) & 0xFFFF;
}

static uint32_t compute_CRC_8bit_fast(const uint8_t* data,int w,int h,int stride)
{
  uint16_t crc = 0xFFFF;

  crc = crc_process_byte_parallel(crc, 0);
  crc = crc_process_byte_parallel(crc, 0);

  for (int y=0; y<h; y++) {
    const uint8_t* d = &data[y*stride];

    for(int x=0; x<w; x++) {
      crc = crc_process_byte_parallel(crc, *d++);
    }
  }

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
  if (ctx->current_sps == NULL || ctx->last_decoded_image == NULL) {
    add_warning(ctx,DE265_ERROR_CANNOT_PROCESS_SEI, false);
    return DE265_OK;
  }

  const sei_decoded_picture_hash* seihash = &sei->data.decoded_picture_hash;

  de265_image* img = ctx->last_decoded_image;
  assert(img != NULL);

  /* Do not check SEI on pictures that are not output.
     Hash may be wrong, because of a broken link (BLA).
     This happens, for example in conformance stream RAP_B, where a EOS-NAL
     appears before a CRA (POC=32). */
  if (img->PicOutputFlag == false) {
    return DE265_OK;
  }

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
        uint16_t crc = compute_CRC_8bit_fast(data,w,h,stride);

        logtrace(LogSEI,"SEI decoded picture hash: %04x <-[%d]-> decoded picture: %04x\n",
                 seihash->crc[i], i, crc);

        if (crc != seihash->crc[i]) {
          fprintf(stderr,"SEI decoded picture hash: %04x, decoded picture: %04x (POC=%d)\n",
                  seihash->crc[i], crc, img->PicOrderCntVal);
          return DE265_ERROR_CHECKSUM_MISMATCH;
        }
      }
      break;

    case sei_decoded_picture_hash_type_checksum:
      {
        uint32_t chksum = compute_checksum_8bit(data,w,h,stride);

        if (chksum != seihash->checksum[i]) {
          fprintf(stderr,"SEI decoded picture hash: %04x, decoded picture: %04x (POC=%d)\n",
                  seihash->checksum[i], chksum, img->PicOrderCntVal);
          return DE265_ERROR_CHECKSUM_MISMATCH;
        }
      }
      break;
    }
  }

  loginfo(LogSEI,"decoded picture hash checked: OK\n");

  return DE265_OK;
}


bool read_sei(bitreader* reader, sei_message* sei, bool suffix, const decoder_context* ctx)
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

  bool success=false;

  switch (sei->payload_type) {
  case sei_payload_type_decoded_picture_hash:
    success = read_sei_decoded_picture_hash(reader,sei,ctx);
    break;

  default:
    // TODO: unknown SEI messages are ignored
    break;
  }

  return success;
}

void dump_sei(const sei_message* sei, const decoder_context* ctx)
{
  loginfo(LogHeaders,"SEI message: %s\n", sei_type_name(sei->payload_type));

  switch (sei->payload_type) {
  case sei_payload_type_decoded_picture_hash:
    dump_sei_decoded_picture_hash(sei, ctx);
    break;

  default:
    // TODO: unknown SEI messages are ignored
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

  default:
    // TODO: unknown SEI messages are ignored
    break;
  }

  return err;
}


const char* sei_type_name(enum sei_payload_type type)
{
  switch (type) {
  case sei_payload_type_buffering_period:
    return "buffering_period";
  case sei_payload_type_pic_timing:
    return "pic_timing";
  case sei_payload_type_pan_scan_rect:
    return "pan_scan_rect";
  case sei_payload_type_filler_payload:
    return "filler_payload";
  case sei_payload_type_user_data_registered_itu_t_t35:
    return "user_data_registered_itu_t_t35";
  case sei_payload_type_user_data_unregistered:
    return "user_data_unregistered";
  case sei_payload_type_recovery_point:
    return "recovery_point";
  case sei_payload_type_scene_info:
    return "scene_info";
  case sei_payload_type_picture_snapshot:
    return "picture_snapshot";
  case sei_payload_type_progressive_refinement_segment_start:
    return "progressive_refinement_segment_start";
  case sei_payload_type_progressive_refinement_segment_end:
    return "progressive_refinement_segment_end";
  case sei_payload_type_film_grain_characteristics:
    return "film_grain_characteristics";
  case sei_payload_type_post_filter_hint:
    return "post_filter_hint";
  case sei_payload_type_tone_mapping_info:
    return "tone_mapping_info";
  case sei_payload_type_frame_packing_arrangement:
    return "frame_packing_arrangement";
  case sei_payload_type_display_orientation:
    return "display_orientation";
  case sei_payload_type_structure_of_pictures_info:
    return "structure_of_pictures_info";
  case sei_payload_type_active_parameter_sets:
    return "active_parameter_sets";
  case sei_payload_type_decoding_unit_info:
    return "decoding_unit_info";
  case sei_payload_type_temporal_sub_layer_zero_index:
    return "temporal_sub_layer_zero_index";
  case sei_payload_type_decoded_picture_hash:
    return "decoded_picture_hash";
  case sei_payload_type_scalable_nesting:
    return "scalable_nesting";
  case sei_payload_type_region_refresh_info:
    return "region_refresh_info";
  case sei_payload_type_no_display:
    return "no_display";
  case sei_payload_type_motion_constrained_tile_sets:
    return "motion_constrained_tile_sets";

  default:
    return "unknown SEI message";
  }
}

