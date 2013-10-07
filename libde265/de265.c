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

#include "de265.h"
#include "decctx.h"
#include "slice_func.h"
#include "pps_func.h"
#include "util.h"
#include "scan.h"
#include "image.h"
#include "sei.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>


int  de265_decode_NAL(de265_decoder_context* de265ctx, rbsp_buffer* data);



const char* de265_get_error_text(de265_error err)
{
  switch (err) {
  case DE265_OK: return "no error";
  case DE265_ERROR_NO_SUCH_FILE: return "no such file";
  case DE265_ERROR_NO_STARTCODE: return "no startcode found";
  case DE265_ERROR_EOF: return "end of file";
  case DE265_ERROR_COEFFICIENT_OUT_OF_IMAGE_BOUNDS: return "coefficient out of image bounds";
  case DE265_ERROR_CHECKSUM_MISMATCH: return "image checksum mismatch";
  case DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA: return "CTB outside of image area";
  case DE265_ERROR_OUT_OF_MEMORY: return "out of memory";
  case DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE: return "coded parameter out of range";
  case DE265_ERROR_IMAGE_BUFFER_FULL: return "DPB/output queue full";
  default: return "unknown error";
  }
}





void de265_init()
{
  init_scan_orders();
}


de265_decoder_context* de265_new_decoder()
{
  decoder_context* ctx = (decoder_context*)calloc(sizeof(decoder_context),1);
  if (!ctx) { return NULL; }

  init_decoder_context(ctx);
  return (de265_decoder_context*)ctx;
}


void de265_free_decoder(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  free_decoder_context(ctx);
  free(de265ctx);
}


de265_error de265_decode_data(de265_decoder_context* de265ctx, uint8_t* data, int len)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  /*
  printf("len=%d\n",len);

  for (int i=0;i<16;i++) {
    printf("%02x ",data[i]);
  }
  printf("\n");
  */

  // Resize output buffer so that complete input would fit.
  // We add 3, because in the worst case 3 extra bytes are created for an input byte.
  rbsp_buffer_resize(&ctx->nal_data, ctx->nal_data.size + len + 3);

  unsigned char* out = ctx->nal_data.data + ctx->nal_data.size;

  if (len==0) {
    if      (ctx->input_push_state<5) { return DE265_ERROR_EOF; }
    else if (ctx->input_push_state==6) { *out++ = 0; }
    else if (ctx->input_push_state==7) { *out++ = 0; *out++ = 0; }

    // decode data
    ctx->nal_data.size = out - ctx->nal_data.data;
    de265_decode_NAL(de265ctx, &ctx->nal_data);

    // clear buffer
    ctx->nal_data.size = 0;
    out = ctx->nal_data.data;

    return DE265_OK;
  }


  for (int i=0;i<len;i++) {
    /*
    printf("state=%d input=%02x (%p) (output size: %d)\n",ctx->input_push_state, *data, data,
           out - ctx->nal_data.data);
    */

    switch (ctx->input_push_state) {
    case 0:
    case 1:
      if (*data == 0) { ctx->input_push_state++; }
      else { return DE265_ERROR_NO_STARTCODE; }
      break;
    case 2:
      if      (*data == 1) { ctx->input_push_state=3; }
      else if (*data == 0) { } // *out++ = 0; }
      else { return DE265_ERROR_NO_STARTCODE; }
      break;
    case 3:
      /*
      *out++ = 0;
      *out++ = 0;
      *out++ = 1;
      */
      *out++ = *data;
      ctx->input_push_state = 4;
      break;
    case 4:
      *out++ = *data;
      ctx->input_push_state = 5;
      break;

    case 5:
      if (*data==0) { ctx->input_push_state=6; }
      else { *out++ = *data; }
      break;

    case 6:
      if (*data==0) { ctx->input_push_state=7; }
      else {
        *out++ = 0;
        *out++ = *data;
        ctx->input_push_state=5;
      }
      break;

    case 7:
      if      (*data==0) { *out++ = 0; }
      else if (*data==3) { *out++ = 0; *out++ = 0; ctx->input_push_state=5; }
      else if (*data==1) {

        // decode this NAL
        ctx->nal_data.size = out - ctx->nal_data.data;
        de265_decode_NAL(de265ctx, &ctx->nal_data);

        // clear buffer for next NAL
        ctx->nal_data.size = 0;
        out = ctx->nal_data.data;

        ctx->input_push_state=3;
      }
      else {
        *out++ = 0;
        *out++ = 0;
        *out++ = *data;

        ctx->input_push_state=5;
      }
      break;
    }

    data++;

    /*
    for (int i=0;i<out - ctx->nal_data.data;i++) {
      printf("%02x ",ctx->nal_data.data[i]);
    }
    printf("\n");
    */
  }

  ctx->nal_data.size = out - ctx->nal_data.data;
  return DE265_OK;
}


int  de265_decode_NAL(de265_decoder_context* de265ctx, rbsp_buffer* data)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  int err = DE265_OK;

  bitreader reader;
  bitreader_init(&reader, data);

  nal_header nal_hdr;
  nal_read_header(&reader, &nal_hdr);
  process_nal_hdr(ctx, &nal_hdr);

  logdebug(LogHighlevel,"NAL: 0x%x 0x%x -  %d %d\n",
           data->data[0], data->data[1],
           nal_hdr.nal_unit_type,
           nal_hdr.nuh_temporal_id);


  if (nal_hdr.nal_unit_type<32) {
    logdebug(LogHeaders,"---> read slice segment header\n");

    int sliceIndex = get_next_slice_index(ctx);
    slice_segment_header* hdr = &ctx->slice[sliceIndex];
    hdr->slice_index = sliceIndex;
    read_slice_segment_header(&reader,hdr,ctx);
    dump_slice_segment_header(hdr, ctx);

    if ((err = process_slice_segment_header(ctx, hdr)) != DE265_OK)
      { return err; }

    skip_bits(&reader,1); // TODO: why?
    prepare_for_CABAC(&reader);
    init_CABAC_decoder(&hdr->cabac_decoder, reader.data, reader.bytes_remaining);

    if ((err=read_slice_segment_data(ctx, hdr)) != DE265_OK)
      { return err; }
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      {
        logdebug(LogHeaders,"---> read VPS\n");

        video_parameter_set vps;
        read_vps(&reader,&vps);
        dump_vps(&vps);

        process_vps(ctx, &vps);
      }
      break;

    case NAL_UNIT_SPS_NUT:
      {
        logdebug(LogHeaders,"----> read SPS\n");

        seq_parameter_set sps;

        if ((err=read_sps(&reader,&sps, &ctx->ref_pic_sets)) != DE265_OK) {
          break;
        }
        dump_sps(&sps, ctx->ref_pic_sets);

        process_sps(ctx, &sps);
      }
      break;

    case NAL_UNIT_PPS_NUT:
      {
        logdebug(LogHeaders,"----> read PPS\n");

        pic_parameter_set pps;

        init_pps(&pps);
        read_pps(&reader,&pps,ctx);
        dump_pps(&pps);

        process_pps(ctx,&pps);
      }
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      logdebug(LogHeaders,"----> read SEI\n");

      sei_message sei;

      read_sei(&reader,&sei, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT, ctx);
      dump_sei(&sei, ctx);

      err = process_sei(&sei, ctx);
      break;
    }

  return err;
}


const struct de265_image* de265_get_next_picture(de265_decoder_context* de265ctx)
{
  const struct de265_image* img = de265_peek_next_picture(de265ctx);
  if (img) {
    de265_release_next_picture(de265ctx);
  }

  return img;
}


const struct de265_image* de265_peek_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  // check for empty queue -> return NULL

  if (ctx->image_output_queue_length==0) {
    return NULL;
  }

  return ctx->image_output_queue[0];
}


void de265_release_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  // no active output picture -> ignore release request

  if (ctx->image_output_queue_length==0) { return; }


  loginfo(LogDPB, "release DPB with POC=%d\n",ctx->image_output_queue[0]->PicOrderCntVal);

  ctx->image_output_queue[0]->PicOutputFlag = false;

  // pop output queue

  for (int i=1;i<ctx->image_output_queue_length;i++)
    {
      ctx->image_output_queue[i-1] = ctx->image_output_queue[i];
    }

  ctx->image_output_queue_length--;

  ctx->image_output_queue[ ctx->image_output_queue_length ] = NULL;


  loginfo(LogDPB, "* DPB output queue: ");
  for (int i=0;i<ctx->image_output_queue_length;i++) {
    loginfo(LogDPB, "*%d ", ctx->image_output_queue[i]->PicOrderCntVal);
  }
  loginfo(LogDPB,"");
}


void de265_set_parameter_bool(de265_decoder_context* de265ctx, enum de265_param param, int value)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  switch (param)
    {
    case DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH:
      ctx->param_sei_check_hash = value;
      break;

    default:
      assert(false);
      break;
    }
}


int de265_get_parameter_bool(de265_decoder_context* de265ctx, enum de265_param param)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  switch (param)
    {
    case DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH:
      return ctx->param_sei_check_hash;
      break;

    default:
      assert(false);
      return false;
    }
}


int de265_get_image_width(const struct de265_image* img,int channel)
{
  switch (channel) {
  case 0:
    return img->width;
  case 1:
  case 2:
    return img->chroma_width;
  default:
    return 0;
  }
}

int de265_get_image_height(const struct de265_image* img,int channel)
{
  switch (channel) {
  case 0:
    return img->height;
  case 1:
  case 2:
    return img->chroma_height;
  default:
    return 0;
  }
}

enum de265_chroma de265_get_chroma_format(const struct de265_image* img)
{
  return img->chroma_format;
}

const uint8_t* de265_get_image_plane(const de265_image* img, int channel, int* out_stride)
{
  switch (channel) {
  case 0:
    if (out_stride) { *out_stride = img->stride; }
    return img->y;

  case 1:
  case 2:
    if (out_stride) { *out_stride = img->chroma_stride; }
    if (channel==1) { return img->cb; }
    else            { return img->cr; }

  default:
    if (out_stride) { *out_stride = 0; }
    return NULL;
  }
}
