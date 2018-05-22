/*
 * H.265 video codec.
 * Copyright (c) 2013-2017 struktur AG, Dirk Farin <farin@struktur.de>
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

#define DEBUG_INSERT_STREAM_ERRORS 0


#include "de265.h"
#include "decctx.h"
#include "util.h"
#include "scan.h"
#include "image.h"
#include "sei.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <mutex>


// TODO: should be in some vps.c related header
de265_error read_vps(decoder_context* ctx, bitreader* reader, video_parameter_set* vps);

extern "C" {
LIBDE265_API const char *de265_get_version(void)
{
    return (LIBDE265_VERSION);
}

LIBDE265_API uint32_t de265_get_version_number(void)
{
    return (LIBDE265_NUMERIC_VERSION);
}

LIBDE265_API int de265_get_version_number_major(void)
{
  return ((LIBDE265_NUMERIC_VERSION)>>24) & 0xFF;
}

LIBDE265_API int de265_get_version_number_minor(void)
{
  return ((LIBDE265_NUMERIC_VERSION)>>16) & 0xFF;
}

LIBDE265_API int de265_get_version_number_maintenance(void)
{
  return ((LIBDE265_NUMERIC_VERSION)>>8) & 0xFF;
}


LIBDE265_API const char* de265_get_error_text(de265_error err)
{
  switch (err) {
  case DE265_OK: return "no error";
    //case DE265_ERROR_NO_STARTCODE: return "no startcode found";
    //case DE265_ERROR_EOF: return "end of file";
  case DE265_WARNING_SEI_CHECKSUM_MISMATCH: return "SEI image checksum mismatch";
  case DE265_WARNING_CTB_OUTSIDE_IMAGE_AREA: return "CTB outside of image area";
  case DE265_ERROR_OUT_OF_MEMORY: return "out of memory";
    //case DE265_WARNING_CODED_PARAMETER_OUT_OF_RANGE: return "coded parameter out of range";
  case DE265_ERROR_CANNOT_START_THREADS: return "cannot start decoding threads";
  case DE265_ERROR_LIBRARY_NOT_INITIALIZED: return "library is not initialized";

  //case DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED:
  //  return "internal error: maximum number of thread contexts exceeded";
  //case DE265_ERROR_MAX_NUMBER_OF_SLICES_EXCEEDED:
  //  return "internal error: maximum number of slices exceeded";
  case DE265_ERROR_MANDATORY_FUNCTIONALITY_NOT_IMPLEMENTED_YET:
    return "unimplemented mandatory decoder feature";
  case DE265_WARNING_OPTIONAL_FUNCTIONALITY_NOT_IMPLEMENTED_YET:
    return "unimplemented optional decoder feature";
    //case DE265_ERROR_SCALING_LIST_NOT_IMPLEMENTED:
    //return "scaling list not implemented";

  case DE265_ERROR_PARAMETER_PARSING:
    return "command-line parameter error";
  case DE265_WARNING_DEPENDENT_SLICE_WITHOUT_INITIAL_SLICE_HEADER:
    return "initial slice header missing, cannot decode dependent slice";
  case DE265_WARNING_PREMATURE_END_OF_SLICE:
    return "premature end of slice data";

  case DE265_WARNING_WARNING_BUFFER_FULL:
    return "Too many warnings queued";
  case DE265_WARNING_INCORRECT_ENTRY_POINT_OFFSET:
    return "Incorrect entry-point offsets";
  case DE265_WARNING_INVALID_SPS_PARAMETER:
    return "SPS header parameter invalid";
  case DE265_WARNING_INVALID_PPS_PARAMETER:
    return "PPS header parameter invalid";
  case DE265_WARNING_INVALID_VUI_PARAMETER:
    return "VUI header parameter invalid";
  case DE265_WARNING_INVALID_VPS_PARAMETER:
    return "VPS header parameter invalid";
  case DE265_WARNING_INVALID_SLICE_PARAMETER:
    return "slice header parameter invalid";
  case DE265_WARNING_SLICEHEADER_INVALID:
    return "slice header invalid";
  case DE265_WARNING_INCORRECT_MOTION_VECTOR_SCALING:
    return "impossible motion vector scaling";
  case DE265_WARNING_NONEXISTING_PPS_REFERENCED:
    return "non-existing PPS referenced";
  case DE265_WARNING_NONEXISTING_SPS_REFERENCED:
    return "non-existing SPS referenced";
  case DE265_WARNING_BOTH_PREDFLAGS_ZERO:
    return "both predFlags[] are zero in MC";
  case DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED:
    return "non-existing reference picture accessed";
  case DE265_WARNING_NUMMVP_NOT_EQUAL_TO_NUMMVQ:
    return "numMV_P != numMV_Q in deblocking";
  case DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE:
    return "number of short-term ref-pic-sets out of range";
  case DE265_WARNING_SHORT_TERM_REF_PIC_SET_OUT_OF_RANGE:
    return "short-term ref-pic-set index out of range";
  case DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE:
    return "parameter in short-term ref-pic-set is out of range";
  case DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST:
    return "faulty reference picture list";
  case DE265_WARNING_END_OF_SUBSTREAM_BIT_NOT_SET:
    return "end_of_sub_stream_one_bit not set to 1 when it should be";
  case DE265_WARNING_INVALID_CHROMA_FORMAT:
    return "invalid chroma format in SPS header";
  case DE265_WARNING_SLICE_SEGMENT_ADDRESS_INVALID:
    return "slice segment address invalid";
  case DE265_WARNING_DEPENDENT_SLICE_WITH_ADDRESS_ZERO:
    return "dependent slice with address 0";
  case DE265_WARNING_INVALID_LT_REFERENCE_CANDIDATE:
    return "invalid long-term reference candidate specified in slice header";
  case DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY:
    return "cannot apply SAO because we ran out of memory";
  case DE265_WARNING_CANNOT_DECODE_SEI_BECAUSE_SPS_IS_MISSING:
    return "cannot decode SEI because SPS header is missing";
  case DE265_WARNING_COLLOCATED_MOTION_VECTOR_OUTSIDE_IMAGE_AREA:
    return "collocated motion-vector is outside image area";
  case DE265_WARNING_DECODING_ERROR:
    return "noncritical error occurred during decoding";

  default: return "unknown error";
  }
}


//LIBDE265_API int de265_isOK(de265_error err)
//{
//  return err == DE265_OK || err < 1000;
//}



static int de265_init_count;

static std::mutex de265_init_mutex;


LIBDE265_API de265_error de265_init()
{
  std::lock_guard<std::mutex> lock(de265_init_mutex);

  de265_init_count++;

  if (de265_init_count > 1) {
    // we are not the first -> already initialized

    return DE265_OK;
  }


  // do initializations

  init_scan_orders();

  de265_error err;

  err = alloc_and_init_significant_coeff_ctxIdx_lookupTable();
  if (err) {
    de265_init_count--;
    return DE265_ERROR_LIBRARY_NOT_INITIALIZED;
  }

  return DE265_OK;
}

LIBDE265_API de265_error de265_free()
{
  std::lock_guard<std::mutex> lock(de265_init_mutex);

  if (de265_init_count<=0) {
    return DE265_ERROR_LIBRARY_NOT_INITIALIZED;
  }

  de265_init_count--;

  if (de265_init_count==0) {
    free_significant_coeff_ctxIdx_lookupTable();
  }

  return DE265_OK;
}


LIBDE265_API de265_decoder_context* de265_new_decoder()
{
  de265_error init_err = de265_init();
  if (init_err != DE265_OK) {
    return NULL;
  }

  decoder_context* ctx = new decoder_context;
  if (!ctx) {
    de265_free();
    return NULL;
  }

  return (de265_decoder_context*)ctx;
}


LIBDE265_API de265_error de265_free_decoder(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  ctx->stop_thread_pool();

  delete ctx;

  return de265_free();
}


LIBDE265_API de265_error de265_start_worker_threads(de265_decoder_context* de265ctx, int number_of_threads)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  if (number_of_threads>0) {
    de265_error err = ctx->start_thread_pool(number_of_threads);
    return err;
  }
  else {
    return DE265_OK;
  }
}


static void dumpdata(const void* data, int len)
{
  for (int i=0;i<len;i++) {
    printf("%02x ", ((uint8_t*)data)[i]);
  }
  printf("\n");
}


LIBDE265_API struct de265_image* de265_alloc_image(int w,int h,enum de265_chroma chroma,
                                                   int bitDepth_luma, int bitDepth_chroma,
                                                   de265_PTS pts,
                                                   const de265_image_allocation* alloc_functions)
{
  de265_image* img = new de265_image();
  img->m_image = std::shared_ptr<image>(new image());
  img->m_image->alloc_image(w,h,chroma,bitDepth_luma,bitDepth_chroma,pts,
                            image::supplementary_data(),nullptr,alloc_functions);
  return img;
}


LIBDE265_API de265_error de265_push_data(de265_decoder_context* de265ctx,
                                         const void* data8, int len,
                                         de265_PTS pts, void* user_data)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  uint8_t* data = (uint8_t*)data8;

  //printf("push data (size %d)\n",len);
  //dumpdata(data8,16);

  return ctx->get_NAL_parser().push_data(data,len,pts,user_data);
}


LIBDE265_API de265_error de265_push_NAL(de265_decoder_context* de265ctx,
                                        const void* data8, int len,
                                        de265_PTS pts, void* user_data)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  uint8_t* data = (uint8_t*)data8;

  //printf("push NAL (size %d)\n",len);
  //dumpdata(data8,16);

  return ctx->get_NAL_parser().push_NAL(data,len,pts,user_data);
}


LIBDE265_API int de265_get_action(de265_decoder_context* de265ctx, int blocking)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  return ctx->get_action(blocking);
}


LIBDE265_API void        de265_push_end_of_NAL(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  ctx->get_NAL_parser().flush_data();
}


LIBDE265_API void        de265_push_end_of_frame(de265_decoder_context* de265ctx)
{
  de265_push_end_of_NAL(de265ctx);

  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->get_NAL_parser().mark_end_of_frame();
}


LIBDE265_API de265_error de265_push_end_of_stream(de265_decoder_context* de265ctx)
{
  de265_push_end_of_NAL(de265ctx);

  decoder_context* ctx = (decoder_context*)de265ctx;

  ctx->get_NAL_parser().flush_data();
  ctx->get_NAL_parser().mark_end_of_stream();

  return DE265_OK;
}


LIBDE265_API void de265_reset(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  //printf("--- reset ---\n");

  ctx->reset();
}


LIBDE265_API int de265_get_number_of_pictures_in_output_queue(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  return ctx->num_pictures_in_output_queue();
}


LIBDE265_API const struct de265_image* de265_get_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  while (ctx->num_pictures_in_output_queue()>0) {
    image_ptr i = ctx->get_next_picture_in_output_queue();

    if (0) {
      printf("output POC %d %d integr:%d\n",i->PicOrderCntVal,
             (i->integrity != INTEGRITY_NOT_DECODED), i->integrity);
    }

    // pop output queue

    ctx->pop_next_picture_in_output_queue();

    if (i->integrity != INTEGRITY_NOT_DECODED) {

      de265_image* img = new de265_image;
      img->m_image = i;

      return img;
    }
  }

  return NULL;
}


LIBDE265_API int de265_get_number_of_frames_pending_at_input(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  return ctx->number_of_frames_pending_at_input();
}


LIBDE265_API int de265_is_decoded_image_correct(const struct de265_image* img)
{
  return img->m_image->integrity == INTEGRITY_CORRECT;
}


LIBDE265_API void de265_skip_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  ctx->pop_next_picture_in_output_queue();
}


LIBDE265_API const struct de265_image* de265_peek_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  while (ctx->num_pictures_in_output_queue()>0) {
    image_ptr i = ctx->get_next_picture_in_output_queue();

    if (i->integrity != INTEGRITY_NOT_DECODED) {
      de265_image* img = new de265_image;
      img->m_image = i;
      return img;
    }
    else {
      ctx->pop_next_picture_in_output_queue();
    }
  }

  return NULL;
}


LIBDE265_API void de265_release_picture(const de265_image* de265img)
{
  assert(de265img);
  delete de265img;
}

LIBDE265_API int  de265_get_highest_TID(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  return ctx->get_frontend_syntax_decoder().get_highest_TID();
}

/*
LIBDE265_API int  de265_get_current_TID(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  return ctx->get_current_TID();
}
*/

LIBDE265_API void de265_set_highest_TID_to_decode(de265_decoder_context* de265ctx,int max_tid)
{
  if (max_tid == de265_limit_TID_unlimited) {
    max_tid = 6;
  }

  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->set_limit_TID(max_tid);
}

LIBDE265_API void de265_set_framerate_ratio(de265_decoder_context* de265ctx,int percent)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->set_framerate_ratio(percent);
}

  /*
LIBDE265_API int  de265_change_framerate(de265_decoder_context* de265ctx,int more)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  return ctx->change_framerate(more);
}
  */

LIBDE265_API de265_error de265_get_warning(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  return ctx->get_next_warning().code;
}


LIBDE265_API void de265_set_CPU_capabilities(de265_decoder_context* de265ctx, int capabilities)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->param_CPU_capabilities = capabilities;
}


LIBDE265_API int de265_get_CPU_capabilites_all_autodetected()
{
  return (de265_CPU_capability_X86_SSE2  |
          de265_CPU_capability_X86_SSE41 |
          de265_CPU_capability_X86_AVX2  |
          de265_CPU_capability_ARM_AARCH64);
}


LIBDE265_API void de265_allow_inexact_decoding(de265_decoder_context* de265ctx, int flags)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->param_inexact_decoding_flags = flags;

  ctx->param_disable_deblocking = !!(flags & de265_inexact_decoding_no_deblocking);
  ctx->param_disable_sao        = !!(flags & de265_inexact_decoding_no_SAO);
}


LIBDE265_API void de265_suppress_faulty_pictures(de265_decoder_context* de265ctx, int suppress)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->param_suppress_faulty_pictures = !!suppress;
}


LIBDE265_API void de265_set_dump_headers_callback(de265_decoder_context* de265ctx,
                                                  void (*callback)(int nal_unit, const char* text))
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->get_frontend_syntax_decoder().param_header_callback = callback;
}


  /*
    case DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH:
      ctx->param_sei_check_hash = !!value;
      break;
  */


LIBDE265_API void de265_set_max_frames_to_decode_in_parallel(de265_decoder_context* de265ctx,
                                                             int parallel_frames)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  ctx->set_max_frames_to_decode_in_parallel(parallel_frames);
}


LIBDE265_API int de265_get_number_of_input_bytes_pending(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  return ctx->get_NAL_parser().bytes_in_input_queue();
}


LIBDE265_API int de265_get_number_of_NAL_units_pending(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  return ctx->get_NAL_parser().number_of_NAL_units_pending();
}


LIBDE265_API void de265_set_max_reorder_buffer_latency(de265_decoder_context* de265ctx, int n)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  ctx->set_max_latency(n);
}


LIBDE265_API int de265_get_image_width(const struct de265_image* img,int channel)
{
  switch (channel) {
  case 0:
    return img->m_image->width_confwin;
  case 1:
  case 2:
    return img->m_image->chroma_width_confwin;
  default:
    return 0;
  }
}

LIBDE265_API int de265_get_image_height(const struct de265_image* img,int channel)
{
  switch (channel) {
  case 0:
    return img->m_image->height_confwin;
  case 1:
  case 2:
    return img->m_image->chroma_height_confwin;
  default:
    return 0;
  }
}


LIBDE265_API int de265_get_bits_per_pixel_from_spec(const struct de265_image_spec* spec,int channel)
{
  switch (channel) {
  case 0:
    return spec->luma_bits_per_pixel;
  case 1:
  case 2:
    return spec->chroma_bits_per_pixel;
  default:
    return 0;
  }
}


LIBDE265_API int de265_get_bits_per_pixel(const struct de265_image* img,int channel)
{
  return img->m_image->get_bit_depth(channel);
}

LIBDE265_API enum de265_chroma de265_get_chroma_format(const struct de265_image* img)
{
  return img->m_image->get_chroma_format();
}

LIBDE265_API const uint8_t* de265_get_image_plane(const de265_image* img, int channel, int* stride)
{
  assert(channel>=0 && channel <= 2);

  uint8_t* data = img->m_image->pixels_confwin[channel];

  if (stride) *stride = img->m_image->get_image_stride(channel) *
                ((de265_get_bits_per_pixel(img, channel)+7) / 8);

  return data;
}

LIBDE265_API void *de265_get_image_plane_user_data(const struct de265_image* img, int channel)
{
  assert(channel>=0 && channel <= 2);

  return img->m_image->plane_user_data[channel];
}

LIBDE265_API void *de265_get_image_plane_user_data_intern(const struct de265_image_intern* img, int channel)
{
  assert(channel>=0 && channel <= 2);

  image* iimg = (image*)img;
  return iimg->plane_user_data[channel];
}

LIBDE265_API void de265_set_image_plane_intern(de265_image_intern* img, int cIdx, void* mem, int stride, void *userdata)
{
  image* iimg = (image*)img;

  // The internal "stride" is the number of pixels per line.
  stride = stride / iimg->get_bytes_per_pixel(cIdx);

  iimg->set_image_plane(cIdx, (uint8_t*)mem, stride, userdata);
}

LIBDE265_API void de265_set_image_allocation_functions(de265_decoder_context* de265ctx,
                                                       de265_image_allocation* allocfunc)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  ctx->set_image_allocation_functions(allocfunc);
}

LIBDE265_API const struct de265_image_allocation *de265_get_default_image_allocation_functions(void)
{
  return &image::default_image_allocation;
}

LIBDE265_API de265_PTS de265_get_image_PTS(const struct de265_image* img)
{
  return img->m_image->pts;
}

LIBDE265_API void* de265_get_image_user_data(const struct de265_image* img)
{
  return img->m_image->user_data;
}

LIBDE265_API void de265_set_image_user_data_intern(struct de265_image_intern* img, void *user_data)
{
  image* iimg = (image*)img;
  iimg->user_data = user_data;
}

LIBDE265_API void* de265_get_image_user_data_intern(const struct de265_image_intern* img)
{
  const image* iimg = (const image*)img;
  return iimg->user_data;
}

  /*
LIBDE265_API void de265_get_image_NAL_header(const struct de265_image* img_de265,
                                             int* nal_unit_type,
                                             const char** nal_unit_name,
                                             int* nuh_layer_id,
                                             int* nuh_temporal_id)
{
  const image_ptr& img = img_de265->m_image;

  if (nal_unit_type)   *nal_unit_type   = img->nal_hdr.nal_unit_type;
  if (nal_unit_name)   *nal_unit_name   = get_NAL_name(img->nal_hdr.nal_unit_type);
  if (nuh_layer_id)    *nuh_layer_id    = img->nal_hdr.nuh_layer_id;
  if (nuh_temporal_id) *nuh_temporal_id = img->nal_hdr.nuh_temporal_id;
}
  */

}
