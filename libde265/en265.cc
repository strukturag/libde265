/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: Dirk Farin <farin@struktur.de>
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

#include "libde265/en265.h"
#include "libde265/encoder/encoder-context.h"


LIBDE265_API en265_encoder_context* en265_new_encoder(void)
{
  de265_error init_err = de265_init();
  if (init_err != DE265_OK) {
    return NULL;
  }

  encoder_context* ectx = new encoder_context();
  return (en265_encoder_context*)ectx;
}


LIBDE265_API de265_error en265_start_worker_threads(en265_encoder_context*, int number_of_threads)
{
  return DE265_OK;
}


LIBDE265_API de265_error en265_free_encoder(en265_encoder_context* e)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;
  delete ectx;

  return de265_free();
}


LIBDE265_API void en265_set_image_release_function(en265_encoder_context* e,
                                                   void (*release_func)(en265_encoder_context*,
                                                                        de265_image*,
                                                                        void* userdata),
                                                   void* alloc_userdata)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  ectx->param_image_allocation_userdata = alloc_userdata;
  ectx->release_func = release_func;
}


// ========== encoder parameters ==========

LIBDE265_API de265_error en265_parse_command_line_parameters(en265_encoder_context* e,
                                                             int* argc, char** argv)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  //if (!ectx->params_config.parse_command_line_params(argc,argv, &ectx->params, true)) {
  int first_idx=1;
  if (!ectx->params_config.parse_command_line_params(argc,argv, &first_idx)) {
    return DE265_ERROR_PARAMETER_PARSING;
  }
  else {
    return DE265_OK;
  }
}

LIBDE265_API void en265_show_params(en265_encoder_context* e)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  //ectx->params_config.show_params(&ectx->params);

  ectx->params_config.print_params();
}


static const char** fill_strings_into_memory(const std::vector<std::string>& strings_list,
                                             void* memory, int memsize)
{
  // calculate memory requirement

  int totalStringLengths = 0;
  for (auto str : strings_list) {
    totalStringLengths += str.length() +1; // +1 for null termination
  }

  int numStrings = strings_list.size();

  int pointersSize = (numStrings+1) * sizeof(const char*);

  if (pointersSize + totalStringLengths > memsize) { return NULL; }


  // copy strings to memory area

  char* stringPtr = ((char*)memory) + (numStrings+1) * sizeof(const char*);
  const char** tablePtr = (const char**)memory;

  for (auto str : strings_list) {
    *tablePtr++ = stringPtr;

    strcpy(stringPtr, str.c_str());
    stringPtr += str.length()+1;
  }

  *tablePtr = NULL;

  return (const char**)memory;
}


LIBDE265_API const char** en265_list_parameters(en265_encoder_context* e, void* memory, int memsize)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  std::vector<std::string> options = ectx->params_config.get_parameter_IDs();

  return fill_strings_into_memory(options, memory, memsize);
}


LIBDE265_API enum en265_parameter_type en265_get_parameter_type(en265_encoder_context* e,
                                                                const char* parametername)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  return ectx->params_config.get_parameter_type(parametername);
}


LIBDE265_API de265_error en265_set_parameter_bool(en265_encoder_context* e,
                                                  const char* param,int value)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  return ectx->params_config.set_bool(param,value) ? DE265_OK : DE265_ERROR_PARAMETER_PARSING;
}


LIBDE265_API de265_error en265_set_parameter_int(en265_encoder_context* e,
                                                 const char* param,int value)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  return ectx->params_config.set_int(param,value) ? DE265_OK : DE265_ERROR_PARAMETER_PARSING;
}

LIBDE265_API de265_error en265_set_parameter_string(en265_encoder_context* e,
                                                    const char* param,const char* value)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  return ectx->params_config.set_string(param,value) ? DE265_OK : DE265_ERROR_PARAMETER_PARSING;
}

LIBDE265_API de265_error en265_set_parameter_choice(en265_encoder_context* e,
                                                    const char* param,const char* value)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  return ectx->params_config.set_choice(param,value) ? DE265_OK : DE265_ERROR_PARAMETER_PARSING;
}


LIBDE265_API const char** en265_list_parameter_choices(en265_encoder_context* e,
                                                       const char* parametername,
                                                       void* memory, int memsize)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  std::vector<std::string> options = ectx->params_config.get_parameter_choices(parametername);

  return fill_strings_into_memory(options, memory, memsize);
}



// ========== encoding loop ==========



LIBDE265_API struct de265_image* en265_allocate_image(en265_encoder_context* e,
                                                      int width, int height, de265_chroma chroma,
                                                      de265_PTS pts, void* image_userdata)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  de265_image* img = new de265_image;
  img->alloc_image(width,height,de265_chroma_420, NULL, false,
                   NULL,ectx, pts, image_userdata, true);

  return img;
}

// Request a specification of the image memory layout for an image of the specified dimensions.
LIBDE265_API void en265_get_image_spec(en265_encoder_context* e,
                                       int width, int height, de265_chroma chroma,
                                       struct de265_image_spec* out_spec)
{
  out_spec->format = de265_image_format_YUV420P8;
  out_spec->width = width;
  out_spec->height= height;
  out_spec->alignment = 1;

  out_spec->crop_left  =0;
  out_spec->crop_right =0;
  out_spec->crop_top   =0;
  out_spec->crop_bottom=0;

  out_spec->visible_width  = out_spec->width  - out_spec->crop_left - out_spec->crop_right;
  out_spec->visible_height = out_spec->height - out_spec->crop_top  - out_spec->crop_bottom;
}

// Image memory layout specification for an image returned by en265_allocate_image().
//LIBDE265_API void de265_get_image_spec_from_image(de265_image* img, struct de265_image_spec* spec);



LIBDE265_API de265_error en265_push_image(en265_encoder_context* e,
                                          struct de265_image* img)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  ectx->sop->insert_new_input_image(img);
  return DE265_OK;
}


LIBDE265_API de265_error en265_push_eof(en265_encoder_context* e)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  ectx->sop->insert_end_of_stream();
  return DE265_OK;
}


LIBDE265_API de265_error en265_block_on_input_queue_length(en265_encoder_context*,
                                                           int max_pending_images,
                                                           int timeout_ms)
{
  // TODO
  return DE265_OK;
}

LIBDE265_API de265_error en265_trim_input_queue(en265_encoder_context*, int max_pending_images)
{
  // TODO
  return DE265_OK;
}

LIBDE265_API int  en265_current_input_queue_length(en265_encoder_context*)
{
  // TODO
  return DE265_OK;
}

LIBDE265_API de265_error en265_encode(en265_encoder_context* e)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  while (ectx->picbuf.have_more_frames_to_encode())
    {
      de265_error result = ectx->encode_picture_from_input_buffer();
      if (result != DE265_OK) return result;
    }

  return DE265_OK;
}

LIBDE265_API enum en265_encoder_state en265_get_encoder_state(en265_encoder_context* e)
{
  // TODO
  return EN265_STATE_IDLE;
}

LIBDE265_API struct en265_packet* en265_get_packet(en265_encoder_context* e, int timeout_ms)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  assert(timeout_ms==0); // TODO: blocking not implemented yet

  if (ectx->output_packets.size()>0) {
    en265_packet* pck = ectx->output_packets.front();
    ectx->output_packets.pop_front();

    return pck;
  }
  else {
    return NULL;
  }
}

LIBDE265_API void en265_free_packet(en265_encoder_context* e, struct en265_packet* pck)
{
  assert(e);
  encoder_context* ectx = (encoder_context*)e;

  // Do not delete images here. They are owned by the EncPicBuf.
  //delete   pck->input_image;
  //delete   pck->reconstruction;

  if (pck->frame_number >= 0) {
    ectx->release_input_image(pck->frame_number);
  }

  delete[] pck->data;
  delete   pck;
}
