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

#ifndef ENCODER_CONTEXT_H
#define ENCODER_CONTEXT_H

#include "libde265/image.h"
#include "libde265/decctx.h"
#include "libde265/image-io.h"
#include "libde265/alloc_pool.h"
#include "libde265/encoder-params.h"
#include "libde265/encpicbuf.h"
#include "libde265/sop.h"
#include "libde265/en265.h"

#include <memory>


struct encoder_context
{
  encoder_context();
  ~encoder_context();

  encoder_params params;
  //config_parameters params_config;
  config_parameters_NEW params_config_NEW;

  EncodingAlgorithm_Custom algo;

  int image_width, image_height;
  bool image_spec_is_defined;  // whether we know the input image size

  error_queue errqueue;
  acceleration_functions accel;

  de265_image* img; // reconstruction

  int pic_qp; // TODO: this should be removed again, eventually

  video_parameter_set  vps;
  seq_parameter_set    sps;
  pic_parameter_set    pps;
  slice_segment_header shdr;

  bool parameters_have_been_set;
  bool headers_have_been_sent;

  encoder_picture_buffer picbuf;
  std::shared_ptr<sop_creator> sop;

  std::deque<en265_packet*> output_packets;


  // --- poor man's garbage collector for CB/TB/PB/coeff data ---

  alloc_pool<enc_cb>  enc_cb_pool;
  alloc_pool<enc_tb>  enc_tb_pool;
  alloc_pool<int16_t> enc_coeff_pool;

  void free_all_pools() {
    enc_cb_pool.free_all();
    enc_tb_pool.free_all();
    enc_coeff_pool.free_all();
  }



  // --- rate-control ---

  float lambda;


  // --- CABAC output and rate estimation ---

  CABAC_encoder*  cabac;      // currently active CABAC output (estim or bitstream)
  context_model*  ctx_model;  // currently active ctx models (estim or bitstream)

  // CABAC bitstream writer
  CABAC_encoder_bitstream cabac_bitstream;
  context_model_table     ctx_model_bitstream;


  void switch_CABAC(context_model_table model, CABAC_encoder* cabac_impl) {
    cabac      = cabac_impl;
    ctx_model  = model;
  }

  void switch_CABAC_to_bitstream() {
    cabac     = &cabac_bitstream;
    ctx_model = ctx_model_bitstream;
  }

  en265_packet* create_packet(en265_packet_content_type t);


  // --- encoding control ---

  de265_error encode_headers();
  de265_error encode_picture_from_input_buffer();


  // Input images can be released after encoding and when the output packet is released.
  // This is important to do as soon as possible, as the image might actually wrap
  // scarce resources like camera picture buffers.
  // This function does release (only) the raw input data.
  void release_input_image(int frame_number) { picbuf.release_input_image(frame_number); }
};


#endif
