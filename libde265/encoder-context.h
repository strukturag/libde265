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


struct encoder_context
{
  encoder_context() {
    img_source = NULL;
    reconstruction_sink = NULL;
    packet_sink = NULL;

    enc_coeff_pool.set_blk_size(64*64*20); // TODO: this a guess

    switch_CABAC_to_bitstream();
  }


  encoder_params params;

  ImageSource*   img_source;
  ImageSink*     reconstruction_sink;
  PacketSink*    packet_sink;

  error_queue errqueue;
  acceleration_functions accel;

  de265_image img;

  video_parameter_set  vps;
  seq_parameter_set    sps;
  pic_parameter_set    pps;
  slice_segment_header shdr;



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

  void write_packet() {
    if (packet_sink) {
      packet_sink->send_packet( cabac_bitstream.data(), cabac_bitstream.size() );
      cabac->reset();
    }
  }
};


#endif
