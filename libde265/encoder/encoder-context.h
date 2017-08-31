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
#include "libde265/encoder/encoder-params.h"
#include "libde265/encoder/encoder-core.h"
#include "libde265/encoder/encpicbuf.h"
#include "libde265/encoder/sop.h"
#include "libde265/en265.h"
#include "libde265/util.h"

#include <memory>


class encoder_context : public base_context
{
 public:
  encoder_context();
  ~encoder_context();

  void set_encoder_core(std::shared_ptr<EncoderCore> core) { algocore=core; }


  // --- image_history (decoded images) ---

  virtual std::shared_ptr<const image> get_image(int frame_id) const {
    return picbuf.get_picture(frame_id)->reconstruction;
  }

  virtual bool has_image(int frame_id) const {
    return picbuf.has_picture(frame_id);
  }


  const image_history& get_input_image_history() const { return m_input_image_history; }



  bool encoder_started;

  config_parameters params_config;

  std::shared_ptr<EncoderCore> algocore;

  int image_width, image_height;
  bool image_spec_is_defined;  // whether we know the input image size

  de265_image_allocation image_allocation_functions;

  /*
  void* param_image_allocation_userdata; // TODO: clean up allocation API
  void (*release_func)(en265_encoder_context*,
                       image*,
                       void* userdata);
  */

  // quick links
  image_ptr img; // reconstruction
  std::shared_ptr<picture_encoding_data> imgdata; // input image
  slice_segment_header* shdr;

  CTBTreeMatrix ctbs;


  int active_qp; // currently active QP
  /*int target_qp;*/ /* QP we want to code at.
    (Not actually the real QP. Check image.get_QPY() for that.) */

  const video_parameter_set& get_vps() const { return *vps; }
  const seq_parameter_set& get_sps() const { return *sps; }
  const pic_parameter_set& get_pps() const { return *pps; }
  std::shared_ptr<const pic_parameter_set> get_pps_ptr() const { return pps; }

  //video_parameter_set& get_vps() { return *vps; }
  seq_parameter_set& get_sps() { return *sps; }
  //pic_parameter_set& get_pps() { return *pps; }

  std::shared_ptr<video_parameter_set>& get_shared_vps() { return vps; }
  std::shared_ptr<seq_parameter_set>& get_shared_sps() { return sps; }
  std::shared_ptr<pic_parameter_set>& get_shared_pps() { return pps; }

 private:
  std::shared_ptr<video_parameter_set>  vps;
  std::shared_ptr<seq_parameter_set>    sps;
  std::shared_ptr<pic_parameter_set>    pps;
  //slice_segment_header shdr;

 public:
  bool parameters_have_been_set;
  bool headers_have_been_sent;

  encoder_picture_buffer picbuf;


  // --- image history that accesses the input images ---

  class image_history_input : public image_history {
  public:
    image_history_input(const encoder_context* ectx) { m_ectx=ectx; }

    virtual std::shared_ptr<const image> get_image(int frame_id) const {
      return m_ectx->picbuf.get_picture(frame_id)->input;
    }

    virtual bool has_image(int frame_id) const {
      return m_ectx->picbuf.has_picture(frame_id);
    }

  private:
    const encoder_context* m_ectx;
  } m_input_image_history;



  std::shared_ptr<sop_creator> sop;

  std::deque<en265_packet*> output_packets;



  // --- rate-control ---

  float lambda;


  // --- CABAC output and rate estimation ---

  //CABAC_encoder*  cabac;      // currently active CABAC output (estim or bitstream)
  //context_model_table2* ctx_model;  // currently active ctx models (estim or bitstream)

  // CABAC bitstream writer
  CABAC_encoder_bitstream cabac_encoder;
  context_model_table     cabac_ctx_models;

  //std::shared_ptr<CABAC_encoder> cabac_estim;

  bool use_adaptive_context;


  /*** TODO: CABAC_encoder direkt an encode-Funktion übergeben, anstatt hier
       aussenrum zwischenzuspeichern (mit undefinierter Lifetime).
       Das Context-Model kann dann gleich mit in den Encoder rein cabac_encoder(ctxtable).
       write_bits() wird dann mit dem context-index aufgerufen, nicht mit dem model direkt.
  ***/


  /*
    void switch_CABAC(context_model_table2* model) {
    cabac      = cabac_estim.get();
    ctx_model  = model;
    }

    void switch_CABAC_to_bitstream() {
    cabac     = &cabac_bitstream;
    ctx_model = &ctx_model_bitstream;
    }
  */

  en265_packet* create_packet(en265_packet_content_type t);


  // --- encoding control ---

  void start_encoder();
  de265_error encode_headers();
  de265_error encode_picture_from_input_buffer();
};


// ================================================================================

class encoder_context_scc
{
 public:
  encoder_context_scc();


  void push_image(image_ptr);
  void push_end_of_video() { }

  std::shared_ptr<video_parameter_set> get_shared_vps() { return vps; }
  std::shared_ptr<seq_parameter_set> get_shared_sps() { return sps; }
  std::shared_ptr<pic_parameter_set> get_shared_pps() { return pps; }

  // Get encoder packet and remove from queue.
  // Returns NULL if there is no packet available.
  en265_packet* get_next_packet();

 private:
  // the currently active parameter sets
  std::shared_ptr<video_parameter_set>  vps;
  std::shared_ptr<seq_parameter_set>    sps;
  std::shared_ptr<pic_parameter_set>    pps;

  enum {
    Uninitialized,
    Encoding,
    Finished
  } state = Uninitialized;

  void set_image_parameters(image_ptr);
  void send_headers();



  CTBTreeMatrix ctbs;


  // --- CABAC output and rate estimation ---

  CABAC_encoder_ref       cabac_encoder;
  context_model_table     cabac_ctx_models;

  en265_packet* copy_encoded_data_into_packet(en265_packet_content_type);


  // --- the encoded packets ---

  std::deque<en265_packet*> output_packets;
};


#endif
