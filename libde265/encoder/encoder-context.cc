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

#include "encoder/encoder-context.h"
#include "encoder/analyze.h"

#include <math.h>


encoder_context::encoder_context()
{
  //img_source = NULL;
  //reconstruction_sink = NULL;
  //packet_sink = NULL;

  image_spec_is_defined = false;
  parameters_have_been_set = false;
  headers_have_been_sent = false;

  param_image_allocation_userdata = NULL;
  release_func = NULL;

  enc_coeff_pool.set_blk_size(64*64*20); // TODO: this a guess

  switch_CABAC_to_bitstream();

  //sop = std::make_shared<sop_creator_trivial_low_delay>();
  sop = std::make_shared<sop_creator_intra_only>();
  sop->set_encoder_picture_buffer(&picbuf);


  params.registerParams(params_config);
  algo.registerParams(params_config);


  // --- initialize encoder ---

  init_acceleration_functions_fallback(&accel);
}


encoder_context::~encoder_context()
{
  while (!output_packets.empty()) {
    en265_free_packet(this, output_packets.front());
    output_packets.pop_front();
  }
}


en265_packet* encoder_context::create_packet(en265_packet_content_type t)
{
  en265_packet* pck = new en265_packet;

  uint8_t* data = new uint8_t[cabac_bitstream.size()];
  memcpy(data, cabac_bitstream.data(), cabac_bitstream.size());

  pck->version = 1;

  pck->data = data;
  pck->length = cabac_bitstream.size();

  pck->frame_number = -1;
  pck->content_type = t;
  pck->complete_picture = 0;
  pck->final_slice = 0;
  pck->dependent_slice = 0;
  //pck->pts = 0;
  //pck->user_data = NULL;

  pck->encoder_context = this;

  pck->input_image = NULL;
  pck->reconstruction = NULL;

  cabac->reset();

  return pck;
}


de265_error encoder_context::encode_headers()
{
  nal_header nal;

  // VPS

  vps.set_defaults(Profile_Main, 6,2);


  // SPS

  sps.set_defaults();
  sps.set_CB_log2size_range( Log2(params.min_cb_size), Log2(params.max_cb_size));
  sps.set_TB_log2size_range( Log2(params.min_tb_size), Log2(params.max_tb_size));
  sps.max_transform_hierarchy_depth_intra = params.max_transform_hierarchy_depth_intra;

  sps.set_resolution(image_width, image_height);
  sps.compute_derived_values();

  // PPS

  pps.set_defaults();
  pps.pic_init_qp = pic_qp;

  // turn off deblocking filter
  pps.deblocking_filter_control_present_flag = true;
  pps.deblocking_filter_override_enabled_flag = false;
  pps.pic_disable_deblocking_filter_flag = true;
  pps.pps_loop_filter_across_slices_enabled_flag = false;

  pps.set_derived_values(&sps);



  // write headers

  en265_packet* pck;

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(cabac);
  vps.write(&errqueue, cabac);
  cabac->flush_VLC();
  pck = create_packet(EN265_PACKET_VPS);
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(cabac);
  sps.write(&errqueue, cabac);
  cabac->flush_VLC();
  pck = create_packet(EN265_PACKET_SPS);
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(cabac);
  pps.write(&errqueue, cabac, &sps);
  cabac->flush_VLC();
  pck = create_packet(EN265_PACKET_PPS);
  output_packets.push_back(pck);



  headers_have_been_sent = true;

  return DE265_OK;
}


de265_error encoder_context::encode_picture_from_input_buffer()
{
  if (!picbuf.have_more_frames_to_encode()) {
    return DE265_OK;
  }


  if (!image_spec_is_defined) {
    const encoder_picture_buffer::image_data* id = picbuf.peek_next_picture_to_encode();
    image_width  = id->input->get_width();
    image_height = id->input->get_height();
    image_spec_is_defined = true;
  }


  if (!parameters_have_been_set) {
    algo.setParams(params);

    // TODO: must be <30, because Y->C mapping (tab8_22) is not implemented yet
    int qp = algo.getPPS_QP();
    pic_qp = qp;

    //lambda = ectx->params.lambda;
    lambda = 0.0242 * pow(1.27245, qp);

    parameters_have_been_set = true;
  }


  if (!headers_have_been_sent) {
    encode_headers();
  }





  encoder_picture_buffer::image_data* imgdata;
  imgdata = picbuf.get_next_picture_to_encode();
  assert(imgdata);
  picbuf.mark_encoding_started(imgdata->frame_number);

  this->imgdata = imgdata;
  this->shdr    = &imgdata->shdr;
  fprintf(stderr,"encoding frame %d\n",imgdata->frame_number);


  // write slice header

  // slice

  imgdata->shdr.slice_deblocking_filter_disabled_flag = true;
  imgdata->shdr.slice_loop_filter_across_slices_enabled_flag = false;
  imgdata->shdr.compute_derived_values(&pps);

  //shdr.slice_pic_order_cnt_lsb = poc & 0xFF;

  nal_header nal;
  nal.set(imgdata->nal_type);
  nal.write(cabac);
  imgdata->shdr.write(&errqueue, cabac, &sps, &pps, nal.nal_unit_type);
  cabac->skip_bits(1);
  cabac->flush_VLC();


  // encode image

  cabac->init_CABAC();
  double psnr = encode_image(this,imgdata->input, algo);
  fprintf(stderr,"  PSNR-Y: %f\n", psnr);
  cabac->flush_CABAC();

  // set reconstruction image

  picbuf.set_reconstruction_image(imgdata->frame_number, img);
  img=NULL;
  this->imgdata = NULL;
  this->shdr = NULL;

  // build output packet

  en265_packet* pck = create_packet(EN265_PACKET_SLICE);
  pck->input_image    = imgdata->input;
  pck->reconstruction = imgdata->reconstruction;
  pck->frame_number   = imgdata->frame_number;
  output_packets.push_back(pck);


  picbuf.mark_encoding_finished(imgdata->frame_number);

  return DE265_OK;
}
