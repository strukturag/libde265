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
 *
 * Author of this file: Christian Feldmann <christian.feldmann@gmx.de>
 *
 */

#include "decctx-multilayer.h"
#include "nal.h"

decoder_context_multilayer::decoder_context_multilayer()
{
  num_worker_threads = 0;
  current_HighestTid = 6;
  limit_HighestTid = 6;   // decode all temporal layers (up to layer 6)
  framerate_ratio = 100;  // decode all 100%
  param_acceleration = de265_acceleration_AUTO;
  currrent_layer = 0;

  // Multilayer decoder parameters
  ml_dec_params.highestTID = 6;
  ml_dec_params.TargetOlsIdx = -1;
  ml_dec_params.TargetLayerId = 1; // Default: Highest layer possible

  // Default parameters
  param_sei_check_hash = false;
  param_suppress_faulty_pictures = false;
  param_disable_deblocking = false;
  param_disable_sao = false;
  param_sps_headers_fd = -1;
  param_vps_headers_fd = -1;
  param_pps_headers_fd = -1;
  param_slice_headers_fd = -1;

  // Init by creating one decoder context (there has to be at least one layer in the bitstream)
  num_layer_decoders = 0;
  for (int i = 0; i<MAX_LAYER_ID; i++) {
    layer_decoders[i] = NULL;
  }
  get_layer_dec(0);
}

decoder_context_multilayer::~decoder_context_multilayer()
{
}

de265_error decoder_context_multilayer::get_warning()
{
  // Do we have a warning/error?
  de265_error err = err_queue.get_warning();
  if (err != DE265_OK)
    return err;

  // Loop through all layer decoders and return the first error that is not DE265_OK
  for (int i = 0; i < num_layer_decoders; i++)
  {
    de265_error err = layer_decoders[i]->get_warning();
    if (err != DE265_OK)
      return err;
  }

  return DE265_OK;
}

void decoder_context_multilayer::reset()
{
  // Reset all layer decoders
  // Todo: Not shure if this is the correct way to handle this
  for (int i = 0; i < num_layer_decoders; i++)
  {
    layer_decoders[i]->reset();
  }
}

int decoder_context_multilayer::num_pictures_in_output_queue()
{
  int nrPics = 0;
  for (int i = 0; i < num_layer_decoders; i++)
  {
    nrPics += layer_decoders[i]->num_pictures_in_output_queue();
  }
  return nrPics;
}

de265_image* decoder_context_multilayer::get_next_picture_in_output_queue()
{
  de265_image* img = NULL;
  for (int i = 0; i < num_layer_decoders; i++)
  {
    if (layer_decoders[i]->num_pictures_in_output_queue() > 0) {
      img = layer_decoders[i]->get_next_picture_in_output_queue();
      if (img != NULL) {
        return img;
      }
    }
  }
  return NULL;
}

void decoder_context_multilayer::pop_next_picture_in_output_queue()
{
  de265_image* img = NULL;
  for (int i = 0; i < num_layer_decoders; i++) {
    if (layer_decoders[i]->num_pictures_in_output_queue() > 0) {
      img = layer_decoders[i]->get_next_picture_in_output_queue();
      if (img != NULL) {
        layer_decoders[i]->pop_next_picture_in_output_queue();
        return;
      }
    }
  }
}

decoder_context* decoder_context_multilayer::get_layer_dec(int layer_id)
{
  assert( layer_id < MAX_LAYER_ID );
  if (layer_decoders[layer_id] == NULL) {
    // The decoder for the layer nuh_layer_id does not exits yet.
    // Create it
    if (layer_id != num_layer_decoders) {
      // NAL unit layer ids should be continuous in the bitstream.
      // TODO Handle the error
    }
    layer_decoders[layer_id] = new decoder_context;
    layer_decoders[layer_id]->set_layer_id(layer_id);
    layer_decoders[layer_id]->set_multi_layer_decoder(this);
    layer_decoders[layer_id]->nal_parser = &nal_parser;

    // Only set the following values if they have been changed (set)
    if (num_worker_threads > 0) {
      // Start thread pool if it has been started
      layer_decoders[layer_id]->start_thread_pool(num_worker_threads);
    }
    if (limit_HighestTid != 6) {
      layer_decoders[layer_id]->set_limit_TID(limit_HighestTid);
    }
    if (framerate_ratio != 100) {
      layer_decoders[layer_id]->set_framerate_ratio(framerate_ratio);
    }

    // Set parameters
    layer_decoders[layer_id]->param_sei_check_hash = param_sei_check_hash;
    layer_decoders[layer_id]->param_suppress_faulty_pictures = param_suppress_faulty_pictures;
    layer_decoders[layer_id]->param_disable_deblocking = param_disable_deblocking;
    layer_decoders[layer_id]->param_disable_sao = param_disable_sao;
    layer_decoders[layer_id]->param_sps_headers_fd = param_sps_headers_fd;
    layer_decoders[layer_id]->param_vps_headers_fd = param_vps_headers_fd;
    layer_decoders[layer_id]->param_pps_headers_fd = param_pps_headers_fd;
    layer_decoders[layer_id]->param_slice_headers_fd = param_slice_headers_fd;
    layer_decoders[layer_id]->set_acceleration_functions(param_acceleration);

    num_layer_decoders++;
  }
  return layer_decoders[layer_id];
}

de265_error decoder_context_multilayer::decode(int* more)
{
  if (nal_parser.get_NAL_queue_length() == 0 &&
     (nal_parser.is_end_of_stream() || nal_parser.is_end_of_frame())) {
    // The stream has ended.
    // The layer decoders might have pending pictures to flush
    int more_temp = 0;
    for (int i = 0; i < num_layer_decoders; i++) {
      layer_decoders[i]->decode(&more_temp);
      if (more_temp != 0) {
        // This has to be called again
        if (more) { *more = 1; }
        return DE265_OK;
      }
    }

    // All decoders returned that they don't need to be called again.
    if (more) { *more = 0; }
    return DE265_OK;
  }

  if (nal_parser.is_end_of_stream() == false &&
      nal_parser.is_end_of_frame() == false &&
      nal_parser.get_NAL_queue_length() == 0) {
    if (more) { *more=1; }

    // The end of the stream has not been set yet but there is no data in the input nal queue.
    return DE265_ERROR_WAITING_FOR_INPUT_DATA;
  }

  if (nal_parser.get_NAL_queue_length()) {
    // Peek the next NAL unit and call the correct decoder function
    NAL_unit* nal = nal_parser.peek_NAL_queue();

    // Parse the header
    bitreader reader;
    bitreader_init(&reader, nal->data(), nal->size());
    nal_header nal_hdr;
    nal_hdr.read(&reader);

    if (nal_hdr.nuh_layer_id > ml_dec_params.TargetLayerId) {
      // Discard all NAL units with nuh_layer_id > (nrLayersToDecode-1)
      nal = nal_parser.pop_from_NAL_queue();
      nal_parser.free_NAL_unit(nal);
      if (more) *more = true;
    }
    else {
      if (nal_hdr.nuh_layer_id != currrent_layer) {
        // We are now decoding another layer. Finish decoding of the last layer first.
        decoder_context *lower_layer_dec = get_layer_dec(currrent_layer);
        bool did_work = true;
        while (did_work) {
          lower_layer_dec->decode_some(&did_work, true);
        }
        currrent_layer = nal_hdr.nuh_layer_id;
      }
      
      decoder_context* layerCtx = get_layer_dec(nal_hdr.nuh_layer_id);

      // Call the decode function for this layer
      if (more) *more = 0;
      de265_error layer_error;
      layer_error = layerCtx->decode(more);

      if (nal_hdr.nal_unit_type == NAL_UNIT_VPS_NUT) {
        // This was a VPS NAL unit. Calculate the OLS (if this VPS has an extension)
        calculate_target_output_layer_set(layerCtx->get_last_parsed_vps());
      }

      return layer_error;
    }
  }

  return DE265_OK;
}

void decoder_context_multilayer::flush_data()
{
  // Flush data and mark as end of stream
  nal_parser.flush_data();
  nal_parser.mark_end_of_stream();
}

de265_error decoder_context_multilayer::start_thread_pool(int nThreads)
{
  num_worker_threads = nThreads;
  de265_error ret_err = DE265_OK;
  de265_error err_tmp = DE265_OK;

  // Start threads for each decoder_context
  // Return the error if one of the call causes an error
  for (int i = 0; i < num_layer_decoders; i++) {
    err_tmp = layer_decoders[i]->start_thread_pool(nThreads);
    if (err_tmp != DE265_OK) {
      ret_err = err_tmp;
    }
  }

  return ret_err;
}

void decoder_context_multilayer::stop_thread_pool()
{
  if (num_worker_threads > 0) {
    // Stop all decoder thread pools
    for (int i = 0; i < num_layer_decoders; i++) {
      layer_decoders[i]->stop_thread_pool();
    }
  }
}

int decoder_context_multilayer::get_highest_TID() const
{
  // Return the highest TID from the base layer
  return layer_decoders[0]->get_highest_TID();
}

void decoder_context_multilayer::set_limit_TID(int max_tid)
{
  limit_HighestTid = max_tid;

  // Set limit for all decoders
  for (int i = 0; i < num_layer_decoders; i++) {
    layer_decoders[i]->set_limit_TID(max_tid);
  }
}

int decoder_context_multilayer::change_framerate(int more)
{
  int ret_val = 0;

  // Change frame rate for all decoders
  for (int i = 0; i < num_layer_decoders; i++) {
    ret_val = layer_decoders[i]->change_framerate(more);
  }

  return ret_val;
}

void decoder_context_multilayer::set_framerate_ratio(int percent)
{
  framerate_ratio = percent;
  // Change frame rate for all decoders
  for (int i = 0; i < num_layer_decoders; i++) {
    layer_decoders[i]->set_framerate_ratio(percent);
  }
}

void decoder_context_multilayer::update_parameters()
{
  for (int i = 0; i < num_layer_decoders; i++) {
    layer_decoders[i]->param_sei_check_hash = param_sei_check_hash;
    layer_decoders[i]->param_suppress_faulty_pictures = param_suppress_faulty_pictures;
    layer_decoders[i]->param_disable_deblocking = param_disable_deblocking;
    layer_decoders[i]->param_disable_sao = param_disable_sao;
    layer_decoders[i]->param_sps_headers_fd = param_sps_headers_fd;
    layer_decoders[i]->param_vps_headers_fd = param_vps_headers_fd;
    layer_decoders[i]->param_pps_headers_fd = param_pps_headers_fd;
    layer_decoders[i]->param_slice_headers_fd = param_slice_headers_fd;
  }
}

void decoder_context_multilayer::set_acceleration_functions(enum de265_acceleration l)
{
  param_acceleration = l;
  // Set acceleration for all decoders
  for (int i = 0; i < num_layer_decoders; i++) {
    layer_decoders[i]->set_acceleration_functions(l);
  }
}

void decoder_context_multilayer::calculate_target_output_layer_set(video_parameter_set *vps)
{
  if (vps==NULL || !vps->vps_extension_flag) {
    // No VPS extension. No multilayer.
    return;
  }
  if (get_target_Layer_ID() == 0) {
    // The vps_extension_flag is true but we are only decoding the base layer.
    // Ignore the existing VPS extension
    return;
  }
  video_parameter_set_extension *vps_ext = &vps->vps_extension;

  // Reading the VPS extension is done.
  // Check (calculate if not set) the output layer set index
  if (!ml_dec_params.values_checked) {
    if (ml_dec_params.TargetOlsIdx == -1) {
      // Target output layer set ID not set yet
      if (ml_dec_params.TargetLayerId > vps->vps_max_layer_id) {
        // Target layer ID too high
        ml_dec_params.TargetLayerId = vps->vps_max_layer_id;
      }

      bool layerSetMatchFound = false;
      // Output layer set index not assigned.
      // Based on the value of targetLayerId, check if any of the output layer matches
      // Currently, the target layer ID in the encoder assumes that all the layers are decoded
      // Check if any of the output layer sets match this description
      for(int i = 0; i < vps_ext->NumOutputLayerSets; i++)
      {
        bool layerSetMatchFlag = false;
        int layerSetIdx = vps_ext->layer_set_idx_for_ols_minus1[i] + 1;

        for(int j = 0; j < vps_ext->NumLayersInIdList[layerSetIdx]; j++)
        {
          if( vps_ext->LayerSetLayerIdList[layerSetIdx][j] == ml_dec_params.TargetLayerId )
          {
            layerSetMatchFlag = true;
            break;
          }
        }

        if( layerSetMatchFlag ) // Potential output layer set candidate found
        {
          // If target dec layer ID list is also included - check if they match
          if( !ml_dec_params.TargetDecLayerSetIdx.empty() )
          {
            for(int j = 0; j < vps_ext->NumLayersInIdList[layerSetIdx]; j++)
            {
              if (ml_dec_params.TargetDecLayerSetIdx[j] != vps_ext->layer_id_in_nuh[vps_ext->LayerSetLayerIdList[layerSetIdx][j]])
              {
                layerSetMatchFlag = false;
              }
            }
          }
          if( layerSetMatchFlag ) // The target dec layer ID list also matches, if present
          {
            // Match found
            layerSetMatchFound = true;
            ml_dec_params.TargetOlsIdx = i;
            ml_dec_params.values_checked = true;
            break;
          }
        }
      }
      if (!layerSetMatchFound) {
        // No output layer set matched the value of either targetLayerId or targetdeclayerIdlist
        set_extensions_decoding_error();
        return;
      }
    }
  }
  else {
    if (ml_dec_params.TargetOlsIdx >= vps_ext->NumOutputLayerSets) {
      set_extensions_decoding_error();
      return;
    }
    int layerSetIdx = vps_ext->layer_set_idx_for_ols_minus1[ ml_dec_params.TargetOlsIdx ] + 1;  // Index to the layer set
    // Check if the targetdeclayerIdlist matches the output layer set
    if( !ml_dec_params.TargetDecLayerSetIdx.empty() ) {
      for(int i = 0; i < vps_ext->NumLayersInIdList[layerSetIdx]; i++)
      {
        if (ml_dec_params.TargetDecLayerSetIdx[i] != vps_ext->layer_id_in_nuh[vps_ext->LayerSetLayerIdList[layerSetIdx][i]]) {
          set_extensions_decoding_error();
          return;
        }
      }
    }
    ml_dec_params.values_checked = true;
  }
}

void decoder_context_multilayer::set_extensions_decoding_error()
{
  // Error in the extension. Switch extensions off. Only decode the base layer.
  err_queue.add_warning(DE265_WARNING_MULTILAYER_ERROR_SWITCH_TO_BASE_LAYER, false);
  ml_dec_params.TargetLayerId = 0;
}
