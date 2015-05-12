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

#ifndef DE265_DECCTX_MULTILAYER_H
#define DE265_DECCTX_MULTILAYER_H

#define MAX_LAYER_ID 63  // Maximum number of layer supported by the standard (see vps_max_layers_minus1)
#include "libde265/decctx.h"
#include "libde265/de265.h"

struct multilayer_decoder_parameters {
  multilayer_decoder_parameters() {
    TargetLayerId = 8; 
    TargetOlsIdx = -1; 
    highestTID = 6; 
    values_checked = false;
  }

  int TargetLayerId;
  int TargetOlsIdx; // The target output layer set index
  std::vector<int> TargetDecLayerSetIdx; // 
  int highestTID;

  bool values_checked;
};

class decoder_context_multilayer : public error_queue {
public:
  decoder_context_multilayer();
  ~decoder_context_multilayer();

  de265_error get_warning();
  void reset();
  de265_error decode(int* more);

  de265_error start_thread_pool(int nThreads);
  void        stop_thread_pool();

  // Get the layer decoder context with the given layer_id.
  // Create it if does not exist yet.
  decoder_context* get_layer_dec(int layer_id);

  // Get the number of picture in the output queue (sum over all layers)
  int num_pictures_in_output_queue();
  // Get next output picture. Lowest layers will be returned first.
  de265_image* get_next_picture_in_output_queue();
  // Pop next output picture. Lowest layers will be poped first.
  void pop_next_picture_in_output_queue();

  // Flush data
  void flush_data();

  NAL_Parser nal_parser;

  // --- frame dropping ---
  void set_limit_TID(int tid);
  int  get_highest_TID() const;
  int  get_current_TID() const { return current_HighestTid; }
  int  change_framerate(int more_vs_less); // 1: more, -1: less
  void set_framerate_ratio(int percent);

  // Parameters
  bool param_sei_check_hash;
  bool param_suppress_faulty_pictures;
  bool param_disable_deblocking;
  bool param_disable_sao;

  int param_sps_headers_fd;
  int param_vps_headers_fd;
  int param_pps_headers_fd;
  int param_slice_headers_fd;

  void set_acceleration_functions(enum de265_acceleration);

  // Update the parameters of the layer decoders if one of the parameters was changed.
  void update_parameters();

  // calculate the output layer set if a vps has been parsed
  void calculate_target_output_layer_set(video_parameter_set *vps);

  int get_target_ols_idx()  { return ml_dec_params.TargetOlsIdx;  }
  int get_target_Layer_ID() { return ml_dec_params.TargetLayerId; }
  
protected:
  decoder_context* layer_decoders[MAX_LAYER_ID];
  int num_layer_decoders;

  int num_worker_threads;

  int current_HighestTid;  // the layer which we are currently decoding
  int limit_HighestTid;    // never switch to a layer above this one
  int framerate_ratio;

  enum de265_acceleration param_acceleration;

  multilayer_decoder_parameters ml_dec_params;
};


#endif
