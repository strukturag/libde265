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

class decoder_context_multilayer {
public:
  decoder_context_multilayer();
  ~decoder_context_multilayer();

  de265_error get_warning();
  void reset();
  de265_error decode(int* more);

  // Get the layer decoder context with the given layer_id.
  // Create it if does not exist yet.
  decoder_context* get_layer_dec(int layer_id);

  // Get the number of picture in the output queue (sum over all layers)
  int num_pictures_in_output_queue();
  // Get next output picture. Lowest layers will be returned first. Return the layer that the image is from.
  de265_image* get_next_picture_in_output_queue(int* layerID);
  // Pop next output picture. Lowest layers will be poped first.
  void pop_next_picture_in_output_queue();

  // Flush data
  void flush_data();

protected:
  decoder_context* layer_decoders[MAX_LAYER_ID];
  int num_layer_decoders;

  NAL_Parser nal_parser;

  multilayer_decoder_parameters ml_dec_params;
};


#endif
