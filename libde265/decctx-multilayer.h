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

protected:
  decoder_context* layer_decoders[MAX_LAYER_ID];
  int num_layer_decoders;

  multilayer_decoder_parameters ml_dec_params;
};


#endif
