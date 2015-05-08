#include "decctx-multilayer.h"

decoder_context_multilayer::decoder_context_multilayer()
{
  // Init by creating one decoder context (there has to be at least one layer in the bitstream)
  num_layer_decoders = 1;
  layer_decoders[0] = new decoder_context;
  //layer_decoders[0]->set_layer_id(0);
  //layer_decoders[0]->set_decoder_ctx_array(layer_decoders);
  //layer_decoders[0]->set_multilayer_decode_parameters( &ml_dec_params );
  for (int i=1; i<MAX_LAYER_ID; i++)
    layer_decoders[i] = NULL;
}