/*
 * H.265 video codec.
 * Copyright (c) 2023 Dirk Farin <dirk.farin@gmail.com>
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

#include <deque>
#include "libde265/de265-multilayer.h"
#include "vps.h"
#include "nal-parser.h"
#include "decctx.h"

#include <map>


struct _de265_vps_scanner
{
  error_queue m_error_queue;
  NAL_Parser m_nal_parser;
};

struct _de265_vps : public video_parameter_set
{
};


de265_vps_scanner* de265_new_vps_scanner()
{
  return new de265_vps_scanner();
}

void de265_vps_scanner_release(de265_vps_scanner* scanner)
{
  delete scanner;
}

de265_error de265_vps_scanner_push_data(de265_vps_scanner* scanner, const void* data, int length)
{
  return scanner->m_nal_parser.push_data((const uint8_t*) data, length, 0, nullptr);
}

de265_error de265_vps_scanner_push_NAL(de265_vps_scanner* scanner, const void* data, int length)
{
  return scanner->m_nal_parser.push_NAL((const uint8_t*) data, length, 0, nullptr);
}

/*
de265_error de265_vps_scanner_flush_nal(de265_vps_scanner* scanner)
{

}
*/

de265_vps* de265_vps_scanner_get_next_vps(de265_vps_scanner* scanner)
{
  for (;;) {
    NAL_unit* nal = scanner->m_nal_parser.pop_from_NAL_queue();
    if (nal == nullptr) {
      return nullptr;
    }

    bitreader reader;
    bitreader_init(&reader, nal->data(), nal->size());

    nal_header nal_hdr;
    nal_hdr.read(&reader);

    if (nal_hdr.nal_unit_type != NAL_UNIT_VPS_NUT) {
      scanner->m_nal_parser.free_NAL_unit(nal);
      continue;
    }

    auto new_vps = new de265_vps();
    de265_error err = new_vps->read(&scanner->m_error_queue, &reader);
    scanner->m_nal_parser.free_NAL_unit(nal);

    if (err != DE265_OK) {
      continue;
    }

    return new_vps;
  }
}


// --- VPS ---

void de265_vps_release(const de265_vps* vps)
{
  delete vps;
}

int de265_vps_get_id(const de265_vps* vps)
{
  return vps->video_parameter_set_id;
}

int de265_vps_get_max_layers(const de265_vps* vps)
{
  return vps->vps_max_layers;
}

uint8_t de265_vps_get_layer_aux_id(const de265_vps* vps, int layer)
{
  if (layer < 0 || layer >= vps->vps_max_layers) {
    return de265_aux_none;
  }

  return vps->AuxId[layer];
}

int de265_vps_get_layer_id_for_aux_id(const de265_vps* vps, uint8_t aux_id)
{
  for (int layer = 0; layer < vps->vps_max_layers; layer++) {
    if (vps->AuxId[layer] == aux_id)
      return layer;
  }

  return -1;
}


// --- access unit ---

struct _de265_access_unit
{
  ~_de265_access_unit();

  struct layer
  {
    const de265_image* image;
    //std::vector<de265_sei*> sei_packets;
  };

  std::map<int, layer> m_layers;
  de265_PTS m_pts;
  void* user_data;

  const de265_vps* m_vps; // the VPS that is active during this access unit
};


_de265_access_unit::~_de265_access_unit()
{
  for (auto& layer : m_layers) {
  }
}


const struct de265_image* de265_access_unit_peek_layer_picture(const de265_access_unit* access_unit, int layer)
{
  auto iter = access_unit->m_layers.find(layer);
  if (iter == access_unit->m_layers.end()) {
    return nullptr;
  }

  return iter->second.image;
}

void de265_access_unit_release(const de265_access_unit* access_unit)
{
  delete access_unit;
}

const de265_vps* de265_access_unit_peek_vps(const de265_access_unit* access_unit)
{
  return access_unit->m_vps;
}


// --- decoder ---

struct _de265_audecoder
{
  ~_de265_audecoder();

  struct layer
  {
    de265_decoder_context* decoder = nullptr;
    uint8_t aux_id = 0;
  };

  enum class State
  {
    Uninitialized,
    Scanning_VPS,
    Decoding,
    Finished
  };

  State m_state = State::Uninitialized;
  de265_vps_scanner* m_vps_scanner = nullptr;
  const de265_vps* m_vps = nullptr; // active VPS (TODO: this is not updated yet)

  struct input_data
  {
    std::vector<uint8_t> data;
    de265_PTS pts = 0;
    void* user_data = nullptr;
    bool is_nal = false;
  };

  std::vector<input_data> m_pending_input;

  bool m_custom_alloc_functions = false;
  struct de265_image_allocation m_image_alloc_functions;
  void* m_image_alloc_userdata = nullptr;

  // TODO: I'm not sure yet how to distribute these over the different decoders.
  // Currently, each decoder receives this number of threads.
  int m_num_worker_threads = 0;

  std::map<int, layer> m_layers;
};

_de265_audecoder::~_de265_audecoder()
{
  for (auto& layer : m_layers) {
    de265_free_decoder(layer.second.decoder);
  }

  de265_vps_release(m_vps);
  de265_vps_scanner_release(m_vps_scanner);
}


de265_audecoder* de265_new_audecoder()
{
  return new de265_audecoder();
}

void de265_audecoder_release(de265_audecoder* decoder)
{
  delete decoder;
}

de265_error de265_audecoder_start_worker_threads(de265_audecoder* decoder, int number_of_threads)
{
  decoder->m_num_worker_threads = number_of_threads;
  return DE265_OK;
}

void de265_audecoder_set_image_allocation_functions(de265_audecoder* decoder,
                                                    struct de265_image_allocation* funcs,
                                                    void* userdata)
{
  // set to all decoders that are already instantiated
  // Note: I'm not sure we have to do this. We should probably set the allocation functions prior to starting the decoder.

  for (auto& layer : decoder->m_layers) {
    de265_set_image_allocation_functions(layer.second.decoder, funcs, userdata);
  }

  decoder->m_custom_alloc_functions = true;
  decoder->m_image_alloc_functions = *funcs;
  decoder->m_image_alloc_userdata = userdata;
}

de265_error de265_audecoder_push_data_common(de265_audecoder* decoder, const void* data, int length,
                                             de265_PTS pts, void* user_data,
                                             bool is_nal_mode)
{
  if (decoder->m_state == de265_audecoder::State::Finished) {
    return de265_error::DE265_ERROR_DATA_PAST_END_OF_STREAM;
  }

  if (decoder->m_state == de265_audecoder::State::Uninitialized) {
    decoder->m_vps_scanner = de265_new_vps_scanner();
    decoder->m_state = de265_audecoder::State::Scanning_VPS;
  }

  assert(decoder->m_state == de265_audecoder::State::Scanning_VPS ||
         decoder->m_state == de265_audecoder::State::Decoding);

  if (decoder->m_state == de265_audecoder::State::Scanning_VPS) {
    if (is_nal_mode) {
      de265_vps_scanner_push_NAL(decoder->m_vps_scanner, data, length);
    }
    else {
      de265_vps_scanner_push_data(decoder->m_vps_scanner, data, length);
    }

    // remember input data so that we can push it into the decoder(s) later

    de265_audecoder::input_data input_data;
    input_data.data.insert(input_data.data.end(),
                           (uint8_t*) data, ((uint8_t*) data) + length);
    input_data.pts = pts;
    input_data.user_data = user_data;
    input_data.is_nal = is_nal_mode;
    decoder->m_pending_input.push_back(input_data);

    // when we decoded the VPS, initialize the layer decoders

    de265_vps* vps = de265_vps_scanner_get_next_vps(decoder->m_vps_scanner);
    if (vps) {
      decoder->m_vps = vps;

      int num_layers = de265_vps_get_max_layers(vps);
      for (int i = 0; i < num_layers; i++) {
        de265_audecoder::layer layer;
        layer.decoder = de265_new_decoder();
        de265_decoder_context_set_layer(layer.decoder, i);

        if (decoder->m_custom_alloc_functions) {
          de265_set_image_allocation_functions(layer.decoder,
                                               &decoder->m_image_alloc_functions,
                                               decoder->m_image_alloc_userdata);
        }

        if (decoder->m_num_worker_threads != 0) {
          de265_error err = de265_start_worker_threads(layer.decoder,
                                                       decoder->m_num_worker_threads);
          if (err != DE265_OK) {
            de265_free_decoder(layer.decoder);
            return err;
          }
        }

        layer.aux_id = de265_vps_get_layer_aux_id(vps, i);
        decoder->m_layers[i] = layer;

        // push pending input data into the decoders

        for (auto& input : decoder->m_pending_input) {
          if (input.is_nal) {
            de265_push_NAL(layer.decoder, input.data.data(), (int) input.data.size(), input.pts, input.user_data);
          }
          else {
            de265_push_data(layer.decoder, input.data.data(), (int) input.data.size(), input.pts, input.user_data);
          }
        }
      }

      de265_vps_scanner_release(decoder->m_vps_scanner);
      decoder->m_vps_scanner = nullptr;
      decoder->m_pending_input.clear();

      decoder->m_state = de265_audecoder::State::Decoding;
    }
  }
  else if (decoder->m_state == de265_audecoder::State::Decoding) {
    // push data into all decoders (each one has an active NAL layer-ID filter)

    for (auto& layer : decoder->m_layers) {
      if (is_nal_mode) {
        de265_push_NAL(layer.second.decoder, data, length, pts, user_data);
      }
      else {
        de265_push_data(layer.second.decoder, data, length, pts, user_data);
      }
    }
  }
  else {
    assert(false);
  }

  return de265_error::DE265_OK;
}


de265_error de265_audecoder_push_data(de265_audecoder* decoder, const void* data, int length,
                                      de265_PTS pts, void* user_data)
{
  return de265_audecoder_push_data_common(decoder, data, length, pts, user_data, false);
}

de265_error de265_audecoder_push_NAL(de265_audecoder* decoder, const void* data, int length,
                                     de265_PTS pts, void* user_data)
{
  de265_set_verbosity(5);

#if 0
  int i;
  for (i = 0; i < length; i++) {
    printf("%02x ", ((uint8_t*) data)[i]);
    if ((i % 16) == 15) {
      printf("\n");
    }
  }
  if ((i % 16) != 0) {
    printf("\n");
  }
#endif

  de265_error e = de265_audecoder_push_data_common(decoder, data, length, pts, user_data, true);

  return e;
}


/* Indicate the end-of-stream. All data pending at the decoder input will be
   pushed into the decoder and the decoded picture queue will be completely emptied.
 */
de265_error de265_audecoder_flush_data(de265_audecoder* decoder)
{
  if (decoder->m_state == de265_audecoder::State::Finished) {
    return de265_error::DE265_OK;
  }
  else if (decoder->m_state == de265_audecoder::State::Uninitialized) {
    decoder->m_state = de265_audecoder::State::Finished;
    return de265_error::DE265_OK;
  }
  else if (decoder->m_state == de265_audecoder::State::Scanning_VPS) {
    // TODO, not relevant except for very small streams (single pictures?)
    decoder->m_state = de265_audecoder::State::Finished;
    return de265_error::DE265_OK;
  }
  else if (decoder->m_state == de265_audecoder::State::Decoding) {
    for (auto& layer : decoder->m_layers) {
      de265_flush_data(layer.second.decoder);
    }

    decoder->m_state = de265_audecoder::State::Finished;
    return de265_error::DE265_OK;
  }
  else {
    assert(false);
    return de265_error::DE265_ERROR_UNSPECIFIED_DECODING_ERROR;
  }
}

const de265_access_unit* de265_audecoder_get_next_picture(de265_audecoder* decoder)
{
  if (decoder->m_layers.empty()) {
    return nullptr;
  }

  for (auto& layer : decoder->m_layers) {
    if (de265_peek_next_picture(layer.second.decoder) == nullptr) {
      return nullptr;
    }
  }

  auto* au = new de265_access_unit;

  const de265_image* img = nullptr;
  for (auto& layer : decoder->m_layers) {
    au->m_layers[layer.first].image = img = de265_get_next_picture(layer.second.decoder);
  }

  assert(img);

  if (img) {
    au->m_pts = img->pts;
    au->user_data = img->user_data;
    au->m_vps = decoder->m_vps;
  }

  return au;
}

de265_error de265_audecoder_decode(de265_audecoder* decoder, int* more)
{
  if (decoder->m_state == de265_audecoder::State::Uninitialized ||
      decoder->m_state == de265_audecoder::State::Scanning_VPS) {
    if (more) *more = 0;
    return DE265_ERROR_WAITING_FOR_INPUT_DATA;
  }
  else if (decoder->m_state == de265_audecoder::State::Finished) {
    if (more) *more = false;
    return DE265_OK;
  }
  else {
    assert(decoder->m_state == de265_audecoder::State::Decoding);

    if (more) *more = 0; // might be overwritten below
    de265_error totalError = DE265_OK;

    for (auto& layer : decoder->m_layers) {
      int decoderMore = 0;
      de265_error err = de265_decode(layer.second.decoder, &decoderMore);
      if (decoderMore && more) *more = 1;

      if (err != DE265_OK) {
        totalError = err;
      }
    }

    return totalError;
  }
}

de265_error de265_audecoder_get_warning(de265_audecoder* decoder)
{
  // collect warnings from all decoders

  for (auto& layer : decoder->m_layers) {
    de265_error err = de265_get_warning(layer.second.decoder);
    if (err != DE265_OK) {
      return err;
    }
  }

  return DE265_OK;
}
