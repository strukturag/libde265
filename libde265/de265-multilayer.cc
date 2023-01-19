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


struct de265_vps_scanner
{
  error_queue m_error_queue;
  NAL_Parser m_nal_parser;
};

struct de265_vps : public video_parameter_set
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

void de265_vps_release(de265_vps* vps)
{
  delete vps;
}

int de265_vps_get_id(de265_vps* vps)
{
  return vps->video_parameter_set_id;
}

int de265_vps_get_max_layers(de265_vps* vps)
{
  return vps->vps_max_layers;
}

uint8_t de265_vps_get_layer_aux_id(de265_vps* vps, int layer)
{
  if (layer < 0 || layer >= vps->vps_max_layers) {
    return de265_aux_none;
  }

  return vps->AuxId[layer];
}
