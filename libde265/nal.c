/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "nal.h"

void nal_read_header(bitreader* reader, nal_header* hdr)
{
  skip_bits(reader,1);
  hdr->nal_unit_type = get_bits(reader,6);
  hdr->nuh_layer_id  = get_bits(reader,6);
  hdr->nuh_temporal_id = get_bits(reader,3) -1;
}


bool isIDR(uint8_t unit_type)
{
  return (unit_type == NAL_UNIT_IDR_W_RADL ||
          unit_type == NAL_UNIT_IDR_N_LP);
}

bool isBLA(uint8_t unit_type)
{
  return (unit_type == NAL_UNIT_BLA_W_LP ||
          unit_type == NAL_UNIT_BLA_W_RADL ||
          unit_type == NAL_UNIT_BLA_N_LP);
}

bool isCRA(uint8_t unit_type)
{
  return unit_type == NAL_UNIT_CRA_NUT;
}

bool isRAP(uint8_t unit_type)
{
  return isIDR(unit_type) || isBLA(unit_type) || isCRA(unit_type);
}

bool isRASL(uint8_t unit_type)
{
  return (unit_type == NAL_UNIT_RASL_N ||
          unit_type == NAL_UNIT_RASL_R);
}

bool isIRAP(uint8_t unit_type)
{
  return (unit_type >= NAL_UNIT_BLA_W_LP &&
          unit_type <= NAL_UNIT_RSV_IRAP_VCL23);
}

bool isRADL(uint8_t unit_type)
{
  return (unit_type == NAL_UNIT_RADL_N ||
          unit_type == NAL_UNIT_RADL_R);
}
