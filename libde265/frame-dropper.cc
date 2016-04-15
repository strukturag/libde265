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


#include "frame-dropper.h"
#include "decctx.h"


void frame_dropper_IRAP_only::send_image_unit(image_unit_ptr imgunit)
{
  if (!imgunit->slice_units.empty()) {
    slice_unit* sunit = imgunit->slice_units[0];
    slice_segment_header* shdr = sunit->shdr;


  }

  m_image_unit_sink->send_image_unit(imgunit);
}
