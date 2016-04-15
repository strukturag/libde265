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

#ifndef DE265_FRAME_DROPPER_H
#define DE265_FRAME_DROPPER_H

#include "libde265/vps.h"
#include "libde265/sps.h"
#include "libde265/pps.h"
#include "libde265/nal.h"
#include "libde265/slice.h"
#include "libde265/image.h"
#include "libde265/motion.h"
#include "libde265/de265.h"
#include "libde265/dpb.h"
#include "libde265/sei.h"
#include "libde265/threads.h"
#include "libde265/image-unit.h"

#include <memory>


class frame_dropper : public image_unit_sink
{
 public:
 frame_dropper() : m_image_unit_sink(nullptr) { }

  void set_image_unit_sink(image_unit_sink* sink) { m_image_unit_sink = sink; }

  virtual void send_image_unit(image_unit_ptr imgunit) {
    m_image_unit_sink->send_image_unit(imgunit);
  }

  virtual void send_end_of_stream() {
    m_image_unit_sink->send_end_of_stream();
  }

 private:
  image_unit_sink* m_image_unit_sink;
};


class frame_dropper_nop : public frame_dropper { };


class frame_dropper_IRAP_only : public image_unit_sink
{
 public:
  virtual void send_image_unit(image_unit_ptr);
};

#endif
