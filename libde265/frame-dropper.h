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
#include <deque>


class frame_dropper : public image_unit_sink
{
 public:
 frame_dropper() : m_image_unit_sink(nullptr) { }

  void reset() { }

  void set_image_unit_sink(image_unit_sink* sink) { m_image_unit_sink = sink; }

  virtual void send_image_unit(image_unit_ptr imgunit) {
    m_image_unit_sink->send_image_unit(imgunit);
  }

  virtual void send_end_of_stream() {
    m_image_unit_sink->send_end_of_stream();
  }

 protected:
  image_unit_sink* m_image_unit_sink;
};


class frame_dropper_nop : public frame_dropper { };


class frame_dropper_IRAP_only : public frame_dropper
{
 public:
  virtual void send_image_unit(image_unit_ptr);
};


class decoder_context;


class frame_dropper_ratio : public frame_dropper
{
public:
  frame_dropper_ratio();

  void reset();

  void set_decoder_context(decoder_context& decctx) { m_decctx=&decctx; }

  virtual void send_image_unit(image_unit_ptr);

  void set_dropping_ratio(float ratio) { m_dropping_ratio = ratio; }
  void send_end_of_stream();

private:
  float m_dropping_ratio;

  decoder_context* m_decctx;

  struct frame_item {
    image_unit_ptr imgunit;
    bool           used_for_reference;
    bool           in_dpb;
  };

  std::deque<frame_item> m_image_queue;

  int m_n_dropped;
  int m_n_total;
  std::deque<bool> m_dropped_history;

  static const int HISTORY_LEN = 50;

  void mark_used(int dpb_idx);
};

#endif
