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
  int   m_max_queue_length;

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



class fps_estimator
{
 public:
  fps_estimator();

  void  set_fps_estimator_min_timespan(float sec);
  void  set_fps_estimator_timespan(float sec);

  void  reset_fps_estimator();
  void  on_frame_decoded(float timestamp_s);

  bool  fps_measurement_available() const;
  float get_fps_measurement() const;

 private:
  float m_estimator_timespan;
  float m_estimator_min_timespan;
  std::deque<float> m_frame_timestamps;
};


/* Definition of different fps values:
   fps_measured = actual fps output of decoder. Dropped frames do not count.
   fps_eff      = video speed as observed by user (as if no frames were dropped)
   fps_100      = decoder fps when running at ratio 100%.
   fps_dec(r)   = decoder fps when running at ratio r
   fps_target   = the effective rate that we want at the output
   dec_ratio    = percentage of frames that are decoded

   fps_eff = fps_measured / dec_ratio

   We assume a model for fps_dec:
   fps_dec(ratio) = fps_100 * (alpha + (1-alpha)*ratio)
   alpha = 1/4 seems to be a typical parameter

   Hence:
   ratio = alpha / (fps_target/fps_100 - (1-alpha))

   It can be observed that for adjusting the decoding rate, it is better to
   assume a too conservative value for alpha (closer to 1). Otherwise, the
   ratio control may start to oscillate.
 */
class frame_drop_ratio_calculator
{
 public:
  frame_drop_ratio_calculator();

  void set_model_parameter(float p) { m_model_param=p; }

  void reset_ratio_levels();
  void add_ratio_level(float ratio);

  void set_target_fps(float fps);
  // ??? void set_safety_offset(float ratio_offset);

  //void  set_decoding_ratio(float ratio) { m_current_decoding_ratio=ratio; }
  float update_decoding_ratio(float measured_fps);

 private:
  std::vector<float> m_ratio_levels;

  float m_target_fps;

  float m_current_decoding_ratio;


  float m_model_param;
  float m_decrease_update_factor;
  float m_increase_update_factor;
};

#endif
