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

#include <algorithm>


void frame_dropper_IRAP_only::send_image_unit(image_unit_ptr imgunit)
{
  if (!isIRAP(imgunit->nal_unit_type)) {
    imgunit->state = image_unit::Dropped;
  }

  m_image_unit_sink->send_image_unit(imgunit);
}


/*
void frame_dropper_ratio::send_image_unit(image_unit_ptr imgunit)
{
  if (imgunit->img->PicOrderCntVal % 2 == 1) {
    imgunit->state = image_unit::Dropped;
    loginfo(LogHighlevel,"...\n");
  }
  else
    loginfo(LogHighlevel,"SHOW\n");

  m_image_unit_sink->send_image_unit(imgunit);
}
*/


frame_dropper_ratio::frame_dropper_ratio()
  : m_dropping_ratio(0.0),
    m_decctx(nullptr)
{
  m_n_dropped = 0;
  m_n_total   = 1;

  /* VLC will show "pictures leaked" errors when we delay the decoding too much.
     As a quick fix, we limit the frame-dropping queue to a maximum length.
     This may affect the dropping capabilities.

     A better solution would be to allocate the image buffer _after_ the dropping queue.
     However, this will need further work on the pipeline.
   */
  m_max_queue_length = 12;
}

void frame_dropper_ratio::reset()
{
  m_image_queue.clear();
  m_n_dropped = 0;
  m_n_total = 0;
  m_dropped_history.clear();
}

void frame_dropper_ratio::mark_used(int dpb_idx)
{
  // find ID of image

  image_ptr img = m_decctx->get_image(dpb_idx);
  int id = img->get_ID();
  for (int i=0;i<m_image_queue.size();i++) {
    if (m_image_queue[i].imgunit->img->get_ID() == id) {
      m_image_queue[i].used_for_reference = true;
      break;
    }
  }
}


void frame_dropper_ratio::send_image_unit(image_unit_ptr imgunit)
{
  frame_item item;
  item.imgunit = imgunit;
  item.used_for_reference = false;
  item.in_dpb = true;

  m_image_queue.push_back(item);
  //printf("-------------------- %d\n",m_image_queue.size());

  slice_unit* sunit = imgunit->get_next_unprocessed_slice_segment();
  if (sunit) {
    slice_segment_header* shdr = sunit->shdr;

    // mark images that are used as reference by this image

    /*
    printf("FRD processing POC %d (ID=%d):\n",
           item.imgunit->img->PicOrderCntVal,
           item.imgunit->img->get_ID());
    */

    for (int i=0;i<shdr->num_ref_idx_l0_active;i++) {
      mark_used(shdr->RefPicList[0][i]);
    }

    for (int j=0;j<shdr->num_ref_idx_l1_active;j++) {
      mark_used(shdr->RefPicList[1][j]);
    }


    // mark images that have been removed from the DPB

    for (int id : shdr->RemoveReferencesList) {

      for (int i=0;i<m_image_queue.size();i++) {
        if (m_image_queue[i].imgunit->img->get_ID() == id) {
          m_image_queue[i].in_dpb = false;
          break;
        }
      }
    }
  }


  while (!m_image_queue.empty() &&

         (m_image_queue.front().in_dpb == false ||
          m_image_queue.front().used_for_reference == true ||
          m_image_queue.size() > m_max_queue_length)
         ) {

    /*
    printf("%d REF: %s\n",
           m_image_queue.front().imgunit->img->get_ID(),
           m_image_queue.front().used_for_reference ? "YES" : "no");
    */

    const frame_item& item = m_image_queue.front();

    bool drop = ( !item.used_for_reference &&
                  float(m_n_dropped)/m_n_total < m_dropping_ratio);

    if (m_image_queue.size() > m_max_queue_length) {
      drop=false;
    }

    /*
    printf("FRD POC %d can be dropped %d  -> drop %d/%d (%f) -> %d\n",
           item.imgunit->img->PicOrderCntVal,
           !item.used_for_reference,
           m_n_dropped,m_n_total, m_dropping_ratio,
           drop);
    */

    if (drop) {
      item.imgunit->state = image_unit::Dropped;
      m_n_dropped++;
    }

    m_n_total++;
    m_dropped_history.push_back(drop);

    if (m_dropped_history.size() > HISTORY_LEN) {
      bool old_drop = m_dropped_history.front();
      m_dropped_history.pop_front();

      m_n_total--;
      if (old_drop) m_n_dropped--;
    }

    m_image_unit_sink->send_image_unit( item.imgunit );
    m_image_queue.pop_front();
  }
}


void frame_dropper_ratio::send_end_of_stream()
{
  while (!m_image_queue.empty()) {
    const frame_item& item = m_image_queue.front();

    if (0 && !item.used_for_reference) {
      item.imgunit->state = image_unit::Dropped;
    }

    m_image_unit_sink->send_image_unit( item.imgunit );
    m_image_queue.pop_front();
  }

  m_image_unit_sink->send_end_of_stream();
}



/* Our model for the fps after frame-dropping to ratio 'p' is:
   fps(p) = fps_max*(1/4 + 3/4*p)

   Hence, the effective frame-rate fps_eff (decoding speed neglecting that some
   frames are dropped) is:

   fps_eff = fps(p) / p
*/

float frame_drop_calc_fps_eff(float fps_max, float ratio)
{
  return fps_max * (1/(4.0*ratio) + 3/4.0);
}

float frame_drop_calc_fps_max(float fps_eff, float ratio)
{
  return fps_eff / (1/(4.0*ratio) + 3/4.0);
}

float frame_drop_calc_ratio  (float fps_max, float fps_eff)
{
  return 1.0/( 4*(fps_eff/fps_max - 3/4.0) );
}



fps_estimator::fps_estimator()
{
  m_estimator_min_timespan = 1.0f; // seconds
  m_estimator_timespan = 5.0f; // 5 seconds
}

void  fps_estimator::reset_fps_estimator()
{
  m_frame_timestamps.clear();
}

void  fps_estimator::set_fps_estimator_timespan(float sec)
{
  m_estimator_timespan = sec;
}

void  fps_estimator::set_fps_estimator_min_timespan(float sec)
{
  m_estimator_min_timespan = sec;
}

void  fps_estimator::on_frame_decoded(float timestamp_s)
{
  m_frame_timestamps.push_back(timestamp_s);

  while (m_frame_timestamps.front() < timestamp_s - m_estimator_timespan) {
    m_frame_timestamps.pop_front();
  }
}

bool  fps_estimator::fps_measurement_available() const
{
  if (m_frame_timestamps.empty()) { return false; }

  return m_frame_timestamps.back() - m_frame_timestamps.front() >= m_estimator_min_timespan;
}

float fps_estimator::get_fps_measurement() const
{
  assert(fps_measurement_available());

  // should always have at least two entries when above test valid
  assert(m_frame_timestamps.size() >= 2);

  float timespan = m_frame_timestamps.back() - m_frame_timestamps.front();
  return (m_frame_timestamps.size()-1) / timespan;
}




frame_drop_ratio_calculator::frame_drop_ratio_calculator()
{
  m_target_fps = 50.0f;
  m_current_decoding_ratio = 1.0f;

  m_model_param = 1.0f; //0.25f; // minimum decoder fps (virtually, at ratio=0%).
  m_decrease_update_factor = 0.5f;
  m_increase_update_factor = 0.1f;
}

void frame_drop_ratio_calculator::reset_ratio_levels()
{
  m_ratio_levels.clear();
}

void frame_drop_ratio_calculator::add_ratio_level(float ratio)
{
  m_ratio_levels.push_back(ratio);
  std::sort(m_ratio_levels.begin(), m_ratio_levels.end());
}


void frame_drop_ratio_calculator::set_target_fps(float fps)
{
  m_target_fps = fps;
}


float frame_drop_ratio_calculator::update_decoding_ratio(float measured_fps)
{
  // effective fps (how fast the video plays, if we ignore the fact that some frames are skipped)
  float fps_eff = measured_fps / m_current_decoding_ratio;
  printf("fps_eff=%f\n",fps_eff);

  // estimated decoder speed at 100% decoding ratio
  float fps_100 = measured_fps / (m_model_param + (1-m_model_param)*m_current_decoding_ratio);
  printf("fps_100=%f\n",fps_100);

  // new decoding ratio
  float ratio = m_model_param / (m_target_fps/fps_100 - (1-m_model_param));


  // --- post-process the new decoding-rate to prevent oscillation (even though that should
  // --- not happen with conservative model_parameter settings) ---

  if (ratio<0) ratio=0;
  if (ratio>1) ratio=1;

  if (ratio<m_current_decoding_ratio) {
    m_current_decoding_ratio = (1-m_decrease_update_factor)*m_current_decoding_ratio
      + m_decrease_update_factor * ratio;
  }
  else {
    m_current_decoding_ratio = (1-m_increase_update_factor)*m_current_decoding_ratio
      + m_increase_update_factor * ratio;
  }

  return m_current_decoding_ratio;
}
