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


  slice_unit* sunit = imgunit->get_next_unprocessed_slice_segment();
  if (sunit) {
    slice_segment_header* shdr = sunit->shdr;

    // mark images that are used as reference by this image

    for (int i=0;i<shdr->num_ref_idx_l0_active;i++) {
      mark_used(shdr->RefPicList[0][i]);
    }

    for (int j=0;j<shdr->num_ref_idx_l0_active;j++) {
      mark_used(shdr->RefPicList[0][j]);
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


  while (m_image_queue.front().in_dpb == false) {
    /*
    printf("%d REF: %s\n",
           m_image_queue.front().imgunit->img->get_ID(),
           m_image_queue.front().used_for_reference ? "YES" : "no");
    */

    const frame_item& item = m_image_queue.front();

    bool drop = ( !item.used_for_reference &&
                  float(m_n_dropped)/m_n_total < m_dropping_ratio);

    /*
    printf("can be dropped %d  -> drop %d/%d (%f) -> %d\n",
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
}
