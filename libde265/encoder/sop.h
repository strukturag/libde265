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

#ifndef DE265_SOP_H
#define DE265_SOP_H

#include "libde265/image.h"
#include "libde265/sps.h"
#include "libde265/encoder/encpicbuf.h"

#include <deque>
#include <vector>


struct refpic_set
{
  std::vector<int> l0;
  std::vector<int> l1;
};


class sop_creator
{
 public:
  sop_creator() { mEncPicBuf=NULL; }
  virtual ~sop_creator() { }

  void         set_encoder_picture_buffer(encoder_picture_buffer* encbuf) { mEncPicBuf=encbuf; }

  virtual void insert_new_input_image(const de265_image*) = 0;
  virtual void insert_end_of_stream() { mEncPicBuf->insert_end_of_stream(); }

  virtual int  get_number_of_temporal_layers() const { return 1; }

  virtual std::vector<refpic_set> get_sps_refpic_sets() const = 0;

 protected:
  encoder_picture_buffer* mEncPicBuf;
};



class sop_creator_intra_only : public sop_creator
{
 public:
  sop_creator_intra_only()
    {
      mNextFrameNumber = 0;
    }

  virtual std::vector<refpic_set> get_sps_refpic_sets() const
  {
    std::vector<refpic_set> refset;
    return refset;
  }

  virtual void insert_new_input_image(const de265_image* img)
  {
    assert(mEncPicBuf);
    mEncPicBuf->insert_next_image_in_encoding_order(img, mNextFrameNumber);
    mEncPicBuf->set_image_intra();
    mEncPicBuf->set_image_NAL_type(NAL_UNIT_IDR_N_LP);
    mEncPicBuf->sop_metadata_commit(mNextFrameNumber);

    mNextFrameNumber++;
  }

 private:
  int  mNextFrameNumber;
};



class sop_creator_trivial_low_delay : public sop_creator
{
 public:
  sop_creator_trivial_low_delay()
    {
      mNextFrameNumber = 0;
    }

  virtual std::vector<refpic_set> get_sps_refpic_sets() const
  {
    refpic_set s;
    s.l0.push_back(-1);

    std::vector<refpic_set> refset;
    refset.push_back(s);

    return refset;
  }

  virtual void insert_new_input_image(const de265_image* img)
  {
    std::vector<int> l0, l1, empty;
    if (mNextFrameNumber>0) {
      l0.push_back(mNextFrameNumber-1);
    }

    assert(mEncPicBuf);
    mEncPicBuf->insert_next_image_in_encoding_order(img, mNextFrameNumber);

    if (mNextFrameNumber==0) {
      mEncPicBuf->set_image_intra();
      mEncPicBuf->set_image_NAL_type(NAL_UNIT_IDR_N_LP);
    } else {
      mEncPicBuf->set_image_references(0, l0,l1, empty,empty);
      mEncPicBuf->set_image_NAL_type(NAL_UNIT_TRAIL_R);
    }
    mEncPicBuf->sop_metadata_commit(mNextFrameNumber);

    mNextFrameNumber++;
  }

 private:
  int  mNextFrameNumber;
};


#endif
