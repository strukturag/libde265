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

/*
struct refpic_set
{
  std::vector<int> l0;
  std::vector<int> l1;
};
*/

class pic_order_counter
{
 public:
  pic_order_counter() { mPOC=0; mNumLsbBits=6; }

  void reset() { mPOC=0; }

  int get_pic_order_count() const { return mPOC; }
  int get_pic_order_count_lsb() const {
    return mPOC & ((1<<mNumLsbBits)-1);
  }

  void advance_poc(int n=1) { mPOC+=n; }

  void set_num_poc_lsb_bits(int n) { mNumLsbBits=n; }
  int  get_num_poc_lsb_bits() const { return mNumLsbBits; }

 private:
  int mPOC;
  int mNumLsbBits;
};


class sop_creator : public pic_order_counter
{
 public:
  sop_creator() { mEncCtx=NULL; mEncPicBuf=NULL; }
  virtual ~sop_creator() { }

  void set_encoder_context(encoder_context* encctx) { mEncCtx=encctx; }
  void set_encoder_picture_buffer(encoder_picture_buffer* encbuf) { mEncPicBuf=encbuf; }

  /* Fills in the following fields:
     - SPS.ref_pic_sets
     - SPS.log2_max_pic_order_cnt_lsb
   */
  virtual void set_SPS_header_values() = 0;

  /* Fills in the following fields:
     - NAL.nal_type
     - SHDR.slice_type
     - SHDR.slice_pic_order_cnt_lsb
     - IMGDATA.references
   */
  virtual void insert_new_input_image(const de265_image*) = 0;
  virtual void insert_end_of_stream() { mEncPicBuf->insert_end_of_stream(); }

  virtual int  get_number_of_temporal_layers() const { return 1; }

  //virtual std::vector<refpic_set> get_sps_refpic_sets() const = 0;

 protected:
  encoder_context*        mEncCtx;
  encoder_picture_buffer* mEncPicBuf;
};



class sop_creator_intra_only : public sop_creator
{
 public:
  sop_creator_intra_only();

  virtual void set_SPS_header_values();
  virtual void insert_new_input_image(const de265_image* img);
};



class sop_creator_trivial_low_delay : public sop_creator
{
 public:
  sop_creator_trivial_low_delay();

  virtual void set_SPS_header_values();
  virtual void insert_new_input_image(const de265_image* img);
};


#endif
