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

#include "libde265/encoder/sop.h"
#include "libde265/encoder/encoder-context.h"


sop_creator_intra_only::sop_creator_intra_only()
{
  mNextFrameNumber = 0;
}


void sop_creator_intra_only::set_SPS_header_values()
{
}


void sop_creator_intra_only::insert_new_input_image(const de265_image* img)
{
  assert(mEncPicBuf);
  image_data* imgdata = mEncPicBuf->insert_next_image_in_encoding_order(img, mNextFrameNumber);

  imgdata->set_intra();
  imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);
  imgdata->shdr.slice_type = SLICE_TYPE_I;
  mEncPicBuf->sop_metadata_commit(mNextFrameNumber);

  mNextFrameNumber++;
}


// ---------------------------------------------------------------------------


sop_creator_trivial_low_delay::sop_creator_trivial_low_delay()
{
  mNextFrameNumber = 0;
}


void sop_creator_trivial_low_delay::set_SPS_header_values()
{
  ref_pic_set rps;
  rps.DeltaPocS0[0] = -1;
  rps.UsedByCurrPicS0[0] = true;
  rps.NumNegativePics = 1;
  rps.NumPositivePics = 0;
  rps.compute_derived_values();
  mEncCtx->sps.ref_pic_sets.push_back(rps);
}


void sop_creator_trivial_low_delay::insert_new_input_image(const de265_image* img)
{
  std::vector<int> l0, l1, empty;
  if (mNextFrameNumber>0) {
    l0.push_back(mNextFrameNumber-1);
  }

  assert(mEncPicBuf);
  image_data* imgdata = mEncPicBuf->insert_next_image_in_encoding_order(img, mNextFrameNumber);

  if (mNextFrameNumber==0) {
    imgdata->set_intra();
    imgdata->set_NAL_type(NAL_UNIT_IDR_N_LP);
    imgdata->shdr.slice_type = SLICE_TYPE_I;
  } else {
    imgdata->set_references(0, l0,l1, empty,empty);
    imgdata->set_NAL_type(NAL_UNIT_TRAIL_R);
    imgdata->shdr.slice_type = SLICE_TYPE_P;
  }
  mEncPicBuf->sop_metadata_commit(mNextFrameNumber);

  mNextFrameNumber++;
}
