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

#ifndef DE265_CODING_DATA_H
#define DE265_CODING_DATA_H

#include <stdint.h>
#include <slice.h>
#include <pps.h>
#include <motion.h>

class base_context;
class image_history;
class slice_segment_header;


template <class T> class CodingDataAccess
{
public:
  const video_parameter_set& get_vps() const;
  const seq_parameter_set& get_sps() const;
  const pic_parameter_set& get_pps() const;

  int  get_POC() const;

  int  get_SliceAddrRS(int ctbX, int ctbY) const;
  enum PartMode get_PartMode(int x,int y) const;
  enum PredMode get_pred_mode(int x,int y) const;
  const PBMotion& get_mv_info(int x,int y) const;
};

#endif
