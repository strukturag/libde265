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

#ifndef DE265_CODING_DATA_IMPL_H
#define DE265_CODING_DATA_IMPL_H

#include <stdint.h>

#include "libde265/codingdata.h"
#include "libde265/image.h"
#include "libde265/encoder/encoder-context.h"


template <> class CodingDataAccess<image>
{
public:
  CodingDataAccess(const image* i) : img(i) { }

  const video_parameter_set& get_vps() const { return img->get_vps(); }
  const seq_parameter_set& get_sps() const { return img->get_sps(); }
  const pic_parameter_set& get_pps() const { return img->get_pps(); }

  int  get_POC() const { return img->PicOrderCntVal; }

  int  get_SliceAddrRS(int ctbX, int ctbY) const { return img->get_SliceAddrRS(ctbX,ctbY); }
  enum PartMode get_PartMode(int x,int y) const { return img->get_PartMode(x,y); }
  enum PredMode get_pred_mode(int x,int y) const { return img->get_pred_mode(x,y); }
  const PBMotion& get_mv_info(int x,int y) const { return img->get_mv_info(x,y); }

private:
  const image* img;
};


template <> class CodingDataAccess<encoder_context>
{
public:
  CodingDataAccess(const encoder_context* e) : ectx(e) { }

  int  get_POC() const { return ectx->img->PicOrderCntVal; }

  const video_parameter_set& get_vps() const { return ectx->get_vps(); }
  const seq_parameter_set& get_sps() const { return ectx->get_sps(); }
  const pic_parameter_set& get_pps() const { return ectx->get_pps(); }

  int  get_SliceAddrRS(int ctbX, int ctbY) const { return ectx->img->get_SliceAddrRS(ctbX,ctbY); }
  enum PartMode get_PartMode(int x,int y) const { return ectx->ctbs.getCB(x,y)->PartMode; }
  enum PredMode get_pred_mode(int x,int y) const { return ectx->ctbs.getCB(x,y)->PredMode; }
  const PBMotion& get_mv_info(int x,int y) const {
    return ectx->ctbs.getPB(x,y)->motion;
  }

private:
  const encoder_context* ectx;
};

#endif
