/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
 *          Min Chen <chenm003@163.com>
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

#include "funcs.h"
#include "codingdata-impl.h"


template <class T>
bool available_zscan(const CodingDataAccess<T>& data,int xCurr,int yCurr, int xN,int yN)
{
  const seq_parameter_set& sps = data.get_sps();
  const pic_parameter_set& pps = data.get_pps();

  if (xN<0 || yN<0) return false;
  if (xN>=sps.pic_width_in_luma_samples ||
      yN>=sps.pic_height_in_luma_samples) return false;

  int minBlockAddrN = pps.MinTbAddrZS[ (xN>>sps.Log2MinTrafoSize) +
                                       (yN>>sps.Log2MinTrafoSize) * sps.PicWidthInTbsY ];
  int minBlockAddrCurr = pps.MinTbAddrZS[ (xCurr>>sps.Log2MinTrafoSize) +
                                          (yCurr>>sps.Log2MinTrafoSize) * sps.PicWidthInTbsY ];

  if (minBlockAddrN > minBlockAddrCurr) return false;

  int xCurrCtb = xCurr >> sps.Log2CtbSizeY;
  int yCurrCtb = yCurr >> sps.Log2CtbSizeY;
  int xNCtb = xN >> sps.Log2CtbSizeY;
  int yNCtb = yN >> sps.Log2CtbSizeY;

  if (data.get_SliceAddrRS(xCurrCtb,yCurrCtb) !=
      data.get_SliceAddrRS(xNCtb,   yNCtb)) {
    return false;
  }

  if (pps.TileIdRS[xCurrCtb + yCurrCtb*sps.PicWidthInCtbsY] !=
      pps.TileIdRS[xNCtb    + yNCtb   *sps.PicWidthInCtbsY]) {
    return false;
  }

  return true;
}


template bool available_zscan<de265_image>(const CodingDataAccess<de265_image>& data,int xCurr,int yCurr, int xN,int yN);
template bool available_zscan<encoder_context>(const CodingDataAccess<encoder_context>& data,int xCurr,int yCurr, int xN,int yN);


template <class T>
bool available_pred_blk(const CodingDataAccess<T>& access,
                        int xC,int yC, int nCbS,
                        int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN)
{
  logtrace(LogMotion,"C:%d;%d P:%d;%d N:%d;%d size=%d;%d\n",xC,yC,xP,yP,xN,yN,nPbW,nPbH);

  int sameCb = (xC <= xN && xN < xC+nCbS &&
                yC <= yN && yN < yC+nCbS);

  bool availableN;

  if (!sameCb) {
    availableN = available_zscan(access,xP,yP,xN,yN);
  }
  else {
    availableN = !(nPbW<<1 == nCbS && nPbH<<1 == nCbS &&  // NxN
                   partIdx==1 &&
                   yN >= yC+nPbH && xN < xC+nPbW);  // xN/yN inside partIdx 2
  }

  if (availableN && access.get_pred_mode(xN,yN) == MODE_INTRA) {
    availableN = false;
  }

  return availableN;
}


template bool available_pred_blk<de265_image>(const CodingDataAccess<de265_image>& access,
                                              int xC,int yC, int nCbS,
                                              int xP, int yP, int nPbW, int nPbH, int partIdx,
                                              int xN,int yN);
template bool available_pred_blk<encoder_context>(const CodingDataAccess<encoder_context>& access,
                                                  int xC,int yC, int nCbS,
                                                  int xP, int yP, int nPbW, int nPbH, int partIdx,
                                                  int xN,int yN);
