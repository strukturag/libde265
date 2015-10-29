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

#ifndef DE265_FUNCS_H
#define DE265_FUNCS_H

#include "libde265/codingdata.h"

template <class T>
bool available_zscan(const CodingDataAccess<T>&,int xCurr,int yCurr, int xN,int yN);


template <class T>
bool available_pred_blk(const CodingDataAccess<T>& access,
                        int xC,int yC, int nCbS,
                        int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN);

#endif
