/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#ifndef DE265_SPS_FUNC_H
#define DE265_SPS_FUNC_H

#include "libde265/sps.h"
#include "libde265/decctx.h"

de265_error read_sps(decoder_context*, bitreader*, seq_parameter_set*, ref_pic_set**);
void dump_sps(seq_parameter_set*, ref_pic_set* sets, int fd);

void free_ref_pic_sets(ref_pic_set**);

#endif
