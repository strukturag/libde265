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

#ifndef DE265_SLICE_FUNC_H
#define DE265_SLICE_FUNC_H

#include "libde265/slice.h"
#include "libde265/decctx.h"
#include "libde265/bitstream.h"
#include "libde265/threads.h"


de265_error read_slice_segment_header(bitreader* br, slice_segment_header* shdr, decoder_context*,
                                      bool* continueDecoding);
void dump_slice_segment_header(const slice_segment_header* shdr, const decoder_context*, int fd);


de265_error read_slice_segment_data(decoder_context*, thread_context* tctx);

bool add_CTB_decode_task_syntax(struct thread_context* tctx, int ctbx,int ctby,  int sx,int sy, thread_task* nextCTBTask);

bool alloc_and_init_significant_coeff_ctxIdx_lookupTable();
void free_significant_coeff_ctxIdx_lookupTable();

#endif
