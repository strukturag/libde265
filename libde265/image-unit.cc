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


#include "libde265/decctx.h"
#include "libde265/image-unit.h"


slice_unit::slice_unit(decoder_context* decctx)
  : shdr(NULL),
    imgunit(NULL),
    flush_reorder_buffer(false),
    nThreads(0),
    first_CTB_TS(-1),
    last_CTB_TS(-1),
    thread_contexts(NULL),
    ctx(decctx)
{
  state = Unprocessed;
  nThreadContexts = 0;
}

slice_unit::~slice_unit()
{
  if (thread_contexts) {
    delete[] thread_contexts;
  }
}


void slice_unit::allocate_thread_contexts(int n)
{
  assert(thread_contexts==NULL);

  thread_contexts = new thread_context[n];
  nThreadContexts = n;
}


thread_context* slice_unit::get_thread_context(int n)
{
  assert(n < nThreadContexts);
  return &thread_contexts[n];
}


void slice_unit::mark_covered_CTBs_as_processed(int progress) const
{
  // mark all CTBs assigned to this slice as processed

  for (int ctb=first_CTB_TS; ctb <= last_CTB_TS; ctb++)
    {
      if (ctb >= imgunit->img->number_of_ctbs())
        break;

      int ctb_rs = shdr->get_pps()->CtbAddrTStoRS[ctb];

      imgunit->img->ctb_progress[ctb_rs].set_progress(progress);
    }
}








image_unit::image_unit()
{
  role=Invalid;
  state=Unprocessed;
}


image_unit::~image_unit()
{
  //wait_to_finish_decoding();

  for (int i=0;i<slice_units.size();i++) {
    delete slice_units[i];
  }
}


bool image_unit::did_finish_decoding() const
{
  for (const auto& task : tasks) {

    if (!task->finished()) {
      return false;
    }
  }

  return true;
}


void image_unit::wait_to_finish_decoding()
{
  for (const auto& task : tasks) {
    task->wait_until_finished();
  }
}
