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

#include "slice.h"
#include <assert.h>


context_model_table2::context_model_table2()
  : model(NULL), refcnt(NULL)
{
  //decouple_or_alloc_with_empty_data();
}

context_model_table2::~context_model_table2()
{
  if (refcnt) {
    *refcnt--;
    if (*refcnt==0) {
      delete[] model;
      delete refcnt;
    }
  }
}

void context_model_table2::init(int initType, int QPY)
{
  decouple_or_alloc_with_empty_data();

  initialize_CABAC_models(model, initType, QPY);
}

void context_model_table2::release()
{
  assert(refcnt); // not necessarily so, but we never use it on an unitialized object

  *refcnt--;
  if (refcnt==0) {
    delete[] model;
    delete refcnt;
  }

  model = nullptr;
  refcnt= nullptr;
}

void context_model_table2::decouple()
{
  assert(refcnt); // not necessarily so, but we never use it on an unitialized object

  if (*refcnt > 1) {
    *refcnt--;

    context_model* oldModel = model;

    model = new context_model[CONTEXT_MODEL_TABLE_LENGTH];
    refcnt= new int;
    *refcnt=1;

    memcpy(model,oldModel,sizeof(*model));
  }
}

context_model_table2& context_model_table2::operator=(const context_model_table2& src)
{
  assert(src.refcnt); // not necessarily so, but we never use it on an unitialized object

  (*(src.refcnt))++;

  release();

  model = src.model;
  refcnt= src.refcnt;
}

void context_model_table2::decouple_or_alloc_with_empty_data()
{
  if (refcnt && *refcnt==1) { return; }

  if (refcnt) {
    assert(*refcnt>1);
    *refcnt--;
  }

  model = new context_model[CONTEXT_MODEL_TABLE_LENGTH];
  refcnt= new int;
  *refcnt=1;
}
