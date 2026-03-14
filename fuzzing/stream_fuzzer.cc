/*
 * H.265 video codec.
 * Copyright (c) 2026 Dirk Farin <dirk.farin@gmail.com>
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

#include "de265.h"
#include <stdint.h>
#include <stddef.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
  de265_decoder_context* ctx = de265_new_decoder();
  if (!ctx) {
    return 0;
  }

  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, 0);
  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_SUPPRESS_FAULTY_PICTURES, 0);
  de265_set_parameter_int(ctx, DE265_DECODER_PARAM_ACCELERATION_CODE, de265_acceleration_SCALAR);

  de265_push_data(ctx, data, size, 0, nullptr);
  de265_flush_data(ctx);

  int more = 1;
  while (more) {
    de265_error err = de265_decode(ctx, &more);
    if (!more || (err != DE265_OK && err != DE265_ERROR_WAITING_FOR_INPUT_DATA)) {
      break;
    }

    while (const de265_image* img = de265_get_next_picture(ctx)) {
      (void)img;
    }
  }

  de265_free_decoder(ctx);
  return 0;
}
