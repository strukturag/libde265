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

  // Interpret input as a sequence of length-prefixed NAL units.
  // Each NAL is preceded by a 4-byte big-endian length.
  size_t pos = 0;
  while (pos + 4 <= size) {
    uint32_t nal_size = (uint32_t(data[pos]) << 24) |
                        (uint32_t(data[pos + 1]) << 16) |
                        (uint32_t(data[pos + 2]) << 8) |
                        uint32_t(data[pos + 3]);
    pos += 4;

    if (nal_size > size - pos) {
      break;
    }

    de265_push_NAL(ctx, data + pos, nal_size, 0, nullptr);
    pos += nal_size;
  }

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
