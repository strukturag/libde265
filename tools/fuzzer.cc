/*
 * H.265 video codec.
 * Copyright (c) 2017 struktur AG, Joachim Bauch <bauch@struktur.de>
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

#include <stddef.h>
#include <stdio.h>

#include "libde265/de265.h"

// Bitmask of actions that should terminate the decoding loop.
static const int kFinishedActions =
  de265_action_push_more_input | de265_action_end_of_stream;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  // We need at least two bytes of data to run.
  if (size < 2) {
    return 0;
  }

  de265_decoder_context* ctx = de265_new_decoder();
  // Let the fuzzer decide on the number of worker threads (1 - 4 threads).
  de265_start_worker_threads(ctx, 1 + (data[0] >> 6));
  // Let the fuzzer decide on the number of frames to decode simultaneously.
  de265_set_max_frames_to_decode_in_parallel(ctx, data[1]);
  data += 2;
  size -= 2;

  // Pass any remaining data to the decoder.
  if (size > 0) {
    de265_push_data(ctx, data, size, 0, nullptr);
  }
  de265_push_end_of_stream(ctx);

  // Decode all images found in the input.
  int action;
  do {
    action = de265_get_action(ctx, true);
    if (action & de265_action_get_image) {
      const struct de265_image* image = de265_get_next_picture(ctx);
      de265_release_picture(image);
    }
  } while ((action & kFinishedActions) == 0);
  de265_free_decoder(ctx);
  return 0;
}
