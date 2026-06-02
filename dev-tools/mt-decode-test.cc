/*
 * H.265 video codec.
 * Copyright (c) 2026 Dirk Farin <dirk.farin@gmail.com>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

// Thread-safety / correctness stress test for the library-scope PPS scan-table
// cache: many independent de265_decoder_context objects decode the SAME stream
// (hence the same geometry) concurrently, so set_derived_values() races on the
// shared cache. Each context runs the SEI hash check, so a wrong/torn cached
// table would surface as a checksum mismatch. Run under helgrind/tsan to also
// check for data races:
//     valgrind --tool=helgrind ./mt-decode-test ../testdata/girlshy.h265 32
//
// Usage: mt-decode-test <stream.h265> [num_threads]

#include "libde265/de265.h"

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <thread>
#include <atomic>

static std::vector<uint8_t> g_data;        // shared, read-only after load
static std::atomic<int> g_failures{0};
static std::atomic<int> g_first_frames{-1}; // frame count of the first finished thread

static void decode_once(int id)
{
  de265_decoder_context* ctx = de265_new_decoder();
  if (!ctx) { g_failures++; return; }
  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, 1);

  de265_push_data(ctx, g_data.data(), (int)g_data.size(), 0, nullptr);
  de265_flush_data(ctx);

  int frames = 0;
  int more = 1;
  while (more) {
    more = 0;
    de265_error err = de265_decode(ctx, &more);
    if (err != DE265_OK && err != DE265_ERROR_WAITING_FOR_INPUT_DATA) {
      fprintf(stderr, "[thread %d] decode error: %s\n", id, de265_get_error_text(err));
      g_failures++;
      break;
    }
    while (de265_get_next_picture(ctx) != nullptr) frames++;

    for (;;) {
      de265_error w = de265_get_warning(ctx);
      if (w == DE265_OK) break;
      if (w == DE265_ERROR_CHECKSUM_MISMATCH) {
        fprintf(stderr, "[thread %d] HASH MISMATCH (cached scan table likely corrupt)\n", id);
        g_failures++;
      }
    }
  }

  de265_free_decoder(ctx);

  // all threads decode the identical stream, so they must agree on frame count
  int expected = g_first_frames.load();
  if (expected < 0) g_first_frames.compare_exchange_strong(expected, frames);
  expected = g_first_frames.load();
  if (frames != expected) {
    fprintf(stderr, "[thread %d] frame count %d != %d\n", id, frames, expected);
    g_failures++;
  }
}

int main(int argc, char** argv)
{
  if (argc < 2) {
    fprintf(stderr, "usage: %s <stream.h265> [num_threads]\n", argv[0]);
    return 2;
  }
  int nThreads = (argc >= 3) ? atoi(argv[2]) : 32;

  FILE* fh = fopen(argv[1], "rb");
  if (!fh) { fprintf(stderr, "cannot open %s\n", argv[1]); return 2; }
  fseek(fh, 0, SEEK_END);
  long sz = ftell(fh);
  fseek(fh, 0, SEEK_SET);
  g_data.resize(sz);
  if (fread(g_data.data(), 1, sz, fh) != (size_t)sz) { fclose(fh); return 2; }
  fclose(fh);

  printf("decoding '%s' in %d concurrent decoder contexts ...\n", argv[1], nThreads);

  std::vector<std::thread> threads;
  for (int i=0;i<nThreads;i++) threads.emplace_back(decode_once, i);
  for (auto& t : threads) t.join();

  int fails = g_failures.load();
  printf("%d threads, %d frames each: %s\n",
         nThreads, g_first_frames.load(), fails==0 ? "all passed" : "FAILURES");
  return fails==0 ? 0 : 1;
}
