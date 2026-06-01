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

#include "tests.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "libde265/fallback-dct.h"

#include <stdio.h>
#include <stdint.h>
#include <vector>

// The SSE add_residual functions only exist when the library was built with
// SSE4.1 support. Without it there is nothing to compare against, so the test
// registers itself as a no-op (always passing).
#if HAVE_SSE4_1
#include "libde265/x86/sse-dct.h"
#endif


namespace {

// Deterministic PRNG (xorshift32) so the test is fully reproducible.
struct RNG {
  uint32_t s;
  explicit RNG(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() { s ^= s<<13; s ^= s>>17; s ^= s<<5; return s; }
  // uniform integer in [lo,hi]
  int range(int lo, int hi) { return lo + (int)(next() % (uint32_t)(hi-lo+1)); }
};

enum Scenario {
  SCEN_RANDOM,     // residual around the clip boundaries -> mix of clipped/unclipped
  SCEN_CLIP_HIGH,  // residual large positive -> everything clips to maxval
  SCEN_CLIP_LOW,   // residual large negative -> everything clips to 0
  SCEN_EXTREMES,   // pixels at 0/maxval, residual large +/-
  SCEN_COUNT
};

const char* scenarioName(int s) {
  switch (s) {
    case SCEN_RANDOM:    return "random";
    case SCEN_CLIP_HIGH: return "clip-high";
    case SCEN_CLIP_LOW:  return "clip-low";
    case SCEN_EXTREMES:  return "extremes";
  }
  return "?";
}

// Run one comparison case: build identical input for the scalar reference and
// the SSE implementation, run both, and verify the whole strided buffer matches
// (which also catches out-of-region writes).
template <class pixel_t, class SSEFunc>
bool runCase(int nT, int bit_depth, int scenario, RNG& rng, SSEFunc sseFunc, bool quiet)
{
  const int maxval = (1<<bit_depth)-1;
  const int stride = nT + 17;   // non-trivial stride: leaves padding after each row

  std::vector<pixel_t> dstRef((size_t)nT*stride);
  std::vector<pixel_t> dstSSE((size_t)nT*stride);
  std::vector<int32_t> r((size_t)nT*nT);

  // identical pixel buffers (including the padding columns)
  for (size_t i=0; i<dstRef.size(); i++) {
    pixel_t p;
    if (scenario == SCEN_EXTREMES) p = (rng.next()&1) ? (pixel_t)maxval : (pixel_t)0;
    else                           p = (pixel_t)rng.range(0, maxval);
    dstRef[i] = p;
    dstSSE[i] = p;
  }

  // residual block (row-major, nT values per row)
  for (int i=0; i<nT*nT; i++) {
    int32_t v;
    switch (scenario) {
      case SCEN_CLIP_HIGH: v =  maxval + rng.range(1, maxval); break;
      case SCEN_CLIP_LOW:  v = -maxval - rng.range(1, maxval); break;
      case SCEN_EXTREMES:  v = (rng.next()&1) ?  (maxval + rng.range(0, maxval))
                                              : -(maxval + rng.range(0, maxval)); break;
      default:             v = rng.range(-maxval-64, maxval+64); break;
    }
    r[i] = v;
  }

  add_residual_fallback<pixel_t>(dstRef.data(), stride, r.data(), nT, bit_depth);
  sseFunc(dstSSE.data(), stride, r.data(), nT, bit_depth);

  for (size_t i=0; i<dstRef.size(); i++) {
    if (dstRef[i] != dstSSE[i]) {
      if (!quiet) {
        printf("  MISMATCH nT=%d depth=%d scen=%s at (x=%d,y=%d): ref=%d sse=%d\n",
               nT, bit_depth, scenarioName(scenario),
               (int)(i%stride), (int)(i/stride), (int)dstRef[i], (int)dstSSE[i]);
      }
      return false;
    }
  }
  return true;
}

} // namespace


class AddResidualTest : public Test
{
public:
  const char* getName() const { return "add-residual"; }
  const char* getDescription() const {
    return "compare SSE add_residual against scalar fallback (random blocks + edge cases)";
  }

  bool work(bool quiet) {
#if HAVE_SSE4_1
    RNG rng(0x00C0FFEE);
    const int blkSizes[] = {4, 8, 16, 32};
    const int depths16[] = {9, 10, 12, 16};
    const int repeats = 8;
    bool ok = true;
    int nCases = 0;

    // 8-bit path (add_residual_8_sse4) -- bit_depth is always 8 here
    for (int b=0; b<4; b++)
      for (int sc=0; sc<SCEN_COUNT; sc++)
        for (int rep=0; rep<repeats; rep++) {
          nCases++;
          if (!runCase<uint8_t>(blkSizes[b], 8, sc, rng, add_residual_8_sse4, quiet))
            ok = false;
        }

    // high bit-depth path (add_residual_16_sse4), depths 9..16
    for (int b=0; b<4; b++)
      for (int d=0; d<4; d++)
        for (int sc=0; sc<SCEN_COUNT; sc++)
          for (int rep=0; rep<repeats; rep++) {
            nCases++;
            if (!runCase<uint16_t>(blkSizes[b], depths16[d], sc, rng, add_residual_16_sse4, quiet))
              ok = false;
          }

    if (!quiet) {
      printf("ran %d add_residual comparison cases: %s\n", nCases, ok ? "all passed" : "FAILURES");
    }
    return ok;
#else
    if (!quiet) {
      printf("SSE4.1 not available in this build -- nothing to compare, skipping.\n");
    }
    return true;
#endif
  }
} addResidualTest;
