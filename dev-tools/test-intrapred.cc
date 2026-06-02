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

#include "libde265/fallback-intrapred.h"

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <sys/time.h>

// The SSE intra-prediction kernels only exist when the library was built with
// SSE4.1 support. Without it there is nothing to compare against / benchmark, so
// the test registers itself as a no-op (always passing).
#if HAVE_SSE4_1
#include "libde265/x86/sse-intrapred.h"
#endif


namespace {

// Deterministic PRNG (xorshift32) so the test is fully reproducible.
struct RNG {
  uint32_t s;
  explicit RNG(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() { s ^= s<<13; s ^= s>>17; s ^= s<<5; return s; }
  int range(int lo, int hi) { return lo + (int)(next() % (uint32_t)(hi-lo+1)); }
};

const int BORDER_MAX = 64;  // == MAX_INTRA_PRED_BLOCK_SIZE

#if HAVE_SSE4_1

double now_sec() { struct timeval tv; gettimeofday(&tv,0); return tv.tv_sec + tv.tv_usec*1e-6; }

const char* modeName(int mode) {
  if (mode==0) return "planar";
  if (mode==1) return "DC";
  return "angular";
}

// Build a fully-initialized random border buffer (centered at index 0, valid
// range [-2*BORDER_MAX, 2*BORDER_MAX], same layout the decoder uses).
void fillBorder(std::vector<uint8_t>& bmem, RNG& rng) {
  bmem.resize(4*BORDER_MAX+1);
  for (size_t i=0;i<bmem.size();i++) bmem[i] = (uint8_t)rng.range(0,255);
}

void runKernelRef(int mode, uint8_t* dst, int stride, int nT, int cIdx, int disable, const uint8_t* border) {
  if      (mode==0) intra_pred_planar_fallback<uint8_t>(dst,stride,nT,cIdx,border);
  else if (mode==1) intra_pred_dc_fallback<uint8_t>(dst,stride,nT,cIdx,border);
  else              intra_pred_angular_fallback<uint8_t>(dst,stride,8,disable,0,0,mode,nT,cIdx,border);
}

void runKernelSSE(int mode, uint8_t* dst, int stride, int nT, int cIdx, int disable, const uint8_t* border) {
  if      (mode==0) intra_pred_planar_8_sse4(dst,stride,nT,cIdx,border);
  else if (mode==1) intra_pred_dc_8_sse4(dst,stride,nT,cIdx,border);
  else              intra_pred_angular_8_sse4(dst,stride,8,disable,0,0,mode,nT,cIdx,border);
}

// One comparison case: identical inputs through scalar reference and SSE, then
// verify the whole strided buffer matches (also catches out-of-region writes).
bool runCase(int mode, int nT, int cIdx, int disable, RNG& rng, bool quiet)
{
  const int stride = nT + 17;   // non-trivial stride: padding after each row

  std::vector<uint8_t> bmem;  fillBorder(bmem, rng);
  const uint8_t* border = &bmem[2*BORDER_MAX];

  std::vector<uint8_t> dstRef((size_t)nT*stride), dstSSE((size_t)nT*stride);
  for (size_t i=0;i<dstRef.size();i++) { uint8_t p=(uint8_t)rng.range(0,255); dstRef[i]=p; dstSSE[i]=p; }

  runKernelRef(mode, dstRef.data(), stride, nT, cIdx, disable, border);
  runKernelSSE(mode, dstSSE.data(), stride, nT, cIdx, disable, border);

  for (size_t i=0;i<dstRef.size();i++) {
    if (dstRef[i] != dstSSE[i]) {
      if (!quiet) {
        printf("  MISMATCH mode=%d(%s) nT=%d cIdx=%d disable=%d at (x=%d,y=%d): ref=%d sse=%d\n",
               mode, modeName(mode), nT, cIdx, disable,
               (int)(i%stride), (int)(i/stride), (int)dstRef[i], (int)dstSSE[i]);
      }
      return false;
    }
  }
  return true;
}

// Time scalar vs SSE for a fixed kernel configuration; returns ns/call via refs.
volatile uint8_t g_sink = 0;

void timeMode(int mode, int nT, int cIdx, RNG& rng, double& nsRef, double& nsSSE)
{
  const int stride = nT + 17;
  std::vector<uint8_t> bmem;  fillBorder(bmem, rng);
  const uint8_t* border = &bmem[2*BORDER_MAX];
  std::vector<uint8_t> dst((size_t)nT*stride, 0);

  // For angular we average the timing over all 33 angular modes (2..34).
  const int mlo = (mode>=2) ? 2  : mode;
  const int mhi = (mode>=2) ? 34 : mode;
  const int nModes = mhi-mlo+1;

  // iteration count chosen so each measurement runs long enough to be stable
  const int iters = 40000;

  // scalar
  for (int m=mlo;m<=mhi;m++) runKernelRef(m, dst.data(), stride, nT, cIdx, 0, border); // warmup
  double t0 = now_sec();
  for (int i=0;i<iters;i++)
    for (int m=mlo;m<=mhi;m++) runKernelRef(m, dst.data(), stride, nT, cIdx, 0, border);
  double t1 = now_sec();
  g_sink ^= dst[0];
  nsRef = (t1-t0) / ((double)iters*nModes) * 1e9;

  // SSE
  for (int m=mlo;m<=mhi;m++) runKernelSSE(m, dst.data(), stride, nT, cIdx, 0, border); // warmup
  double t2 = now_sec();
  for (int i=0;i<iters;i++)
    for (int m=mlo;m<=mhi;m++) runKernelSSE(m, dst.data(), stride, nT, cIdx, 0, border);
  double t3 = now_sec();
  g_sink ^= dst[0];
  nsSSE = (t3-t2) / ((double)iters*nModes) * 1e9;
}

#endif // HAVE_SSE4_1

} // namespace


class IntraPredTest : public Test
{
public:
  const char* getName() const { return "intra-prediction"; }
  const char* getDescription() const {
    return "compare SSE intra prediction (DC/planar/angular) against scalar fallback, and benchmark";
  }

  bool work(bool quiet) {
#if HAVE_SSE4_1
    RNG rng(0x1234ABCD);
    const int blkSizes[] = {4, 8, 16, 32};
    const int repeats = 4;
    bool ok = true;
    int nCases = 0;

    // --- correctness: every mode x block-size x component x boundary-filter flag ---
    for (int b=0;b<4;b++)
      for (int mode=0;mode<=34;mode++)
        for (int cIdx=0;cIdx<=1;cIdx++)
          for (int disable=0;disable<=1;disable++)
            for (int rep=0;rep<repeats;rep++) {
              nCases++;
              if (!runCase(mode, blkSizes[b], cIdx, disable, rng, quiet))
                ok = false;
            }

    if (!quiet) {
      printf("ran %d intra-prediction comparison cases: %s\n\n", nCases, ok ? "all passed" : "FAILURES");

      // --- benchmark: scalar vs SSE, separately for DC / planar / angular ---
      printf("per-kernel timing (luma, cIdx=0), ns/call:\n");
      const int modes[]  = {1, 0, 2};   // DC, planar, angular(avg 2..34)
      for (int mi=0; mi<3; mi++) {
        for (int b=0;b<4;b++) {
          double nsRef=0, nsSSE=0;
          timeMode(modes[mi], blkSizes[b], 0, rng, nsRef, nsSSE);
          const char* tag = (modes[mi]==1)?"DC     " : (modes[mi]==0)?"planar " : "angular";
          printf("  intra %s nT=%2d  scalar=%8.2f  sse=%8.2f  speedup x%.2f%s\n",
                 tag, blkSizes[b], nsRef, nsSSE, nsRef/nsSSE,
                 (modes[mi]==2)?"  (avg modes 2..34)":"");
        }
      }
    }
    return ok;
#else
    if (!quiet) {
      printf("SSE4.1 not available in this build -- nothing to compare, skipping.\n");
    }
    return true;
#endif
  }
} intraPredTest;
