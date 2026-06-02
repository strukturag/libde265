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
#if HAVE_SSE4_1
#include "libde265/x86/sse-dct.h"
#endif
#if HAVE_AVX2
#include "libde265/x86/transform-avx2.h"
#endif

#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>

namespace {

struct RNG {
  uint32_t s;
  explicit RNG(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() { s ^= s<<13; s ^= s>>17; s ^= s<<5; return s; }
  int range(int lo, int hi) { return lo + (int)(next() % (uint32_t)(hi-lo+1)); }
};

double now_sec() { struct timeval tv; gettimeofday(&tv,0); return tv.tv_sec + tv.tv_usec*1e-6; }

typedef void (*tfunc)(uint8_t*, const int16_t*, ptrdiff_t);

#if HAVE_AVX2

const int STRIDE = 32; // 16-byte aligned rows (SSE kernels use aligned stores)

// Fill a coefficient block. Most HEVC blocks are sparse with small values, but
// we also stress full-range inputs to exercise the intermediate int16 clipping.
void fillCoeffs(int16_t* c, int nT, int scenario, RNG& rng) {
  const int N = nT*nT;
  for (int i=0;i<N;i++) c[i]=0;
  if (scenario==0) {                       // sparse, small (typical)
    int nnz = rng.range(1, N/8 + 1);
    for (int k=0;k<nnz;k++) c[rng.range(0,N-1)] = (int16_t)rng.range(-512,512);
  } else if (scenario==1) {                // dense moderate
    for (int i=0;i<N;i++) c[i]=(int16_t)rng.range(-2048,2048);
  } else {                                 // full int16 range (saturation stress)
    for (int i=0;i<N;i++) c[i]=(int16_t)rng.range(-32768,32767);
  }
}

// one comparison: run reference and candidate on identical inputs, compare.
bool runCase(int nT, tfunc ref, tfunc cand, int scenario, RNG& rng, const char* tag, bool quiet) {
  alignas(32) int16_t coeffs[32*32];
  alignas(32) uint8_t dstRef[32*STRIDE];
  alignas(32) uint8_t dstCand[32*STRIDE];

  fillCoeffs(coeffs, nT, scenario, rng);
  for (int i=0;i<nT*STRIDE;i++) { uint8_t p=(uint8_t)rng.range(0,255); dstRef[i]=p; dstCand[i]=p; }

  ref (dstRef,  coeffs, STRIDE);
  cand(dstCand, coeffs, STRIDE);

  for (int i=0;i<nT*STRIDE;i++) {
    if (dstRef[i]!=dstCand[i]) {
      if (!quiet)
        printf("  MISMATCH %s nT=%d scen=%d at (x=%d,y=%d): ref=%d cand=%d\n",
               tag, nT, scenario, (int)(i%STRIDE),(int)(i/STRIDE),(int)dstRef[i],(int)dstCand[i]);
      return false;
    }
  }
  return true;
}

double timeFunc(int nT, tfunc f, RNG& rng) {
  alignas(32) int16_t coeffs[32*32];
  alignas(32) uint8_t dst[32*STRIDE];
  fillCoeffs(coeffs, nT, 1, rng);
  for (int i=0;i<nT*STRIDE;i++) dst[i]=(uint8_t)rng.range(0,255);
  const int iters = 200000;
  f(dst, coeffs, STRIDE); // warmup
  double t0=now_sec();
  for (int i=0;i<iters;i++) f(dst, coeffs, STRIDE);
  double t1=now_sec();
  return (t1-t0)/iters*1e9;
}

#endif // HAVE_AVX2

} // namespace


class TransformAVX2Test : public Test
{
public:
  const char* getName() const { return "transform-avx2"; }
  const char* getDescription() const {
    return "compare AVX2 inverse transforms against SSE/scalar, and benchmark";
  }

  bool work(bool quiet) {
#if HAVE_AVX2 && HAVE_SSE4_1
    RNG rng(0xBEEF1234);
    bool ok = true;
    int n = 0;

    struct Kern { int nT; tfunc scalar, sse, avx2; const char* name; };
    Kern kerns[] = {
      {16, transform_16x16_add_8_fallback, ff_hevc_transform_16x16_add_8_sse4, transform_16x16_add_8_avx2, "16x16"},
      {32, transform_32x32_add_8_fallback, ff_hevc_transform_32x32_add_8_sse4, transform_32x32_add_8_avx2, "32x32"},
    };

    for (auto& k : kerns)
      for (int scen=0; scen<3; scen++)
        for (int rep=0; rep<20; rep++) {
          n++;
          if (!runCase(k.nT, k.sse,    k.avx2, scen, rng, "avx2-vs-sse",    quiet)) ok=false;
          if (!runCase(k.nT, k.scalar, k.avx2, scen, rng, "avx2-vs-scalar", quiet)) ok=false;
        }

    if (!quiet) {
      printf("ran %d transform comparison cases: %s\n\n", n, ok?"all passed":"FAILURES");
      printf("inverse-transform timing, ns/call:\n");
      for (auto& k : kerns) {
        double s = timeFunc(k.nT, k.scalar, rng);
        double e = timeFunc(k.nT, k.sse,    rng);
        double a = timeFunc(k.nT, k.avx2,   rng);
        printf("  idct %s  scalar=%8.2f  sse=%8.2f  avx2=%8.2f   avx2-vs-sse x%.2f\n",
               k.name, s, e, a, e/a);
      }
    }
    return ok;
#else
    if (!quiet) printf("AVX2/SSE4.1 not available in this build -- skipping.\n");
    return true;
#endif
  }
} transformAVX2Test;
