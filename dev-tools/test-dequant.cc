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

#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <sys/time.h>

namespace {

struct RNG {
  uint32_t s;
  explicit RNG(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() { s ^= s<<13; s ^= s>>17; s ^= s<<5; return s; }
  int range(int lo, int hi) { return lo + (int)(next() % (uint32_t)(hi-lo+1)); }
};

double now_sec() { struct timeval tv; gettimeofday(&tv,0); return tv.tv_sec + tv.tv_usec*1e-6; }

static int clip16(int v){ return v < -32768 ? -32768 : (v > 32767 ? 32767 : v); }

// reference == the original int64 scalar loop in scale_coefficients_internal()
void dequant_ref(int16_t* buf, const int16_t* list, const int16_t* pos, int n,
                 int32_t fact, int32_t offset, int32_t bdShift) {
  for (int i=0;i<n;i++) {
    int64_t c = list[i];
    c = clip16((int)((c*fact + offset) >> bdShift));
    buf[ pos[i] ] = (int16_t)c;
  }
}

typedef void (*dqfunc)(int16_t*, const int16_t*, const int16_t*, int, int32_t, int32_t, int32_t);

// build a sparse coefficient list for an nT x nT block with `nnz` nonzeros at
// distinct positions; levels mostly small with occasional large values.
void build(int nT, int nnz, RNG& rng, std::vector<int16_t>& list, std::vector<int16_t>& pos) {
  const int N = nT*nT;
  std::vector<uint8_t> used(N, 0);
  list.clear(); pos.clear();
  for (int k=0;k<nnz;k++) {
    int p;
    do { p = rng.range(0, N-1); } while (used[p]);
    used[p]=1;
    pos.push_back((int16_t)p);
    int lvl = (rng.range(0,9)==0) ? rng.range(-32768,32767) : rng.range(-300,300);
    list.push_back((int16_t)lvl);
  }
}

#if HAVE_SSE4_1
bool runCase(int nT, int qP, int nnz, RNG& rng, dqfunc cand, bool quiet) {
  const int N = nT*nT;
  const int levelScale[] = {40,45,51,57,64,72};
  int bdShift = 8 + (nT==4?2:nT==8?3:nT==16?4:5) - 9;  // 8-bit
  int offset  = 1 << (bdShift-1);
  int fact    = levelScale[qP%6] << (qP/6);

  std::vector<int16_t> list, pos;
  build(nT, nnz, rng, list, pos);

  std::vector<int16_t> bufRef(N,0), bufCand(N,0);
  dequant_ref(bufRef.data(), list.data(), pos.data(), nnz, fact, offset, bdShift);
  cand        (bufCand.data(),list.data(), pos.data(), nnz, fact, offset, bdShift);

  for (int i=0;i<N;i++)
    if (bufRef[i]!=bufCand[i]) {
      if (!quiet) printf("  MISMATCH nT=%d qP=%d nnz=%d at pos=%d: ref=%d cand=%d\n",
                         nT,qP,nnz,i,(int)bufRef[i],(int)bufCand[i]);
      return false;
    }
  return true;
}

double timeFunc(int nT, int nnz, dqfunc f, RNG& rng) {
  const int levelScale[] = {40,45,51,57,64,72};
  int qP=32;
  int bdShift = 8 + (nT==4?2:nT==8?3:nT==16?4:5) - 9;
  int offset  = 1 << (bdShift-1);
  int fact    = levelScale[qP%6] << (qP/6);
  std::vector<int16_t> list, pos;
  build(nT, nnz, rng, list, pos);
  std::vector<int16_t> buf(nT*nT, 0);
  const int iters = 300000;
  f(buf.data(), list.data(), pos.data(), nnz, fact, offset, bdShift);
  double t0=now_sec();
  for (int i=0;i<iters;i++) f(buf.data(), list.data(), pos.data(), nnz, fact, offset, bdShift);
  double t1=now_sec();
  return (t1-t0)/iters*1e9;
}
#endif

} // namespace


class DequantTest : public Test
{
public:
  const char* getName() const { return "dequant"; }
  const char* getDescription() const {
    return "compare SSE / fallback inverse quantization against the int64 reference, and benchmark";
  }

  bool work(bool quiet) {
#if HAVE_SSE4_1
    RNG rng(0xD2C0FFEE);
    bool ok=true; int n=0;
    const int sizes[]={4,8,16,32};
    for (int s=0;s<4;s++)
      for (int qP=0; qP<=51; qP+=3)
        for (int frac=1; frac<=8; frac++) {        // sparsity: nnz = N/frac
          int N=sizes[s]*sizes[s];
          int nnz = N/frac; if (nnz<1) nnz=1;
          n++;
          if (!runCase(sizes[s], qP, nnz, rng, dequant_coeff_block_fallback, quiet)) ok=false;
          if (!runCase(sizes[s], qP, nnz, rng, dequant_coeff_block_sse4,     quiet)) ok=false;
        }

    if (!quiet) {
      printf("ran %d dequant comparison cases: %s\n\n", n, ok?"all passed":"FAILURES");
      printf("inverse-quantization timing, ns/call (32x32 block, varying density):\n");
      const int densities[] = {1024, 256, 64, 16}; // nnz: full, 1/4, 1/16, 1/64
      for (int d=0; d<4; d++) {
        int nnz=densities[d];
        double r = timeFunc(32, nnz, dequant_ref,                   rng);
        double e = timeFunc(32, nnz, dequant_coeff_block_sse4,      rng);
        printf("  nnz=%4d  int64-ref=%8.2f  sse=%8.2f   sse-vs-ref x%.2f\n",
               nnz, r, e, r/e);
      }
    }
    return ok;
#else
    if (!quiet) printf("SSE4.1 not available in this build -- skipping.\n");
    return true;
#endif
  }
} dequantTest;
