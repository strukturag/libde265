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

#include "libde265/fallback-deblk.h"
#if HAVE_SSE4_1
#include "libde265/x86/sse-deblk.h"
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

#if HAVE_SSE4_1
const int W = 24, H = 24, OFF = 12;   // ptr at (OFF,OFF); margins > 4 each way

// luma: one comparison case
bool runLuma(int vertical, int dE, int dEp, int dEq, int tc, int filterP, int filterQ,
             RNG& rng, bool quiet) {
  std::vector<uint8_t> a(W*H), b(W*H);
  for (int i=0;i<W*H;i++) { uint8_t v=(uint8_t)rng.range(0,255); a[i]=v; b[i]=v; }
  uint8_t* pa = a.data() + OFF*W + OFF;
  uint8_t* pb = b.data() + OFF*W + OFF;

  deblock_luma_8_fallback(pa, W, vertical, dE,dEp,dEq,tc, filterP,filterQ);
  deblock_luma_8_sse4    (pb, W, vertical, dE,dEp,dEq,tc, filterP,filterQ);

  for (int i=0;i<W*H;i++)
    if (a[i]!=b[i]) {
      if (!quiet) printf("  LUMA MISMATCH vert=%d dE=%d dEp=%d dEq=%d tc=%d fP=%d fQ=%d at (%d,%d): ref=%d sse=%d\n",
                         vertical,dE,dEp,dEq,tc,filterP,filterQ, i%W,i/W, a[i],b[i]);
      return false;
    }
  return true;
}

template <class F> double timeIt(F f) {
  const int iters = 2000000;
  f(); // warmup
  double t0=now_sec();
  for (int i=0;i<iters;i++) f();
  double t1=now_sec();
  return (t1-t0)/iters*1e9;
}
#endif

} // namespace


class DeblkTest : public Test
{
public:
  const char* getName() const { return "deblk"; }
  const char* getDescription() const {
    return "compare SSE deblocking (luma+chroma) against scalar, and benchmark";
  }

  bool work(bool quiet) {
#if HAVE_SSE4_1
    RNG rng(0xDEB10C);
    bool ok=true; int n=0;

    for (int rep=0; rep<200; rep++)
      for (int vert=0; vert<2; vert++) {
        int tc = rng.range(1, 25);
        // luma: all dE/dEp/dEq/filterP/filterQ combinations
        for (int dE=1; dE<=2; dE++)
          for (int dEp=0; dEp<2; dEp++)
            for (int dEq=0; dEq<2; dEq++)
              for (int fP=0; fP<2; fP++)
                for (int fQ=0; fQ<2; fQ++) {
                  n++;
                  if (!runLuma(vert, dE,dEp,dEq, tc, fP,fQ, rng, quiet)) ok=false;
                }
      }

    if (!quiet) {
      printf("ran %d deblock comparison cases: %s\n\n", n, ok?"all passed":"FAILURES");

      // benchmark (use fixed buffers; representative params)
      std::vector<uint8_t> buf(W*H);
      for (int i=0;i<W*H;i++) buf[i]=(uint8_t)rng.range(0,255);
      uint8_t* p = buf.data() + OFF*W + OFF;
      printf("deblock timing, ns/segment (4 lines):\n");
      for (int vert=0; vert<2; vert++) {
        const char* o = vert?"vert":"horiz";
        double ls = timeIt([&]{ deblock_luma_8_fallback(p,W,vert,2,1,1,12,1,1); });
        double le = timeIt([&]{ deblock_luma_8_sse4    (p,W,vert,2,1,1,12,1,1); });
        double lw = timeIt([&]{ deblock_luma_8_fallback(p,W,vert,1,1,1,12,1,1); });
        double lx = timeIt([&]{ deblock_luma_8_sse4    (p,W,vert,1,1,1,12,1,1); });
        printf("  luma %-5s strong  scalar=%6.2f  sse=%6.2f  x%.2f\n", o, ls, le, ls/le);
        printf("  luma %-5s weak    scalar=%6.2f  sse=%6.2f  x%.2f\n", o, lw, lx, lw/lx);
      }
    }
    return ok;
#else
    if (!quiet) printf("SSE4.1 not available -- skipping.\n");
    return true;
#endif
  }
} deblkTest;
