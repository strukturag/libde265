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

#include "libde265/cabac.h"
#include "libde265/contextmodel.h"

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
double now_sec(){ struct timeval tv; gettimeofday(&tv,0); return tv.tv_sec + tv.tv_usec*1e-6; }

// --- reference: the ORIGINAL scalar CABAC arithmetic (pre-branchless) ---
const uint8_t REF_LPS[64][4] = {
  {128,176,208,240},{128,167,197,227},{128,158,187,216},{123,150,178,205},
  {116,142,169,195},{111,135,160,185},{105,128,152,175},{100,122,144,166},
  { 95,116,137,158},{ 90,110,130,150},{ 85,104,123,142},{ 81, 99,117,135},
  { 77, 94,111,128},{ 73, 89,105,122},{ 69, 85,100,116},{ 66, 80, 95,110},
  { 62, 76, 90,104},{ 59, 72, 86, 99},{ 56, 69, 81, 94},{ 53, 65, 77, 89},
  { 51, 62, 73, 85},{ 48, 59, 69, 80},{ 46, 56, 66, 76},{ 43, 53, 63, 72},
  { 41, 50, 59, 69},{ 39, 48, 56, 65},{ 37, 45, 54, 62},{ 35, 43, 51, 59},
  { 33, 41, 48, 56},{ 32, 39, 46, 53},{ 30, 37, 43, 50},{ 29, 35, 41, 48},
  { 27, 33, 39, 45},{ 26, 31, 37, 43},{ 24, 30, 35, 41},{ 23, 28, 33, 39},
  { 22, 27, 32, 37},{ 21, 26, 30, 35},{ 20, 24, 29, 33},{ 19, 23, 27, 31},
  { 18, 22, 26, 30},{ 17, 21, 25, 28},{ 16, 20, 23, 27},{ 15, 19, 22, 25},
  { 14, 18, 21, 24},{ 14, 17, 20, 23},{ 13, 16, 19, 22},{ 12, 15, 18, 21},
  { 12, 14, 17, 20},{ 11, 14, 16, 19},{ 11, 13, 15, 18},{ 10, 12, 15, 17},
  { 10, 12, 14, 16},{  9, 11, 13, 15},{  9, 11, 12, 14},{  8, 10, 12, 14},
  {  8,  9, 11, 13},{  7,  9, 11, 12},{  7,  9, 10, 12},{  7,  8, 10, 11},
  {  6,  8,  9, 11},{  6,  7,  9, 10},{  6,  7,  8,  9},{  2,  2,  2,  2}};
const uint8_t REF_renorm[32] = {6,5,4,4,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
const uint8_t REF_nMPS[64] = {
 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
 33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,62,63};
const uint8_t REF_nLPS[64] = {
 0,0,1,2,2,4,4,5,6,7,8,9,9,11,11,12,13,13,15,15,16,16,18,18,19,19,21,21,22,22,23,24,
 24,25,26,26,27,27,28,29,29,30,30,30,31,32,32,33,33,33,34,34,35,35,35,36,36,36,37,37,37,38,38,63};

struct RefCABAC {
  uint32_t range, value; int bits_needed;
  const uint8_t* curr; const uint8_t* end;
  void init(const uint8_t* b, int len) {
    range=510; bits_needed=8; value=0; curr=b; end=b+len;
    if (len>0){ value=(uint32_t)(*curr++)<<8; bits_needed-=8; }
    if (len>1){ value|=(*curr++);            bits_needed-=8; }
  }
  int decode_bit(int& state, int& mps) {
    int decoded_bit;
    int LPS = REF_LPS[state][(range>>6)-4];
    range -= LPS;
    uint32_t scaled_range = range<<7;
    if (value < scaled_range) {
      decoded_bit = mps;
      state = REF_nMPS[state];
      if (scaled_range < (256u<<7)) {
        range = scaled_range>>6; value<<=1; bits_needed++;
        if (bits_needed==0){ bits_needed=-8; if(curr<end) value|=*curr++; }
      }
    } else {
      value -= scaled_range;
      int num_bits = REF_renorm[LPS>>3];
      value<<=num_bits; range = (uint32_t)LPS<<num_bits;
      decoded_bit = 1-mps;
      if (state==0) mps = 1-mps;
      state = REF_nLPS[state];
      bits_needed += num_bits;
      if (bits_needed>=0){ if(curr<end) value|=(uint32_t)(*curr++)<<bits_needed; bits_needed-=8; }
    }
    return decoded_bit;
  }
  int decode_bypass() {
    value<<=1; bits_needed++;
    if (bits_needed>=0){ bits_needed=-8; if(end>curr) value|=*curr++; }
    uint32_t scaled_range = range<<7;
    if (value>=scaled_range){ value-=scaled_range; return 1; }
    return 0;
  }
  int decode_term() {
    range-=2; uint32_t sr=range<<7;
    if (value>=sr) return 1;
    if (sr<(256u<<7)){ range=sr>>6; value*=2; bits_needed++;
      if (bits_needed==0){ bits_needed=-8; if(curr<end) value+=*curr++; } }
    return 0;
  }
};

} // namespace


class CabacTest : public Test
{
public:
  const char* getName() const { return "cabac"; }
  const char* getDescription() const {
    return "verify the branchless CABAC decoder bin-by-bin against the original scalar algorithm";
  }

  bool work(bool quiet) {
    const int N = 400;               // number of context models
    const int LEN = 4*1024*1024;     // random bitstream
    RNG rng(0x5AB0C123);

    std::vector<uint8_t> buf(LEN);
    for (int i=0;i<LEN;i++) buf[i]=(uint8_t)rng.next();

    // init both decoders with identical context state
    CABAC_decoder dec;
    dec.init(buf.data(), LEN);
    dec.init_CABAC();
    std::vector<context_model> models(N);
    std::vector<int> rstate(N), rmps(N);
    RefCABAC ref; ref.init(buf.data(), LEN);
    for (int i=0;i<N;i++) {
      int st = rng.range(0,62), m = rng.range(0,1);
      models[i].state = st; models[i].MPSbit = m;
      rstate[i]=st; rmps[i]=m;
    }

    const int NOPS = 6*1000*1000;
    bool ok = true;
    int mismatches = 0;
    for (int i=0;i<NOPS && mismatches<5;i++) {
      int r = (int)(rng.next() % 100);
      int got, want;
      if (r < 90) {                          // decode_bit (the changed path)
        int idx = rng.range(0,N-1);
        got  = dec.decode_bit(&models[idx]);
        want = ref.decode_bit(rstate[idx], rmps[idx]);
      } else {                               // bypass (shares the same bit buffer)
        got  = dec.decode_bypass();
        want = ref.decode_bypass();
      }
      if (got != want) {
        if (!quiet) printf("  MISMATCH at op %d: got %d want %d\n", i, got, want);
        ok = false; mismatches++;
      }
    }

    // final context-state must also match exactly
    for (int i=0;i<N;i++) {
      if (models[i].state != rstate[i] || models[i].MPSbit != rmps[i]) {
        if (!quiet) printf("  STATE MISMATCH ctx %d: sse(s%d,m%d) ref(s%d,m%d)\n",
                           i, models[i].state, models[i].MPSbit, rstate[i], rmps[i]);
        ok = false;
      }
    }

    if (!quiet) {
      printf("decoded %d bins, context state %s\n", NOPS, ok?"all match":"MISMATCH");
      // throughput
      CABAC_decoder d2; d2.init(buf.data(),LEN); d2.init_CABAC();
      std::vector<context_model> m2(N);
      for (int i=0;i<N;i++){ m2[i].state=rng.range(0,62); m2[i].MPSbit=rng.range(0,1); }
      const int BINS = 20*1000*1000;
      double t0=now_sec();
      volatile int sink=0;
      for (int i=0;i<BINS;i++) sink ^= d2.decode_bit(&m2[i*2654435761u % N]);
      double t1=now_sec();
      printf("decode_bit throughput: %.2f Mbins/s (%.2f ns/bin)\n",
             BINS/(t1-t0)/1e6, (t1-t0)/BINS*1e9);
    }
    return ok;
  }
} cabacTest;
