

#include "libde265/x86/sse-intra-dc.h"
#include "libde265/fallback-intra-dc.h"
#include "iacaMarks.h"

#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <iostream>

#include "cpucounters.h"

#define D 0

#if D
#define Deb(x) if (D) { print128(x); printf(" " #x "\n"); }
#else
#define Deb(x)
#endif

void print128(__m128i m);
void print128(__m128i m,int w);


void intra_dc()
{
  const int w = 8;

  int imgw=640,imgh=320;

  uint8_t dstc_raw[imgw*imgh],dstsse_raw[imgw*imgh];

  uint8_t* dstc   = dstc_raw + 0;
  uint8_t* dstsse = dstsse_raw + 0;

  uint8_t border[4*w+1];
  for (int i=0;i<2*w;i++) {
    border[2*w+1+i] = 3*i+1;
    border[2*w-1-i] = 128 - 3*i+1;
  }
  border[2*w] = 0xC0;


  //if (!D)
  {
    srand(time(0));
  }

  for (int i=0;i<4*w+1;i++) {
    border[i] = rand()&0xFF;
  }

  for (int i=0;i<2*2*w+1;i++) {
    if (i==2*w || i==2*w+1) printf("| ");
    printf("%02x ", border[i]);
  }
  printf("\n");

#if 0
  PCM * m = PCM::getInstance();
  int err;
  m->resetPMU();
  if ((err=m->program()) != PCM::Success) {
    printf("err %d\n",err);
    return;
  }

  SystemCounterState before_sstate = getSystemCounterState();
#endif

  const uint64_t nIter = (D ? 1 : 1000000000);
  for (int i=0;i<nIter;i++) {
    if (w==4) {
      intra_dc_noavg_8_4x4_fallback(dstc,imgw, border+2*w);
      //intra_dc_sse_8bit_4x4<false>(dstsse,imgw, border+2*w);

      //intra_dc_avg_8_4x4_fallback(dstc,imgw, border+2*w);
      //intra_dc_sse_8bit_4x4<true>(dstsse,imgw, border+2*w);
    }

    if (w==8) {
      //intra_dc_noavg_8_8x8_fallback(dstc,imgw, border+2*w);
      //intra_dc_noavg_8_8x8_sse4(dstc,imgw, border+2*w);

      //intra_dc_avg_8_8x8_fallback(dstc,imgw, border+2*w);
      //intra_dc_avg_8_8x8_sse4(dstc,imgw, border+2*w);
    }

    if (w==16) {
      //intra_dc_noavg_8_16x16_fallback(dstc,imgw, border+2*w);
      //intra_dc_noavg_8_16x16_sse4(dstsse,imgw, border+2*w);

      //intra_dc_avg_8_16x16_fallback(dstc,imgw, border+2*w);
      //intra_dc_avg_8_16x16_sse4(dstsse,imgw, border+2*w);
    }

    if (w==32) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 5, false);
      //intra_dc_sse_noavg_8bit_32x32(dstsse,imgw, border+2*w);
    }

    for (int i=0;i<w*w;i++) {
      //assert(dstsse[i] == dstc[i]);
    }
  }

#if 0
  SystemCounterState after_sstate = getSystemCounterState();
  std::cout << "Instructions per clock:" << getIPC(before_sstate,after_sstate)
            << " L3 cache hit ratio:" << getL3CacheHitRatio(before_sstate,after_sstate)
            << " Bytes read:" << getBytesReadFromMC(before_sstate,after_sstate)
            << " cycles: " << getCycles(before_sstate,after_sstate)
            << " instr: " << getInstructionsRetired(before_sstate,after_sstate)
            << "\n";
#endif


  if (1) {
    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      for (int x=0;x<w;x++) {
        printf("%02x ",dstsse[i*imgw+x]);
      }
      printf("\n");
    }

    printf("\n");

    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      for (int x=0;x<w;x++) {
        printf("%02x ",dstc[i*imgw+x]);
      }
      printf("\n");
    }
  }
}
