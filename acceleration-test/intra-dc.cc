

#include "libde265/x86/sse-intra-dc.h"
#include "libde265/fallback-intra-dc.h"

#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <time.h>

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
  const int w = 16;

  int imgw=640,imgh=320;

  uint8_t dstc[imgw*imgh],dstsse[imgw*imgh];

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

  const uint64_t nIter = (D ? 1 : 1000000000);
  for (int i=0;i<nIter;i++) {
    if (w==4) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 2, false);
      //intra_dc_sse_8bit_4x4<false>(dstsse,imgw, border+2*w);

      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 2, true);
      //intra_dc_sse_8bit_4x4<true>(dstsse,imgw, border+2*w);
    }

    if (w==8) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 3, false);
      //intra_dc_sse_8bit_8x8<false>(dstsse,imgw, border+2*w);

      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 3, true);
      //intra_dc_sse_8bit_8x8<true>(dstsse,imgw, border+2*w);
    }

    if (w==16) {
      //intra_dc_noavg_8_16x16_fallback(dstc,imgw, border+2*w);
      intra_dc_noavg_8_16x16_sse4(dstsse,imgw, border+2*w);

      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 4, true);
      //intra_dc_sse_8bit_16x16<true>(dstsse,imgw, border+2*w);
    }

    if (w==32) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 5, false);
      //intra_dc_sse_noavg_8bit_32x32(dstsse,imgw, border+2*w);
    }

    for (int i=0;i<w*w;i++) {
      //assert(dstsse[i] == dstc[i]);
    }
  }

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
