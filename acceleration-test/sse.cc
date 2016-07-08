
#include "libde265/x86/sse-sao.h"
#include "libde265/fallback-sao.h"

#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

const bool D=0;


void intra_dc();


void print128(__m128i m)
{
  for (int i=0;i<16;i++) {
    uint8_t v = ((uint8_t*)&m)[15-i];
    printf("%02x",v);
  }
}

void print128(__m128i m,int w)
{
  for (int i=0;i<16;i++) {
    if (i && (i%w)==0) printf(" ");

    uint8_t v = ((uint8_t*)&m)[i];
    printf("%02x",v);
  }
}



void sao()
{
  const int w = 16;

  uint8_t blk[w*w]; // must be 16-byte aligned
  uint8_t dstc[w*w],dstsse[w*w];

  for (int i=0;i<w*w;i++) {
    blk[i] = i;
    dstc[i] = i;
  }

  if (D)
    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      print128(*(__m128i*)&blk[i*w],1);
      printf("\n");
    }

  for (int i=0;i<10000;i++) {
    for (int j=0;j<1000;j++) {
      sao_band_sse_8bit     (dstsse,16, blk,16, 16,16,  31, 10,-10,5,-5);
      //sao_band_fallback_8bit(dstc,  16, blk,16, 16,16,  31, 10,-10,5,-5);
    }
  }

  for (int i=0;i<w*w;i++) {
    //assert(dstsse[i] == dstc[i]);
  }

  if (1) {
    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      print128(*(__m128i*)&dstsse[i*w],1);
      printf("\n");
    }

    printf("\n");

    for (int i=0;i<w;i++) {
      printf("%02d: ",i);
      print128(*(__m128i*)&dstc[i*w],1);
      printf("\n");
    }
  }
}


int main()
{
  //sao();
  intra_dc();

  return 0;
}
