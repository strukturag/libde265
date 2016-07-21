
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
void motion();


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


void memory()
{
  uint64_t size = 10000000;

  uint8_t* m = new uint8_t[size];
  printf("%p\n",m);

  uint64_t pos = 0;
  for (int k=0;k<100;k++)
    for (int i=0;i<10000000;i++) {
      pos = (pos*173+58) % size;

      m[pos] = i;
    }

  delete[] m;
}


void matrix()
{
  int w = 32768;
  int32_t* mm = new int32_t[w*w+64];
  int32_t* m = (int32_t*)(((uint64_t)mm+63) & ~0x3F); // align to 64-byte boundary (cache line)
  printf("%p %p\n",m,mm);

  for (int x=0;x<w;x+=32)
    for (int y=0;y<w;y++)
      {
        for (int i=0;i<16;i++) {
          _mm_stream_si32(&m[x+y*w+i], 1);

          //m[x+y*w+i] = 1;
        }
      }

  delete[] mm;
}


int main()
{
  //sao();
  //intra_dc();
  motion();

  //memory();
  //matrix();

  return 0;
}
