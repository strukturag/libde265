

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

//#include "cpucounters.h"

#include "libde265/util.h"

#define D 1

#if D
#define Deb(x) if (D) { print128(x); printf(" " #x "\n"); }
#else
#define Deb(x)
#endif

void print128(__m128i m);
void print128(__m128i m,int w);


void put_weighted_pred_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                  const int16_t *src, ptrdiff_t srcstride,
                                  int width, int height,
                                  int w,int o,int log2WD)
{
  assert(log2WD>=1); // TODO

  const int rnd = (1<<(log2WD-1));

  for (int y=0;y<height;y++) {
    const int16_t* in  = &src[y*srcstride];
    uint8_t* out = &dst[y*dststride];

    for (int x=0;x<width;x++) {
      out[0] = Clip1_8bit(((in[0]*w + rnd)>>log2WD) + o);
      out++; in++;
    }
  }
}


void put_weighted_pred_8_w8_sse4(uint8_t *dst, ptrdiff_t dststride,
                                 const int16_t *src, ptrdiff_t srcstride,
                                 int height,
                                 int w,int o,int log2WD)
{
  //__m128i rnd = _mm_set1_epi16( 1<<(log2WD-1) );
  //Deb(rnd);

  __m128i flat_w = _mm_set1_epi16( w<<(15-log2WD) );
  Deb(flat_w);

  __m128i offset = _mm_set1_epi16( o );
  Deb(offset);

  for (int y=0;y<height;y++) {
    __m128i in = _mm_load_si128((const __m128i*)(&src[y*srcstride]));
    Deb(in);

    __m128i mul = _mm_mulhrs_epi16(in, flat_w);
    Deb(mul);

    __m128i result16 = _mm_add_epi16(mul,offset);
    Deb(result16);

    __m128i result8  = _mm_packus_epi16(result16, result16);
    Deb(result8);

    _mm_storel_epi64 ((__m128i*)(dst+y*dststride), result8);
  }
}


void put_weighted_bipred_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                    const int16_t *src1, const int16_t *src2, ptrdiff_t srcstride,
                                    int width, int height,
                                    int w1,int o1, int w2,int o2, int log2WD)
{
  assert(log2WD>=1); // TODO

  const int rnd = ((o1+o2+1) << log2WD);

  for (int y=0;y<height;y++) {
    const int16_t* in1 = &src1[y*srcstride];
    const int16_t* in2 = &src2[y*srcstride];
    uint8_t* out = &dst[y*dststride];

    for (int x=0;x<width;x++) {
      out[0] = Clip1_8bit((in1[0]*w1 + in2[0]*w2 + rnd)>>(log2WD+1));
      out++; in1++; in2++;
    }
  }
}


void motion()
{
  const int w = 8;

  int imgw=640,imgh=320;

  int16_t src[imgw*imgh];

  uint8_t dstc_raw[imgw*imgh],dstsse_raw[imgw*imgh];

  uint8_t* dstc   = dstc_raw + 0;
  uint8_t* dstsse = dstsse_raw + 0;

  for (int i=0;i<imgw*imgh;i++) {
    src[i]=(i + (i/imgw)) & 0xFF;
  }


  //if (!D)
  {
    srand(time(0));
  }

  for (int i=0;i<w;i++) {
    printf("%02d: ",i);
    for (int x=0;x<w;x++) {
      printf("%02x ",src[i*imgw+x]);
    }
    printf("\n");
  }

  printf("\n");


  int param_w = 1;
  int param_o =  0;
  int log2WD  =  6;

  const uint64_t nIter = (D ? 1 : 1000000000);
  for (int i=0;i<nIter;i++) {
    if (w==4) {
      //intra_dc_noavg_8_4x4_fallback(dstc,imgw, border+2*w);
      //intra_dc_sse_8bit_4x4<false>(dstsse,imgw, border+2*w);

      //intra_dc_avg_8_4x4_fallback(dstc,imgw, border+2*w);
      //intra_dc_sse_8bit_4x4<true>(dstsse,imgw, border+2*w);
    }

    if (w==8) {
      //intra_dc_noavg_8_8x8_fallback(dstc,imgw, border+2*w);
      //intra_dc_noavg_8_8x8_sse4(dstc,imgw, border+2*w);

      //intra_dc_avg_8_8x8_fallback(dstc,imgw, border+2*w);
      put_weighted_pred_8_fallback(dstc, imgw,
                                   src, imgw,
                                   w,w, param_w, param_o, log2WD);

      put_weighted_pred_8_w8_sse4(dstsse, imgw,
                                  src, imgw,
                                  w, param_w, param_o, log2WD);
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
