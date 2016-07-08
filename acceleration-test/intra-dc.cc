

#include "libde265/x86/sse-sao.h"
#include "libde265/fallback-sao.h"

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


// 2.18x faster
void __attribute__ ((noinline)) intra_dc_see_noavg_8bit_4x4(uint8_t* dst,int dstStride, uint8_t* border)
{
  __m128i zero   = _mm_setzero_si128();
  __m128i ones16 = _mm_set_epi16(1,1,1,1,0,0,0,0);
  __m128i rawborder = _mm_loadu_si128((const __m128i*)(border-4));
  Deb(rawborder);

#if 1
  // seems to be slightly faster than the version using unpacklo() below
  __m128i mask = _mm_set_epi8(0,0,0,0, 0,0,0, 0xFF,0xFF,0xFF,0xFF,0,0xFF,0xFF,0xFF,0xFF);
  __m128i maskedborder = _mm_and_si128(rawborder, mask);
  __m128i border16 = _mm_shuffle_epi8(maskedborder,
                                      _mm_set_epi8(15,0,15,1,15,2,15,3,15,5,15,6,15,7,15,8));
  Deb(maskedborder);
#endif

#if 0
  __m128i compactborder = _mm_shuffle_epi8(rawborder,
                                           _mm_set_epi8(0,0,0,0,0,0,0,0, 0,1,2,3,5,6,7,8));
  __m128i border16  = _mm_unpacklo_epi8(compactborder ,zero);

  Deb(compactborder);
#endif

  Deb(border16);
  Deb(ones16);

  __m128i bordersum = _mm_add_epi16(border16, ones16);

  Deb(bordersum);

  __m128i dcsum4 = _mm_hadd_epi16(bordersum, zero);
  __m128i dcsum2 = _mm_hadd_epi16(dcsum4, zero);
  __m128i dcsum1 = _mm_hadd_epi16(dcsum2, zero);

  Deb(dcsum4);
  Deb(dcsum2);
  Deb(dcsum1);

  const int log_nT = 2;
  __m128i dcsum = _mm_srai_epi16(dcsum1, log_nT+1);

  Deb(dcsum);

  __m128i flatdc = _mm_shuffle_epi8(dcsum, zero);

  Deb(flatdc);

  uint32_t flatdc32 = _mm_cvtsi128_si32(flatdc);

  for (int y=0;y<4;y++) {
    *(uint32_t*)(dst+y*dstStride) = flatdc32;
  }
}



// 27x faster
void __attribute__ ((noinline)) intra_dc_see_noavg_8bit_8x8(uint8_t* dst,int dstStride, uint8_t* border)
{
  __m128i zero   = _mm_setzero_si128();
  __m128i ones16 = _mm_set1_epi16(1);
  __m128i leftborder = _mm_loadl_epi64((const __m128i*)(border-8));

  __m128i topborder  = _mm_loadl_epi64((const __m128i*)(border+1));
  //__m128i topborder  = _mm_load_si128((const __m128i*)(border));
  //topborder = _mm_srli_si128(topborder, 1);

  Deb(topborder);
  Deb(leftborder);
  Deb(ones16);

  __m128i topborder16  = _mm_unpacklo_epi8(topborder ,zero);
  __m128i leftborder16 = _mm_unpacklo_epi8(leftborder,zero);

  Deb(topborder16);
  Deb(leftborder16);

  __m128i bordersum_a = _mm_add_epi16(topborder16, leftborder16);
  __m128i bordersum_b = _mm_add_epi16(bordersum_a, ones16);

  Deb(bordersum_a);
  Deb(bordersum_b);

  __m128i dcsum4 = _mm_hadd_epi16(bordersum_b, zero);
  __m128i dcsum2 = _mm_hadd_epi16(dcsum4, zero);
  __m128i dcsum1 = _mm_hadd_epi16(dcsum2, zero);

  Deb(dcsum4);
  Deb(dcsum2);
  Deb(dcsum1);

  const int log_nT = 3;
  __m128i dcsum = _mm_srai_epi16(dcsum1, log_nT+1);

  Deb(dcsum);

  __m128i flatdc = _mm_shuffle_epi8(dcsum, zero);

  Deb(flatdc);

  for (int y=0;y<8;y++) {
    _mm_storel_epi64((__m128i*)(dst+y*dstStride), flatdc);
  }
}


// 4.64x faster
void __attribute__ ((noinline)) intra_dc_see_noavg_8bit_16x16(uint8_t* dst,int dstStride, uint8_t* border)
{
  __m128i zero   = _mm_setzero_si128();
  __m128i ones16 = _mm_set1_epi16(2);
  __m128i leftborder = _mm_load_si128((const __m128i*)(border-16));
  __m128i topborder  = _mm_loadu_si128((const __m128i*)(border+1));
  //__m128i topborder  = _mm_load_si128((const __m128i*)(border));
  //topborder = _mm_srli_si128(topborder, 1);

  Deb(topborder);
  Deb(leftborder);
  Deb(ones16);

  __m128i topborder16_lo  = _mm_unpacklo_epi8(topborder ,zero);
  __m128i leftborder16_lo = _mm_unpacklo_epi8(leftborder,zero);

  __m128i topborder16_hi  = _mm_unpackhi_epi8(topborder ,zero);
  __m128i leftborder16_hi = _mm_unpackhi_epi8(leftborder,zero);

  Deb(topborder16_lo);
  Deb(leftborder16_lo);
  Deb(topborder16_hi);
  Deb(leftborder16_hi);

  __m128i bordersum_lo = _mm_add_epi16(topborder16_lo, leftborder16_lo);
  __m128i bordersum_hi = _mm_add_epi16(topborder16_hi, leftborder16_hi);
  __m128i bordersum_a = _mm_add_epi16(bordersum_lo, bordersum_hi);
  __m128i bordersum_b = _mm_add_epi16(bordersum_a, ones16);

  Deb(bordersum_a);
  Deb(bordersum_b);

  __m128i dcsum4 = _mm_hadd_epi16(bordersum_b, zero);
  __m128i dcsum2 = _mm_hadd_epi16(dcsum4, zero);
  __m128i dcsum1 = _mm_hadd_epi16(dcsum2, zero);

  Deb(dcsum4);
  Deb(dcsum2);
  Deb(dcsum1);

  const int log_nT = 4;
  __m128i dcsum = _mm_srai_epi16(dcsum1, log_nT+1);

  Deb(dcsum);

  __m128i flatdc = _mm_shuffle_epi8(dcsum, zero);

  Deb(flatdc);

  for (int y=0;y<16;y++) {
    _mm_store_si128((__m128i*)(dst+y*dstStride), flatdc);
  }
}


// 1.97x faster
void __attribute__ ((noinline)) intra_dc_see_noavg_8bit_32x32(uint8_t* dst,int dstStride, uint8_t* border)
{
  __m128i zero   = _mm_setzero_si128();
  __m128i ones16 = _mm_set1_epi16(4);
  __m128i leftborder_a = _mm_load_si128((const __m128i*)(border-16));
  __m128i leftborder_b = _mm_load_si128((const __m128i*)(border-32));
  __m128i topborder_a  = _mm_loadu_si128((const __m128i*)(border+1));
  __m128i topborder_b  = _mm_loadu_si128((const __m128i*)(border+17));
  //__m128i topborder  = _mm_load_si128((const __m128i*)(border));
  //topborder = _mm_srli_si128(topborder, 1);

  Deb(topborder_a);
  Deb(topborder_b);
  Deb(leftborder_a);
  Deb(leftborder_b);
  Deb(ones16);

  __m128i topborder16_lo  = _mm_unpacklo_epi8(topborder_a ,zero);
  __m128i leftborder16_lo = _mm_unpacklo_epi8(leftborder_a,zero);

  __m128i topborder16_hi  = _mm_unpackhi_epi8(topborder_a ,zero);
  __m128i leftborder16_hi = _mm_unpackhi_epi8(leftborder_a,zero);

  __m128i topborder16b_lo  = _mm_unpacklo_epi8(topborder_b ,zero);
  __m128i leftborder16b_lo = _mm_unpacklo_epi8(leftborder_b,zero);

  __m128i topborder16b_hi  = _mm_unpackhi_epi8(topborder_b ,zero);
  __m128i leftborder16b_hi = _mm_unpackhi_epi8(leftborder_b,zero);

  Deb(topborder16_lo);
  Deb(leftborder16_lo);
  Deb(topborder16_hi);
  Deb(leftborder16_hi);

  __m128i bordersum_lo = _mm_add_epi16(topborder16_lo, leftborder16_lo);
  __m128i bordersum_hi = _mm_add_epi16(topborder16_hi, leftborder16_hi);
  __m128i bordersumb_lo = _mm_add_epi16(topborder16b_lo, leftborder16b_lo);
  __m128i bordersumb_hi = _mm_add_epi16(topborder16b_hi, leftborder16b_hi);
  bordersum_lo = _mm_add_epi16(bordersum_lo, bordersumb_lo);
  bordersum_hi = _mm_add_epi16(bordersum_hi, bordersumb_hi);
  __m128i bordersum_a = _mm_add_epi16(bordersum_lo, bordersum_hi);
  __m128i bordersum_b = _mm_add_epi16(bordersum_a, ones16);

  Deb(bordersum_a);
  Deb(bordersum_b);

  __m128i dcsum4 = _mm_hadd_epi16(bordersum_b, zero);
  __m128i dcsum2 = _mm_hadd_epi16(dcsum4, zero);
  __m128i dcsum1 = _mm_hadd_epi16(dcsum2, zero);

  Deb(dcsum4);
  Deb(dcsum2);
  Deb(dcsum1);

  const int log_nT = 5;
  __m128i dcsum = _mm_srai_epi16(dcsum1, log_nT+1);

  Deb(dcsum);

  __m128i flatdc = _mm_shuffle_epi8(dcsum, zero);

  Deb(flatdc);

  for (int y=0;y<32;y++) {
    _mm_store_si128((__m128i*)(dst+y*dstStride   ), flatdc);
    _mm_store_si128((__m128i*)(dst+y*dstStride+16), flatdc);
  }
}


template <class pixel_t>
void __attribute__ ((noinline)) intra_dc_fallback_8bit(pixel_t* dst,int dstStride, pixel_t* border, int Log2_nT, bool avg)
{
  int nT = 1<<Log2_nT;

  int dcVal = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal += border[ i+1];
      dcVal += border[-i-1];
    }

  dcVal += nT;
  dcVal >>= Log2_nT+1;

  if (avg) {
    dst[0] = (border[-1] + 2*dcVal + border[1] +2) >> 2;

    for (int x=1;x<nT;x++) { dst[x]           = (border[ x+1] + 3*dcVal+2)>>2; }
    for (int y=1;y<nT;y++) { dst[y*dstStride] = (border[-y-1] + 3*dcVal+2)>>2; }
    for (int y=1;y<nT;y++)
      for (int x=1;x<nT;x++)
        {
          dst[x+y*dstStride] = dcVal;
        }
  } else {

    if (sizeof(pixel_t)==1) {
      for (int y=0;y<nT;y++)
        memset(dst+y*dstStride,dcVal, nT);
    }
    else {
      for (int y=0;y<nT;y++)
        for (int x=0;x<nT;x++)
          {
            dst[x+y*dstStride] = dcVal;
          }
    }
  }
}


void intra_dc()
{
  const int w = 32;

  int imgw=640,imgh=320;

  uint8_t dstc[imgw*imgh],dstsse[imgw*imgh];

  uint8_t border[4*w+1];
  for (int i=0;i<2*w;i++) {
    border[2*w+1+i] = 3*i+1;
    border[2*w-1-i] = 128 - 3*i+1;
  }
  border[2*w] = 0xC0;


  srand(time(0));
  for (int i=0;i<4*w+1;i++) {
    border[i] = rand()&0xFF;
  }

  for (int i=0;i<2*2*w+1;i++) {
    printf("%02x ", border[i]);
  }
  printf("\n");

  for (int i=0;i<100000000;i++) {
    if (w==4) {
      intra_dc_fallback_8bit(dstc,imgw, border+2*w, 2, false);
      intra_dc_see_noavg_8bit_4x4(dstsse,imgw, border+2*w);
    }

    if (w==8) {
      intra_dc_fallback_8bit(dstc,imgw, border+2*w, 3, false);
      intra_dc_see_noavg_8bit_8x8(dstsse,imgw, border+2*w);
    }

    if (w==16) {
      intra_dc_fallback_8bit(dstc,imgw, border+2*w, 4, false);
      intra_dc_see_noavg_8bit_16x16(dstsse,imgw, border+2*w);
    }

    if (w==32) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 5, false);
      intra_dc_see_noavg_8bit_32x32(dstsse,imgw, border+2*w);
    }
  }

  for (int i=0;i<w*w;i++) {
    //assert(dstsse[i] == dstc[i]);
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
