

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


// without avg: slightly faster
// with    avg: slower
template <bool avg>
void /*__attribute__ ((noinline))*/ intra_dc_see_8bit_4x4(uint8_t* dst,int dstStride, uint8_t* border)
{
  __m128i zero   = _mm_setzero_si128();
  __m128i ones16 = _mm_set_epi16(1,1,1,1,0,0,0,0);
  __m128i rawborder = _mm_loadu_si128((const __m128i*)(border-8));
  Deb(rawborder);

  // --- fill border16 with the pixels from the left and top border ---

#if 1
  __m128i mask = _mm_set_epi8(0,0,0, 0xFF,0xFF,0xFF,0xFF,0,0xFF,0xFF,0xFF,0xFF,0,0,0,0);
  __m128i maskedborder = _mm_and_si128(rawborder, mask);
  __m128i border16 = _mm_shuffle_epi8(maskedborder,
                                      _mm_set_epi8(15,4,15,5,15,6,15,7,15,12,15,11,15,10,15,9));
  Deb(maskedborder);
#endif

#if 0
  // this variant is not possible anymore with the 'avg' implementation
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

  if (!avg) {
    uint32_t flatdc32 = _mm_cvtsi128_si32(flatdc);

    for (int y=0;y<4;y++) {
      *(uint32_t*)(dst+y*dstStride) = flatdc32;
    }
  }
  else {
    // --- DC-value times 3 ---

    __m128i dcsumoffset = _mm_add_epi16(dcsum, _mm_set1_epi16(2));
    __m128i dcx3 = _mm_add_epi16(dcsumoffset, _mm_slli_epi16(dcsum,1));
    Deb(dcx3);

    __m128i flatdcx3 = _mm_shuffle_epi8(dcx3, _mm_set_epi8(1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0));
    Deb(flatdcx3);


    // --- sum left column and top row at once ---

    __m128i bordersum = _mm_add_epi16(flatdcx3,border16);
    Deb(bordersum);

    __m128i bordersum_shifted = _mm_srai_epi16(bordersum, 2);
    Deb(bordersum_shifted);

    __m128i border_packed = _mm_packus_epi16(bordersum_shifted, bordersum_shifted);
    Deb(border_packed);

    uint32_t toprow32 = _mm_cvtsi128_si32(border_packed);
    *(uint32_t*)dst = toprow32;

    uint32_t flatdc32 = _mm_cvtsi128_si32(dcsum);

    // mask for swapping in left column pixel
    __m128i leftbytemask = _mm_set_epi8(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0x80);

    border_packed = _mm_srli_si128(border_packed, 5);

    for (int y=1;y<4;y++) {
      __m128i row = _mm_blendv_epi8(flatdc, border_packed, leftbytemask);
      Deb(row);

      border_packed = _mm_srli_si128(border_packed, 1);

      uint32_t row32 = _mm_cvtsi128_si32(row);
      *(uint32_t*)(dst+y*dstStride) = row32;
    }

    // --- special processing of top-left corner ---

    *dst = (2*flatdc32 + 2 + border[-1] + border[1]) >> 2;
  }
}


template void intra_dc_see_8bit_4x4<true>(uint8_t* dst,int dstStride, uint8_t* border);
template void intra_dc_see_8bit_4x4<false>(uint8_t* dst,int dstStride, uint8_t* border);



// with    avg: 2.43x faster
// without avg: 1.95x faster
template <bool avg>
void //__attribute__ ((noinline))
intra_dc_see_8bit_8x8(uint8_t* dst,int dstStride, uint8_t* border)
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

  if (!avg) {
    for (int y=0;y<8;y++) {
      _mm_storel_epi64((__m128i*)(dst+y*dstStride), flatdc);
    }
  }
  else {
    // --- DC-value times 3 ---

    __m128i dcsumoffset = _mm_add_epi16(dcsum, _mm_set1_epi16(2));
    __m128i dcx3 = _mm_add_epi16(dcsumoffset, _mm_slli_epi16(dcsum,1));
    Deb(dcx3);

    __m128i flatdcx3 = _mm_shuffle_epi8(dcx3, _mm_set_epi8(1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0));
    Deb(flatdcx3);


    // --- compute top row ---

    __m128i toprowsum = _mm_add_epi16(flatdcx3,topborder16);
    Deb(toprowsum);

    __m128i toprowsum_shifted = _mm_srai_epi16(toprowsum, 2);
    Deb(toprowsum_shifted);

    __m128i toprow_packed = _mm_packus_epi16(toprowsum_shifted, toprowsum_shifted);
    Deb(toprow_packed);

    _mm_storel_epi64((__m128i*)(dst+0*dstStride), toprow_packed);


    // --- compute left column and main block DC value ---

    // left border pixels except first line
    __m128i leftborderrev = _mm_shuffle_epi8(leftborder16,
                                             _mm_set_epi8(1,1,1,0,3,2,5,4, 7,6,9,8,11,10,13,12));
    Deb(leftborderrev);

    __m128i leftborderadd = _mm_add_epi16(leftborderrev, flatdcx3);
    leftborderadd = _mm_srai_epi16(leftborderadd,2);
    Deb(leftborderadd);

    __m128i leftborder_packed = _mm_packus_epi16(leftborderadd,leftborderadd);
    Deb(leftborder_packed);

    uint32_t flatdc32 = _mm_cvtsi128_si32(dcsum);

    // mask for swapping in left column pixel
    __m128i leftbytemask = _mm_set_epi8(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0x80);

    for (int y=1;y<8;y++) {
      __m128i row = _mm_blendv_epi8(flatdc, leftborder_packed, leftbytemask);
      Deb(row);

      leftborder_packed = _mm_srli_si128(leftborder_packed, 1);

      _mm_storel_epi64((__m128i*)(dst+y*dstStride), row);
    }

    // --- special processing of top-left corner ---

    *dst = (2*flatdc32 + 2 + border[-1] + border[1]) >> 2;
  }
}


// with    avg : 5.3x
// without avg : 2.9x
template <bool avg>
void __attribute__ ((noinline))
intra_dc_see_8bit_16x16(uint8_t* dst,int dstStride, uint8_t* border)
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

  if (!avg) {
    for (int y=0;y<16;y++) {
      _mm_store_si128((__m128i*)(dst+y*dstStride), flatdc);
    }
  }
  else {
    // --- DC-value times 3 ---

    __m128i dcsumoffset = _mm_add_epi16(dcsum, _mm_set1_epi16(2));
    __m128i dcx3 = _mm_add_epi16(dcsumoffset, _mm_slli_epi16(dcsum,1));
    Deb(dcx3);

    __m128i flatdcx3 = _mm_shuffle_epi8(dcx3, _mm_set_epi8(1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0));
    Deb(flatdcx3);


    // --- compute top row ---

    __m128i toprowsum_hi = _mm_add_epi16(flatdcx3,topborder16_hi);
    __m128i toprowsum_lo = _mm_add_epi16(flatdcx3,topborder16_lo);
    Deb(toprowsum_hi);
    Deb(toprowsum_lo);

    __m128i toprowsum_hi_shifted = _mm_srai_epi16(toprowsum_hi, 2);
    __m128i toprowsum_lo_shifted = _mm_srai_epi16(toprowsum_lo, 2);
    Deb(toprowsum_hi_shifted);
    Deb(toprowsum_lo_shifted);

    __m128i toprow_packed = _mm_packus_epi16(toprowsum_lo_shifted, toprowsum_hi_shifted);
    Deb(toprow_packed);

    _mm_store_si128((__m128i*)(dst+0*dstStride), toprow_packed);


    // --- compute left column and main block DC value ---

    // left border pixels except first line
    __m128i leftborder_shuffle= _mm_set_epi8(1,0,3,2,5,4, 7,6,9,8,11,10,13,12,15,14);
    __m128i leftborder_lo_rev = _mm_shuffle_epi8(leftborder16_lo,leftborder_shuffle);
    __m128i leftborder_hi_rev = _mm_shuffle_epi8(leftborder16_hi,leftborder_shuffle);
    Deb(leftborder_lo_rev);
    Deb(leftborder_hi_rev);

    __m128i leftborder_lo_add = _mm_add_epi16(leftborder_lo_rev, flatdcx3);
    leftborder_lo_add = _mm_srai_epi16(leftborder_lo_add,2);
    __m128i leftborder_hi_add = _mm_add_epi16(leftborder_hi_rev, flatdcx3);
    leftborder_hi_add = _mm_srai_epi16(leftborder_hi_add,2);
    Deb(leftborder_lo_add);
    Deb(leftborder_hi_add);

    __m128i leftborder_packed = _mm_packus_epi16(leftborder_hi_add,leftborder_lo_add);
    Deb(leftborder_packed);

    uint32_t flatdc32 = _mm_cvtsi128_si32(dcsum);

    // mask for swapping in left column pixel
    __m128i leftbytemask = _mm_set_epi8(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0x80);

    for (int y=1;y<16;y++) {
      leftborder_packed = _mm_srli_si128(leftborder_packed, 1);

      __m128i row = _mm_blendv_epi8(flatdc, leftborder_packed, leftbytemask);
      Deb(row);

      _mm_store_si128((__m128i*)(dst+y*dstStride), row);
    }

    // --- special processing of top-left corner ---

    *dst = (2*flatdc32 + 2 + border[-1] + border[1]) >> 2;
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

    for (int x=1;x<nT;x++) {
      dst[x] = (border[ x+1] + 3*dcVal+2)>>2;
    }

    for (int y=1;y<nT;y++) {
      dst[y*dstStride] = (border[-y-1] + 3*dcVal+2)>>2;

      for (int x=1;x<nT;x++)
        {
          dst[x+y*dstStride] = dcVal;
        }
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
  const int w = 4;

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

  imgw=16;

  const uint64_t nIter = (D ? 1 : 1000000000);
  for (int i=0;i<nIter;i++) {
    if (w==4) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 2, false);
      //intra_dc_see_8bit_4x4<false>(dstsse,imgw, border+2*w);

      intra_dc_fallback_8bit(dstc,imgw, border+2*w, 2, true);
      intra_dc_see_8bit_4x4<true>(dstsse,imgw, border+2*w);
    }

    if (w==8) {
      intra_dc_fallback_8bit(dstc,imgw, border+2*w, 3, false);
      intra_dc_see_8bit_8x8<false>(dstsse,imgw, border+2*w);

      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 3, true);
      //intra_dc_see_8bit_8x8<true>(dstsse,imgw, border+2*w);
    }

    if (w==16) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 4, false);
      intra_dc_see_8bit_16x16<false>(dstsse,imgw, border+2*w);

      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 4, true);
      //intra_dc_see_8bit_16x16<true>(dstsse,imgw, border+2*w);
    }

    if (w==32) {
      //intra_dc_fallback_8bit(dstc,imgw, border+2*w, 5, false);
      intra_dc_see_noavg_8bit_32x32(dstsse,imgw, border+2*w);
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
