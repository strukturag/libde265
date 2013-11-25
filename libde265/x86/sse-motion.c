/*
  Code taken over from openHEVC.
 */

#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>

#include "sse-motion.h"


void put_unweighted_pred_8_sse(uint8_t *_dst, ptrdiff_t dststride,
                               int16_t *src, ptrdiff_t srcstride, int width, int height)
{
  int x, y;
  uint8_t *dst = (uint8_t*) _dst;
  __m128i r0, r1, f0;

  f0 = _mm_set1_epi16(32);


  if(!(width & 15))
    {
      for (y = 0; y < height; y++) {
        for (x = 0; x < width; x += 16) {
          r0 = _mm_load_si128((__m128i *) (src+x));

          r1 = _mm_load_si128((__m128i *) (src+x + 8));
          r0 = _mm_adds_epi16(r0, f0);

          r1 = _mm_adds_epi16(r1, f0);
          r0 = _mm_srai_epi16(r0, 6);
          r1 = _mm_srai_epi16(r1, 6);
          r0 = _mm_packus_epi16(r0, r1);

          _mm_storeu_si128((__m128i *) (dst+x), r0);
        }
        dst += dststride;
        src += srcstride;
      }
    }else if(!(width & 7))
    {
      for (y = 0; y < height; y++) {
        for (x = 0; x < width; x += 8) {
          r0 = _mm_load_si128((__m128i *) (src+x));

          r0 = _mm_adds_epi16(r0, f0);

          r0 = _mm_srai_epi16(r0, 6);
          r0 = _mm_packus_epi16(r0, r0);

          _mm_storel_epi64((__m128i *) (dst+x), r0);
        }
        dst += dststride;
        src += srcstride;
      }
    }else if(!(width & 3)){
    for (y = 0; y < height; y++) {
      for(x = 0;x < width; x+=4){
        r0 = _mm_loadl_epi64((__m128i *) (src+x));
        r0 = _mm_adds_epi16(r0, f0);

        r0 = _mm_srai_epi16(r0, 6);
        r0 = _mm_packus_epi16(r0, r0);
        _mm_maskmoveu_si128(r0,_mm_set_epi8(0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,-1,-1),(char *) (dst+x));
      }
      dst += dststride;
      src += srcstride;
    }
  }else{
    for (y = 0; y < height; y++) {
      for(x = 0;x < width; x+=2){
        r0 = _mm_loadl_epi64((__m128i *) (src+x));
        r0 = _mm_adds_epi16(r0, f0);

        r0 = _mm_srai_epi16(r0, 6);
        r0 = _mm_packus_epi16(r0, r0);
        _mm_maskmoveu_si128(r0,_mm_set_epi8(0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1),(char *) (dst+x));
      }
      dst += dststride;
      src += srcstride;
    }
  }
}



void put_weighted_pred_avg_8_sse(uint8_t *_dst, ptrdiff_t dststride,
                                 int16_t *src1, int16_t *src2,
                                 ptrdiff_t srcstride, int width,
                                 int height)
{
  int x, y;
  uint8_t *dst = (uint8_t*) _dst;
  __m128i r0, r1, f0, r2, r3;


  f0 = _mm_set1_epi16(64);
  if(!(width & 15)){
    for (y = 0; y < height; y++) {

      for (x = 0; x < width; x += 16) {
        r0 = _mm_load_si128((__m128i *) &src1[x]);
        r1 = _mm_load_si128((__m128i *) &src1[x + 8]);
        r2 = _mm_load_si128((__m128i *) &src2[x]);
        r3 = _mm_load_si128((__m128i *) &src2[x + 8]);

        r0 = _mm_adds_epi16(r0, f0);
        r1 = _mm_adds_epi16(r1, f0);
        r0 = _mm_adds_epi16(r0, r2);
        r1 = _mm_adds_epi16(r1, r3);
        r0 = _mm_srai_epi16(r0, 7);
        r1 = _mm_srai_epi16(r1, 7);
        r0 = _mm_packus_epi16(r0, r1);

        _mm_storeu_si128((__m128i *) (dst + x), r0);
      }
      dst += dststride;
      src1 += srcstride;
      src2 += srcstride;
    }
  }else if(!(width & 7)){
    for (y = 0; y < height; y++) {
      for(x=0;x<width;x+=8){
        r0 = _mm_load_si128((__m128i *) (src1+x));
        r2 = _mm_load_si128((__m128i *) (src2+x));

        r0 = _mm_adds_epi16(r0, f0);
        r0 = _mm_adds_epi16(r0, r2);
        r0 = _mm_srai_epi16(r0, 7);
        r0 = _mm_packus_epi16(r0, r0);

        _mm_storel_epi64((__m128i *) (dst+x), r0);
      }
      dst += dststride;
      src1 += srcstride;
      src2 += srcstride;
    }
  }else if(!(width & 3)){
    r1= _mm_set_epi8(0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,-1,-1);
    for (y = 0; y < height; y++) {

      for(x=0;x<width;x+=4)
        {
          r0 = _mm_loadl_epi64((__m128i *) (src1+x));
          r2 = _mm_loadl_epi64((__m128i *) (src2+x));

          r0 = _mm_adds_epi16(r0, f0);
          r0 = _mm_adds_epi16(r0, r2);
          r0 = _mm_srai_epi16(r0, 7);
          r0 = _mm_packus_epi16(r0, r0);

          _mm_maskmoveu_si128(r0,r1,(char *) (dst+x));
        }
      dst += dststride;
      src1 += srcstride;
      src2 += srcstride;
    }
  }else{
    r1= _mm_set_epi8(0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1);
    for (y = 0; y < height; y++) {
      for(x=0;x<width;x+=2)
        {
          r0 = _mm_loadl_epi64((__m128i *) (src1+x));
          r2 = _mm_loadl_epi64((__m128i *) (src2+x));

          r0 = _mm_adds_epi16(r0, f0);
          r0 = _mm_adds_epi16(r0, r2);
          r0 = _mm_srai_epi16(r0, 7);
          r0 = _mm_packus_epi16(r0, r0);


          _mm_maskmoveu_si128(r0,r1,(char *) (dst+x));
        }
      dst += dststride;
      src1 += srcstride;
      src2 += srcstride;
    }
  }
}
