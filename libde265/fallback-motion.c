
#include "fallback-motion.h"
#include "util.h"

#include <assert.h>


void put_unweighted_pred_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                    int16_t *src, ptrdiff_t srcstride,
                                    int width, int height)
{
  int offset8bit = 32;
  int shift8bit = 6;

  assert((width&1)==0);

  for (int y=0;y<height;y++) {
    int16_t* in  = &src[y*srcstride];
    uint8_t* out = &dst[y*dststride];

    for (int x=0;x<width;x+=2) {
      out[0] = Clip1_8bit((in[0] + offset8bit)>>shift8bit);
      out[1] = Clip1_8bit((in[1] + offset8bit)>>shift8bit);
      out+=2; in+=2;
    }
  }
}


void put_weighted_pred_avg_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                      int16_t *src1, int16_t *src2,
                                      ptrdiff_t srcstride, int width,
                                      int height)
{
  int offset8bit = 64;
  int shift8bit = 7;

  assert((width&1)==0);

  // I had a special case for 8-pixel parallel, unrolled code,
  // but I did not see any speedup.

#if 0
  for (int y=0;y<height;y++) {
    int16_t* in1 = &src1[y*srcstride];
    int16_t* in2 = &src2[y*srcstride];
    uint8_t* out = &dst[y*dststride];

    for (int x=0;x<width;x++) {
      out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
      out++; in1++; in2++;
    }
  }
#endif

#if 0
  if ((width&7)==0) {
    for (int y=0;y<height;y++) {
      int16_t* in1 = &src1[y*srcstride];
      int16_t* in2 = &src2[y*srcstride];
      uint8_t* out = &dst[y*dststride];

      for (int x=0;x<width;x+=8) {
        out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
        out[1] = Clip1_8bit((in1[1] + in2[1] + offset8bit)>>shift8bit);
        out[2] = Clip1_8bit((in1[2] + in2[2] + offset8bit)>>shift8bit);
        out[3] = Clip1_8bit((in1[3] + in2[3] + offset8bit)>>shift8bit);
        out[4] = Clip1_8bit((in1[4] + in2[4] + offset8bit)>>shift8bit);
        out[5] = Clip1_8bit((in1[5] + in2[5] + offset8bit)>>shift8bit);
        out[6] = Clip1_8bit((in1[6] + in2[6] + offset8bit)>>shift8bit);
        out[7] = Clip1_8bit((in1[7] + in2[7] + offset8bit)>>shift8bit);
        out+=8; in1+=8; in2+=8;
      }
    }
  }
  else
#endif
    {
      for (int y=0;y<height;y++) {
        int16_t* in1 = &src1[y*srcstride];
        int16_t* in2 = &src2[y*srcstride];
        uint8_t* out = &dst[y*dststride];

        for (int x=0;x<width;x+=2) {
          out[0] = Clip1_8bit((in1[0] + in2[0] + offset8bit)>>shift8bit);
          out[1] = Clip1_8bit((in1[1] + in2[1] + offset8bit)>>shift8bit);
          out+=2; in1+=2; in2+=2;
        }
      }
    }
}
