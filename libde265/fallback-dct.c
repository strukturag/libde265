
#include "fallback-motion.h"
#include "util.h"

#ifdef _MSC_VER
# include <malloc.h>
#else
# include <alloca.h>
#endif

#include <assert.h>


static int8_t mat_8_357[4][4] = {
  { 29, 55, 74, 84 },
  { 74, 74,  0,-74 },
  { 84,-29,-74, 55 },
  { 55,-84, 74,-29 }
};



void transform_4x4_luma_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride)
{
  int16_t g[4][4];

  int postShift = 20-8; // 8 bit
  int rndV = 1<<(7-1);
  int rndH = 1<<(postShift-1);


  // --- V ---

  for (int c=0;c<4;c++) {

    logtrace(LogTransform,"DST-V: ");
    for (int r=0;r<4;r++) {
      logtrace(LogTransform,"%d ",coeffs[c+r*4]);
    }
    logtrace(LogTransform,"* -> ");


    for (int i=0;i<4;i++) {
      int sum=0;

      for (int j=0;j<4;j++) {
        sum += mat_8_357[j][i] * coeffs[c+j*4];
      }

      g[i][c] = Clip3(-32768,32767, (sum+rndV)>>7);
    }


    for (int y=0;y<4;y++) {
      logtrace(LogTransform,"*%d ",g[y][c]);
    }
    logtrace(LogTransform,"*\n");
  }


  // --- H ---

  for (int y=0;y<4;y++) {

    logtrace(LogTransform,"DST-H: ");
    for (int c=0;c<4;c++) {
      logtrace(LogTransform,"%d ",g[y][c]);
    }
    logtrace(LogTransform,"* -> ");


    for (int i=0;i<4;i++) {
      int sum=0;

      for (int j=0;j<4;j++) {
        sum += mat_8_357[j][i] * g[y][j];
      }

      int out = Clip3(-32768,32767, (sum+rndH)>>postShift);

      dst[y*stride+i] = Clip1_8bit(dst[y*stride+i] + out);

      logtrace(LogTransform,"*%d ",out);
    }

    logtrace(LogTransform,"*\n");
  }
}
