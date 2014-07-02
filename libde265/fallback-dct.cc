/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "fallback-motion.h"
#include "util.h"

#if defined(_MSC_VER) || defined(__MINGW32__)
# include <malloc.h>
#else
# include <alloca.h>
#endif

#include <assert.h>


void transform_skip_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride)
{
  int nT = 4;
  int bdShift2 = 20-8;

  for (int y=0;y<nT;y++)
    for (int x=0;x<nT;x++) {
      int32_t c = coeffs[x+y*nT] << 7;
      c = (c+(1<<(bdShift2-1)))>>bdShift2;

      dst[y*stride+x] = Clip1_8bit(dst[y*stride+x] + c);
    }
}


void transform_bypass_8_fallback(uint8_t *dst, int16_t *coeffs, int nT, ptrdiff_t stride)
{
  int bdShift2 = 20-8;

  for (int y=0;y<nT;y++)
    for (int x=0;x<nT;x++) {
      int32_t c = coeffs[x+y*nT];

      dst[y*stride+x] = Clip1_8bit(dst[y*stride+x] + c);
    }
}
        

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



static int8_t mat_dct[32][32] = {
  { 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
  { 90, 90, 88, 85, 82, 78, 73, 67, 61, 54, 46, 38, 31, 22, 13,  4,      -4,-13,-22,-31,-38,-46,-54,-61,-67,-73,-78,-82,-85,-88,-90,-90},
  { 90, 87, 80, 70, 57, 43, 25,  9, -9,-25,-43,-57,-70,-80,-87,-90,     -90,-87,-80,-70,-57,-43,-25, -9,  9, 25, 43, 57, 70, 80, 87, 90},
  { 90, 82, 67, 46, 22, -4,-31,-54,-73,-85,-90,-88,-78,-61,-38,-13,      13, 38, 61, 78, 88, 90, 85, 73, 54, 31,  4,-22,-46,-67,-82,-90},
  { 89, 75, 50, 18,-18,-50,-75,-89,-89,-75,-50,-18, 18, 50, 75, 89,      89, 75, 50, 18,-18,-50,-75,-89,-89,-75,-50,-18, 18, 50, 75, 89},
  { 88, 67, 31,-13,-54,-82,-90,-78,-46, -4, 38, 73, 90, 85, 61, 22,     -22,-61,-85,-90,-73,-38,  4, 46, 78, 90, 82, 54, 13,-31,-67,-88},
  { 87, 57,  9,-43,-80,-90,-70,-25, 25, 70, 90, 80, 43, -9,-57,-87,     -87,-57, -9, 43, 80, 90, 70, 25,-25,-70,-90,-80,-43,  9, 57, 87},
  { 85, 46,-13,-67,-90,-73,-22, 38, 82, 88, 54, -4,-61,-90,-78,-31,      31, 78, 90, 61,  4,-54,-88,-82,-38, 22, 73, 90, 67, 13,-46,-85},
  { 83, 36,-36,-83,-83,-36, 36, 83, 83, 36,-36,-83,-83,-36, 36, 83,      83, 36,-36,-83,-83,-36, 36, 83, 83, 36,-36,-83,-83,-36, 36, 83},
  { 82, 22,-54,-90,-61, 13, 78, 85, 31,-46,-90,-67,  4, 73, 88, 38,     -38,-88,-73, -4, 67, 90, 46,-31,-85,-78,-13, 61, 90, 54,-22,-82},
  { 80,  9,-70,-87,-25, 57, 90, 43,-43,-90,-57, 25, 87, 70, -9,-80,     -80, -9, 70, 87, 25,-57,-90,-43, 43, 90, 57,-25,-87,-70,  9, 80},
  { 78, -4,-82,-73, 13, 85, 67,-22,-88,-61, 31, 90, 54,-38,-90,-46,      46, 90, 38,-54,-90,-31, 61, 88, 22,-67,-85,-13, 73, 82,  4,-78},
  { 75,-18,-89,-50, 50, 89, 18,-75,-75, 18, 89, 50,-50,-89,-18, 75,      75,-18,-89,-50, 50, 89, 18,-75,-75, 18, 89, 50,-50,-89,-18, 75},
  { 73,-31,-90,-22, 78, 67,-38,-90,-13, 82, 61,-46,-88, -4, 85, 54,     -54,-85,  4, 88, 46,-61,-82, 13, 90, 38,-67,-78, 22, 90, 31,-73},
  { 70,-43,-87,  9, 90, 25,-80,-57, 57, 80,-25,-90, -9, 87, 43,-70,     -70, 43, 87, -9,-90,-25, 80, 57,-57,-80, 25, 90,  9,-87,-43, 70},
  { 67,-54,-78, 38, 85,-22,-90,  4, 90, 13,-88,-31, 82, 46,-73,-61,      61, 73,-46,-82, 31, 88,-13,-90, -4, 90, 22,-85,-38, 78, 54,-67},
  { 64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64,      64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64, 64,-64,-64, 64},
  { 61,-73,-46, 82, 31,-88,-13, 90, -4,-90, 22, 85,-38,-78, 54, 67,     -67,-54, 78, 38,-85,-22, 90,  4,-90, 13, 88,-31,-82, 46, 73,-61},
  { 57,-80,-25, 90, -9,-87, 43, 70,-70,-43, 87,  9,-90, 25, 80,-57,     -57, 80, 25,-90,  9, 87,-43,-70, 70, 43,-87, -9, 90,-25,-80, 57},
  { 54,-85, -4, 88,-46,-61, 82, 13,-90, 38, 67,-78,-22, 90,-31,-73,      73, 31,-90, 22, 78,-67,-38, 90,-13,-82, 61, 46,-88,  4, 85,-54},
  { 50,-89, 18, 75,-75,-18, 89,-50,-50, 89,-18,-75, 75, 18,-89, 50,      50,-89, 18, 75,-75,-18, 89,-50,-50, 89,-18,-75, 75, 18,-89, 50},
  { 46,-90, 38, 54,-90, 31, 61,-88, 22, 67,-85, 13, 73,-82,  4, 78,     -78, -4, 82,-73,-13, 85,-67,-22, 88,-61,-31, 90,-54,-38, 90,-46},
  { 43,-90, 57, 25,-87, 70,  9,-80, 80, -9,-70, 87,-25,-57, 90,-43,     -43, 90,-57,-25, 87,-70, -9, 80,-80,  9, 70,-87, 25, 57,-90, 43},
  { 38,-88, 73, -4,-67, 90,-46,-31, 85,-78, 13, 61,-90, 54, 22,-82,      82,-22,-54, 90,-61,-13, 78,-85, 31, 46,-90, 67,  4,-73, 88,-38},
  { 36,-83, 83,-36,-36, 83,-83, 36, 36,-83, 83,-36,-36, 83,-83, 36,      36,-83, 83,-36,-36, 83,-83, 36, 36,-83, 83,-36,-36, 83,-83, 36},
  { 31,-78, 90,-61,  4, 54,-88, 82,-38,-22, 73,-90, 67,-13,-46, 85,     -85, 46, 13,-67, 90,-73, 22, 38,-82, 88,-54, -4, 61,-90, 78,-31},
  { 25,-70, 90,-80, 43,  9,-57, 87,-87, 57, -9,-43, 80,-90, 70,-25,     -25, 70,-90, 80,-43, -9, 57,-87, 87,-57,  9, 43,-80, 90,-70, 25},
  { 22,-61, 85,-90, 73,-38, -4, 46,-78, 90,-82, 54,-13,-31, 67,-88,      88,-67, 31, 13,-54, 82,-90, 78,-46,  4, 38,-73, 90,-85, 61,-22},
  { 18,-50, 75,-89, 89,-75, 50,-18,-18, 50,-75, 89,-89, 75,-50, 18,      18,-50, 75,-89, 89,-75, 50,-18,-18, 50,-75, 89,-89, 75,-50, 18},
  { 13,-38, 61,-78, 88,-90, 85,-73, 54,-31,  4, 22,-46, 67,-82, 90,     -90, 82,-67, 46,-22, -4, 31,-54, 73,-85, 90,-88, 78,-61, 38,-13},
  {  9,-25, 43,-57, 70,-80, 87,-90, 90,-87, 80,-70, 57,-43, 25, -9,      -9, 25,-43, 57,-70, 80,-87, 90,-90, 87,-80, 70,-57, 43,-25,  9},
  {  4,-13, 22,-31, 38,-46, 54,-61, 67,-73, 78,-82, 85,-88, 90,-90,      90,-90, 88,-85, 82,-78, 73,-67, 61,-54, 46,-38, 31,-22, 13, -4}
};




static void transform_idct_add_8(uint8_t *dst, ptrdiff_t stride,
                                 int nT, int16_t *coeffs)
{
  /*
    The effective shift is
    7 bits right for bit-depth 8,
    6 bits right for bit-depth 9,
    5 bits right for bit-depth 10.

    Computation is independent of the block size.
    Each multiplication with the table includes a left shift of 6 bits.
    Hence, we have 2* 6 bits = 12 bits left shift.
    V-pass has fixed 7 bit right shift.
    H-pass has 20-BitDepth bit right shift;

    Effective shift 's' means: residual value 1 gives DC-coeff (1<<s).
   */


  int postShift = 20-8; // 8 bit
  int rnd1 = 1<<(7-1);
  int rnd2 = 1<<(postShift-1);
  int fact = (1<<(5-Log2(nT)));

  int16_t g[32*32];  // actually, only [nT*nT] used

  // TODO: valgrind reports that dst[] contains uninitialized data.
  // Probably from intra-prediction.

  /*
  for (int i=0;i<nT*nT;i++) {
    printf("%d\n",coeffs[i]);
  }

  for (int y=0;y<nT;y++) {
    for (int i=0;i<nT;i++) {
      printf("%d ",dst[y*stride+i]);
    }
  }
  printf("\n");
  */

  /*
  printf("--- input\n");
  for (int r=0;r<nT;r++, printf("\n"))
    for (int c=0;c<nT;c++) {
      printf("%3d ",coeffs[c+r*nT]);
    }
  */

  for (int c=0;c<nT;c++) {

    logtrace(LogTransform,"DCT-V: ");
    for (int i=0;i<nT;i++) {
      logtrace(LogTransform,"*%d ",coeffs[c+i*nT]);
    }
    logtrace(LogTransform,"* -> ");


    // find last non-zero coefficient to reduce computations carried out in DCT

    int lastCol = nT-1;
    for (;lastCol>=0;lastCol--) {
      if (coeffs[c+lastCol*nT]) { break; }
    }

    for (int i=0;i<nT;i++) {
      int sum=0;

      /*
      printf("input: ");
      for (int j=0;j<nT;j++) {
        printf("%3d ",coeffs[c+j*nT]);
      }
      printf("\n");

      printf("mat: ");
      for (int j=0;j<nT;j++) {
        printf("%3d ",mat_dct[fact*j][i]);
      }
      printf("\n");
      */

      for (int j=0;j<=lastCol /*nT*/;j++) {
        sum += mat_dct[fact*j][i] * coeffs[c+j*nT];
      }
      
      g[c+i*nT] = Clip3(-32768,32767, (sum+rnd1)>>7);

      logtrace(LogTransform,"*%d ",g[c+i*nT]);
    }
    logtrace(LogTransform,"*\n");
  }

  /*
  printf("--- temp\n");
  for (int r=0;r<nT;r++, printf("\n"))
    for (int c=0;c<nT;c++) {
      printf("%3d ",g[c+r*nT]);
    }
  */

  for (int y=0;y<nT;y++) {

    logtrace(LogTransform,"DCT-H: ");
    for (int i=0;i<nT;i++) {
      logtrace(LogTransform,"*%d ",g[i+y*nT]);
    }
    logtrace(LogTransform,"* -> ");


    // find last non-zero coefficient to reduce computations carried out in DCT

    int lastCol = nT-1;
    for (;lastCol>=0;lastCol--) {
      if (g[y*nT+lastCol]) { break; }
    }


    for (int i=0;i<nT;i++) {
      int sum=0;
      
      for (int j=0;j<=lastCol /*nT*/;j++) {
        sum += mat_dct[fact*j][i] * g[y*nT+j];
      }
      
      //int out = Clip3(-32768,32767, (sum+rnd2)>>postShift);
      int out = (sum+rnd2)>>postShift;

      //fprintf(stderr,"%d*%d+%d = %d\n",y,stride,i,y*stride+i);
      //fprintf(stderr,"[%p]=%d\n",&dst[y*stride+i], Clip1_8bit(dst[y*stride+i]));
      dst[y*stride+i] = Clip1_8bit(dst[y*stride+i] + out);

      logtrace(LogTransform,"*%d ",out);
    }
    logtrace(LogTransform,"*\n");
  }
}


void transform_4x4_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride)
{
  transform_idct_add_8(dst,stride,  4, coeffs);
}

void transform_8x8_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride)
{
  transform_idct_add_8(dst,stride,  8, coeffs);
}

void transform_16x16_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride)
{
  transform_idct_add_8(dst,stride,  16, coeffs);
}

void transform_32x32_add_8_fallback(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride)
{
  transform_idct_add_8(dst,stride,  32, coeffs);
}





static void transform_fdct_8(int16_t* coeffs, int nT,
                             const int16_t *input, ptrdiff_t stride)
{
  /*
    Each sum over a basis vector sums nT elements, which is compensated by
    shifting right by Log2(nT). Do this in each of the H/V passes.

    Each multiplication with the table includes a left shift of 6 bits.
    Hence, we have 2* 6 bits = 12 bits left shift. Additionally,
    V-pass has BitDepth-9 bit right shift,
    H-pass has fixed 6 bit right shift.

    For bit-depth 8, the effective shift is 7 bits left.
    For bit-depth 9, the effective shift is 6 bits left.
    For bit-depth 10, the effective shift is 5 bits left.

    Effective shift 's' means: DC-coeff (1<<s) gives residual value 1.
   */

  int BD = 8;
  int shift1 = Log2(nT) + BD -9;  // 12-9=3
  int shift2 = Log2(nT) + 6;      // 10

  int rnd1 = 1<<(shift1-1);
  int rnd2 = 1<<(shift2-1);
  int fact = (1<<(5-Log2(nT)));

  int16_t g[32*32];  // actually, only [nT*nT] used

  for (int c=0;c<nT;c++) {

    for (int i=0;i<nT;i++) {
      int sum=0;
      
      for (int j=0;j<nT;j++) {
        sum += mat_dct[fact*i][j] * input[c+j*stride];
      }
      
      g[c+i*nT] = Clip3(-32768,32767, (sum+rnd1)>>shift1);

      logtrace(LogTransform,"*%d ",g[c+i*nT]);
    }
    logtrace(LogTransform,"*\n");
  }


  for (int y=0;y<nT;y++) {

    logtrace(LogTransform,"DCT-H: ");
    for (int i=0;i<nT;i++) {
      logtrace(LogTransform,"*%d ",g[i+y*nT]);
    }
    logtrace(LogTransform,"* -> ");


    for (int i=0;i<nT;i++) {
      int sum=0;
      
      for (int j=0;j<nT;j++) {
        sum += mat_dct[fact*i][j] * g[y*nT+j];
      }
      
      int out = (sum+rnd2)>>shift2;

      coeffs[y*nT+i] = out;

      logtrace(LogTransform,"*%d ",out);
    }
    logtrace(LogTransform,"*\n");
  }
}


void fdct_4x4_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride)
{
  transform_fdct_8(coeffs, 4, input,stride);
}

void fdct_8x8_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride)
{
  transform_fdct_8(coeffs, 8, input,stride);
}

void fdct_16x16_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride)
{
  transform_fdct_8(coeffs, 16, input,stride);
}

void fdct_32x32_8_fallback(int16_t *coeffs, const int16_t *input, ptrdiff_t stride)
{
  transform_fdct_8(coeffs, 32, input,stride);
}
