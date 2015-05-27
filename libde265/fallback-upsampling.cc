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

#include "fallback-upsampling.h"
#include "util.h"

// Use a faster implementation of the upsampling filter. The downside is, that this upsampling 
// implementation allocates a temporary buffer. This buffer has dimension SrcPicHeigh * DestPicWidth
#define USE_FASTER_IMPLEMENTATION 1

void resampling_process_of_luma_sample_values_fallback( uint8_t *src, ptrdiff_t srcstride, int src_size[2],
                                                        uint8_t *dst, ptrdiff_t dststride, int dst_size[2],
                                                        int position_params[8], int BitDepthRefLayerY, int BitDepthCurrY)
{
  // Reference layer size
  int PicHeightInSamplesRefLayerY = src_size[1];
  int PicWidthInSamplesRefLayerY = src_size[0];

  // Current layer size
  int PicHeightInSamplesCurLayerY = dst_size[1];
  int PicWidthInSamplesCurLayerY = dst_size[0];

  int xRef16, yRef16, xRef, xPhase, yRef, yPhase;
  int shift1, shift2, offset;
  int yPosRL;
  uint8_t *rlPicSampleL;
  uint8_t *rsLumaSample;

  int refW = PicWidthInSamplesRefLayerY;   // (H 37)

  // Table H.1 – 16-phase luma resampling filter
  int fL[16][8] = { { 0, 0,   0, 64,  0,   0, 0,  0},
                    { 0, 1,  -3, 63,  4,  -2, 1,  0},
                    {-1, 2,  -5, 62,  8,  -3, 1,  0},
                    {-1, 3,  -8, 60, 13,  -4, 1,  0},
                    {-1, 4, -10, 58, 17,  -5, 1,  0},
                    {-1, 4, -11, 52, 26,  -8, 3, -1},
                    {-1, 3,   9, 47, 31, -10, 4, -1},
                    {-1, 4, -11, 45, 34, -10, 4, -1},
                    {-1, 4, -11, 40, 40, -11, 4, -1},
                    {-1, 4, -10, 34, 45, -11, 4, -1},
                    {-1, 4, -10, 31, 47,  -9, 3, -1},
                    {-1, 3,  -8, 26, 52, -11, 4, -1},
                    { 0, 1,  -5, 17, 58, -10, 4, -1},
                    { 0, 1,  -4, 13, 60,  -8, 3, -1},
                    { 0, 1,  -3,  8, 62,  -5, 2, -1},
                    { 0, 1,  -2,  4, 63,  -3, 1,  0} };

  // 4. The variables shift1, shift2 and offset are derived as follows:
  shift1 = BitDepthRefLayerY - 8;  // (H 33)
  shift2 = 20 - BitDepthCurrY;     // (H 34)
  offset = 1 << (shift2 - 1);      // (H 35)
  
#if USE_FASTER_IMPLEMENTATION
  // Perform horizontal / vertical upsampling seperately.
  // Allocate temporaray buffer
  int16_t *tmp = new int16_t[PicHeightInSamplesRefLayerY * PicWidthInSamplesCurLayerY];
  int16_t *tmpSample;
  int tmpStride = PicWidthInSamplesCurLayerY;

  // Horizontal upsampling
  for (int y = 0; y < PicHeightInSamplesRefLayerY; y++) {

    // Get pointer to the source for this y position
    rlPicSampleL = src + y * srcstride;
    // Get pointer to the temp array for this y position
    tmpSample = tmp + y * tmpStride;

    for (int xP = 0; xP < PicWidthInSamplesCurLayerY; xP++) {
      // 1.
      // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
      // The position_params array contains the precomputed values needed for this.
      xRef16 = (((xP - position_params[0]) * position_params[4] + position_params[6] + (1 << 11)) >> 12) + position_params[2];  // (H 63)
        
      // 2. The variables xRef and xPhase are derived as follows:
      xRef   = xRef16 >> 4;  // (H 29)
      xPhase = xRef16 % 16;  // (H 30)

      tmpSample[xP] = (fL[xPhase][0] * rlPicSampleL[ Clip3(0, refW - 1, xRef - 3)] +
                       fL[xPhase][1] * rlPicSampleL[ Clip3(0, refW - 1, xRef - 2)] +
                       fL[xPhase][2] * rlPicSampleL[ Clip3(0, refW - 1, xRef - 1)] +
                       fL[xPhase][3] * rlPicSampleL[ Clip3(0, refW - 1, xRef    )] +
                       fL[xPhase][4] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 1)] +
                       fL[xPhase][5] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 2)] +
                       fL[xPhase][6] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 3)] +
                       fL[xPhase][7] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 4)] ) >> shift1; // (H 38)
    }
  }
  
  // Vertical upsampling
  int refY = PicHeightInSamplesRefLayerY;

  for (int yP = 0; yP < PicHeightInSamplesCurLayerY; yP++) {
    // Get pointer to destination y line
    rsLumaSample = dst + yP * dststride;
    
    // 1.
    // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
    // The position_params array contains the precomputed values needed for this.
    yRef16 = (((yP - position_params[1]) * position_params[5] + position_params[7] + (1 << 11)) >> 12) + position_params[3];  // (H 64)

    // 3. The variables yRef and yPhase are derived as follows:
    yPhase = yRef16 % 16;  // (H 32)
    yRef   = yRef16 >> 4;  // (H 31)

    for (int x = 0; x < PicWidthInSamplesCurLayerY; x++) {
      // Get pointer to temp array y line
      tmpSample = tmp + x;

       // 6. The resampled luma sample value rsLumaSample is derived as follows:
      rsLumaSample[x] = ( fL[yPhase][0] * tmpSample[ Clip3(0, refY - 1, yRef - 3) * tmpStride ] +
                          fL[yPhase][1] * tmpSample[ Clip3(0, refY - 1, yRef - 2) * tmpStride ] +
                          fL[yPhase][2] * tmpSample[ Clip3(0, refY - 1, yRef - 1) * tmpStride ] +
                          fL[yPhase][3] * tmpSample[ Clip3(0, refY - 1, yRef    ) * tmpStride ] +
                          fL[yPhase][4] * tmpSample[ Clip3(0, refY - 1, yRef + 1) * tmpStride ] +
                          fL[yPhase][5] * tmpSample[ Clip3(0, refY - 1, yRef + 2) * tmpStride ] +
                          fL[yPhase][6] * tmpSample[ Clip3(0, refY - 1, yRef + 3) * tmpStride ] +
                          fL[yPhase][7] * tmpSample[ Clip3(0, refY - 1, yRef + 4) * tmpStride ] + offset ) >> shift2;  // (H 39)
    }
  }
  
  delete[] tmp;
  tmp = NULL;

#else
  int tempArray[8];

  // H.8.1.4.1.1 Resampling process of luma sample values
  for (int yP = 0; yP < PicHeightInSamplesCurLayerY; yP++) {
    rsLumaSample = dst + yP * dststride;

    // 1.
    // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
    // The position_params array contains the precomputed values needed for this.
    yRef16 = (((yP - position_params[1]) * position_params[5] + position_params[7] + (1 << 11)) >> 12) + position_params[3];  // (H 64)

    // 3. The variables yRef and yPhase are derived as follows:
    yPhase = yRef16 % 16;  // (H 32)
    yRef   = yRef16 >> 4;  // (H 31)

    for (int xP = 0; xP < PicWidthInSamplesCurLayerY; xP++) {
      // 1.
      // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
      // The position_params array contains the precomputed values needed for this.
      xRef16 = (((xP - position_params[0]) * position_params[4] + position_params[6] + (1 << 11)) >> 12) + position_params[2];  // (H 63)
        
      // 2. The variables xRef and xPhase are derived as follows:
      xRef   = xRef16 >> 4;  // (H 29)
      xPhase = xRef16 % 16;  // (H 30)

      // 5. The sample value tempArray[ n ] with n = 0..7, is derived as follows:
      for (int n = 0; n<8; n++) {
        yPosRL = Clip3( 0, PicHeightInSamplesRefLayerY - 1, yRef + n - 3 );  // (H 36)

        rlPicSampleL = src + yPosRL * srcstride;
        tempArray[n] = (fL[xPhase][0] * rlPicSampleL[ Clip3(0, refW - 1, xRef - 3)] +
                        fL[xPhase][1] * rlPicSampleL[ Clip3(0, refW - 1, xRef - 2)] +
                        fL[xPhase][2] * rlPicSampleL[ Clip3(0, refW - 1, xRef - 1)] +
                        fL[xPhase][3] * rlPicSampleL[ Clip3(0, refW - 1, xRef    )] +
                        fL[xPhase][4] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 1)] +
                        fL[xPhase][5] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 2)] +
                        fL[xPhase][6] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 3)] +
                        fL[xPhase][7] * rlPicSampleL[ Clip3(0, refW - 1, xRef + 4)] ) >> shift1; // (H 38)
      }

      // 6. The resampled luma sample value rsLumaSample is derived as follows:
      rsLumaSample[xP] = ( fL[yPhase][0] * tempArray[0] +
                            fL[yPhase][1] * tempArray[1] +
                            fL[yPhase][2] * tempArray[2] +
                            fL[yPhase][3] * tempArray[3] +
                            fL[yPhase][4] * tempArray[4] +
                            fL[yPhase][5] * tempArray[5] +
                            fL[yPhase][6] * tempArray[6] +
                            fL[yPhase][7] * tempArray[7] + offset ) >> shift2;  // (H 39)
    }
  }
#endif
}
void resampling_process_of_chroma_sample_values_fallback( uint8_t *src, ptrdiff_t srcstride, int src_size[2],
                                                          uint8_t *dst, ptrdiff_t dststride, int dst_size[2],
                                                          int position_params[8], int BitDepthRefLayerC, int BitDepthCurrC)
{
  int PicHeightInSamplesRefLayerC = src_size[1];
  int PicWidthInSamplesRefLayerC = src_size[0];

  int PicHeightInSamplesCurLayerC = dst_size[1];
  int PicWidthInSamplesCurLayerC = dst_size[0];
  
  int xRef16, yRef16, xRef, xPhase, yRef, yPhase;
  int shift1, shift2, offset;
  int yPosRL;
  int tempArray[4];
  uint8_t *rlPicSampleC;
  uint8_t *rsChromaSample;

  // 4. The variables shift1, shift2 and offset are derived as follows:
  shift1 = BitDepthRefLayerC - 8;  // (H 45)
  shift2 = 20 - BitDepthCurrC;     // (H 46)
  offset = 1 << (shift2 - 1);      // (H 47)

  int refWC = PicWidthInSamplesRefLayerC; // (H 49)

  // Table H.2 – 16-phase chroma resampling filter
  int fC[16][4] = { { 0, 64,  0,  0},
                    {-2, 62,  4,  0},
                    {-2, 58, 10, -2},
                    {-4, 56, 14, -2},
                    {-4, 54, 16, -2},
                    {-6, 52, 20, -2},
                    {-6, 46, 28, -4},
                    {-4, 42, 30, -4},
                    {-4, 36, 36, -4},
                    {-4, 30, 42, -4},
                    {-4, 28, 46, -6},
                    {-2, 20, 52, -6},
                    {-2, 16, 54, -4},
                    {-2, 14, 56, -4},
                    {-2, 10, 58, -2},
                    { 0,  4, 62, -2} };

#if USE_FASTER_IMPLEMENTATION
  // Perform horizontal / vertical upsampling seperately.
  // Allocate temporaray buffer
  int16_t *tmp = new int16_t[PicHeightInSamplesRefLayerC * PicWidthInSamplesCurLayerC];
  int16_t *tmpSample;
  int tmpStride = PicWidthInSamplesCurLayerC;

  int refW = PicWidthInSamplesRefLayerC;   // (H 37)

  // Horizontal upsampling
  for (int y = 0; y < PicHeightInSamplesRefLayerC; y++) {

    // Get pointer to the source for this y position
    rlPicSampleC = src + y * srcstride;
    // Get pointer to the temp array for this y position
    tmpSample = tmp + y * tmpStride;

    for (int xP = 0; xP < PicWidthInSamplesCurLayerC; xP++) {
      // 1.
      // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
      // The position_params array contains the precomputed values needed for this.
      xRef16 = (((xP - position_params[0]) * position_params[4] + position_params[6] + (1 << 11)) >> 12) + position_params[2];  // (H 63)

      // 2. The variables xRef and xPhase are derived as follows:
      xRef   = xRef16 >> 4;  // (H 29)
      xPhase = xRef16 % 16;  // (H 30)

      tmpSample[xP] = (fC[xPhase][0] * rlPicSampleC[ Clip3(0, refWC - 1, xRef - 1)] +
                       fC[xPhase][1] * rlPicSampleC[ Clip3(0, refWC - 1, xRef    )] +
                       fC[xPhase][2] * rlPicSampleC[ Clip3(0, refWC - 1, xRef + 1)] +
                       fC[xPhase][3] * rlPicSampleC[ Clip3(0, refWC - 1, xRef + 2)] ) >> shift1; // (H 50)
    }
  }

  // Vertical upsampling
  int refY = PicHeightInSamplesRefLayerC;

  for (int yP = 0; yP < PicHeightInSamplesCurLayerC; yP++) {
    // Get pointer to destination y line
    rsChromaSample = dst + yP * dststride;
    
    // 1.
    // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
    // The position_params array contains the precomputed values needed for this.
    yRef16 = (((yP - position_params[1]) * position_params[5] + position_params[7] + (1 << 11)) >> 12) + position_params[3];  // (H 64)

    // 3. The variables yRef and yPhase are derived as follows:
    //yPhase = yRef16 % 16;  // (H 32)
    yPhase = yRef16 & 15;  // This is what the reference software does. TODO: Double check with the latest standard.
    yRef   = yRef16 >> 4;  // (H 31)

    for (int x = 0; x < PicWidthInSamplesCurLayerC; x++) {
      // Get pointer to temp array y line
      tmpSample = tmp + x;

      // 6. The resampled chroma sample value rsChromaSample is derived as follows:
      rsChromaSample[x] = ( fC[yPhase][0] * tmpSample[ Clip3(0, refY - 1, yRef - 1) * tmpStride ] +
                            fC[yPhase][1] * tmpSample[ Clip3(0, refY - 1, yRef    ) * tmpStride ] +
                            fC[yPhase][2] * tmpSample[ Clip3(0, refY - 1, yRef + 1) * tmpStride ] +
                            fC[yPhase][3] * tmpSample[ Clip3(0, refY - 1, yRef + 2) * tmpStride ] + offset ) >> shift2;  // (H 51)
    }
  }

  delete[] tmp;
  tmp = NULL;

#else
  // H.8.1.4.1.2 Resampling process of chroma sample values
  for (int yP = 0; yP < PicHeightInSamplesCurLayerC; yP++) {
    rsChromaSample = dst + yP * dststride;

    // 1.
    // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
    // The position_params array contains the precomputed values needed for this.
    yRef16 = (((yP - position_params[1]) * position_params[5] + position_params[7] + (1 << 11)) >> 12) + position_params[3];  // (H 64)

    // 3. The variables yRef and yPhase are derived as follows:
    yRef   = yRef16 >> 4;  // (H 43)
    //yPhase = yRef16 % 16;  // (H 44)
    yPhase = yRef16 & 15;  // This is what the reference software does. TODO: Double check with the latest standard.
    
    for (int xP = 0; xP < PicWidthInSamplesCurLayerC; xP++) {
      // 1.
      // H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
      // The position_params array contains the precomputed values needed for this.
      xRef16 = (((xP - position_params[0]) * position_params[4] + position_params[6] + (1 << 11)) >> 12) + position_params[2];  // (H 63)
        
      // 2. The variables xRef and xPhase are derived as follows:
      xRef   = xRef16 >> 4;  // (H 41)
      xPhase = xRef16 % 16;  // (H 42)

      // 5. The sample value tempArray[ n ] with n = 0..3, is derived as follows:
      for (int n = 0; n<4; n++) {
        yPosRL = Clip3( 0, PicHeightInSamplesRefLayerC - 1, yRef + n - 1 );  // (H 48)

        rlPicSampleC = src + yPosRL * srcstride;
        tempArray[n] = (fC[xPhase][0] * rlPicSampleC[ Clip3(0, refWC - 1, xRef - 1)] +
                        fC[xPhase][1] * rlPicSampleC[ Clip3(0, refWC - 1, xRef    )] +
                        fC[xPhase][2] * rlPicSampleC[ Clip3(0, refWC - 1, xRef + 1)] +
                        fC[xPhase][3] * rlPicSampleC[ Clip3(0, refWC - 1, xRef + 2)] ) >> shift1; // (H 50)
      }

      // 6. The resampled chroma sample value rsChromaSample is derived as follows:
      rsChromaSample[xP] = ( fC[yPhase][0] * tempArray[0] +
                             fC[yPhase][1] * tempArray[1] +
                             fC[yPhase][2] * tempArray[2] +
                             fC[yPhase][3] * tempArray[3] + offset ) >> shift2;  // (H 51)

      rsChromaSample[xP] = Clip3(0, ( 1 << BitDepthCurrC ) - 1, rsChromaSample[xP]);  // (H 52)
    }
  }
#endif
}




  