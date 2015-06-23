/*
 * H.265 video codec.
 * Copyright (c) 2013 openHEVC contributors
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

#include <emmintrin.h>
#include <tmmintrin.h> // SSSE3
#if HAVE_SSE4_1
#include <smmintrin.h>
#endif

#include "sse-upsampling.h"
#include "libde265/util.h"
#include <assert.h>

/*

Possible operations to use:
- _mm_madd_epi16(a,b) : 
r0 := (a0 * b0) + (a1 * b1)
r1 := (a2 * b2) + (a3 * b3)
r2 := (a4 * b4) + (a5 * b5)
r3 := (a6 * b6) + (a7 * b7)

- _mm_insert_epi16(a,b,im) : Inserts the least significant 16 bits of b into the selected 16-bit integer of a
r0 := (imm == 0) ? b : a0;
r1 := (imm == 1) ? b : a1;
...
r7 := (imm == 7) ? b : a7;

- _mm_maddubs_epi16 : multiplies and adds integers. (16*8bit unsigned, 16*8bit signed) (SSSE3)
r0 := SATURATE_16((a0 * b0) + (a1 * b1))
r1 := SATURATE_16((a2 * b2) + (a3 * b3))
...
r7 := SATURATE_16((a14 * b14) + (a15 * b15))

- _mm_mullo_epi16 : Multiplies the 8 signed or unsigned 16-bit integers from a by the 8 signed or unsigned 16-bit integers from b.
r0 := (a0 * b0)[15:0]
r1 := (a1 * b1)[15:0]
...
r7 := (a7 * b7)[15:0]

- _mm_shuffle_epi8 : This instruction shuffles 16-byte parameters from a 128-bit parameter.
  Maybe for resorting something ...

// horizontal add of four 32 bit partial sums and return result
// Two steps to add 4 32 bit counters
vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 8));
vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));

- _mm_cvtsi128_si32: Moves the least significant 32 bits of a to a 32-bit integer.

- _mm_setr_epi64: Set the 2 __m64 values

- SSE4: _mm_insert_epi8 (Insert byte into 128bit vector)
- SSE4: _mm_extract_epi8 (Extract 8bit)
- SSE4: _mm_blendv_epi8 (Reorder from a,b to output per byte)

- SSE2: _mm_slli_epi64  (Shift left 2 64 bit values)
- SSE2: _mm_unpackhi_epi8 Interleaves 8 bit wise


*/

#ifdef HAVE_SSE4_1
#define MEMORY_PADDING  8
#else
#define MEMORY_PADDING  0
#endif

#define STANDARD_ALIGNMENT 16

#ifdef HAVE___MINGW_ALIGNED_MALLOC
#define ALLOC_ALIGNED(alignment, size)         __mingw_aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                      __mingw_aligned_free((mem))
#elif _WIN32
#define ALLOC_ALIGNED(alignment, size)         _aligned_malloc((size), (alignment))
#define FREE_ALIGNED(mem)                      _aligned_free((mem))
#elif __APPLE__
static inline void *ALLOC_ALIGNED(size_t alignment, size_t size) {
    void *mem = NULL;
    if (posix_memalign(&mem, alignment, size) != 0) {
        return NULL;
    }
    return mem;
};
#define FREE_ALIGNED(mem)                      free((mem))
#else
#define ALLOC_ALIGNED(alignment, size)      memalign((alignment), (size))
#define FREE_ALIGNED(mem)                   free((mem))
#endif

#define ALLOC_ALIGNED_16(size)              ALLOC_ALIGNED(16, size)

// H.8.1.4.1.3 Derivation process for reference layer sample location in units of 1/16-th sample
// The position_params array contains the precomputed values needed for this. (H 63)
#define MAP_X_TO_REF_16(xP) (((xP - position_params[0]) * position_params[4] + position_params[6] + (1 << 11)) >> 12) + position_params[2]
#define MAP_Y_TO_REF_16(yP) (((yP - position_params[1]) * position_params[5] + position_params[7] + (1 << 11)) >> 12) + position_params[3]
// Get xRef in samples positions and the phase (H 63) plus (H 29) and (H 30).
#define MAP_X_TO_REF_PHASE(xP, xRef, xPhase) int xRef16 = MAP_X_TO_REF_16(xP); xRef = xRef16 >> 4; xPhase = xRef16 % 16;
#define MAP_Y_TO_REF_PHASE(yP, yRef, yPhase) int yRef16 = MAP_Y_TO_REF_16(yP); yRef = yRef16 >> 4; yPhase = yRef16 % 16;

//// Table H.1 – 16-phase luma resampling filter
//int fL[16][8] = { { 0, 0,   0, 64,  0,   0, 0,  0},
//                  { 0, 1,  -3, 63,  4,  -2, 1,  0},
//                  {-1, 2,  -5, 62,  8,  -3, 1,  0},
//                  {-1, 3,  -8, 60, 13,  -4, 1,  0},
//                  {-1, 4, -10, 58, 17,  -5, 1,  0},
//                  {-1, 4, -11, 52, 26,  -8, 3, -1},
//                  {-1, 3,  -9, 47, 31, -10, 4, -1},
//                  {-1, 4, -11, 45, 34, -10, 4, -1},
//                  {-1, 4, -11, 40, 40, -11, 4, -1},
//                  {-1, 4, -10, 34, 45, -11, 4, -1},
//                  {-1, 4, -10, 31, 47,  -9, 3, -1},
//                  {-1, 3,  -8, 26, 52, -11, 4, -1},
//                  { 0, 1,  -5, 17, 58, -10, 4, -1},
//                  { 0, 1,  -4, 13, 60,  -8, 3, -1},
//                  { 0, 1,  -3,  8, 62,  -5, 2, -1},
//                  { 0, 1,  -2,  4, 63,  -3, 1,  0} };

// 8 bit representation of the above filters. Each 8 bits are one coefficient.
ALIGNED_16(const __m64) fL_8bit[16] = { 0x0000004000000000,
                                        0x0001fd3f04fe0100,
                                        0xff02fb3e08fd0100,
                                        0xff03f83c0dfc0100,
                                        0xff04f63a11fb0100,
                                        0xff04f5341af803ff,
                                        0xff03f72f1ff604ff,
                                        0xff04f52d22f604ff,
                                        0xff04f52828f504ff,
                                        0xff04f6222df504ff,
                                        0xff04f61f2ff703ff,
                                        0xff03f81a34f504ff,
                                        0x0001fb113af604ff,
                                        0x0001fc0d3cf803ff,
                                        0x0001fd083efb02ff,
                                        0x0001fe043ffd0100 };

// 16 bit representation of the above filters. Each 16 bits are one coefficient.
ALIGNED_16(const __m64) fL_16bit[16][2] = { {0x0000000000000040, 0x0000000000000000},
                                            {0x00000001fffd003f, 0x0004fffe00010000},
                                            {0xffff0002fffb003e, 0x0008fffd00010000},
                                            {0xffff0003fff8003c, 0x000dfffc00010000},
                                            {0xffff0004fff6003a, 0x0011fffb00010000},
                                            {0xffff0004fff50034, 0x001afff80003ffff},
                                            {0xffff0003fff7002f, 0x001ffff60004ffff},
                                            {0xffff0004fff5002d, 0x0022fff60004ffff},
                                            {0xffff0004fff50028, 0x0028fff50004ffff},
                                            {0xffff0004fff60022, 0x002dfff50004ffff},
                                            {0xffff0004fff6001f, 0x002ffff70003ffff},
                                            {0xffff0003fff8001a, 0x0034fff50004ffff},
                                            {0x00000001fffb0011, 0x003afff60004ffff},
                                            {0x00000001fffc000d, 0x003cfff80003ffff},
                                            {0x00000001fffd0008, 0x003efffb0002ffff},
                                            {0x00000001fffe0004, 0x003ffffd00010000} };

// Measure the execution time of the upsampling and print it to stdout
#define MEASURE_EXECUTION_TIME 0
// Profile by getting CPU ticks and running the upsampling function multiple times
#define PROFILE_FUNCTION 1

#if MEASURE_EXECUTION_TIME
#define INTMAX_MAX 9223372036854775807LL
#include <chrono>
using namespace std;
using namespace std::chrono;
#endif
#if PROFILE_FUNCTION
#ifdef _WIN32
#include <intrin.h>
#include <inttypes.h>
uint64_t rdtsc(){
    return __rdtsc();
}
#else
// Linux
uint64_t rdtsc(){
    unsigned int lo,hi;
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    return ((uint64_t)hi << 32) | lo;
}
#endif
#endif

// Upsampling process for 8 bit input ...
void resampling_process_of_luma_sample_values_sse(uint8_t *src, ptrdiff_t srcstride, int src_size[2],
                                                  uint8_t *dst, ptrdiff_t dststride, int dst_size[2],
                                                  int position_params[10])
{
#if MEASURE_EXECUTION_TIME
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
#endif
#if PROFILE_FUNCTION

  // Measure the operations (loading/saving/arithmetic)
  uint64_t lastTicks1;
  uint64_t loadTicks_ver = 0;
  uint64_t loadTicks_hor = 0;
  uint64_t arithmeticTicks_ver = 0;
  uint64_t arithmeticTicks_hor = 0;
  uint64_t saveTicks_ver = 0;
  uint64_t saveTicks_hor = 0;

  // Measure the parts (memAlloc/Hor/Ver)
  uint64_t lastTicksPart;
  uint64_t horizontalTicks = 0;
  uint64_t verticalTicks = 0;
  uint64_t allocBuffer = 0;
    
  uint64_t beforeWholeFunction = rdtsc();
  // Run this function 100 times
  for (int run=0; run<100; run++) {
#endif

  static int16_t* s_tmp = NULL;  // Init static pointer to tempraray array for upsampling
  static int s_tmp_size = -1;

  // Reference layer size
  int PicHeightInSamplesRefLayerY = src_size[1];
  int PicWidthInSamplesRefLayerY  = src_size[0];
  assert(PicHeightInSamplesRefLayerY % 2 == 0);   // Pic height must be dividible by 2 for this function to work. This is a requirement of HEVC so this should always be the case.

  // Current layer size
  int PicHeightInSamplesCurLayerY = dst_size[1];
  int PicWidthInSamplesCurLayerY  = dst_size[0];

  int BitDepthRefLayerY = position_params[8];
  int BitDepthCurrY     = position_params[9];
  int clipMax           = (1 << BitDepthCurrY) - 1;

  // 4. The variables shift1, shift2 and offset are derived as follows:
  int shift2 = 20 - BitDepthCurrY;     // (H 34)
  int offset = 1 << (shift2 - 1);      // (H 35)

  // Perform horizontal / vertical upsampling seperately.
  
  // Allocate temporaray buffer
#if PROFILE_FUNCTION
  lastTicksPart = rdtsc();
#endif
  int alignment = STANDARD_ALIGNMENT;
  int tmpStride = ((PicWidthInSamplesCurLayerY + alignment-1) / alignment * alignment); // In bytes
  int tmpHeight = PicHeightInSamplesRefLayerY;
  if (s_tmp_size == -1 && s_tmp == NULL) {
    // Allocate the temporaray tmp buffer
    s_tmp = (int16_t *)ALLOC_ALIGNED_16(tmpStride * tmpHeight * sizeof(int16_t) + MEMORY_PADDING);
    s_tmp_size = tmpStride * tmpHeight * sizeof(int16_t) + MEMORY_PADDING;
  }
  else if (s_tmp_size < tmpStride * tmpHeight * sizeof(int16_t) + MEMORY_PADDING) {
    // The buffer is not big enough. TODO (Maybe a version of realloc would be better??)
    // Allocate buffer that is big enough
    FREE_ALIGNED(s_tmp);
    s_tmp = (int16_t *)ALLOC_ALIGNED_16(tmpStride * tmpHeight * sizeof(int16_t) + MEMORY_PADDING);
    s_tmp_size = tmpStride * tmpHeight * sizeof(int16_t) + MEMORY_PADDING;
  }
  int16_t* tmp = s_tmp;
#if PROFILE_FUNCTION
  allocBuffer += rdtsc() - lastTicksPart;
#endif

  // -------- Horizontal upsampling ------------ //
#if PROFILE_FUNCTION
  lastTicksPart = rdtsc();
#endif
  __m128i in;           // Contains 16 input sample values that we will perform the multiplication on
  __m128i filter;       // Contains two filters
  __m128i result;       // Result of multiplication

  in = _mm_setzero_si128(); // Set in to 0
  
  // Get the number of samples we have to pad
  int xRef, xPhase;
  MAP_X_TO_REF_PHASE(0, xRef, xPhase);
  xRef -= 3;  // For the filter (3 coefficients on the left)
  assert(xRef > -8 && xRef <= 0); // No shift for more than 8 bytes. (Is this even possible?)
  int shift = abs(xRef);
  
  // Get pointers to y lines
  uint8_t *yLine0 = src;
  uint8_t *yLine1 = src + srcstride;
  int16_t *yLineOut0 = tmp;
  int16_t *yLineOut1 = tmp + tmpStride;
  for (int y = 0; y < PicHeightInSamplesRefLayerY; y += 2) {    // Always process pairs of y lines

    int xReadPos = 0;
    
#if PROFILE_FUNCTION
    lastTicks1 = rdtsc();
#endif
    // Fill the input buffer (in) and apply padding.
    int byte0 = *yLine0;    // Get the left most byte of this and the next line
    int byte1 = *yLine1;
    int iByteCount;
    for (iByteCount = 0; iByteCount < shift; iByteCount++) { 
      // shift left
      in = _mm_slli_epi64(in, 8);

      // Insert the left most byte 'shift' times
      in = _mm_insert_epi8(in, byte0, 8);
      in = _mm_insert_epi8(in, byte1, 0);
    }
    
    // Fill the rest of the buffer with bytes from input
    for (xReadPos = 0; iByteCount < 8; iByteCount++) {
      // Get bytes
      byte0 = *(yLine0 + xReadPos);
      byte1 = *(yLine1 + xReadPos++);

      // shift left
      in = _mm_slli_epi64(in, 8);

      // insert bytes
      in = _mm_insert_epi8(in, byte0, 8);
      in = _mm_insert_epi8(in, byte1, 0);
    }
    int xRefBuf = 0; // The buffer is now ready to start at xRef of 0
#if PROFILE_FUNCTION
    loadTicks_hor += rdtsc() - lastTicks1;
#endif
    
    // The input buffer is set up. Here we go:
    for (int xP=0; xP < PicWidthInSamplesCurLayerY; xP++) {
      // Map from this layer x to refLayer x
      MAP_X_TO_REF_PHASE(xP, xRef, xPhase);

#if PROFILE_FUNCTION
      lastTicks1 = rdtsc();
#endif
      while (xRef != xRefBuf) {
        // We have to load bytes until xRefBuf and xRef match again
        if (xReadPos < PicWidthInSamplesRefLayerY) {  // Otherwise repeat last byte (padding)
          // Get bytes
          byte0 = *(yLine0 + xReadPos);
          byte1 = *(yLine1 + xReadPos++);
        }

        // shift left
        in = _mm_slli_epi64(in, 8);

        // insert bytes
        in = _mm_insert_epi8(in, byte0, 8);
        in = _mm_insert_epi8(in, byte1, 0);

        xRefBuf++;
      }
#if PROFILE_FUNCTION
      loadTicks_hor += rdtsc() - lastTicks1;
#endif

#if PROFILE_FUNCTION
      lastTicks1 = rdtsc();
#endif
      // Get the correct filter (according to xPhase)
      filter = _mm_setr_epi64(fL_8bit[xPhase], fL_8bit[xPhase]);

      // multiply
      result = _mm_maddubs_epi16(in, filter);
      
      // Add up results
      result = _mm_add_epi16(result, _mm_srli_si128(result, 4));
      result = _mm_add_epi16(result, _mm_srli_si128(result, 2));

#if PROFILE_FUNCTION
      arithmeticTicks_hor += rdtsc() - lastTicks1;
      lastTicks1 = rdtsc();
#endif

      // Save results
      *(yLineOut0 + xP ) = _mm_extract_epi16(result, 4);
      *(yLineOut1 + xP ) = _mm_extract_epi16(result, 0);

#if PROFILE_FUNCTION
      saveTicks_hor += rdtsc() - lastTicks1;
#endif
    }

    // Go to next y lines
    yLine0 += srcstride * 2;
    yLine1 += srcstride * 2;
    yLineOut0 += tmpStride * 2;
    yLineOut1 += tmpStride * 2;
  }

  // -------- Vertical upsampling ------------ ////
#if PROFILE_FUNCTION
  horizontalTicks += rdtsc() - lastTicksPart;
  lastTicksPart = rdtsc();
#endif
  int yRef, yPhase;
  MAP_Y_TO_REF_PHASE(0, yRef, yPhase);
  yRef -= 3;  // For the filter (3 coefficients on the top)
  assert(yRef > -8 && yRef <= 0); // No shift for more than 8 bytes. (Is this even possible?)
  shift = abs(yRef);

  int16_t *yLineIn;
  uint8_t  *yLineOut;
  for (int xP=0; xP < PicWidthInSamplesCurLayerY; xP++) {
    // Get top y buffer for x position
    yLineIn = tmp + xP;
    yLineOut = dst + xP;
    
    int yReadPos = 0;

    // Fill the input buffer (in) and apply padding.
#if PROFILE_FUNCTION
    lastTicks1 = rdtsc();
#endif
    int iByteCount;
    int byte = *(yLineIn); // Read top val

    for (iByteCount = 0; iByteCount < shift; iByteCount++) { 
      // shift left
      in = _mm_slli_si128(in, 2);

      // Insert the top most byte 'shift' times
      in = _mm_insert_epi16(in, byte, 0);
    }
    
    // Fill the rest of the buffer with bytes from input
    for (; iByteCount < 8; iByteCount++) {
      // Get bytes
      byte = *(yLineIn); yLineIn += tmpStride;  // Read val and goto next y line
      yReadPos++;

      // shift left
      in = _mm_slli_si128(in, 2);

      // insert bytes
      in = _mm_insert_epi16(in, byte, 0);
    }
    int yRefBuf = 0; // The buffer is now ready to start at yRef of 0
#if PROFILE_FUNCTION
    loadTicks_ver += rdtsc() - lastTicks1;
#endif

    // The input buffer is set up. Here we go:
    for (int y = 0; y < PicHeightInSamplesCurLayerY; y++) {
      // Map from this layer y to refLayer y
      MAP_X_TO_REF_PHASE(y, yRef, yPhase);

#if PROFILE_FUNCTION
      lastTicks1 = rdtsc();
#endif
      while (yRef != yRefBuf) {
        // We have to load bytes until yRefBuf and yRef match again
        if (yReadPos < PicHeightInSamplesRefLayerY) {  // Otherwise repeat last byte (padding)
          // Get bytes
         byte = *(yLineIn); yLineIn += tmpStride;  // Read val and goto next y line
         yReadPos++;
        }

        // shift left
        in = _mm_slli_si128(in, 2);

        // insert bytes
        in = _mm_insert_epi16(in, byte, 0);
        
        yRefBuf++;
      }
#if PROFILE_FUNCTION
      loadTicks_ver += rdtsc() - lastTicks1;
      lastTicks1 = rdtsc();
#endif

      // Get the correct filter (according to yPhase)
      filter = _mm_setr_epi64(fL_16bit[yPhase][1], fL_16bit[yPhase][0]);

      // multiply (16 bit input, out 32bit)
      result = _mm_madd_epi16 (in, filter);

      // Add up results
      result = _mm_add_epi32(result, _mm_srli_si128(result, 8));
      result = _mm_add_epi32(result, _mm_srli_si128(result, 4));
#if PROFILE_FUNCTION
      arithmeticTicks_ver += rdtsc() - lastTicks1;
      lastTicks1 = rdtsc();
#endif
            
      // Save results
      *(yLineOut) = Clip3(0, 255, (( _mm_extract_epi32(result, 0) + offset ) >> shift2) );
      yLineOut += dststride;

#if PROFILE_FUNCTION
      saveTicks_ver += rdtsc() - lastTicks1;
#endif

    }
  }

#if PROFILE_FUNCTION
  verticalTicks += rdtsc() - lastTicksPart;
  }

  uint64_t wholeTime = rdtsc() - beforeWholeFunction;
  double d_wholeTime = wholeTime;
  double p1 = allocBuffer / d_wholeTime * 100;
  double p2 = horizontalTicks / d_wholeTime * 100;
  double p3 = verticalTicks / d_wholeTime * 100;
  // Print values
  printf("Mem Alloc: %" PRIu64 " (%f)\n", allocBuffer, p1);
  printf("Hor Filt : %" PRIu64 " (%f)\n", horizontalTicks, p2);
  printf("Ver Filt : %" PRIu64 " (%f)\n", verticalTicks, p3);
  printf("\n");

  uint64_t verSum = loadTicks_ver + arithmeticTicks_ver + saveTicks_ver;
  uint64_t horSum = loadTicks_hor + arithmeticTicks_hor + saveTicks_hor;
  double d_verSum = verSum;
  double d_horSum = horSum;
  double p4 = loadTicks_ver / d_verSum * 100;
  double p5 = arithmeticTicks_ver / d_verSum * 100;
  double p6 = saveTicks_ver / d_verSum * 100;
  double p7 = loadTicks_hor / d_horSum * 100;
  double p8 = arithmeticTicks_hor / d_horSum * 100;
  double p9 = saveTicks_hor / d_horSum * 100;

  printf("Load  Ver: %" PRIu64 " (%f)\n", loadTicks_ver, p4);
  printf("Load  Hor: %" PRIu64 " (%f)\n", loadTicks_hor, p7);
  printf("Arith Ver: %" PRIu64 " (%f)\n", arithmeticTicks_ver, p5);
  printf("Arith Hor: %" PRIu64 " (%f)\n", arithmeticTicks_hor, p8);
  printf("Save  Ver: %" PRIu64 " (%f)\n", saveTicks_ver, p6);
  printf("Save  Hor: %" PRIu64 " (%f)\n", saveTicks_hor, p9);
#endif
#if MEASURE_EXECUTION_TIME
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  printf("Upsampling Y from (%dx%d) to (%dx%d) took %d us\n", PicWidthInSamplesRefLayerY, PicHeightInSamplesRefLayerY, PicWidthInSamplesCurLayerY, PicHeightInSamplesCurLayerY, duration);
#endif
} 




//// The padding/shifting values. Needed for the operation: Shift right and shift in the highest value (padding).
  //// Shifts from 0-7 are possible. padding0 are the lower 64 bits, padding1 are the upper 64 bits.
  //__m128i padding_mask; // The mask used for padding (use _mm_shuffle_epi8 with this mask to perform right shift and padding)
  // padding_mask = _mm_setr_epi64(padding0[shift], padding1[shift]);
  //__m64 padding1[8] = { 0x0001020304050607,
  //                      0x0000010203040506,
  //                      0x0000000102030405,
  //                      0x0000000001020304,
  //                      0x0000000000010203,
  //                      0x0000000000000102,
  //                      0x0000000000000001,
  //                      0x0000000000000000
  //                      /*0x0000000000000000,
  //                      0x0000000000000000,
  //                      0x0000000000000000,
  //                      0x0000000000000000,
  //                      0x0000000000000000,
  //                      0x0000000000000000,
  //                      0x0000000000000000,
  //                      0x0000000000000000*/ };

  //__m64 padding0[8] = { 0x08090a0b0c0d0e0f,
  //                      0x0808090a0b0c0d0e,
  //                      0x080808090a0b0c0d,
  //                      0x08080808090a0b0c,
  //                      0x0808080808090a0b,
  //                      0x080808080808090a,
  //                      0x0808080808080809,
  //                      0x0808080808080808
  //                      /*0x0001020304050607,
  //                      0x0000010203040506,
  //                      0x0000000102030405,
  //                      0x0000000001020304,
  //                      0x0000000000010203,
  //                      0x0000000000000102,
  //                      0x0000000000000001,
  //                      0x0000000000000000*/ };