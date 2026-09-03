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

#ifndef FALLBACK_MOTION_H
#define FALLBACK_MOTION_H

#include <stddef.h>
#include <stdint.h>


void put_weighted_pred_avg_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                      const int16_t *src1, const int16_t *src2,
                                      ptrdiff_t srcstride, int width,
                                      int height);

void put_unweighted_pred_8_fallback(uint8_t *_dst, ptrdiff_t dststride,
                                    const int16_t *src, ptrdiff_t srcstride,
                                    int width, int height);

void put_weighted_pred_8_fallback(uint8_t *_dst, ptrdiff_t dststride,
                                  const int16_t *src, ptrdiff_t srcstride,
                                  int width, int height,
                                  int w,int o,int log2WD);
void put_weighted_bipred_8_fallback(uint8_t *_dst, ptrdiff_t dststride,
                                    const int16_t *src1, const int16_t *src2, ptrdiff_t srcstride,
                                    int width, int height,
                                    int w1,int o1, int w2,int o2, int log2WD);

// The 16-bit pixel kernels are templates on the type of the intermediate
// prediction samples (inter_t): int16_t up to MC_MAX_BIT_DEPTH_INT16 (see
// acceleration.h), int32_t above. Instantiated for both types in fallback-motion.cc.

template <class inter_t>
void put_weighted_pred_avg_16_fallback(uint16_t *dst, ptrdiff_t dststride,
                                       const inter_t *src1, const inter_t *src2,
                                       ptrdiff_t srcstride, int width,
                                       int height, int bit_depth);

template <class inter_t>
void put_unweighted_pred_16_fallback(uint16_t *_dst, ptrdiff_t dststride,
                                     const inter_t *src, ptrdiff_t srcstride,
                                     int width, int height, int bit_depth);

template <class inter_t>
void put_weighted_pred_16_fallback(uint16_t *_dst, ptrdiff_t dststride,
                                   const inter_t *src, ptrdiff_t srcstride,
                                   int width, int height,
                                   int w,int o,int log2WD, int bit_depth);
template <class inter_t>
void put_weighted_bipred_16_fallback(uint16_t *_dst, ptrdiff_t dststride,
                                     const inter_t *src1, const inter_t *src2, ptrdiff_t srcstride,
                                     int width, int height,
                                     int w1,int o1, int w2,int o2, int log2WD, int bit_depth);



void put_epel_8_fallback(int16_t *dst, ptrdiff_t dststride,
                         const uint8_t *_src, ptrdiff_t srcstride,
                         int width, int height,
                         int mx, int my, int16_t* mcbuffer);

template <class inter_t>
void put_epel_16_fallback(inter_t *out, ptrdiff_t out_stride,
                          const uint16_t *src, ptrdiff_t src_stride,
                          int width, int height,
                          int mx, int my, inter_t* mcbuffer, int bit_depth);

template <class pixel_t, class inter_t>
void put_epel_hv_fallback(inter_t *dst, ptrdiff_t dststride,
                          const pixel_t *_src, ptrdiff_t srcstride,
                          int width, int height,
                          int mx, int my, inter_t* mcbuffer, int bit_depth);


#define QPEL(x,y) void put_qpel_ ## x ## _ ## y ## _fallback(int16_t *out, ptrdiff_t out_stride, \
                           const uint8_t *src, ptrdiff_t srcstride, \
                           int nPbW, int nPbH, int16_t* mcbuffer)
QPEL(0,0); QPEL(0,1); QPEL(0,2); QPEL(0,3);
QPEL(1,0); QPEL(1,1); QPEL(1,2); QPEL(1,3);
QPEL(2,0); QPEL(2,1); QPEL(2,2); QPEL(2,3);
QPEL(3,0); QPEL(3,1); QPEL(3,2); QPEL(3,3);

#undef QPEL


template <class inter_t>
void put_qpel_0_0_fallback_16(inter_t *out, ptrdiff_t out_stride,
                              const uint16_t *src, ptrdiff_t srcstride,
                              int nPbW, int nPbH, inter_t* mcbuffer, int bit_depth);

#define QPEL(x,y) void put_qpel_ ## x ## _ ## y ## _fallback_16(int16_t *out, ptrdiff_t out_stride, \
                           const uint16_t *src, ptrdiff_t srcstride, \
                           int nPbW, int nPbH, int16_t* mcbuffer, int bit_depth)
/*       */ QPEL(0,1); QPEL(0,2); QPEL(0,3);
QPEL(1,0); QPEL(1,1); QPEL(1,2); QPEL(1,3);
QPEL(2,0); QPEL(2,1); QPEL(2,2); QPEL(2,3);
QPEL(3,0); QPEL(3,1); QPEL(3,2); QPEL(3,3);

#undef QPEL

// Same with int32_t intermediates (BitDepth > 12), for all fractional
// positions including full-sample (xFracL == yFracL == 0).
void put_qpel_fallback_16_32(int32_t *out, ptrdiff_t out_stride,
                             const uint16_t *src, ptrdiff_t srcstride,
                             int nPbW, int nPbH, int32_t* mcbuffer,
                             int xFracL, int yFracL, int bit_depth);

#endif
