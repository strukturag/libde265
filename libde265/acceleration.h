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

#ifndef DE265_ACCELERATION_H
#define DE265_ACCELERATION_H

#include <stddef.h>
#include <stdint.h>

struct acceleration_functions
{
  void (*put_weighted_pred_avg_8)(uint8_t *_dst, ptrdiff_t dststride,
                                  const int16_t *src1, const int16_t *src2, ptrdiff_t srcstride,
                                  int width, int height);

  void (*put_unweighted_pred_8)(uint8_t *_dst, ptrdiff_t dststride,
                                const int16_t *src, ptrdiff_t srcstride,
                                int width, int height);

  void (*put_weighted_pred_8)(uint8_t *_dst, ptrdiff_t dststride,
                              const int16_t *src, ptrdiff_t srcstride,
                              int width, int height,
                              int w,int o,int log2WD);
  void (*put_weighted_bipred_8)(uint8_t *_dst, ptrdiff_t dststride,
                                const int16_t *src1, const int16_t *src2, ptrdiff_t srcstride,
                                int width, int height,
                                int w1,int o1, int w2,int o2, int log2WD);

  void (*put_hevc_epel_8)(int16_t *dst, ptrdiff_t dststride,
                          const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                          int mx, int my, int16_t* mcbuffer);
  void (*put_hevc_epel_h_8)(int16_t *dst, ptrdiff_t dststride,
                            const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                            int mx, int my, int16_t* mcbuffer);
  void (*put_hevc_epel_v_8)(int16_t *dst, ptrdiff_t dststride,
                            const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                            int mx, int my, int16_t* mcbuffer);
  void (*put_hevc_epel_hv_8)(int16_t *dst, ptrdiff_t dststride,
                             const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                             int mx, int my, int16_t* mcbuffer);

  void (*put_hevc_qpel_8[4][4])(int16_t *dst, ptrdiff_t dststride,
                                const uint8_t *src, ptrdiff_t srcstride, int width, int height,
                                int16_t* mcbuffer);

  // --- inverse transforms ---

  void (*transform_skip_8)(uint8_t *_dst, const int16_t *coeffs, ptrdiff_t _stride); // no transform
  void (*transform_bypass_8)(uint8_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride);
  void (*transform_4x4_dst_add_8)(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride); // iDST
  void (*transform_add_8[4])(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride); // iDCT

  void (*transform_skip_16)(uint16_t *_dst, const int16_t *coeffs, ptrdiff_t _stride, int bit_depth); // no transform
  void (*transform_bypass_16)(uint16_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride, int bit_depth);
  void (*transform_4x4_dst_add_16)(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth); // iDST
  void (*transform_add_16[4])(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth); // iDCT


  template <class pixel_t> void transform_skip(pixel_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
  template <class pixel_t> void transform_bypass(pixel_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride, int bit_depth);
  template <class pixel_t> void transform_4x4_dst_add(pixel_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);
  template <class pixel_t> void transform_add(int sizeIdx, pixel_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth);


  // --- forward transforms ---

  void (*fwd_transform_4x4_dst_8)(int16_t *coeffs, const int16_t* src, ptrdiff_t stride); // fDST

  // indexed with (log2TbSize-2)
  void (*fwd_transform_8[4])     (int16_t *coeffs, const int16_t *src, ptrdiff_t stride); // fDCT


  // forward Hadamard transform (without scaling factor)
  // (4x4,8x8,16x16,32x32) indexed with (log2TbSize-2)
  void (*hadamard_transform_8[4])     (int16_t *coeffs, const int16_t *src, ptrdiff_t stride);
};


template <> inline void acceleration_functions::transform_skip<uint8_t>(uint8_t *dst, const int16_t *coeffs,ptrdiff_t stride, int bit_depth) { transform_skip_8(dst,coeffs,stride); }
template <> inline void acceleration_functions::transform_skip<uint16_t>(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth) { transform_skip_16(dst,coeffs,stride, bit_depth); }

template <> inline void acceleration_functions::transform_bypass<uint8_t>(uint8_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride, int bit_depth) { transform_bypass_8(dst,coeffs,nT,stride); }
template <> inline void acceleration_functions::transform_bypass<uint16_t>(uint16_t *dst, const int16_t *coeffs, int nT, ptrdiff_t stride, int bit_depth) { transform_bypass_16(dst,coeffs,nT,stride, bit_depth); }

template <> inline void acceleration_functions::transform_4x4_dst_add<uint8_t>(uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride,int bit_depth) { transform_4x4_dst_add_8(dst,coeffs,stride); }
template <> inline void acceleration_functions::transform_4x4_dst_add<uint16_t>(uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride,int bit_depth) { transform_4x4_dst_add_16(dst,coeffs,stride,bit_depth); }

template <> inline void acceleration_functions::transform_add<uint8_t>(int sizeIdx, uint8_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth) { transform_add_8[sizeIdx](dst,coeffs,stride); }
template <> inline void acceleration_functions::transform_add<uint16_t>(int sizeIdx, uint16_t *dst, const int16_t *coeffs, ptrdiff_t stride, int bit_depth) { transform_add_16[sizeIdx](dst,coeffs,stride,bit_depth); }

#endif
