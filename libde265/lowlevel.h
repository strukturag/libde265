
#ifndef DE265_LOWLEVEL_H
#define DE265_LOWLEVEL_H

#include <stddef.h>
#include <stdint.h>

struct lowlevel_functions
{
  void (*put_weighted_pred_avg_8)(uint8_t *_dst, ptrdiff_t dststride,
                                  int16_t *src1, int16_t *src2, ptrdiff_t srcstride,
                                  int width, int height);

  void (*put_unweighted_pred_8)(uint8_t *_dst, ptrdiff_t dststride,
                                int16_t *src, ptrdiff_t srcstride,
                                int width, int height);

  void (*put_weighted_pred_8)(uint8_t *_dst, ptrdiff_t dststride,
                              int16_t *src, ptrdiff_t srcstride,
                              int width, int height,
                              int w,int o,int log2WD);
  void (*put_weighted_bipred_8)(uint8_t *_dst, ptrdiff_t dststride,
                                int16_t *src1, int16_t *src2, ptrdiff_t srcstride,
                                int width, int height,
                                int w1,int o1, int w2,int o2, int log2WD);

  void (*put_hevc_epel_8)(int16_t *dst, ptrdiff_t dststride,
                          uint8_t *src, ptrdiff_t srcstride, int width, int height,
                          int mx, int my, int16_t* mcbuffer);
  void (*put_hevc_epel_h_8)(int16_t *dst, ptrdiff_t dststride,
                            uint8_t *src, ptrdiff_t srcstride, int width, int height,
                            int mx, int my, int16_t* mcbuffer);
  void (*put_hevc_epel_v_8)(int16_t *dst, ptrdiff_t dststride,
                            uint8_t *src, ptrdiff_t srcstride, int width, int height,
                            int mx, int my, int16_t* mcbuffer);
  void (*put_hevc_epel_hv_8)(int16_t *dst, ptrdiff_t dststride,
                             uint8_t *src, ptrdiff_t srcstride, int width, int height,
                             int mx, int my, int16_t* mcbuffer);

  void (*put_hevc_qpel_8[4][4])(int16_t *dst, ptrdiff_t dststride,
                                uint8_t *src, ptrdiff_t srcstride, int width, int height,
                                int16_t* mcbuffer);

  void (*transform_skip_8)(uint8_t *_dst, int16_t *coeffs, ptrdiff_t _stride); // no transform
  void (*transform_4x4_luma_add_8)(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride); // iDST

  void (*transform_4x4_add_8)(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride); // iDCT
  void (*transform_8x8_add_8)(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride); // iDCT
  void (*transform_16x16_add_8)(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride); // iDCT
  void (*transform_32x32_add_8)(uint8_t *dst, int16_t *coeffs, ptrdiff_t stride); // iDCT
};

#endif
