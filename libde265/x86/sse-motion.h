
#ifndef SSE_MOTION_H
#define SSE_MOTION_H

#include <stddef.h>
#include <stdint.h>


void ff_hevc_put_unweighted_pred_8_sse(uint8_t *_dst, ptrdiff_t dststride,
                               int16_t *src, ptrdiff_t srcstride,
                               int width, int height);

void ff_hevc_put_weighted_pred_avg_8_sse(uint8_t *_dst, ptrdiff_t dststride,
                                 int16_t *src1, int16_t *src2,
                                 ptrdiff_t srcstride, int width,
                                 int height);

void ff_hevc_put_hevc_epel_pixels_8_sse(int16_t *dst, ptrdiff_t dststride,
                                        uint8_t *_src, ptrdiff_t srcstride,
                                        int width, int height,
                                        int mx, int my, int16_t* mcbuffer);
void ff_hevc_put_hevc_epel_h_8_sse(int16_t *dst, ptrdiff_t dststride,
                                   uint8_t *_src, ptrdiff_t srcstride,
                                   int width, int height,
                                   int mx, int my, int16_t* mcbuffer);
void ff_hevc_put_hevc_epel_v_8_sse(int16_t *dst, ptrdiff_t dststride,
                                   uint8_t *_src, ptrdiff_t srcstride,
                                   int width, int height,
                                   int mx, int my, int16_t* mcbuffer);
void ff_hevc_put_hevc_epel_hv_8_sse(int16_t *dst, ptrdiff_t dststride,
                                    uint8_t *_src, ptrdiff_t srcstride,
                                    int width, int height,
                                    int mx, int my, int16_t* mcbuffer);

#endif
