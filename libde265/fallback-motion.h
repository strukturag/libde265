
#ifndef FALLBACK_MOTION_H
#define FALLBACK_MOTION_H

#include <stddef.h>
#include <stdint.h>


void put_weighted_pred_avg_8_fallback(uint8_t *dst, ptrdiff_t dststride,
                                      int16_t *src1, int16_t *src2,
                                      ptrdiff_t srcstride, int width,
                                      int height);

void put_unweighted_pred_8_fallback(uint8_t *_dst, ptrdiff_t dststride,
                                    int16_t *src, ptrdiff_t srcstride,
                                    int width, int height);

void put_epel_8_fallback(int16_t *dst, ptrdiff_t dststride,
                    uint8_t *_src, ptrdiff_t srcstride,
                    int width, int height,
                    int mx, int my, int16_t* mcbuffer);
void put_epel_hv_8_fallback(int16_t *dst, ptrdiff_t dststride,
                            uint8_t *_src, ptrdiff_t srcstride,
                            int width, int height,
                            int mx, int my, int16_t* mcbuffer);

#endif
