
#ifndef SSE_MOTION_H
#define SSE_MOTION_H

#include <stddef.h>
#include <stdint.h>


void put_unweighted_pred_8_sse(uint8_t *_dst, ptrdiff_t dststride,
                               int16_t *src, ptrdiff_t srcstride,
                               int width, int height);

void put_weighted_pred_avg_8_sse(uint8_t *_dst, ptrdiff_t dststride,
                                 int16_t *src1, int16_t *src2,
                                 ptrdiff_t srcstride, int width,
                                 int height);

#endif
