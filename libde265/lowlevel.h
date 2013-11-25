
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
};

#endif
