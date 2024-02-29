#ifndef LIBDE265_NEON_INTRAPRED
#define LIBDE265_NEON_INTRAPRED

#include <stdint.h>
#include "neon_common.h"
#include "slice.h"


void intra_prediction_DC_neon_8(uint8_t *dst, int dstStride, int nT, int cIdx, /* const */ uint8_t *border);

void intra_prediction_angular_27_34_neon(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border);

void intra_prediction_angular_18_26_neon(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border);

void intra_prediction_angular_10_17_neon(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border);

void intra_prediction_angular_2_9_neon(uint8_t* dst, int dstStride,
                                       int bit_depth, bool disableIntraBoundaryFilter,
                                       int xB0,int yB0,
                                       enum IntraPredMode intraPredMode,
                                       int nT,int cIdx,
                                       uint8_t * border);

void intra_prediction_sample_filtering_neon(const seq_parameter_set& sps,
                                            uint8_t* p,
                                            int nT, int cIdx,
                                            enum IntraPredMode intraPredMode);

void intra_prediction_planar_neon(uint8_t *dst, int dstStride, int nT,int cIdx, uint8_t *border);

#endif