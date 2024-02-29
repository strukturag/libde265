#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <iostream>

#include <cassert>
#include <cstring>
#include <arm_neon.h>

#include "./libde265/util.h"
#include "neon_intrapred.h"
#include "intrapred.h"

#include <sys/time.h>


// Sum all the elements in the vector into the low 32 bits.
inline uint32x2_t Sum(const uint16x4_t val) {
  const uint32x2_t sum = vpaddl_u16(val);
  return vpadd_u32(sum, sum);
}

// Sum all the elements in the vector into the low 32 bits.
inline uint32x2_t Sum(const uint16x8_t val) {
  const uint32x4_t sum_0 = vpaddlq_u16(val);
  const uint64x2_t sum_1 = vpaddlq_u32(sum_0);
  return vadd_u32(vget_low_u32(vreinterpretq_u32_u64(sum_1)),
                  vget_high_u32(vreinterpretq_u32_u64(sum_1)));
}


LIBDE265_INLINE uint32x2_t intra_pred_DcVal_neon(uint8_t *dst, int dstStride, int nT, int cIdx, const uint8_t *border)
{
  uint32x2_t DcSum ;
  uint32x2_t DcnT  ;

  if(nT == 4) {
    uint8x8_t  val = vdup_n_u8(0);
    val = Load4<0>(border+1,val);
    val = Load4<1>(border-nT,val);
    DcSum = Sum(vpaddl_u8(val));
    DcnT  = vdup_n_u32(4);
    return vshr_n_u32(vadd_u32(DcSum, DcnT),3);
  }
  else if(nT == 8) {
    uint8x16_t val_t0 = vdupq_n_u8(0);
    val_t0 = Load8<0>(border+1,val_t0);
    val_t0 = Load8<1>(border-nT,val_t0);
    DcSum = Sum(vpaddlq_u8(val_t0));
    DcnT  = vdup_n_u32(8);
    return vshr_n_u32(vadd_u32(DcSum, DcnT),4);
  }
  else if(nT == 16) {
    uint8x16_t  val_t0 = vdupq_n_u8(0);
    uint8x16_t  val_l0 = vdupq_n_u8(0);
    val_t0 = Load8<0>(border+1,val_t0);
    val_t0 = Load8<1>(border+1+nT/2,val_t0);
    val_l0 = Load8<0>(border-nT,val_l0);
    val_l0 = Load8<1>(border-nT/2,val_l0);
    uint32x2_t sum_t0 = Sum(vpaddlq_u8(val_t0));
    uint32x2_t sum_l0 = Sum(vpaddlq_u8(val_l0));
    DcSum = vadd_u32(sum_t0,sum_l0);
    DcnT  = vdup_n_u32(16);
    return vshr_n_u32(vadd_u32(DcSum, DcnT),5);
  }
  else { //32x32

    uint8x16_t  val_t0 = vdupq_n_u8(0);
    uint8x16_t  val_t1 = vdupq_n_u8(0);
    uint8x16_t  val_l0 = vdupq_n_u8(0);
    uint8x16_t  val_l1 = vdupq_n_u8(0);

    val_t0 = Load8<0>(border+1+8*0,val_t0);
    val_t0 = Load8<1>(border+1+8*1,val_t0);
    val_t1 = Load8<0>(border+1+8*2,val_t1);
    val_t1 = Load8<1>(border+1+8*3,val_t1);
    uint32x2_t sum_t0 = vadd_u32(Sum(vpaddlq_u8(val_t0)), Sum(vpaddlq_u8(val_t1)));

    val_l0 = Load8<0>(border-8*1,val_l0);
    val_l0 = Load8<1>(border-8*2,val_l0);
    val_l1 = Load8<0>(border-8*3,val_l1);
    val_l1 = Load8<1>(border-8*4,val_l1);
    uint32x2_t sum_l0 = vadd_u32(Sum(vpaddlq_u8(val_l0)), Sum(vpaddlq_u8(val_l1)));

    DcSum = vadd_u32(sum_t0,sum_l0);
    DcnT  = vdup_n_u32(32);
    return vshr_n_u32(vadd_u32(DcSum, DcnT),6);
  }

}

void intra_prediction_DC_neon_8(uint8_t *dst, int dstStride, int nT, int cIdx, /* const */ uint8_t *border)
{

  uint32x2_t dc_dup = intra_pred_DcVal_neon(dst, dstStride, nT, cIdx, border);
  uint32_t   DcVal;
  vst1_lane_u32(&DcVal, dc_dup, 0);

  // load dc_dup
  intra_pred_DcStore_neon<0>((void *)dst, dstStride, nT, dc_dup );

  if(cIdx==0 && nT<32) {
    uint16_t dcval_dup= 3*DcVal+2 ;

    if(nT == 4) {
      uint8x8_t val = vdup_n_u8(0);
      val = Load4<1>(border+1,val);
      for(int y=0;y<4;y++) {
        switch (y) {
          case 0 : val = vld1_lane_u8(&border[-y-1],val, 0); break;
          case 1 : val = vld1_lane_u8(&border[-y-1],val, 1); break;
          case 2 : val = vld1_lane_u8(&border[-y-1],val, 2); break;
          case 3 : val = vld1_lane_u8(&border[-y-1],val, 3); break;
        }
      } 
      uint16x8_t dat  = vld1q_dup_u16(&dcval_dup);
      dat = vaddw_u8(dat,val);
      uint8x8_t res = vshrn_n_u16(dat,2);
      ValueToMem<int32_t>(dst,vget_lane_u32(vreinterpret_u32_u8(res),1));
      for(int y=0;y<4;y++) {
        switch (y) {
          case 0 : vst1_lane_u8(&dst[y*dstStride],res,0); break;
          case 1 : vst1_lane_u8(&dst[y*dstStride],res,1); break;
          case 2 : vst1_lane_u8(&dst[y*dstStride],res,2); break;
          case 3 : vst1_lane_u8(&dst[y*dstStride],res,3); break;
        }
      } 
    }
    else if(nT == 8) {
      uint8x8_t val = vdup_n_u8(0);
      val = vld1_u8(border+1);
      uint16x8_t dat = vld1q_dup_u16(&dcval_dup);
      dat = vaddw_u8(dat,val);
      uint8x8_t res = vshrn_n_u16(dat,2);
      vst1_u8(dst,res);

      for(int y=0;y<8;y++) {
        switch (y) {
          case 0 : val = vld1_lane_u8(&border[-y-1],val, 0); break;
          case 1 : val = vld1_lane_u8(&border[-y-1],val, 1); break;
          case 2 : val = vld1_lane_u8(&border[-y-1],val, 2); break;
          case 3 : val = vld1_lane_u8(&border[-y-1],val, 3); break;
          case 4 : val = vld1_lane_u8(&border[-y-1],val, 4); break;
          case 5 : val = vld1_lane_u8(&border[-y-1],val, 5); break;
          case 6 : val = vld1_lane_u8(&border[-y-1],val, 6); break;
          case 7 : val = vld1_lane_u8(&border[-y-1],val, 7); break;
        }
      } 
      dat = vld1q_dup_u16(&dcval_dup);
      dat = vaddw_u8(dat,val);
      res = vshrn_n_u16(dat,2);
      for(int y=0;y<8;y++) {
        switch (y) {
          case 0 :  vst1_lane_u8(&dst[y*dstStride],res,0); break;
          case 1 :  vst1_lane_u8(&dst[y*dstStride],res,1); break;
          case 2 :  vst1_lane_u8(&dst[y*dstStride],res,2); break;
          case 3 :  vst1_lane_u8(&dst[y*dstStride],res,3); break;
          case 4 :  vst1_lane_u8(&dst[y*dstStride],res,4); break;
          case 5 :  vst1_lane_u8(&dst[y*dstStride],res,5); break;
          case 6 :  vst1_lane_u8(&dst[y*dstStride],res,6); break;
          case 7 :  vst1_lane_u8(&dst[y*dstStride],res,7); break;
        }
      } 
    }
    else {
      assert(nT == 16);
      uint8x8_t val = vdup_n_u8(0);
      val = vld1_u8(border+1);
      uint16x8_t dat = vld1q_dup_u16(&dcval_dup);
      uint16x8_t sum = vaddw_u8(dat,val);
      uint8x8_t res = vshrn_n_u16(sum,2);
      vst1_u8(dst,res);
      val = vld1_u8(border+1+8);
      sum = vaddw_u8(dat,val);
      res = vshrn_n_u16(sum,2);
      vst1_u8(dst+8,res);

      for(int y=0;y<8;y++) {
        switch (y) {
          case 0 : val = vld1_lane_u8(&border[-y-1],val, 0); break;
          case 1 : val = vld1_lane_u8(&border[-y-1],val, 1); break;
          case 2 : val = vld1_lane_u8(&border[-y-1],val, 2); break;
          case 3 : val = vld1_lane_u8(&border[-y-1],val, 3); break;
          case 4 : val = vld1_lane_u8(&border[-y-1],val, 4); break;
          case 5 : val = vld1_lane_u8(&border[-y-1],val, 5); break;
          case 6 : val = vld1_lane_u8(&border[-y-1],val, 6); break;
          case 7 : val = vld1_lane_u8(&border[-y-1],val, 7); break;
        }
      } 
      sum = vaddw_u8(dat,val);
      res = vshrn_n_u16(sum,2);
      for(int y=0;y<8;y++) {
        switch (y) {
          case 0 :  vst1_lane_u8(&dst[y*dstStride],res,0); break;
          case 1 :  vst1_lane_u8(&dst[y*dstStride],res,1); break;
          case 2 :  vst1_lane_u8(&dst[y*dstStride],res,2); break;
          case 3 :  vst1_lane_u8(&dst[y*dstStride],res,3); break;
          case 4 :  vst1_lane_u8(&dst[y*dstStride],res,4); break;
          case 5 :  vst1_lane_u8(&dst[y*dstStride],res,5); break;
          case 6 :  vst1_lane_u8(&dst[y*dstStride],res,6); break;
          case 7 :  vst1_lane_u8(&dst[y*dstStride],res,7); break;
        }
      } 

      for(int y=8;y<16;y++) {
        switch (y-8) {
          case 0 : val = vld1_lane_u8(&border[-y-1],val, 0); break;
          case 1 : val = vld1_lane_u8(&border[-y-1],val, 1); break;
          case 2 : val = vld1_lane_u8(&border[-y-1],val, 2); break;
          case 3 : val = vld1_lane_u8(&border[-y-1],val, 3); break;
          case 4 : val = vld1_lane_u8(&border[-y-1],val, 4); break;
          case 5 : val = vld1_lane_u8(&border[-y-1],val, 5); break;
          case 6 : val = vld1_lane_u8(&border[-y-1],val, 6); break;
          case 7 : val = vld1_lane_u8(&border[-y-1],val, 7); break;
        }
      } 
      sum = vaddw_u8(dat,val);
      res = vshrn_n_u16(sum,2);
      for(int y=8;y<16;y++) {
        switch (y-8) {
          case 0 :  vst1_lane_u8(&dst[y*dstStride],res,0); break;
          case 1 :  vst1_lane_u8(&dst[y*dstStride],res,1); break;
          case 2 :  vst1_lane_u8(&dst[y*dstStride],res,2); break;
          case 3 :  vst1_lane_u8(&dst[y*dstStride],res,3); break;
          case 4 :  vst1_lane_u8(&dst[y*dstStride],res,4); break;
          case 5 :  vst1_lane_u8(&dst[y*dstStride],res,5); break;
          case 6 :  vst1_lane_u8(&dst[y*dstStride],res,6); break;
          case 7 :  vst1_lane_u8(&dst[y*dstStride],res,7); break;
        }
      } 
    }

    dst[0] = (border[-1] + 2*DcVal + border[1] + 2) >> 2;
  }

  return ;
}


extern const int intraPredAngle_table[1+34];

// angle 27 ~ 34
void intra_prediction_angular_27_34_neon(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border)
{

  int intraPredAngle = intraPredAngle_table[intraPredMode];

  // 4x4
  if(nT == 4) {

    int y = 0;
    do {
      int iIdx = ((y+1)*intraPredAngle)>>5 ;
      int iFact= ((y+1)*intraPredAngle)&31 ;
      uint8x8_t vref_l = vld1_u8(border+iIdx+1);
      uint8x8_t vref_r = vld1_u8(border+iIdx+2);
      uint16x8_t weight_a = vmull_u8(vref_l,vdup_n_u8(32-(uint8_t)iFact));
      uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vdup_n_u8(iFact));
      uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
      ValueToMem<int32_t>(dst+y*dstStride,vget_lane_u32(vreinterpret_u32_u8(val),0));  // ? lane 0 or 1
    } while (++y < nT);
  }
  else {

    int y = 0;
    do {
      int iIdx = ((y+1)*intraPredAngle)>>5 ;
      int iFact= ((y+1)*intraPredAngle)&31 ;
      int lidx = 0 ;

      do {
        uint8x8_t vref_l = vld1_u8(border+lidx*8+iIdx+1);
        uint8x8_t vref_r = vld1_u8(border+lidx*8+iIdx+2);
        uint16x8_t weight_a = vmull_u8(vref_l,vdup_n_u8(32-(uint8_t)iFact));
        uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vdup_n_u8(iFact));
        uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
        vst1_u8(dst+y*dstStride+lidx*8,val);

      } while (++lidx < (nT/8)) ;

    } while (++y < nT) ;
  }

  return ;
}

// angle 18 ~ 26
void intra_prediction_angular_18_26_neon(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border)
{
  uint8_t ref_mem[4*MAX_INTRA_PRED_BLOCK_SIZE+1]; 
  uint8_t *ref=&ref_mem[2*MAX_INTRA_PRED_BLOCK_SIZE];

  int intraPredAngle = intraPredAngle_table[intraPredMode];
  int invAngle = invAngle_table[intraPredMode-11];

  // prepare ref pixel
  for (int x=0;x<=nT;x++)
    { ref[x] = border[x]; }

  int s_ref = (nT*intraPredAngle)>>5;
  if (s_ref < -1) {
    for (int x= s_ref; x<=-1; x++) {
      ref[x] = border[0-((x*invAngle+128)>>8)];
    }
  }

  // 4x4
  if(nT == 4) {

    int y = 0;
    do {
      int iIdx = ((y+1)*intraPredAngle)>>5 ;
      int iFact= ((y+1)*intraPredAngle)&31 ;
      uint8x8_t vref_l = vld1_u8(ref+iIdx+1);
      uint8x8_t vref_r = vld1_u8(ref+iIdx+2);
      uint16x8_t weight_a = vmull_u8(vref_l,vdup_n_u8(32-(uint8_t)iFact));
      uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vdup_n_u8(iFact));
      uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
      ValueToMem<int32_t>(dst+y*dstStride,vget_lane_u32(vreinterpret_u32_u8(val),0)); 
    } while (++y < nT);
  }
  else {

    int y = 0;
    do {
      int iIdx = ((y+1)*intraPredAngle)>>5 ;
      int iFact= ((y+1)*intraPredAngle)&31 ;
      int lidx = 0 ;

      do {
        uint8x8_t vref_l = vld1_u8(ref+lidx*8+iIdx+1);
        uint8x8_t vref_r = vld1_u8(ref+lidx*8+iIdx+2);
        uint16x8_t weight_a = vmull_u8(vref_l,vdup_n_u8(32-(uint8_t)iFact));
        uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vdup_n_u8(iFact));
        uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
        vst1_u8(dst+y*dstStride+lidx*8,val);

      } while (++lidx < (nT/8)) ;

    } while (++y < nT) ;
  }

  if (intraPredMode==26 && cIdx==0 && nT<32 && !disableIntraBoundaryFilter) {
    for (int y=0;y<nT;y++) {
      dst[0+y*dstStride] = Clip_BitDepth(border[1] + ((border[-1-y] - border[0])>>1), bit_depth);
    }
  }

  return ;
}

// angle 10 ~ 17
void intra_prediction_angular_10_17_neon(uint8_t* dst, int dstStride,
                                         int bit_depth, bool disableIntraBoundaryFilter,
                                         int xB0,int yB0,
                                         enum IntraPredMode intraPredMode,
                                         int nT,int cIdx,
                                         uint8_t * border)
{
  
  uint8_t  ref_mem[4*MAX_INTRA_PRED_BLOCK_SIZE+1]; 
  uint8_t* ref=&ref_mem[2*MAX_INTRA_PRED_BLOCK_SIZE];

  int intraPredAngle = intraPredAngle_table[intraPredMode];
  int invAngle = invAngle_table[intraPredMode-11];

  // prepare ref pixel
  for (int x=0;x<=nT;x++)
    { ref[x] = border[-x]; }

  int s_ref = (nT*intraPredAngle)>>5;
  if (s_ref < -1) {
    for (int x= s_ref; x<=-1; x++) {
      ref[x] = border[((x*invAngle+128)>>8)]; // DIFF (neg)
    }
  }

  // 4x4
  if(nT == 4) {

    int y = 0;
    int8x8_t   vidx_x = vcreate_s8(0x0403020104030201);
    int16x8_t  vidx_lx= vmull_s8(vidx_x, vdup_n_s8(intraPredAngle));
    int8x8_t   viIdx  = vshrn_n_s16(vidx_lx,5) ;
               viIdx  = vadd_s8(viIdx,vdup_n_s8(nT));
    int8x8_t   viFact = vmovn_s16(vandq_s16(vidx_lx, vdupq_n_s16(31)));
    uint8x8_t  vsFact = vsub_u8(vdup_n_u8(32),vreinterpret_u8_s8(viFact));

    do {
      uint8x8_t tref_l = vld1_u8(ref+y-nT+1);
      uint8x8_t vref_l = vtbl1_u8(tref_l,vreinterpret_u8_s8(viIdx));
      uint8x8_t tref_r = vld1_u8(ref+y-nT+2); // border-y-iIdx-2 -7
      uint8x8_t vref_r = vtbl1_u8(tref_r,vreinterpret_u8_s8(viIdx)); // border-y-iIdx-2 -7
      uint16x8_t weight_a = vmull_u8(vref_l,vsFact);
      uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vreinterpret_u8_s8(viFact));
      uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
      ValueToMem<int32_t>(dst+y*dstStride,vget_lane_u32(vreinterpret_u32_u8(val),0));
    } while (++y < nT);
  }
  else if(nT == 8) {

    int y = 0;
    int8_t     iIdx   = (nT*intraPredAngle)>>5;  // iIdx < nT
    int8x8_t   vidx_x = vcreate_s8(0x0807060504030201); 
    int16x8_t  vidx_lx= vmull_s8(vidx_x, vdup_n_s8(intraPredAngle));
    int8x8_t   viIdx  = vshrn_n_s16(vidx_lx,5) ;
               viIdx  = vsub_s8(viIdx,vdup_n_s8(iIdx));
    int8x8_t   viFact = vmovn_s16(vandq_s16(vidx_lx, vdupq_n_s16(31)));
    uint8x8_t  vsFact = vsub_u8(vdup_n_u8(32),vreinterpret_u8_s8(viFact));

    do {
      uint8x8_t tref_l, tref_r;
      tref_l  = vld1_u8(ref+y+iIdx+1);
      tref_r  = vld1_u8(ref+y+iIdx+2);

      uint8x8_t vref_l = vtbl1_u8(tref_l,vreinterpret_u8_s8(viIdx));
      uint8x8_t vref_r = vtbl1_u8(tref_r,vreinterpret_u8_s8(viIdx));
      uint16x8_t weight_a = vmull_u8(vref_l,vsFact);
      uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vreinterpret_u8_s8(viFact));
      uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
      vst1_u8(dst+y*dstStride,val);

    } while (++y < nT) ;
  }
  else if(nT == 16) {

    int y = 0;
    int8_t     iIdx   = (nT*intraPredAngle)>>5;  // iIdx < nT
    int8x8_t   vidx_x = vdup_n_s8(0);
    do {
      int lidx = 0 ;
      uint8x8x2_t tref_l, tref_r;
      tref_l.val[0] = vld1_u8(ref+y+iIdx+1);
      tref_l.val[1] = vld1_u8(ref+y+iIdx+1+8);
      tref_r.val[0] = vld1_u8(ref+y+iIdx+2);
      tref_r.val[1] = vld1_u8(ref+y+iIdx+2+8);

      do {
        switch(lidx) {
          case 0 : vidx_x = vcreate_s8(0x0807060504030201); break;
          case 1 : vidx_x = vcreate_s8(0x100f0e0d0c0b0a09); break;
        }
        int16x8_t  vidx_lx= vmull_s8(vidx_x, vdup_n_s8(intraPredAngle));
        int8x8_t   viIdx  = vshrn_n_s16(vidx_lx,5) ;
                   viIdx  = vsub_s8(viIdx,vdup_n_s8(iIdx));
        int8x8_t   viFact = vmovn_s16(vandq_s16(vidx_lx, vdupq_n_s16(31)));
        uint8x8_t  vsFact = vsub_u8(vdup_n_u8(32),vreinterpret_u8_s8(viFact));

        uint8x8_t vref_l = vtbl2_u8(tref_l,vreinterpret_u8_s8(viIdx));
        uint8x8_t vref_r = vtbl2_u8(tref_r,vreinterpret_u8_s8(viIdx));
        uint16x8_t weight_a = vmull_u8(vref_l,vsFact);
        uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vreinterpret_u8_s8(viFact));
        uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
        vst1_u8(dst+y*dstStride+lidx*8,val);
      } while (++lidx < (nT/8)) ;

    } while (++y < nT) ;
  }
  else {
    assert(nT == 32);
    int y = 0;
    int8_t     iIdx   = (nT*intraPredAngle)>>5;  // iIdx < nT
    int8x8_t   vidx_x = vdup_n_s8(0);
    do {
      int lidx = 0 ;
      uint8x8x4_t tref_l, tref_r;
      tref_l.val[0] = vld1_u8(ref+y+iIdx+1);     tref_l.val[1] = vld1_u8(ref+y+iIdx+1+8);
      tref_l.val[2] = vld1_u8(ref+y+iIdx+1+16);  tref_l.val[3] = vld1_u8(ref+y+iIdx+1+24);
      tref_r.val[0] = vld1_u8(ref+y+iIdx+2);     tref_r.val[1] = vld1_u8(ref+y+iIdx+2+8);
      tref_r.val[2] = vld1_u8(ref+y+iIdx+2+16);  tref_r.val[3] = vld1_u8(ref+y+iIdx+2+24);

      do {
        switch(lidx) {
          case 0 : vidx_x = vcreate_s8(0x0807060504030201); break;
          case 1 : vidx_x = vcreate_s8(0x100f0e0d0c0b0a09); break;
          case 2 : vidx_x = vcreate_s8(0x1817161514131211); break;
          case 3 : vidx_x = vcreate_s8(0x201f1e1d1c1b1a19); break;
        }
        int16x8_t  vidx_lx= vmull_s8(vidx_x, vdup_n_s8(intraPredAngle));
        int8x8_t   viIdx  = vshrn_n_s16(vidx_lx,5) ;
                   viIdx  = vsub_s8(viIdx,vdup_n_s8(iIdx));
        int8x8_t   viFact = vmovn_s16(vandq_s16(vidx_lx, vdupq_n_s16(31)));
        uint8x8_t  vsFact = vsub_u8(vdup_n_u8(32),vreinterpret_u8_s8(viFact));

        uint8x8_t vref_l = vtbl4_u8(tref_l,vreinterpret_u8_s8(viIdx));
        uint8x8_t vref_r = vtbl4_u8(tref_r,vreinterpret_u8_s8(viIdx));
        uint16x8_t weight_a = vmull_u8(vref_l,vsFact);
        uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,vreinterpret_u8_s8(viFact));
        uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
        vst1_u8(dst+y*dstStride+lidx*8,val);
      } while (++lidx < (nT/8)) ;

    } while (++y < nT) ;
  }

  if (intraPredMode==10 && cIdx==0 && nT<32 && !disableIntraBoundaryFilter) {  // DIFF 26->10
    for (int x=0;x<nT;x++) { // DIFF (x<->y)
      dst[x] = Clip_BitDepth(border[-1] + ((border[1+x] - border[0])>>1), bit_depth); // DIFF (x<->y && neg)
    }
  }
  return ;
}

// angle 2 ~ 9
void intra_prediction_angular_2_9_neon(uint8_t* dst, int dstStride,
                                       int bit_depth, bool disableIntraBoundaryFilter,
                                       int xB0,int yB0,
                                       IntraPredMode intraPredMode,
                                       int nT,int cIdx,
                                       uint8_t * border)
{
  int intraPredAngle = intraPredAngle_table[intraPredMode];

  // 4x4
  if(nT == 4) {

    int y = 0;
    uint8x8_t  vidx_x = vcreate_u8(0x0807060504030201);
    uint16x8_t vidx_lx= vmull_u8(vidx_x, vdup_n_u8(intraPredAngle));
    uint8x8_t  viIdx  = vshrn_n_u16(vidx_lx,5) ;
    uint8x8_t  viFact = vmovn_u16(vandq_u16(vidx_lx, vdupq_n_u16(31)));
    uint8x8_t  vsFact = vsub_u8(vdup_n_u8(32),viFact);

    do {
      uint8x8_t tref_l = vld1_u8(border-y-1-7);
                tref_l = vrev64_u8(tref_l);
      uint8x8_t vref_l = vtbl1_u8(tref_l,viIdx);
      uint8x8_t tref_r = vld1_u8(border-y-2-7); // border-y-iIdx-2 -7
                tref_r = vrev64_u8(tref_r);
      uint8x8_t vref_r = vtbl1_u8(tref_r,viIdx); // border-y-iIdx-2 -7
      uint16x8_t weight_a = vmull_u8(vref_l,vsFact);
      uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,viFact);
      uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
      ValueToMem<int32_t>(dst+y*dstStride,vget_lane_u32(vreinterpret_u32_u8(val),0));  // ? lane 0 or 1
    } while (++y < nT);
  }
  else {

    int y = 0;
    do {
      int lidx = 0 ;
      uint8x8x2_t tref_l, tref_r;

      do {
        uint8_t    iIdx   = ((lidx*8+1)*intraPredAngle)>>5;  // iIdx < nT
        uint8x8_t  vidx_x = vdup_n_u8(0);
        switch(lidx) {
          case 0 : vidx_x = vcreate_u8(0x0807060504030201); break;
          case 1 : vidx_x = vcreate_u8(0x100f0e0d0c0b0a09); break;
          case 2 : vidx_x = vcreate_u8(0x1817161514131211); break;
          case 3 : vidx_x = vcreate_u8(0x201f1e1d1c1b1a19); break;
        }
        uint16x8_t vidx_lx= vmull_u8(vidx_x, vdup_n_u8(intraPredAngle));
        uint8x8_t  viIdx  = vshrn_n_u16(vidx_lx,5) ;
                   viIdx  = vsub_u8(viIdx,vdup_n_u8(iIdx));
        uint8x8_t  viFact = vmovn_u16(vandq_u16(vidx_lx, vdupq_n_u16(31)));
        uint8x8_t  vsFact = vsub_u8(vdup_n_u8(32),viFact);

        tref_l.val[0] = vld1_u8(border-y-iIdx-1-7);  tref_l.val[0] = vrev64_u8(tref_l.val[0]);
        tref_l.val[1] = vld1_u8(border-y-iIdx-1-15); tref_l.val[1] = vrev64_u8(tref_l.val[1]);
        tref_r.val[0] = vld1_u8(border-y-iIdx-2-7);  tref_r.val[0] = vrev64_u8(tref_r.val[0]);
        tref_r.val[1] = vld1_u8(border-y-iIdx-2-15); tref_r.val[1] = vrev64_u8(tref_r.val[1]);
        uint8x8_t vref_l = vtbl2_u8(tref_l,viIdx);
        uint8x8_t vref_r = vtbl2_u8(tref_r,viIdx);
        uint16x8_t weight_a = vmull_u8(vref_l,vsFact);
        uint16x8_t weight_s = vmlal_u8(weight_a,vref_r,viFact);
        uint8x8_t  val  = vrshrn_n_u16(weight_s ,5);
        vst1_u8(dst+y*dstStride+lidx*8,val);
      } while (++lidx < (nT/8)) ;

    } while (++y < nT) ;
  }

  return ;
}


void intra_prediction_sample_filtering_neon(const seq_parameter_set& sps,
                                            uint8_t* p,
                                            int nT, int cIdx,
                                            enum IntraPredMode intraPredMode)
{
  int filterFlag;

  if (intraPredMode==INTRA_DC || nT==4) {
    filterFlag = 0;
  } else {
    // int-cast below prevents a typing problem that leads to wrong results when abs_value is a macro
    int minDistVerHor = libde265_min( abs_value((int)intraPredMode-26),
                                      abs_value((int)intraPredMode-10) );

    switch (nT) {
    case 8:  filterFlag = (minDistVerHor>7) ? 1 : 0; break;
    case 16: filterFlag = (minDistVerHor>1) ? 1 : 0; break;
    case 32: filterFlag = (minDistVerHor>0) ? 1 : 0; break;
      // there is no official 64x64 TB block, but we call this for some intra-pred mode algorithms
      // on the whole CB (2Nx2N mode for the whole CTB)
    case 64: filterFlag = 0; break;
    default: filterFlag = -1; assert(false); break; // should never happen
    }
  }


  if (filterFlag) {
    int biIntFlag = (sps.strong_intra_smoothing_enable_flag &&
                     cIdx==0 &&
                     nT==32 &&
                     abs_value(p[0]+p[ 64]-2*p[ 32]) < (1<<(sps.bit_depth_luma-5)) &&
                     abs_value(p[0]+p[-64]-2*p[-32]) < (1<<(sps.bit_depth_luma-5)))
                     ? 1 : 0;

    uint8_t  pF_mem[4*32+1];
    uint8_t* pF = &pF_mem[2*32];
    uint8_t  p0 = p[0];

    if (biIntFlag) {
      pF[-2*nT] = p[-2*nT];
      pF[ 2*nT] = p[ 2*nT];
      pF[    0] = p[    0];

//      for (int i=1;i<=63;i++) {
//        pF[-i] = p[0] + ((i*(p[-64]-p[0])+32)>>6);
//        pF[ i] = p[0] + ((i*(p[ 64]-p[0])+32)>>6);
//      }
      // neon 
      int16_t pFp = p[-64] - p[0];
      int16_t pFn = p[ 64] - p[0];
      int16x8_t vpFp ;
      int16x8_t vpFn ;
      int icnt = 0;
      int16x8_t vi = vcombine_s16(vcreate_s16(0x0003000200010000),vcreate_s16(0x0007000600050004));
      do {
        vpFp  = vdupq_n_s16(pFp);
        vpFp  = vmlaq_s16(vdupq_n_s16(32),vpFp,vi);
        vpFp  = vshrq_n_s16(vpFp,6);
        vpFp  = vaddq_s16(vpFp,vdupq_n_s16(p0));
        uint8x8_t vpFps = vmovn_u16(vreinterpretq_u16_s16(vpFp));
        vst1_u8(pF-icnt-7, vrev64_u8(vpFps));

        vpFn  = vdupq_n_s16(pFn);
        vpFn  = vmlaq_s16(vdupq_n_s16(32),vpFn,vi);
        vpFn  = vshrq_n_s16(vpFn,6);
        vpFn  = vaddq_s16(vpFn,vdupq_n_s16(p0));
        uint8x8_t vpFns = vmovn_u16(vreinterpretq_u16_s16(vpFn));
        vst1_u8(pF+icnt,vpFns);

        vi = vaddq_s16(vi, vdupq_n_s16(8));
        icnt +=8;
      } while (icnt < 64);
      pF[0]  = p0;

    } else {
      pF[-2*nT] = p[-2*nT];
      pF[ 2*nT] = p[ 2*nT];
//      for (int i=-(2*nT-1) ; i<=2*nT-1 ; i++)
//        {
//          pF[i] = (p[i+1] + 2*p[i] + p[i-1] + 2) >> 2;
//        }
      int y = 0 ;
      uint8_t   *pnew = p-(2*nT-1)  ;
      uint8_t   *pFnew= pF-(2*nT-1) ;
      do {
        uint8x8_t  pl = vld1_u8(pnew-1);
        uint8x8_t  pm = vld1_u8(pnew);
        uint8x8_t  pr = vld1_u8(pnew+1);
        uint16x8_t ps = vaddq_u16(vaddl_u8(pl,pm),vaddl_u8(pm,pr));
        uint8x8_t  po = vqrshrn_n_u16(ps,2);
        vst1_u8(pFnew,po);
        pnew += ((y==(nT*2/8-1))?7:8); // when pnew = p[0], +7
        pFnew+= ((y==(nT*2/8-1))?7:8); 
      } while ( ++y < nT*2*2/8);
    }

    // copy back to original array

    memcpy(p-2*nT, pF-2*nT, (4*nT+1) * sizeof(uint8_t));
  }
  else {
    // do nothing ?
  }

  logtrace(LogIntraPred,"post filtering: ");
  print_border(p,NULL,nT);
  logtrace(LogIntraPred,"\n");
}


void intra_prediction_planar_neon(uint8_t *dst, int dstStride, int nT,int cIdx, uint8_t *border)
{

  if(nT == 4) {
    uint32_t   temp ;
    memcpy(&temp, border+1, 4);
    uint32x2_t border_te ; 
               border_te = vld1_lane_u32(&temp, border_te, 0);    // vreinterpret_u8_u32();
               border_te = vld1_lane_u32(&temp, border_te, 1);    // vreinterpret_u8_u32();
    uint8x8_t  border_tc = vreinterpret_u8_u32(border_te);
    uint8x8_t  border_l0 = vext_u8(vdup_n_u8(border[-1]),vdup_n_u8(border[-2]),4);
    uint8x8_t  border_l2 = vext_u8(vdup_n_u8(border[-3]),vdup_n_u8(border[-4]),4);

    uint8x8_t  border_tr = vdup_n_u8(border[1+nT]);
    uint8x8_t  border_lb = vdup_n_u8(border[-1-nT]);

    uint8x8_t  coef_tr   = vcreate_u8(0x0403020104030201);
    uint8x8_t  coef_lc   = vcreate_u8(0x0001020300010203);
    uint8x8_t  coef_tc0  = vcreate_u8(0x0202020203030303);
    uint8x8_t  coef_tc2  = vcreate_u8(0x0000000001010101);
    uint8x8_t  coef_lb0  = vcreate_u8(0x0202020201010101);
    uint8x8_t  coef_lb2  = vcreate_u8(0x0404040403030303);

    uint16x8_t sum_t0, sum_t2, sum_l0, sum_l2, sum_tl0, sum_tl2;
    uint8x8_t  shf_tl0, shf_tl2;      

    sum_t0   = vmull_u8(border_tr,coef_tr);
    sum_l0   = vmull_u8(border_lb,coef_lb0);
    sum_t0   = vmlal_u8(sum_t0, border_tc, coef_tc0);
    sum_l0   = vmlal_u8(sum_l0, border_l0, coef_lc) ;
    sum_tl0  = vaddq_u16(sum_t0, sum_l0);
    
    sum_t2   = vmull_u8(border_tr, coef_tr);
    sum_l2   = vmull_u8(border_lb, coef_lb2);
    sum_t2   = vmlal_u8(sum_t2, border_tc, coef_tc2);
    sum_l2   = vmlal_u8(sum_l2, border_l2, coef_lc) ;
    sum_tl2  = vaddq_u16(sum_t2, sum_l2);

    shf_tl0  = vrshrn_n_u16(sum_tl0, 3);
    shf_tl2  = vrshrn_n_u16(sum_tl2, 3);

    ValueToMem<int32_t>(dst+0*dstStride,vget_lane_u32(vreinterpret_u32_u8(shf_tl0),0)); 
    ValueToMem<int32_t>(dst+1*dstStride,vget_lane_u32(vreinterpret_u32_u8(shf_tl0),1)); 
    ValueToMem<int32_t>(dst+2*dstStride,vget_lane_u32(vreinterpret_u32_u8(shf_tl2),0)); 
    ValueToMem<int32_t>(dst+3*dstStride,vget_lane_u32(vreinterpret_u32_u8(shf_tl2),1)); 

  }
  else {
    uint8x8_t  border_tr = vdup_n_u8(border[1+nT]);
    uint8x8_t  border_lb = vdup_n_u8(border[-1-nT]);
    uint8x8_t  base      = vcreate_u8(0x0807060504030201);

    for(int y=0; y<nT; y++) {
      uint8x8_t coef_lb   = vdup_n_u8(y+1);
      uint8x8_t coef_tc   = vdup_n_u8(nT-1-y);
      uint8x8_t border_lc = vdup_n_u8(border[-1-y]); 

      for(int x=0; x<nT; x+=8) {
        uint8x8_t border_tc  = vld1_u8(border+1+x);
        uint8x8_t coef_tr    = vadd_u8(base, vdup_n_u8(x));
        uint8x8_t coef_lc    = vsub_u8(vdup_n_u8(nT),coef_tr);

        uint16x8_t sum_tc, sum_lc, sum_tr, sum_lb, sum_t,sum_l, sum;
        uint8x8_t  shf_tl;

        sum_tc  = vmull_u8(border_tc, coef_tc);
        sum_tr  = vmull_u8(border_tr, coef_tr);
        sum_lc  = vmull_u8(border_lc, coef_lc);
        sum_lb  = vmull_u8(border_lb, coef_lb);
        sum_t   = vaddq_u16(sum_tc,sum_tr);
        sum_l   = vaddq_u16(sum_lc,sum_lb);
        sum     = vaddq_u16(sum_t,sum_l);
        switch(nT) {
          case 8 : shf_tl  = vrshrn_n_u16(sum, 4);  break ;
          case 16: shf_tl  = vrshrn_n_u16(sum, 5);  break ;
          case 32: shf_tl  = vrshrn_n_u16(sum, 6);  break ;
        }
        vst1_u8(dst+y*dstStride+x,shf_tl);
      }
    }
  }
  
}