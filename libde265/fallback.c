
#include "fallback.h"
#include "fallback-motion.h"
#include "fallback-dct.h"


void init_lowlevel_functions_fallback(struct lowlevel_functions* lowlevel)
{
  lowlevel->put_weighted_pred_avg_8 = put_weighted_pred_avg_8_fallback;
  lowlevel->put_unweighted_pred_8   = put_unweighted_pred_8_fallback;

  lowlevel->put_weighted_pred_8 = put_weighted_pred_8_fallback;
  lowlevel->put_weighted_bipred_8 = put_weighted_bipred_8_fallback;

  lowlevel->put_hevc_epel_8    = put_epel_8_fallback;
  lowlevel->put_hevc_epel_h_8  = put_epel_hv_8_fallback;
  lowlevel->put_hevc_epel_v_8  = put_epel_hv_8_fallback;
  lowlevel->put_hevc_epel_hv_8 = put_epel_hv_8_fallback;

  lowlevel->put_hevc_qpel_8[0][0] = put_qpel_0_0_fallback;
  lowlevel->put_hevc_qpel_8[0][1] = put_qpel_0_1_fallback;
  lowlevel->put_hevc_qpel_8[0][2] = put_qpel_0_2_fallback;
  lowlevel->put_hevc_qpel_8[0][3] = put_qpel_0_3_fallback;
  lowlevel->put_hevc_qpel_8[1][0] = put_qpel_1_0_fallback;
  lowlevel->put_hevc_qpel_8[1][1] = put_qpel_1_1_fallback;
  lowlevel->put_hevc_qpel_8[1][2] = put_qpel_1_2_fallback;
  lowlevel->put_hevc_qpel_8[1][3] = put_qpel_1_3_fallback;
  lowlevel->put_hevc_qpel_8[2][0] = put_qpel_2_0_fallback;
  lowlevel->put_hevc_qpel_8[2][1] = put_qpel_2_1_fallback;
  lowlevel->put_hevc_qpel_8[2][2] = put_qpel_2_2_fallback;
  lowlevel->put_hevc_qpel_8[2][3] = put_qpel_2_3_fallback;
  lowlevel->put_hevc_qpel_8[3][0] = put_qpel_3_0_fallback;
  lowlevel->put_hevc_qpel_8[3][1] = put_qpel_3_1_fallback;
  lowlevel->put_hevc_qpel_8[3][2] = put_qpel_3_2_fallback;
  lowlevel->put_hevc_qpel_8[3][3] = put_qpel_3_3_fallback;

  lowlevel->transform_skip_8 = transform_skip_8_fallback;
  lowlevel->transform_bypass_8 = transform_bypass_8_fallback;
  lowlevel->transform_4x4_luma_add_8 = transform_4x4_luma_add_8_fallback;
  lowlevel->transform_4x4_add_8   = transform_4x4_add_8_fallback;
  lowlevel->transform_8x8_add_8   = transform_8x8_add_8_fallback;
  lowlevel->transform_16x16_add_8 = transform_16x16_add_8_fallback;
  lowlevel->transform_32x32_add_8 = transform_32x32_add_8_fallback;
}
