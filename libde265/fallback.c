
#include "fallback.h"
#include "fallback-motion.h"
#include "fallback-dct.h"


void init_lowlevel_functions_fallback(struct lowlevel_functions* lowlevel)
{
  lowlevel->put_weighted_pred_avg_8 = put_weighted_pred_avg_8_fallback;
  lowlevel->put_unweighted_pred_8   = put_unweighted_pred_8_fallback;

  lowlevel->put_hevc_epel_8    = put_epel_8_fallback;
  lowlevel->put_hevc_epel_h_8  = put_epel_hv_8_fallback;
  lowlevel->put_hevc_epel_v_8  = put_epel_hv_8_fallback;
  lowlevel->put_hevc_epel_hv_8 = put_epel_hv_8_fallback;

  lowlevel->transform_4x4_luma_add_8 = transform_4x4_luma_add_8_fallback;
}
