
#include "x86/sse.h"
#include "x86/sse-motion.h"


void init_lowlevel_functions_sse(struct lowlevel_functions* lowlevel)
{
  lowlevel->put_unweighted_pred_8   = ff_hevc_put_unweighted_pred_8_sse;
  lowlevel->put_weighted_pred_avg_8 = ff_hevc_put_weighted_pred_avg_8_sse;

  lowlevel->put_hevc_epel_8    = ff_hevc_put_hevc_epel_pixels_8_sse;
  lowlevel->put_hevc_epel_h_8  = ff_hevc_put_hevc_epel_h_8_sse;
  lowlevel->put_hevc_epel_v_8  = ff_hevc_put_hevc_epel_v_8_sse;
  lowlevel->put_hevc_epel_hv_8 = ff_hevc_put_hevc_epel_hv_8_sse;
}

