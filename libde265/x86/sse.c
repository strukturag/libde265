
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


  lowlevel->put_hevc_qpel_8[0][0] = ff_hevc_put_hevc_qpel_pixels_8_sse;
  lowlevel->put_hevc_qpel_8[0][1] = ff_hevc_put_hevc_qpel_v_1_8_sse;
  lowlevel->put_hevc_qpel_8[0][2] = ff_hevc_put_hevc_qpel_v_2_8_sse;
  lowlevel->put_hevc_qpel_8[0][3] = ff_hevc_put_hevc_qpel_v_3_8_sse;
  lowlevel->put_hevc_qpel_8[1][0] = ff_hevc_put_hevc_qpel_h_1_8_sse;
  lowlevel->put_hevc_qpel_8[1][1] = ff_hevc_put_hevc_qpel_h_1_v_1_sse;
  lowlevel->put_hevc_qpel_8[1][2] = ff_hevc_put_hevc_qpel_h_1_v_2_sse;
  lowlevel->put_hevc_qpel_8[1][3] = ff_hevc_put_hevc_qpel_h_1_v_3_sse;
  lowlevel->put_hevc_qpel_8[2][0] = ff_hevc_put_hevc_qpel_h_2_8_sse;
  lowlevel->put_hevc_qpel_8[2][1] = ff_hevc_put_hevc_qpel_h_2_v_1_sse;
  lowlevel->put_hevc_qpel_8[2][2] = ff_hevc_put_hevc_qpel_h_2_v_2_sse;
  lowlevel->put_hevc_qpel_8[2][3] = ff_hevc_put_hevc_qpel_h_2_v_3_sse;
  lowlevel->put_hevc_qpel_8[3][0] = ff_hevc_put_hevc_qpel_h_3_8_sse;
  lowlevel->put_hevc_qpel_8[3][1] = ff_hevc_put_hevc_qpel_h_3_v_1_sse;
  lowlevel->put_hevc_qpel_8[3][2] = ff_hevc_put_hevc_qpel_h_3_v_2_sse;
  lowlevel->put_hevc_qpel_8[3][3] = ff_hevc_put_hevc_qpel_h_3_v_3_sse;
}

