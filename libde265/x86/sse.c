
#include "x86/sse.h"
#include "x86/sse-motion.h"


void init_lowlevel_functions_sse(struct lowlevel_functions* lowlevel)
{
  lowlevel->put_unweighted_pred_8   = put_unweighted_pred_8_sse;
  lowlevel->put_weighted_pred_avg_8 = put_weighted_pred_avg_8_sse;
}

