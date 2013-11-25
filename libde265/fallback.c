
#include "fallback.h"
#include "fallback-motion.h"


void init_lowlevel_functions_fallback(struct lowlevel_functions* lowlevel)
{
  lowlevel->put_weighted_pred_avg_8 = put_weighted_pred_avg_8_fallback;
  lowlevel->put_unweighted_pred_8   = put_unweighted_pred_8_fallback;
}
