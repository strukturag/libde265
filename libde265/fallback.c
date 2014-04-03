/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "fallback.h"
#include "fallback-motion.h"
#include "fallback-dct.h"


void init_acceleration_functions_fallback(struct acceleration_functions* accel)
{
  accel->put_weighted_pred_avg_8 = put_weighted_pred_avg_8_fallback;
  accel->put_unweighted_pred_8   = put_unweighted_pred_8_fallback;

  accel->put_weighted_pred_8 = put_weighted_pred_8_fallback;
  accel->put_weighted_bipred_8 = put_weighted_bipred_8_fallback;

  accel->put_hevc_epel_8    = put_epel_8_fallback;
  accel->put_hevc_epel_h_8  = put_epel_hv_8_fallback;
  accel->put_hevc_epel_v_8  = put_epel_hv_8_fallback;
  accel->put_hevc_epel_hv_8 = put_epel_hv_8_fallback;

  accel->put_hevc_qpel_8[0][0] = put_qpel_0_0_fallback;
  accel->put_hevc_qpel_8[0][1] = put_qpel_0_1_fallback;
  accel->put_hevc_qpel_8[0][2] = put_qpel_0_2_fallback;
  accel->put_hevc_qpel_8[0][3] = put_qpel_0_3_fallback;
  accel->put_hevc_qpel_8[1][0] = put_qpel_1_0_fallback;
  accel->put_hevc_qpel_8[1][1] = put_qpel_1_1_fallback;
  accel->put_hevc_qpel_8[1][2] = put_qpel_1_2_fallback;
  accel->put_hevc_qpel_8[1][3] = put_qpel_1_3_fallback;
  accel->put_hevc_qpel_8[2][0] = put_qpel_2_0_fallback;
  accel->put_hevc_qpel_8[2][1] = put_qpel_2_1_fallback;
  accel->put_hevc_qpel_8[2][2] = put_qpel_2_2_fallback;
  accel->put_hevc_qpel_8[2][3] = put_qpel_2_3_fallback;
  accel->put_hevc_qpel_8[3][0] = put_qpel_3_0_fallback;
  accel->put_hevc_qpel_8[3][1] = put_qpel_3_1_fallback;
  accel->put_hevc_qpel_8[3][2] = put_qpel_3_2_fallback;
  accel->put_hevc_qpel_8[3][3] = put_qpel_3_3_fallback;

  accel->transform_skip_8 = transform_skip_8_fallback;
  accel->transform_bypass_8 = transform_bypass_8_fallback;
  accel->transform_4x4_luma_add_8 = transform_4x4_luma_add_8_fallback;
  accel->transform_4x4_add_8   = transform_4x4_add_8_fallback;
  accel->transform_8x8_add_8   = transform_8x8_add_8_fallback;
  accel->transform_16x16_add_8 = transform_16x16_add_8_fallback;
  accel->transform_32x32_add_8 = transform_32x32_add_8_fallback;
}
