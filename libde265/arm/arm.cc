/*
 * H.265 video codec.
 * Copyright (c) 2013-2015 struktur AG, Joachim Bauch <bauch@struktur.de>
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm-motion.h"

#include <stdio.h>

// TODO: on linux, use getauxval(AT_HWCAP);
static bool detect_neon()
{
  return true;
}


void init_acceleration_functions_neon(struct acceleration_functions* accel)
{
#ifdef HAVE_NEON
  printf("neon\n");
#endif
}

void init_acceleration_functions_aarch64(struct acceleration_functions* accel)
{
#ifdef HAVE_AARCH64
  printf("aarch64\n");
#endif

  accel->put_unweighted_pred_8   = put_pred_8_neon;
  accel->put_weighted_pred_avg_8 = put_bipred_8_neon;

  accel->put_hevc_qpel_8[1][0] = mc_qpel_h1_8_neon;
  accel->put_hevc_qpel_8[2][0] = mc_qpel_h2_8_neon;
  accel->put_hevc_qpel_8[3][0] = mc_qpel_h3_8_neon;

  accel->put_hevc_epel_8       = mc_noshift_8_chroma_neon;
  accel->put_hevc_qpel_8[0][0] = mc_noshift_8_luma_neon;
}






#if 0
#define QPEL_FUNC(name)							\
    extern "C" void ff_##name(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride, \
                                   int height, int width); \
    void libde265_##name(int16_t *dst, ptrdiff_t dststride, const uint8_t *src, ptrdiff_t srcstride, \
                                   int width, int height, int16_t* mcbuffer) { \
      ff_##name(dst, dststride, src, srcstride, height, width); \
    }

QPEL_FUNC(hevc_put_qpel_v1_neon_8);
QPEL_FUNC(hevc_put_qpel_v2_neon_8);
QPEL_FUNC(hevc_put_qpel_v3_neon_8);
QPEL_FUNC(hevc_put_qpel_h1_neon_8);
QPEL_FUNC(hevc_put_qpel_h2_neon_8);
QPEL_FUNC(hevc_put_qpel_h3_neon_8);
QPEL_FUNC(hevc_put_qpel_h1v1_neon_8);
QPEL_FUNC(hevc_put_qpel_h1v2_neon_8);
QPEL_FUNC(hevc_put_qpel_h1v3_neon_8);
QPEL_FUNC(hevc_put_qpel_h2v1_neon_8);
QPEL_FUNC(hevc_put_qpel_h2v2_neon_8);
QPEL_FUNC(hevc_put_qpel_h2v3_neon_8);
QPEL_FUNC(hevc_put_qpel_h3v1_neon_8);
QPEL_FUNC(hevc_put_qpel_h3v2_neon_8);
QPEL_FUNC(hevc_put_qpel_h3v3_neon_8);
#undef QPEL_FUNC

void init_acceleration_functions_arm(struct acceleration_functions* accel)
{
#ifdef HAVE_NEON
  if (has_NEON()) {
    accel->put_hevc_qpel_8[0][1] = libde265_hevc_put_qpel_v1_neon_8;
    accel->put_hevc_qpel_8[0][2] = libde265_hevc_put_qpel_v2_neon_8;
    accel->put_hevc_qpel_8[0][3] = libde265_hevc_put_qpel_v3_neon_8;
    accel->put_hevc_qpel_8[1][0] = libde265_hevc_put_qpel_h1_neon_8;
    accel->put_hevc_qpel_8[1][1] = libde265_hevc_put_qpel_h1v1_neon_8;
    accel->put_hevc_qpel_8[1][2] = libde265_hevc_put_qpel_h1v2_neon_8;
    accel->put_hevc_qpel_8[1][3] = libde265_hevc_put_qpel_h1v3_neon_8;
    accel->put_hevc_qpel_8[2][0] = libde265_hevc_put_qpel_h2_neon_8;
    accel->put_hevc_qpel_8[2][1] = libde265_hevc_put_qpel_h2v1_neon_8;
    accel->put_hevc_qpel_8[2][2] = libde265_hevc_put_qpel_h2v2_neon_8;
    accel->put_hevc_qpel_8[2][3] = libde265_hevc_put_qpel_h2v3_neon_8;
    accel->put_hevc_qpel_8[3][0] = libde265_hevc_put_qpel_h3_neon_8;
    accel->put_hevc_qpel_8[3][1] = libde265_hevc_put_qpel_h3v1_neon_8;
    accel->put_hevc_qpel_8[3][2] = libde265_hevc_put_qpel_h3v2_neon_8;
    accel->put_hevc_qpel_8[3][3] = libde265_hevc_put_qpel_h3v3_neon_8;
  }
#endif  // #ifdef HAVE_NEON
}
#endif
