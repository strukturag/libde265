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
#include "neon_common.h"
#include "neon_dct.h"
#include "neon_intrapred.h"

#ifdef HAVE_NEON

#define QPEL_FUNC(name) \
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

#if defined(HAVE_SIGNAL_H) && defined(HAVE_SETJMP_H)

#include <signal.h>
#include <setjmp.h>

extern "C" void libde265_detect_neon(void);

static jmp_buf jump_env;

static void sighandler(int sig) {
  (void)sig;
  longjmp(jump_env, 1);
}

static bool has_NEON() {
  static bool checked_NEON = false;
  static bool have_NEON = false;

  if (!checked_NEON) {
    void (*oldsignal)(int);

    checked_NEON = true;
    oldsignal = signal(SIGILL, sighandler);
    if (setjmp(jump_env)) {
      signal(SIGILL, oldsignal);
      have_NEON = false;
      return false;
    }
    libde265_detect_neon();
    signal(SIGILL, oldsignal);
    have_NEON = true;
  }

  return have_NEON;
}

#else  // #if defined(HAVE_SIGNAL_H) && defined(HAVE_SETJMP_H)

#warning "Don't know how to detect NEON support at runtime- will be disabled"

static bool has_NEON() {
  return false;
}

#endif

#endif  // #ifdef HAVE_NEON

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
      /*inverse transform*/
      accel->transform_4x4_dst_add_8 = ff_hevc_transform_4x4_luma_add_8_neon;
    
      accel->transform_dc_add_8[0] = ff_hevc_transform_4x4_dc_add_8_neon;
      accel->transform_dc_add_8[1] = ff_hevc_transform_8x8_dc_add_8_neon;
      accel->transform_dc_add_8[2] = ff_hevc_transform_16x16_dc_add_8_neon;
      accel->transform_dc_add_8[3] = ff_hevc_transform_32x32_dc_add_8_neon;
    
      accel->transform_add_8[0] = ff_hevc_transform_4x4_add_8_neon;
      accel->transform_add_8[1] = ff_hevc_transform_8x8_add_8_neon;
      accel->transform_add_8[2] = ff_hevc_transform_16x16_add_8_neon;
      accel->transform_add_8[3] = ff_hevc_transform_32x32_add_8_neon;

      accel->transform_skip_residual16 = ff_hevc_transform_skip_residual16;

      accel->add_residual16_8  = ff_hevc_residual16_add_8_neon;

      /* intra prediction*/ 
      accel->intra_pred_dc_8[0]  = intra_prediction_DC_neon_8 ; 
      accel->intra_pred_dc_8[1]  = intra_prediction_DC_neon_8 ;
      accel->intra_pred_dc_8[2]  = intra_prediction_DC_neon_8 ;
      accel->intra_pred_dc_8[3]  = intra_prediction_DC_neon_8 ;
    
      accel->intra_prediction_angular_8[0]  = intra_prediction_angular_2_9_neon ;
      accel->intra_prediction_angular_8[1]  = intra_prediction_angular_10_17_neon;
      accel->intra_prediction_angular_8[2]  = intra_prediction_angular_18_26_neon;
      accel->intra_prediction_angular_8[3]  = intra_prediction_angular_27_34_neon;

      accel->intra_prediction_sample_filtering_8 = intra_prediction_sample_filtering_neon;

      accel->intra_prediction_planar_8    = intra_prediction_planar_neon ;
    
      /* TODO, 16bit*/
      // accel->intra_pred_dc_16 = intra_prediction_DC_neon_16 ;
      // accel->intra_prediction_angular_16[0] = intra_prediction_angular_2_9_neon ;
      // accel->intra_prediction_angular_16[1] = intra_prediction_angular_10_17_neon;
      // accel->intra_prediction_angular_16[2] = intra_prediction_angular_18_26_neon;
      // accel->intra_prediction_angular_16[3] = intra_prediction_angular_27_34_neon;
      // accel->intra_prediction_sample_filtering_16= intra_prediction_sample_filtering_neon ;
      // accel->intra_prediction_planar_16   = intra_prediction_planar_neon ;

      /*deblocking filter*/
      /* TODO */
    
      /*SAO filtering*/
      /* TODO */
}
