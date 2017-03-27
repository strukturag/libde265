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

#ifdef _MSC_VER
#include <intrin.h>
#endif

#include "x86/sse.h"
#include "x86/sse-motion.h"
#include "x86/sse-motion-new.h"
#include "x86/sse-dct.h"
#include "x86/sse-sao.h"
#include "x86/sse-intra-dc.h"
#include "de265.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef __GNUC__
#include <cpuid.h>
#endif

#include <stdio.h>


static void cpuid(uint32_t in_eax,
                  uint32_t& out_eax, uint32_t& out_ebx,
                  uint32_t& out_ecx, uint32_t& out_edx)
{
#ifdef _MSC_VER
  uint32_t regs[4];

  __cpuid((int *)regs, (int)in_eax);

  out_eax = regs[0];
  out_ebx = regs[1];
  out_ecx = regs[2];
  out_edx = regs[3];
#else
  out_eax=0; out_ebx=0;
  out_ecx=0; out_edx=0;

  __get_cpuid(1, &out_eax,&out_ebx,&out_ecx,&out_edx);
#endif
}


void init_acceleration_functions_sse(struct acceleration_functions* accel,
                                     uint32_t inexact_computation_flags)
{
  uint32_t eax,ebx,ecx,edx;

  cpuid(1, eax,ebx,ecx,edx);

  // printf("CPUID EAX=1 -> ECX=%x EDX=%x\n", regs[2], regs[3]);

  int have_MMX    = !!(edx & (1<<23));
  int have_SSE    = !!(edx & (1<<25));
  int have_SSE2   = !!(edx & (1<<26));
  int have_SSE3   = !!(ecx & (1<< 0));
  int have_SSSE3  = !!(ecx & (1<< 9));
  int have_SSE4_1 = !!(ecx & (1<<19));
  int have_SSE4_2 = !!(ecx & (1<<20));
  int have_AVX    = !!(ecx & (1<<28));

  cpuid(0x80000001, eax,ebx,ecx,edx);
  int have_SSE4a  = !!(ecx & (1<< 6));

  cpuid(7, eax,ebx,ecx,edx);
  int have_AVX2   = !!(ebx & (1<< 5));

  if (0) {
    printf("MMX:%d SSE:%d SSE2:%d SSE3:%d SSSE3:%d SSE4a:%d SSE4_1:%d SSE4_2:%d AVX:%d AVX2:%d\n",
           have_MMX,have_SSE,have_SSE2,have_SSE3,have_SSSE3,
           have_SSE4a,have_SSE4_1,have_SSE4_2,have_AVX,have_AVX2);
  }

#if HAVE_SSE4_1
  if (have_SSE2) {
    accel->put_unweighted_pred_8   = put_pred_8_sse2;
    accel->put_weighted_pred_avg_8 = put_bipred_8_sse2;
  }

  if (have_SSSE3) { accel->put_weighted_pred_8 = put_weighted_pred_8_ssse3; }
  if (have_SSE2)  { accel->put_weighted_bipred_8 = put_weighted_bipred_8_sse2; }

  if (have_SSE2) { accel->put_hevc_epel_8    = put_hevc_chroma_direct_8_sse2; }
  if (have_SSE4_1) { // TODO: check requirements
    accel->put_hevc_epel_h_8  = ff_hevc_put_hevc_epel_h_8_sse;
    accel->put_hevc_epel_v_8  = ff_hevc_put_hevc_epel_v_8_sse;
    accel->put_hevc_epel_hv_8 = ff_hevc_put_hevc_epel_hv_8_sse;
  }

  if (have_SSE2) { accel->put_hevc_qpel_8[0][0] = put_hevc_luma_direct_8_sse2; }
  if (have_SSE4_1) { // TODO: check requirements
    accel->put_hevc_qpel_8[0][1] = ff_hevc_put_hevc_qpel_v_1_8_sse;
    accel->put_hevc_qpel_8[0][2] = ff_hevc_put_hevc_qpel_v_2_8_sse;
    accel->put_hevc_qpel_8[0][3] = ff_hevc_put_hevc_qpel_v_3_8_sse;
    accel->put_hevc_qpel_8[1][0] = ff_hevc_put_hevc_qpel_h_1_8_sse;
    accel->put_hevc_qpel_8[1][1] = ff_hevc_put_hevc_qpel_h_1_v_1_sse;
    accel->put_hevc_qpel_8[1][2] = ff_hevc_put_hevc_qpel_h_1_v_2_sse;
    accel->put_hevc_qpel_8[1][3] = ff_hevc_put_hevc_qpel_h_1_v_3_sse;
    accel->put_hevc_qpel_8[2][0] = ff_hevc_put_hevc_qpel_h_2_8_sse;
    accel->put_hevc_qpel_8[2][1] = ff_hevc_put_hevc_qpel_h_2_v_1_sse;
    accel->put_hevc_qpel_8[2][2] = ff_hevc_put_hevc_qpel_h_2_v_2_sse;
    accel->put_hevc_qpel_8[2][3] = ff_hevc_put_hevc_qpel_h_2_v_3_sse;
    accel->put_hevc_qpel_8[3][0] = ff_hevc_put_hevc_qpel_h_3_8_sse;
    accel->put_hevc_qpel_8[3][1] = ff_hevc_put_hevc_qpel_h_3_v_1_sse;
    accel->put_hevc_qpel_8[3][2] = ff_hevc_put_hevc_qpel_h_3_v_2_sse;
    accel->put_hevc_qpel_8[3][3] = ff_hevc_put_hevc_qpel_h_3_v_3_sse;

    accel->transform_skip_8 = ff_hevc_transform_skip_8_sse;

    // actually, for these two functions, the scalar fallback seems to be faster than the SSE code
    //accel->transform_4x4_luma_add_8 = ff_hevc_transform_4x4_luma_add_8_sse4; // SSE-4 only TODO

    accel->transform_add_8[0] = idct_4x4_add_8_sse4;
    accel->transform_add_8[1] = ff_hevc_transform_8x8_add_8_sse4;
    accel->transform_add_8[2] = ff_hevc_transform_16x16_add_8_sse4;
    accel->transform_add_8[3] = ff_hevc_transform_32x32_add_8_sse4;
  }

  if (have_SSSE3 && have_SSE4_1) {
#if 0
    accel->intra_dc_noavg_8[0] = intra_dc_noavg_8_4x4_ssse3_sse41;
    accel->intra_dc_avg_8[0]   = intra_dc_avg_8_4x4_ssse3_sse41;
#endif
    accel->intra_dc_noavg_8[1] = intra_dc_noavg_8_8x8_ssse3_sse41;
    accel->intra_dc_avg_8[1]   = intra_dc_avg_8_8x8_ssse3_sse41;
    accel->intra_dc_noavg_8[2] = intra_dc_noavg_8_16x16_ssse3_sse41;
    accel->intra_dc_avg_8[2]   = intra_dc_avg_8_16x16_ssse3_sse41;
  }

  if (have_SSSE3) { accel->intra_dc_noavg_8[3] = intra_dc_noavg_8_32x32_ssse3; }
  accel->intra_dc_avg_8[3]   = nullptr;

  if (have_SSE2) { accel->sao_band_8 = sao_band_8bit_sse2; }
#endif
}
