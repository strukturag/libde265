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

#include "libde265/acceleration.h"
#include "libde265/de265.h"

#include "fallback.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_SSE4_1
#include "x86/sse.h"
#endif

#ifdef HAVE_NEON
#include "arm/arm.h"
#endif


acceleration_functions::acceleration_functions()
{
  init_acceleration_functions_fallback(this);
}


void acceleration_functions::init(int cpu_capabilities, int inexact_decoding_flags)
{
  // fill scalar functions first (so that function table is completely filled)

  init_acceleration_functions_fallback(this);


  // override functions with optimized variants

#ifdef HAVE_SSE4_1
  if (cpu_capabilities & (de265_CPU_capability_X86_SSE2 | de265_CPU_capability_X86_SSE41)) {
    init_acceleration_functions_sse(this, inexact_decoding_flags);
  }
#endif

#ifdef HAVE_NEON
  if (cpu_capabilities & de265_CPU_capability_ARM_NEON) {
    init_acceleration_functions_neon(this);
  }
#endif

#ifdef HAVE_AARCH64
  if (cpu_capabilities & de265_CPU_capability_ARM_AARCH64) {
    init_acceleration_functions_aarch64(this);
  }
#endif
}
