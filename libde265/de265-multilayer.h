/*
 * H.265 video codec.
 * Copyright (c) 2023 Dirk Farin <dirk.farin@gmail.com>
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

#ifndef LIBDE265_DE265_MULTILAYER_H
#define LIBDE265_DE265_MULTILAYER_H

#include "de265.h"

#ifdef __cplusplus
extern "C" {
#endif

struct de265_vps_scanner; // private structure

struct de265_vps; // private structure

enum de265_aux_id {
  de265_aux_none = 0,
  de265_aux_alpha = 1,
  de265_aux_depth = 2
};


// --- VPS scanner ---

/* Get a new vps scanner. */
LIBDE265_API de265_vps_scanner* de265_new_vps_scanner(void);

LIBDE265_API void de265_vps_scanner_release(de265_vps_scanner*);

LIBDE265_API de265_error de265_vps_scanner_push_data(de265_vps_scanner*, const void* data, int length);

//LIBDE265_API de265_error de265_vps_scanner_flush_nal(de265_vps_scanner*);

LIBDE265_API de265_vps* de265_vps_scanner_get_next_vps(de265_vps_scanner*);


// --- VPS ---

LIBDE265_API void de265_vps_release(de265_vps*);

LIBDE265_API int de265_vps_get_id(de265_vps*);

LIBDE265_API int de265_vps_get_max_layers(de265_vps*);

// returns de265_aux_id or any other unspecified ID
LIBDE265_API uint8_t de265_vps_get_layer_aux_id(de265_vps*, int layer);


#ifdef __cplusplus
}
#endif

#endif //LIBDE265_DE265_MULTILAYER_H
