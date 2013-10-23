/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#ifndef DE265_VPS_H
#define DE265_VPS_H

#include <stdbool.h>

#include "libde265/bitstream.h"


#define MAX_TEMPORAL_SUBLAYERS 8


struct profile_data {
  char sub_layer_profile_present_flag;
  char sub_layer_level_present_flag;

  char sub_layer_profile_space;
  char sub_layer_tier_flag;
  char sub_layer_profile_idc;

  char sub_layer_profile_compatibility_flag[32];

  char sub_layer_progressive_source_flag;
  char sub_layer_interlaced_source_flag;
  char sub_layer_non_packed_constraint_flag;
  char sub_layer_frame_only_constraint_flag;

  int sub_layer_level_idc;
};


struct profile_tier_level {
  int general_profile_space;
  int general_tier_flag;
  int general_profile_idc;

  char general_profile_compatibility_flag[32];

  char general_progressive_source_flag;
  char general_interlaced_source_flag;
  char general_non_packed_constraint_flag;
  char general_frame_only_constraint_flag;

  int general_level_idc;

  char sub_layer_profile_present_flag[MAX_TEMPORAL_SUBLAYERS];
  char sub_layer_level_present_flag  [MAX_TEMPORAL_SUBLAYERS];

  struct profile_data profile[MAX_TEMPORAL_SUBLAYERS];
};


void read_profile_tier_level(bitreader* reader,
                             struct profile_tier_level* hdr,
                             int max_sub_layers);

void dump_profile_tier_level(struct profile_tier_level* hdr,
                             int max_sub_layers);


struct bit_rate_pic_rate_info {
  char bit_rate_info_present_flag[8];
  char pic_rate_info_present_flag[8];

  int avg_bit_rate[8];
  int max_bit_rate[8];

  char constant_pic_rate_idc[8];
  int  avg_pic_rate[8];

};

void read_bit_rate_pic_rate_info(bitreader* reader,
                                 struct bit_rate_pic_rate_info* hdr,
                                 int TempLevelLow,
                                 int TempLevelHigh);

void dump_bit_rate_pic_rate_info(struct bit_rate_pic_rate_info* hdr,
                                 int TempLevelLow,
                                 int TempLevelHigh);



typedef struct {
  int vps_max_dec_pic_buffering;
  int vps_max_num_reorder_pics;
  int vps_max_latency_increase;
} layer_data;

typedef struct {
  int video_parameter_set_id;
  int vps_max_layers;
  int vps_max_sub_layers;
  int vps_temporal_id_nesting_flag;
  struct profile_tier_level profile_tier_level;
  struct bit_rate_pic_rate_info bit_rate_pic_rate_info;
  int vps_sub_layer_ordering_info_present_flag;

  layer_data layer[MAX_TEMPORAL_SUBLAYERS];

  int vps_num_hrd_parameters;

  /*
    for( opIdx = 0; opIdx < vps_num_hrd_parameters; opIdx++ ) {

      if( opIdx > 0 )

        operation_point_layer_id_flags(opIdx)

          hrd_parameters(opIdx == 0, vps_max_sub_layers_minus1)

          }
  */

  /*
  vps_extension_flag
  u(1)
    if( vps_extension_flag )

      while( more_rbsp_data() )

        vps_extension_data_flag
          u(1)
  */

} video_parameter_set;

void read_vps(bitreader* reader, video_parameter_set* vps);
void dump_vps(video_parameter_set*);

#endif
