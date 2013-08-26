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

#include "vps.h"
#include "util.h"


void read_vps(bitreader* reader, video_parameter_set* vps)
{
  vps->video_parameter_set_id = get_bits(reader, 4);
  skip_bits(reader, 2+6);
  vps->vps_max_sub_layers = get_bits(reader,3) +1;
  vps->vps_temporal_id_nesting_flag = get_bits(reader,1);
  skip_bits(reader, 16);

  read_profile_tier_level(reader, &vps->profile_tier_level,
                          true, vps->vps_max_sub_layers);
  read_bit_rate_pic_rate_info(reader, &vps->bit_rate_pic_rate_info,
                              0, vps->vps_max_sub_layers-1);

  vps->vps_sub_layer_ordering_info_present_flag = get_bits(reader,1);
}


void read_profile_tier_level(bitreader* reader,
                             struct profile_tier_level* hdr,
                             bool profile_present, int max_sub_layers)
{
  hdr->ProfilePresentFlag = profile_present;

  if (profile_present) {
    hdr->general_profile_space = get_bits(reader,2);
    hdr->general_tier_flag = get_bits(reader,1);
    hdr->general_profile_idc = get_bits(reader,5);

    for (int i=0; i<32; i++) {
      hdr->general_profile_compatibility_flag[i] = get_bits(reader,1);
    }

    hdr->general_progressive_source_flag = get_bits(reader,1);
    hdr->general_interlaced_source_flag  = get_bits(reader,1);
    hdr->general_non_packed_constraint_flag = get_bits(reader,1);
    hdr->general_frame_only_constraint_flag = get_bits(reader,1);
    skip_bits(reader,44);
  }

  hdr->general_level_idc = get_bits(reader,8);


  for (int i=0; i<max_sub_layers-1; i++)
    {
      if (profile_present) {
        hdr->profile_data[i].sub_layer_profile_present_flag = get_bits(reader,1);
      }

      hdr->profile_data[i].sub_layer_level_present_flag = get_bits(reader,1);

      if (profile_present && hdr->profile_data[i].sub_layer_profile_present_flag) {

        hdr->profile_data[i].sub_layer_profile_space = get_bits(reader,2);
        hdr->profile_data[i].sub_layer_tier_flag = get_bits(reader,1);
        hdr->profile_data[i].sub_layer_profile_idc = get_bits(reader,5);

        for (int j=0; j<32; j++) {
          hdr->profile_data[i].sub_layer_profile_compatibility_flag[j] = get_bits(reader,1);
        }

        skip_bits(reader,16);
      }


      if (hdr->profile_data[i].sub_layer_level_present_flag) {
        hdr->profile_data[i].sub_layer_level_idc = get_bits(reader,8);
      }
    }

}



void read_bit_rate_pic_rate_info(bitreader* reader,
                                 struct bit_rate_pic_rate_info* hdr,
                                 int TempLevelLow,
                                 int TempLevelHigh)
{
  for (int i=TempLevelLow; i<=TempLevelHigh; i++) {

    hdr->bit_rate_info_present_flag[i] = get_bits(reader,1);
    hdr->pic_rate_info_present_flag[i] = get_bits(reader,1);

    if (hdr->bit_rate_info_present_flag[i]) {
      hdr->avg_bit_rate[i] = get_bits(reader,16);
      hdr->max_bit_rate[i] = get_bits(reader,16);
    }

    if (hdr->pic_rate_info_present_flag[i]) {
      hdr->constant_pic_rate_idc[i] = get_bits(reader,2);
      hdr->avg_pic_rate[i] = get_bits(reader,16);
    }
  }
}




void dump_vps(video_parameter_set* vps)
{
#define LOG(...) loginfo(LogHeaders, __VA_ARGS__)

  LOG("----------------- VPS -----------------\n");
  LOG("video_parameter_set_id                : %d\n", vps->video_parameter_set_id);
  LOG("vps_max_sub_layers                    : %d\n", vps->vps_max_sub_layers);
  LOG("vps_temporal_id_nesting_flag          : %d\n", vps->vps_temporal_id_nesting_flag);

  dump_profile_tier_level(&vps->profile_tier_level, vps->vps_max_sub_layers);
  dump_bit_rate_pic_rate_info(&vps->bit_rate_pic_rate_info, 0, vps->vps_max_sub_layers-1);

  LOG("vps_sub_layer_ordering_info_present_flag : %d\n",
         vps->vps_sub_layer_ordering_info_present_flag);
}


void dump_profile_tier_level(struct profile_tier_level* hdr,
                             int max_sub_layers)
{
  bool profile_present = hdr->ProfilePresentFlag;

  if (profile_present) {
    LOG("  general_profile_space     : %d\n", hdr->general_profile_space);
    LOG("  general_tier_flag         : %d\n", hdr->general_tier_flag);
    LOG("  general_profile_idc       : %d\n", hdr->general_profile_idc);

    LOG("  general_profile_compatibility_flags: ");
    for (int i=0; i<32; i++) {
      if (i) LOG("*,");
      LOG("*%d",hdr->general_profile_compatibility_flag[i]);
    }
    LOG("*\n");
  }

  LOG("  general_level_idc         : %d\n", hdr->general_level_idc);

  for (int i=0; i<max_sub_layers-1; i++)
    {
      LOG("  Profile/Tier/Level [Layer %d]\n",i);

      if (profile_present && hdr->profile_data[i].sub_layer_profile_present_flag) {

        LOG("    sub_layer_profile_space : %d\n",hdr->profile_data[i].sub_layer_profile_space);
        LOG("    sub_layer_tier_flag     : %d\n",hdr->profile_data[i].sub_layer_tier_flag);
        LOG("    sub_layer_profile_idc   : %d\n",hdr->profile_data[i].sub_layer_profile_idc);

        LOG("    sub_layer_profile_compatibility_flags: ");
        for (int j=0; j<32; j++) {
          if (j) LOG(",");
          LOG("%d",hdr->profile_data[i].sub_layer_profile_compatibility_flag[j]);
        }
        LOG("\n");
      }


      if (hdr->profile_data[i].sub_layer_level_present_flag) {
        LOG("    sub_layer_level_idc   : %d\n", hdr->profile_data[i].sub_layer_level_idc);
      }
    }
}



void dump_bit_rate_pic_rate_info(struct bit_rate_pic_rate_info* hdr,
                                 int TempLevelLow,
                                 int TempLevelHigh)
{
  for (int i=TempLevelLow; i<=TempLevelHigh; i++) {

    LOG("  Bitrate [Layer %d]\n", i);

    if (hdr->bit_rate_info_present_flag[i]) {
      LOG("    avg_bit_rate : %d\n", hdr->avg_bit_rate[i]);
      LOG("    max_bit_rate : %d\n", hdr->max_bit_rate[i]);
    }

    if (hdr->pic_rate_info_present_flag[i]) {
      LOG("    constant_pic_rate_idc : %d\n", hdr->constant_pic_rate_idc[i]);
      LOG("    avg_pic_rate[i]       : %d\n", hdr->avg_pic_rate[i]);
    }
  }
}

