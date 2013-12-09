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

#include <assert.h>


void read_vps(bitreader* reader, video_parameter_set* vps)
{
  vps->video_parameter_set_id = get_bits(reader, 4);
  skip_bits(reader, 2);
  vps->vps_max_layers = get_bits(reader,6) +1;
  vps->vps_max_sub_layers = get_bits(reader,3) +1;
  vps->vps_temporal_id_nesting_flag = get_bits(reader,1);
  skip_bits(reader, 16);

  read_profile_tier_level(reader, &vps->profile_tier_level,
                          vps->vps_max_sub_layers);

  /*
  read_bit_rate_pic_rate_info(reader, &vps->bit_rate_pic_rate_info,
                              0, vps->vps_max_sub_layers-1);
  */

  vps->vps_sub_layer_ordering_info_present_flag = get_bits(reader,1);

  int firstLayerRead = vps->vps_sub_layer_ordering_info_present_flag ? 0 : vps->vps_max_sub_layers-1;

  for (int i=firstLayerRead;i<vps->vps_max_sub_layers;i++) {
    vps->layer[i].vps_max_dec_pic_buffering = get_uvlc(reader);
    vps->layer[i].vps_max_num_reorder_pics  = get_uvlc(reader);
    vps->layer[i].vps_max_latency_increase  = get_uvlc(reader);
  }

  if (!vps->vps_sub_layer_ordering_info_present_flag) {
    for (int i=0;i<firstLayerRead;i++) {
      vps->layer[i].vps_max_dec_pic_buffering = vps->layer[firstLayerRead].vps_max_dec_pic_buffering;
      vps->layer[i].vps_max_num_reorder_pics  = vps->layer[firstLayerRead].vps_max_num_reorder_pics;
      vps->layer[i].vps_max_latency_increase  = vps->layer[firstLayerRead].vps_max_latency_increase;
    }
  }


  vps->vps_max_layer_id = get_bits(reader,6);
  vps->vps_num_layer_sets = get_uvlc(reader)+1;


  for (int i=1; i <= vps->vps_num_layer_sets-1; i++)
    for (int j=0; j <= vps->vps_max_layer_id; j++)
      {
        vps->layer_id_included_flag[i][j] = get_bits(reader,1);
      }

  vps->vps_timing_info_present_flag = get_bits(reader,1);

  if (vps->vps_timing_info_present_flag) {
    vps->vps_num_units_in_tick = get_bits(reader,32);
    vps->vps_time_scale        = get_bits(reader,32);
    vps->vps_poc_proportional_to_timing_flag = get_bits(reader,1);

    if (vps->vps_poc_proportional_to_timing_flag) {
      vps->vps_num_ticks_poc_diff_one = get_uvlc(reader)+1;
      vps->vps_num_hrd_parameters     = get_uvlc(reader);

      if (vps->vps_num_hrd_parameters >= 1024) {
        assert(false); // TODO: return bitstream error
      }

      for (int i=0; i<vps->vps_num_hrd_parameters; i++) {
        vps->hrd_layer_set_idx[i] = get_uvlc(reader);

        if (i > 0) {
          vps->cprms_present_flag[i] = get_bits(reader,1);
        }

        //hrd_parameters(cprms_present_flag[i], vps_max_sub_layers_minus1)

        return; // TODO: decode hrd_parameters()
      }
    }
  }
  
  vps->vps_extension_flag = get_bits(reader,1);

  if (vps->vps_extension_flag) {
    /*
    while( more_rbsp_data() )
    vps_extension_data_flag u(1)
    rbsp_trailing_bits()
    */
  }
}


void read_profile_tier_level(bitreader* reader,
                             struct profile_tier_level* hdr,
                             int max_sub_layers)
{
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

  hdr->general_level_idc = get_bits(reader,8);


  for (int i=0; i<max_sub_layers-1; i++)
    {
      hdr->profile[i].sub_layer_profile_present_flag = get_bits(reader,1);
      hdr->profile[i].sub_layer_level_present_flag   = get_bits(reader,1);
    }

  if (max_sub_layers > 1)
    {
      for (int i=max_sub_layers-1; i<8; i++)
        {
          skip_bits(reader,2);
        }
    }

  for (int i=0; i<max_sub_layers-1; i++)
    {
      if (hdr->profile[i].sub_layer_profile_present_flag)
        {
          hdr->profile[i].sub_layer_profile_space = get_bits(reader,2);
          hdr->profile[i].sub_layer_tier_flag = get_bits(reader,1);
          hdr->profile[i].sub_layer_profile_idc = get_bits(reader,5);

          for (int j=0; j<32; j++)
            {
              hdr->profile[i].sub_layer_profile_compatibility_flag[j] = get_bits(reader,1);
            }

          hdr->profile[i].sub_layer_progressive_source_flag = get_bits(reader,1);
          hdr->profile[i].sub_layer_interlaced_source_flag  = get_bits(reader,1);
          hdr->profile[i].sub_layer_non_packed_constraint_flag = get_bits(reader,1);
          hdr->profile[i].sub_layer_frame_only_constraint_flag = get_bits(reader,1);
          skip_bits(reader,44);
        }

      if (hdr->profile[i].sub_layer_level_present_flag)
        {
          hdr->profile[i].sub_layer_level_idc = get_bits(reader,8);
        }
    }
}


/*
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
*/



void dump_vps(video_parameter_set* vps)
{
#if (_MSC_VER >= 1500)
#define LOG(...) loginfo(LogHeaders, __VA_ARGS__)

  LOG("----------------- VPS -----------------\n");
  LOG("video_parameter_set_id                : %d\n", vps->video_parameter_set_id);
  LOG("vps_max_layers                        : %d\n", vps->vps_max_layers);
  LOG("vps_max_sub_layers                    : %d\n", vps->vps_max_sub_layers);
  LOG("vps_temporal_id_nesting_flag          : %d\n", vps->vps_temporal_id_nesting_flag);

  dump_profile_tier_level(&vps->profile_tier_level, vps->vps_max_sub_layers);
  //dump_bit_rate_pic_rate_info(&vps->bit_rate_pic_rate_info, 0, vps->vps_max_sub_layers-1);

  LOG("vps_sub_layer_ordering_info_present_flag : %d\n",
         vps->vps_sub_layer_ordering_info_present_flag);

  if (vps->vps_sub_layer_ordering_info_present_flag) {
    for (int i=0;i<vps->vps_max_sub_layers;i++) {
      LOG("layer %d: vps_max_dec_pic_buffering = %d\n",i,vps->layer[i].vps_max_dec_pic_buffering);
      LOG("         vps_max_num_reorder_pics  = %d\n",vps->layer[i].vps_max_num_reorder_pics);
      LOG("         vps_max_latency_increase  = %d\n",vps->layer[i].vps_max_latency_increase);
    }
  }
  else {
    LOG("layer (all): vps_max_dec_pic_buffering = %d\n",vps->layer[0].vps_max_dec_pic_buffering);
    LOG("             vps_max_num_reorder_pics  = %d\n",vps->layer[0].vps_max_num_reorder_pics);
    LOG("             vps_max_latency_increase  = %d\n",vps->layer[0].vps_max_latency_increase);
  }


  LOG("vps_max_layer_id   = %d\n", vps->vps_max_layer_id);
  LOG("vps_num_layer_sets = %d\n", vps->vps_num_layer_sets);

  for (int i=1; i <= vps->vps_num_layer_sets-1; i++)
    for (int j=0; j <= vps->vps_max_layer_id; j++)
      {
        LOG("layer_id_included_flag[%d][%d] = %d\n",i,j,
            vps->layer_id_included_flag[i][j]);
      }

  LOG("vps_timing_info_present_flag = %d\n",
      vps->vps_timing_info_present_flag);

  if (vps->vps_timing_info_present_flag) {
    LOG("vps_num_units_in_tick = %d\n", vps->vps_num_units_in_tick);
    LOG("vps_time_scale        = %d\n", vps->vps_time_scale);
    LOG("vps_poc_proportional_to_timing_flag = %d\n", vps->vps_poc_proportional_to_timing_flag);

    if (vps->vps_poc_proportional_to_timing_flag) {
      LOG("vps_num_ticks_poc_diff_one = %d\n", vps->vps_num_ticks_poc_diff_one);
      LOG("vps_num_hrd_parameters     = %d\n", vps->vps_num_hrd_parameters);

      for (int i=0; i<vps->vps_num_hrd_parameters; i++) {
        LOG("hrd_layer_set_idx[%d] = %d\n", vps->hrd_layer_set_idx[i]);

        if (i > 0) {
          LOG("cprms_present_flag[i] = %d\n", vps->cprms_present_flag[i]);
        }

        //hrd_parameters(cprms_present_flag[i], vps_max_sub_layers_minus1)

        return; // TODO: decode hrd_parameters()
      }
    }
  }
  
  LOG("vps_extension_flag = %d\n", vps->vps_extension_flag);
#undef LOG
#endif
}


void dump_profile_tier_level(struct profile_tier_level* hdr,
                             int max_sub_layers)
{
#if (_MSC_VER >= 1500)
#define LOG(...) loginfo(LogHeaders, __VA_ARGS__)
  LOG("  general_profile_space     : %d\n", hdr->general_profile_space);
  LOG("  general_tier_flag         : %d\n", hdr->general_tier_flag);
  LOG("  general_profile_idc       : %d\n", hdr->general_profile_idc);

  LOG("  general_profile_compatibility_flags: ");
  for (int i=0; i<32; i++) {
    if (i) LOG("*,");
    LOG("*%d",hdr->general_profile_compatibility_flag[i]);
  }
  LOG("*\n");

  LOG("  general_level_idc         : %d\n", hdr->general_level_idc);

  for (int i=0; i<max_sub_layers-1; i++)
    {
      LOG("  Profile/Tier/Level [Layer %d]\n",i);

      if (hdr->profile[i].sub_layer_profile_present_flag) {

        LOG("    sub_layer_profile_space : %d\n",hdr->profile[i].sub_layer_profile_space);
        LOG("    sub_layer_tier_flag     : %d\n",hdr->profile[i].sub_layer_tier_flag);
        LOG("    sub_layer_profile_idc   : %d\n",hdr->profile[i].sub_layer_profile_idc);

        LOG("    sub_layer_profile_compatibility_flags: ");
        for (int j=0; j<32; j++) {
          if (j) LOG(",");
          LOG("%d",hdr->profile[i].sub_layer_profile_compatibility_flag[j]);
        }
        LOG("\n");

        LOG("    sub_layer_progressive_source_flag : %d\n",hdr->profile[i].sub_layer_progressive_source_flag);
        LOG("    sub_layer_interlaced_source_flag : %d\n",hdr->profile[i].sub_layer_interlaced_source_flag);
        LOG("    sub_layer_non_packed_constraint_flag : %d\n",hdr->profile[i].sub_layer_non_packed_constraint_flag);
        LOG("    sub_layer_frame_only_constraint_flag : %d\n",hdr->profile[i].sub_layer_frame_only_constraint_flag);
      }


      if (hdr->profile[i].sub_layer_level_present_flag) {
        LOG("    sub_layer_level_idc   : %d\n", hdr->profile[i].sub_layer_level_idc);
      }
    }
#undef LOG
#endif
}



/*
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
*/
