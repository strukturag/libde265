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

#include "vps.h"
#include "util.h"
#include "decctx.h"

#include <assert.h>


de265_error video_parameter_set::read(decoder_context* ctx, bitreader* reader)
{
  int vlc;

  video_parameter_set_id = vlc = get_bits(reader, 4);
  if (vlc >= DE265_MAX_VPS_SETS) return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;

  skip_bits(reader, 2);
  vps_max_layers = vlc = get_bits(reader,6) +1;
  if (vlc != 1) return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE; // TODO: out of specification

  vps_max_sub_layers = vlc = get_bits(reader,3) +1;
  if (vlc >= MAX_TEMPORAL_SUBLAYERS) return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;

  vps_temporal_id_nesting_flag = get_bits(reader,1);
  skip_bits(reader, 16);

  profile_tier_level.read(reader, vps_max_sub_layers);

  /*
  read_bit_rate_pic_rate_info(reader, &bit_rate_pic_rate_info,
                              0, vps_max_sub_layers-1);
  */

  vps_sub_layer_ordering_info_present_flag = get_bits(reader,1);
  //assert(vps_max_sub_layers-1 < MAX_TEMPORAL_SUBLAYERS);

  int firstLayerRead = vps_sub_layer_ordering_info_present_flag ? 0 : (vps_max_sub_layers-1);

  for (int i=firstLayerRead;i<vps_max_sub_layers;i++) {
    layer[i].vps_max_dec_pic_buffering = get_uvlc(reader);
    layer[i].vps_max_num_reorder_pics  = get_uvlc(reader);
    layer[i].vps_max_latency_increase  = get_uvlc(reader);
  }

  if (!vps_sub_layer_ordering_info_present_flag) {
    assert(firstLayerRead < MAX_TEMPORAL_SUBLAYERS);

    for (int i=0;i<firstLayerRead;i++) {
      layer[i].vps_max_dec_pic_buffering = layer[firstLayerRead].vps_max_dec_pic_buffering;
      layer[i].vps_max_num_reorder_pics  = layer[firstLayerRead].vps_max_num_reorder_pics;
      layer[i].vps_max_latency_increase  = layer[firstLayerRead].vps_max_latency_increase;
    }
  }


  vps_max_layer_id = get_bits(reader,6);
  vps_num_layer_sets = get_uvlc(reader)+1;

  if (vps_num_layer_sets<0 ||
      vps_num_layer_sets>=1024) {
    ctx->add_warning(DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE, false);
    return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;
  }

  for (int i=1; i <= vps_num_layer_sets-1; i++)
    for (int j=0; j <= vps_max_layer_id; j++)
      {
        layer_id_included_flag[i][j] = get_bits(reader,1);
      }

  vps_timing_info_present_flag = get_bits(reader,1);

  if (vps_timing_info_present_flag) {
    vps_num_units_in_tick = get_bits(reader,32);
    vps_time_scale        = get_bits(reader,32);
    vps_poc_proportional_to_timing_flag = get_bits(reader,1);

    if (vps_poc_proportional_to_timing_flag) {
      vps_num_ticks_poc_diff_one = get_uvlc(reader)+1;
      vps_num_hrd_parameters     = get_uvlc(reader);

      if (vps_num_hrd_parameters >= 1024) {
        assert(false); // TODO: return bitstream error
      }

      for (int i=0; i<vps_num_hrd_parameters; i++) {
        hrd_layer_set_idx[i] = get_uvlc(reader);

        if (i > 0) {
          cprms_present_flag[i] = get_bits(reader,1);
        }

        //hrd_parameters(cprms_present_flag[i], vps_max_sub_layers_minus1)

        return DE265_OK; // TODO: decode hrd_parameters()
      }
    }
  }
  
  vps_extension_flag = get_bits(reader,1);

  if (vps_extension_flag) {
    /*
    while( more_rbsp_data() )
    vps_extension_data_flag u(1)
    rbsp_trailing_bits()
    */
  }

  return DE265_OK;
}


void profile_tier_level::read(bitreader* reader,
                              int max_sub_layers)
{
  general_profile_space = get_bits(reader,2);
  general_tier_flag = get_bits(reader,1);
  general_profile_idc = get_bits(reader,5);

  for (int i=0; i<32; i++) {
    general_profile_compatibility_flag[i] = get_bits(reader,1);
  }

  general_progressive_source_flag = get_bits(reader,1);
  general_interlaced_source_flag  = get_bits(reader,1);
  general_non_packed_constraint_flag = get_bits(reader,1);
  general_frame_only_constraint_flag = get_bits(reader,1);
  skip_bits(reader,44);

  general_level_idc = get_bits(reader,8);


  for (int i=0; i<max_sub_layers-1; i++)
    {
      profile[i].sub_layer_profile_present_flag = get_bits(reader,1);
      profile[i].sub_layer_level_present_flag   = get_bits(reader,1);
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
      if (profile[i].sub_layer_profile_present_flag)
        {
          profile[i].sub_layer_profile_space = get_bits(reader,2);
          profile[i].sub_layer_tier_flag = get_bits(reader,1);
          profile[i].sub_layer_profile_idc = get_bits(reader,5);

          for (int j=0; j<32; j++)
            {
              profile[i].sub_layer_profile_compatibility_flag[j] = get_bits(reader,1);
            }

          profile[i].sub_layer_progressive_source_flag = get_bits(reader,1);
          profile[i].sub_layer_interlaced_source_flag  = get_bits(reader,1);
          profile[i].sub_layer_non_packed_constraint_flag = get_bits(reader,1);
          profile[i].sub_layer_frame_only_constraint_flag = get_bits(reader,1);
          skip_bits(reader,44);
        }

      if (profile[i].sub_layer_level_present_flag)
        {
          profile[i].sub_layer_level_idc = get_bits(reader,8);
        }
    }
}


void profile_tier_level::write(CABAC_encoder* out, int max_sub_layers) const
{
  out->write_bits(general_profile_space,2);
  out->write_bit (general_tier_flag);
  out->write_bits(general_profile_idc, 5);

  for (int i=0; i<32; i++) {
    out->write_bit(general_profile_compatibility_flag[i]);
  }

  out->write_bit(general_progressive_source_flag);
  out->write_bit(general_interlaced_source_flag);
  out->write_bit(general_non_packed_constraint_flag);
  out->write_bit(general_frame_only_constraint_flag);
  out->skip_bits(44);

  out->write_bits(general_level_idc,8);


  for (int i=0; i<max_sub_layers-1; i++)
    {
      out->write_bit(profile[i].sub_layer_profile_present_flag);
      out->write_bit(profile[i].sub_layer_level_present_flag);
    }

  if (max_sub_layers > 1)
    {
      for (int i=max_sub_layers-1; i<8; i++)
        {
          out->skip_bits(2);
        }
    }

  for (int i=0; i<max_sub_layers-1; i++)
    {
      if (profile[i].sub_layer_profile_present_flag)
        {
          out->write_bits(profile[i].sub_layer_profile_space,2);
          out->write_bit (profile[i].sub_layer_tier_flag);
          out->write_bits(profile[i].sub_layer_profile_idc,5);

          for (int j=0; j<32; j++)
            {
              out->write_bit(profile[i].sub_layer_profile_compatibility_flag[j]);
            }

          out->write_bit(profile[i].sub_layer_progressive_source_flag);
          out->write_bit(profile[i].sub_layer_interlaced_source_flag);
          out->write_bit(profile[i].sub_layer_non_packed_constraint_flag);
          out->write_bit(profile[i].sub_layer_frame_only_constraint_flag);
          out->skip_bits(44);
        }

      if (profile[i].sub_layer_level_present_flag)
        {
          out->write_bits(profile[i].sub_layer_level_idc,8);
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



#define LOG0(t) log2fh(fh, t)
#define LOG1(t,d) log2fh(fh, t,d)
#define LOG2(t,d1,d2) log2fh(fh, t,d1,d2)
#define LOG3(t,d1,d2,d3) log2fh(fh, t,d1,d2,d3)

void video_parameter_set::dump(int fd) const
{
  FILE* fh;
  if (fd==1) fh=stdout;
  else if (fd==2) fh=stderr;
  else { return; }

  LOG0("----------------- VPS -----------------\n");
  LOG1("video_parameter_set_id                : %d\n", video_parameter_set_id);
  LOG1("vps_max_layers                        : %d\n", vps_max_layers);
  LOG1("vps_max_sub_layers                    : %d\n", vps_max_sub_layers);
  LOG1("vps_temporal_id_nesting_flag          : %d\n", vps_temporal_id_nesting_flag);

  profile_tier_level.dump(vps_max_sub_layers, fh);
  //dump_bit_rate_pic_rate_info(&bit_rate_pic_rate_info, 0, vps_max_sub_layers-1);

  LOG1("vps_sub_layer_ordering_info_present_flag : %d\n",
       vps_sub_layer_ordering_info_present_flag);

  if (vps_sub_layer_ordering_info_present_flag) {
    for (int i=0;i<vps_max_sub_layers;i++) {
      LOG2("layer %d: vps_max_dec_pic_buffering = %d\n",i,layer[i].vps_max_dec_pic_buffering);
      LOG1("         vps_max_num_reorder_pics  = %d\n",layer[i].vps_max_num_reorder_pics);
      LOG1("         vps_max_latency_increase  = %d\n",layer[i].vps_max_latency_increase);
    }
  }
  else {
    LOG1("layer (all): vps_max_dec_pic_buffering = %d\n",layer[0].vps_max_dec_pic_buffering);
    LOG1("             vps_max_num_reorder_pics  = %d\n",layer[0].vps_max_num_reorder_pics);
    LOG1("             vps_max_latency_increase  = %d\n",layer[0].vps_max_latency_increase);
  }


  LOG1("vps_max_layer_id   = %d\n", vps_max_layer_id);
  LOG1("vps_num_layer_sets = %d\n", vps_num_layer_sets);

  for (int i=1; i <= vps_num_layer_sets-1; i++)
    for (int j=0; j <= vps_max_layer_id; j++)
      {
        LOG3("layer_id_included_flag[%d][%d] = %d\n",i,j,
             layer_id_included_flag[i][j]);
      }

  LOG1("vps_timing_info_present_flag = %d\n",
       vps_timing_info_present_flag);

  if (vps_timing_info_present_flag) {
    LOG1("vps_num_units_in_tick = %d\n", vps_num_units_in_tick);
    LOG1("vps_time_scale        = %d\n", vps_time_scale);
    LOG1("vps_poc_proportional_to_timing_flag = %d\n", vps_poc_proportional_to_timing_flag);

    if (vps_poc_proportional_to_timing_flag) {
      LOG1("vps_num_ticks_poc_diff_one = %d\n", vps_num_ticks_poc_diff_one);
      LOG1("vps_num_hrd_parameters     = %d\n", vps_num_hrd_parameters);

      for (int i=0; i<vps_num_hrd_parameters; i++) {
        LOG2("hrd_layer_set_idx[%d] = %d\n", i, hrd_layer_set_idx[i]);

        if (i > 0) {
          LOG2("cprms_present_flag[%d] = %d\n", i, cprms_present_flag[i]);
        }

        //hrd_parameters(cprms_present_flag[i], vps_max_sub_layers_minus1)

        return; // TODO: decode hrd_parameters()
      }
    }
  }
  
  LOG1("vps_extension_flag = %d\n", vps_extension_flag);
}


void profile_tier_level::dump(int max_sub_layers, FILE* fh) const
{
  LOG1("  general_profile_space     : %d\n", general_profile_space);
  LOG1("  general_tier_flag         : %d\n", general_tier_flag);
  LOG1("  general_profile_idc       : %d\n", general_profile_idc);

  LOG0("  general_profile_compatibility_flags: ");
  for (int i=0; i<32; i++) {
    if (i) LOG0("*,");
    LOG1("*%d",general_profile_compatibility_flag[i]);
  }
  LOG0("*\n");

  LOG1("  general_level_idc         : %d\n", general_level_idc);

  for (int i=0; i<max_sub_layers-1; i++)
    {
      LOG1("  Profile/Tier/Level [Layer %d]\n",i);

      if (profile[i].sub_layer_profile_present_flag) {

        LOG1("    sub_layer_profile_space : %d\n",profile[i].sub_layer_profile_space);
        LOG1("    sub_layer_tier_flag     : %d\n",profile[i].sub_layer_tier_flag);
        LOG1("    sub_layer_profile_idc   : %d\n",profile[i].sub_layer_profile_idc);

        LOG0("    sub_layer_profile_compatibility_flags: ");
        for (int j=0; j<32; j++) {
          if (j) LOG0(",");
          LOG1("%d",profile[i].sub_layer_profile_compatibility_flag[j]);
        }
        LOG0("\n");

        LOG1("    sub_layer_progressive_source_flag : %d\n",profile[i].sub_layer_progressive_source_flag);
        LOG1("    sub_layer_interlaced_source_flag : %d\n",profile[i].sub_layer_interlaced_source_flag);
        LOG1("    sub_layer_non_packed_constraint_flag : %d\n",profile[i].sub_layer_non_packed_constraint_flag);
        LOG1("    sub_layer_frame_only_constraint_flag : %d\n",profile[i].sub_layer_frame_only_constraint_flag);
      }


      if (profile[i].sub_layer_level_present_flag) {
        LOG1("    sub_layer_level_idc   : %d\n", profile[i].sub_layer_level_idc);
      }
    }
}

#undef LOG0
#undef LOG1
#undef LOG2
#undef LOG3


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
