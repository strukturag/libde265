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
#include "vps-extension.h"
#include "bitstream.h"


video_parameter_set_extension::video_parameter_set_extension()
{
  for (int i = 0; i < 8; i++)
  {
    poc_lsb_not_present_flag[i] = false;
    for (int j = 0; j < 8; j++)
    {
      direct_dependency_flag[i][j] = false;
      direct_dependency_type[i][j] = 0;
      VpsInterLayerSamplePredictionEnabled[i][j] = false;
      VpsInterLayerMotionPredictionEnabled[i][j] = false;
    }
    view_id_val[i] = 0;

    for (int j = 0; j<16; j++) {
      dimension_id[i][j] = 0;
    }
  }

  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 8; j++) {
      max_tid_il_ref_pics_plus1[i][j] = 7;
    }
  }

  num_add_olss = 0;
  rep_format_idx_present_flag = false;
  NumOutputLayerSets = 0;
}


de265_error video_parameter_set_extension::read(bitreader* reader, video_parameter_set *vps)
{
  // Byte alignment (vps_extension_alignment_bit_equal_to_one)
  for (int nrBits = bits_to_byte_boundary(reader); nrBits > 0; nrBits--) {
    if (get_bits(reader, 1) != 1) {
      return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;
    }
  }

  if (vps->vps_max_layers - 1 > 0 && vps->vps_base_layer_internal_flag) {
    vps_ext_PTL[0].read(reader, false, vps->vps_max_sub_layers);
  }

  int vlc;
  splitting_flag = (get_bits(reader,1) != 0);
  int NumScalabilityTypes = 0;
  for (int i = 0; i < 16; i++)
  {
    vlc = get_bits(reader,1);
    scalability_mask_flag[i] = (vlc != 0);
    NumScalabilityTypes += vlc;
  }

  for (int j = 0; j < NumScalabilityTypes - splitting_flag; j++)
  {
    dimension_id_len_minus1[j] = get_bits(reader,3);
  }

  if (splitting_flag) {
    int numBits = 0;
    for(int i = 0; i < NumScalabilityTypes - i; i++)
    {
      numBits += dimension_id_len_minus1[i] + 1;
    }
    if (numBits < 6) return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;
    dimension_id_len_minus1[NumScalabilityTypes-1] = 6 - numBits;
    numBits = 6;
  }

  vps_nuh_layer_id_present_flag = (get_bits(reader,1) != 0);

  int MaxLayersMinus1 = libde265_min(62, vps->vps_max_layers-1);

  layer_id_in_nuh[0] = 0;
  for (int i = 1; i <= MaxLayersMinus1; i++)
  {
    if (vps_nuh_layer_id_present_flag) {
      layer_id_in_nuh[i] = vlc = get_bits(reader,6);
      if (vlc > layer_id_in_nuh[i-1]) return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;
    }
    else {
      layer_id_in_nuh[i] = i;
    }
    if (!splitting_flag) {
      for (int j = 0; j < NumScalabilityTypes; j++)
      {
        int nrBits = dimension_id_len_minus1[j] + 1;
        dimension_id[i][j] = get_bits(reader,nrBits);
      }
    }
  }

  // Standard:
  // For i from 0 to MaxLayersMinus1, inclusive, the variable LayerIdxInVps[ layer_id_in_nuh[ i ] ] is set equal to i.
  for (int i = 1; i <= MaxLayersMinus1; i++) {
    LayerIdxInVps[ layer_id_in_nuh[ i ] ] = 1;
  }

  view_id_len = vlc = get_bits(reader,4);
  if (vlc > 0) {

    // Standard F.7.4.3.1.1 (F-2) (JCTVC-R1008_v7)
    // The variable ScalabilityId[ i ][ smIdx ] specifying the identifier of the smIdx-th scalability dimension type of the i-th layer, and the variables ViewOrderIdx[ lId ], DependencyId[ lId ], and AuxId[ lId ] specifying the view order index, the spatial/quality scalability identifier, and the auxiliary identifier, respectively, of the layer with nuh_layer_id equal to lId  are derived as follows:
    int NumViews = 1;
    for (int i = 0; i <= MaxLayersMinus1; i++) {
      int lId = layer_id_in_nuh[i];
      int_2d ScalabilityId;
      for (int smIdx = 0, j = 0; smIdx < 16; smIdx++)
      {
        if (scalability_mask_flag[smIdx])
          ScalabilityId[i][smIdx] = dimension_id[i][j++];
        else
          ScalabilityId[i][smIdx] = 0;
      }
      int_1d ViewOrderIdx;
      int_1d DependencyId;
      ViewOrderIdx[ lId ] = ScalabilityId[ i ][ 1 ];
      DependencyId[ lId ] = ScalabilityId[ i ][ 2 ];
      if (i > 0) {
        int newViewFlag  = 1;
        for (int j = 0; j < i; j++) {
          if (ViewOrderIdx[ lId ] == ViewOrderIdx[ layer_id_in_nuh[j] ] )
            newViewFlag  = 0;
        }
        NumViews += newViewFlag ;
      }
    }

    for (int i = 0; i < NumViews; i++)
    {
      view_id_val[i] = get_bits(reader,vlc);
    }
  }

  for (int i = 1; i <= MaxLayersMinus1; i++) {
    for (int j = 0; j < i; j++)
    {
      direct_dependency_flag[i][j] = (get_bits(reader,1) != 0);
    }
  }

  // Standard F.7.4.3.1.1 (F-3) (JCTVC-R1008_v7)
  // The variable DependencyFlag[ i ][ j ] is derived as follows:
  for( int i = 0; i <= MaxLayersMinus1; i++ ) {
    for( int j = 0; j <= MaxLayersMinus1; j++ ) {
      DependencyFlag[i][j] = direct_dependency_flag[i][j];
      for (int k = 0; k < i; k++) {
        if (direct_dependency_flag[i][k] && DependencyFlag[k][j])
          DependencyFlag[i][j] = true;
      }
    }
  }

  // Standard F.7.4.3.1.1 (F-4) (JCTVC-R1008_v7)
  // The variables NumDirectRefLayers[ iNuhLId ], IdDirectRefLayer[ iNuhLId ][ d ], NumRefLayers[ iNuhLId ], IdRefLayer[ iNuhLId ][ r ], NumPredictedLayers[ iNuhLId ], and IdPredictedLayer[ iNuhLId ][ p ] are derived as follows:
  for( int i = 0; i <= MaxLayersMinus1; i++ ) {
    int iNuhLId = layer_id_in_nuh[i];
    int d = 0; int r = 0; int p = 0;
    for( int j = 0; j <= MaxLayersMinus1; j++ ) {
      int jNuhLid = layer_id_in_nuh[ j ];
      if( direct_dependency_flag[i][j] )
        IdDirectRefLayer[ iNuhLId ][ d++ ] = jNuhLid;
      if( DependencyFlag[ i ][ j ] )
        IdRefLayer[ iNuhLId ][ r++ ] = jNuhLid;
      if( DependencyFlag[ j ][ i ] )
        IdPredictedLayer[ iNuhLId ][ p++ ] = jNuhLid;
    }
    NumDirectRefLayers[ iNuhLId ] = d;
    NumRefLayers[ iNuhLId ] = r;
    NumPredictedLayers[ iNuhLId ] = p;
  }

  // Standard F.7.4.3.1.1 (F-5) (JCTVC-R1008_v7)
  // The variables NumIndependentLayers, NumLayersInTreePartition[ i ], and TreePartitionLayerIdList[ i ][ j ] for i in the range of 0 to NumIndependentLayers ? 1, inclusive, and j in the range of 0 to NumLayersInTreePartition[ i ] ? 1, inclusive, are derived as follows:
  int_2d   TreePartitionLayerIdList;
  int_1d   NumLayersInTreePartition;
  bool     layerIdInListFlag[64];
  for (int i = 0; i <= 63; i++) {
    layerIdInListFlag[i] = false;
  }
  int k = 0;
  for ( int i = 0; i <= MaxLayersMinus1; i++ ) {
    int iNuhLId = layer_id_in_nuh[i];
    if( NumDirectRefLayers[ iNuhLId ] == 0 ) {
      TreePartitionLayerIdList[ k ][ 0 ] = iNuhLId;
      int h = 1;
      for( int j = 0; j < NumPredictedLayers[ iNuhLId ]; j++ ) {
        int predLId = IdPredictedLayer[ iNuhLId ][ j ];
        if( !layerIdInListFlag[ predLId ] ) {
          TreePartitionLayerIdList[ k ][ h++ ] = predLId;
          layerIdInListFlag[ predLId ] = 1;
        }
      }
      NumLayersInTreePartition[ k++ ] = h;
    }
  }
  int NumIndependentLayers = k;

  num_add_layer_sets = 0;
  if (NumIndependentLayers > 1) {
    num_add_layer_sets = get_uvlc(reader);
  }

  for( int i = 0; i < num_add_layer_sets; i++ ) {
    for( int j = 1; j < NumIndependentLayers; j++ ) {
      int nr_bits = ceil_log2(NumLayersInTreePartition[j] + 1);
      highest_layer_idx_plus1[ i ][ j ] = get_bits(reader,1);
    }
  }

  // Standard F.7.4.3 - layer_id_included_flag - (7-3) (JCTVC-R1013_v6)
  // The value of NumLayersInIdList[ 0 ] is set equal to 1 and the value of LayerSetLayerIdList[ 0 ][ 0 ] is set equal to 0
  NumLayersInIdList[0] = 1;
  LayerSetLayerIdList[0][0] = 0;
  //For each value of i in the range of 1 to vps_num_layer_sets_minus1, inclusive, the variable NumLayersInIdList[ i ] and the layer identifier list LayerSetLayerIdList[ i ] are derived as follows:
  for (int i = 1; i <= vps->vps_num_layer_sets-1; i++) {
    int n = 0;
    for (int m = 0; m <= vps->vps_max_layer_id; m++) {
      if( vps->layer_id_included_flag[ i ][ m ] ) {
        LayerSetLayerIdList[ i ][ n++ ] = m;
      }
    }
    NumLayersInIdList[ i ] = n;

    // For each value of i in the range of 1 to vps_num_layer_sets_minus1, inclusive, NumLayersInIdList[ i ] shall be in the range of 1 to vps_max_layers_minus1 + 1, inclusive.
    if (n > vps->vps_max_layers) return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;
  }

  // Standard F.7.4.3.1.1 (F-8) (JCTVC-R1008_v7)
  // specifies the values of NumLayersInIdList[ vps_num_layer_sets_minus1 + 1 + i ] and LayerSetLayerIdList[ vps_num_layer_sets_minus1 + 1 + i ][ layerNum ] as follows
  for( int i = 0; i < num_add_layer_sets; i++ ) {
    int layerNum = 0;
    int lsIdx = vps->vps_num_layer_sets + i;
    for (int treeIdx = 1; treeIdx < NumIndependentLayers; treeIdx++) {
      for( int layerCnt = 0; layerCnt < highest_layer_idx_plus1[ i ][ treeIdx ]; layerCnt++ ) {
        LayerSetLayerIdList[ lsIdx ][ layerNum++ ] = TreePartitionLayerIdList[ treeIdx ][ layerCnt ];
      }
    }
    NumLayersInIdList[ lsIdx ] = layerNum;
  }

  vps_sub_layers_max_minus1_present_flag = (get_bits(reader,1) != 0);
  if (vps_sub_layers_max_minus1_present_flag) {
    for (int i = 0; i <= MaxLayersMinus1; i++) {
      sub_layers_vps_max_minus1[i] = get_bits(reader,3);
    }
  }
  else {
    // When not present, the value of sub_layers_vps_max_minus1[ i ] is inferred to be equal to vps_max_sub_layers_minus1.
    for (int i = 0; i <= MaxLayersMinus1; i++) {
      sub_layers_vps_max_minus1[i] = vps->vps_max_sub_layers - 1;
    }
  }

  max_tid_ref_present_flag = (get_bits(reader,1) != 0);
  if (max_tid_ref_present_flag) {
    for (int i = 0; i <= MaxLayersMinus1; i++) {
      for( int j = i + 1; j <= MaxLayersMinus1; j++ ) {
        if (direct_dependency_flag[j][i])
          max_tid_il_ref_pics_plus1[i][j] = get_bits(reader,3);
      }
    }
  }

  default_ref_layers_active_flag = (get_bits(reader,1) != 0);
  vps_num_profile_tier_level_minus1 = get_uvlc(reader);
  for( int i = vps->vps_base_layer_internal_flag ? 2 : 1;
             i <= vps_num_profile_tier_level_minus1; i++ ) {
    vlc = get_bits(reader,1);
    vps_profile_present_flag[i] = (vlc != 0);
    vps_ext_PTL[i].read(reader, (vlc != 0), vps->vps_max_sub_layers);
  }

  // Standard F.7.4.3.1.1 (F-6) (JCTVC-R1008_v7)
  // The variable NumLayerSets is derived as follows:
  NumLayerSets = vps->vps_num_layer_sets + num_add_layer_sets;
  if (NumLayerSets > 1) {
    num_add_olss = get_uvlc(reader);
    default_output_layer_idc = get_bits(reader,2);
  }

  output_layer_flag[0][0] = 1;
  OutputLayerFlag[0][0] = 1;

  NumOutputLayerSets = num_add_olss + NumLayerSets;
  layer_set_idx_for_ols_minus1[0] = -1;
  for( int i = 1; i < NumOutputLayerSets; i++ ) {
    if (NumLayerSets > 2 && i >= NumLayerSets) {
      int nr_bits = ceil_log2(NumLayerSets - 1);
      layer_set_idx_for_ols_minus1[i] = get_bits(reader,nr_bits);
    }
    else {
      layer_set_idx_for_ols_minus1[i] = i - 1;
    }
    int defaultOutputLayerIdc = libde265_min(default_output_layer_idc, 2);

    // Standard F.7.4.3.1.1 (F-10) (JCTVC-R1008_v7)
    // For i in the range of 0 to NumOutputLayerSets ? 1, inclusive, the variable OlsIdxToLsIdx[ i ] is derived as specified in the following:
    OlsIdxToLsIdx[ i ] =  ( i < NumLayerSets ) ? i : ( layer_set_idx_for_ols_minus1[ i ] + 1 );

    if (i > vps->vps_num_layer_sets - 1 || defaultOutputLayerIdc == 2) {
      // i in the range of ( defaultOutputLayerIdc  = =  2 ) ? 0 : ( vps_num_layer_sets_minus1 + 1 ) to NumOutputLayerSets ? 1, inclusive,
      for (int j = 0; j < NumLayersInIdList[OlsIdxToLsIdx[i]]; j++) {
        output_layer_flag[i][j] = (get_bits(reader,1) != 0);
        OutputLayerFlag[i][j] = output_layer_flag[ i ][ j ];
      }
    }
    else {
      for (int j = 0; j < NumLayersInIdList[OlsIdxToLsIdx[i]]; j++) {
        if (defaultOutputLayerIdc == 0 || defaultOutputLayerIdc == 1) {

          // with nuhLayerIdA being the highest value in LayerSetLayerIdList[ OlsIdxToLsIdx[ i ] ],
          int nuhLayerIdA = 0;
          for (int l = 0; l < NumLayersInIdList[OlsIdxToLsIdx[i]]; l++) {
            if (LayerSetLayerIdList[OlsIdxToLsIdx[i]][l] > nuhLayerIdA) {
              nuhLayerIdA = LayerSetLayerIdList[OlsIdxToLsIdx[i]][l];
            }
          }
          if (defaultOutputLayerIdc == 0 || LayerSetLayerIdList[OlsIdxToLsIdx[i]][j] == nuhLayerIdA) {
            OutputLayerFlag[i][j] = true;
          }
          else {
            OutputLayerFlag[i][j] = false;
          }
        }
      }
    }

    // Standard F.7.4.3.1.1 (F-11) (JCTVC-R1008_v7)
    // The variable NumOutputLayersInOutputLayerSet[ i ] is derived as follows:
    NumOutputLayersInOutputLayerSet[ i ] = 0;
    for( int j = 0; j < NumLayersInIdList[ OlsIdxToLsIdx[ i ] ]; j++ ) {
      NumOutputLayersInOutputLayerSet[ i ] += OutputLayerFlag[ i ][ j ];
      if( OutputLayerFlag[ i ][ j ] ) {
        OlsHighestOutputLayerId[ i ] = LayerSetLayerIdList[ OlsIdxToLsIdx[ i ] ][ j ];
      }
    }

    // Standard F.7.4.3.1.1 (F-12) (JCTVC-R1008_v7)
    // The variables NumNecessaryLayers[ olsIdx ] and NecessaryLayerFlag[ olsIdx ][ lIdx ] are derived as follows:
    for( int olsIdx = 0; olsIdx < NumOutputLayerSets; olsIdx++ ) {
      int lsIdx = OlsIdxToLsIdx[ olsIdx ];
      for( int lsLayerIdx = 0; lsLayerIdx < NumLayersInIdList[ lsIdx ]; lsLayerIdx++ ) {
        NecessaryLayerFlag[ olsIdx ][ lsLayerIdx ] = 0;
      }
      for( int lsLayerIdx = 0; lsLayerIdx < NumLayersInIdList[ lsIdx ]; lsLayerIdx++ ) {
        if( OutputLayerFlag[ olsIdx ][ lsLayerIdx ] ) {
          NecessaryLayerFlag[ olsIdx ][ lsLayerIdx ] = 1;
          int currLayerId = LayerSetLayerIdList[ lsIdx ][ lsLayerIdx ];
          for( int rLsLayerIdx = 0; rLsLayerIdx < lsLayerIdx; rLsLayerIdx++ ) {
            int refLayerId = LayerSetLayerIdList[ lsIdx ][ rLsLayerIdx ];
            if( DependencyFlag[ LayerIdxInVps[ currLayerId ] ][ LayerIdxInVps[ refLayerId ] ] ) {
              NecessaryLayerFlag[ olsIdx ][ rLsLayerIdx ] = 1;
            }
          }
        }
      }
      NumNecessaryLayers[ olsIdx ] = 0;
      for (int lsLayerIdx = 0; lsLayerIdx < NumLayersInIdList[lsIdx]; lsLayerIdx++) {
        NumNecessaryLayers[ olsIdx ] += NecessaryLayerFlag[ olsIdx ][ lsLayerIdx ];
      }
    }

    for (int j = 0; j < NumLayersInIdList[OlsIdxToLsIdx[i]]; j++) {
      if( NecessaryLayerFlag[ i ][ j ] && vps_num_profile_tier_level_minus1 > 0 ) {
        int nr_bits = ceil_log2(vps_num_profile_tier_level_minus1 + 1 );
        profile_tier_level_idx[ i ][ j ] = get_bits(reader,nr_bits);
      }
    }

    if (NumOutputLayersInOutputLayerSet[i] == 1 && NumDirectRefLayers[OlsHighestOutputLayerId[i]] > 0) {
      alt_output_layer_flag[i] = (get_bits(reader,1) != 0);
    }
  }

  vps_num_rep_formats_minus1 = get_uvlc(reader);
  for (int i = 0; i <= vps_num_rep_formats_minus1; i++) {
    de265_error err = vps_ext_rep_format[i].read(reader);
    if ( err != DE265_OK) {
      return err;
    }
  }

  if (vps_num_rep_formats_minus1 > 0) {
    rep_format_idx_present_flag = (get_bits(reader,1) != 0);
  }
  for (int i = 0; i < 16; i++) {
    if (rep_format_idx_present_flag && i <= MaxLayersMinus1 && 
        !(i==0 && vps->vps_base_layer_internal_flag)) {
      // Read vps_rep_format_idx[i] from file
      int nr_bits = ceil_log2( vps_num_rep_formats_minus1 + 1);
      vps_rep_format_idx[ i ] = get_bits(reader,nr_bits);
    }
    else {
      // Infer vps_rep_format_idx[i]
      vps_rep_format_idx[ i ] = libde265_min( i, vps_num_rep_formats_minus1 );
    }
  }
  
  max_one_active_ref_layer_flag = (get_bits(reader,1) != 0);
  vps_poc_lsb_aligned_flag = (get_bits(reader,1) != 0);

  for (int i = 1; i <= MaxLayersMinus1; i++) {
    if( NumDirectRefLayers[ layer_id_in_nuh[ i ] ] == 0 ) {
      poc_lsb_not_present_flag[ i ] = (get_bits(reader,1) != 0);
    }
  }

  // Standard F.7.4.3.1.1 (F-9) (JCTVC-R1008_v7)
  // The variable MaxSubLayersInLayerSetMinus1[ i ] is derived as follows:
  MaxSubLayersInLayerSetMinus1;
  for( int i = 0; i < NumLayerSets; i++ ) {
    int maxSlMinus1 = 0;
    for( int k = 0; k < NumLayersInIdList[ i ]; k++ ) {
      int lId = LayerSetLayerIdList[ i ][ k ];
      maxSlMinus1 = libde265_max( maxSlMinus1, sub_layers_vps_max_minus1[ LayerIdxInVps[ lId ] ] );
    }
    MaxSubLayersInLayerSetMinus1[ i ] = maxSlMinus1;
  }

  // dpb_size() (F.7.3.2.1.3) (JCTVC-R1008_v7)
  dpb_size_table.read(reader, vps);

  direct_dep_type_len_minus2 = get_uvlc(reader);
  direct_dependency_all_layers_flag = (get_bits(reader,1) != 0);
  if (direct_dependency_all_layers_flag) {
    int nr_bits = direct_dep_type_len_minus2 + 2;
    direct_dependency_all_layers_type = get_bits(reader,nr_bits);
    for (int i = vps->vps_base_layer_internal_flag ? 1 : 2; i <= MaxLayersMinus1; i++) {
      for( int j = vps->vps_base_layer_internal_flag ? 0 : 1; j < i; j++ ) {
        direct_dependency_type[i][j] = direct_dependency_all_layers_type;
      }
    }
  }
  else {
    for (int i = vps->vps_base_layer_internal_flag ? 1 : 2; i <= MaxLayersMinus1; i++) {
      for( int j = vps->vps_base_layer_internal_flag ? 0 : 1; j < i; j++ ) {
        if( direct_dependency_flag[ i ][ j ] ) {
          int nr_bits = direct_dep_type_len_minus2 + 2;
          direct_dependency_type[ i ][ j ] = get_bits(reader,nr_bits);
        }
      }
    }
  }

  // Derive VpsInterLayerSamplePredictionEnabled and VpsInterLayerMotionPredictionEnabled // F.7.4.3.1.1 (F-13)
  for (int i = vps->vps_base_layer_internal_flag ? 1 : 2; i <= MaxLayersMinus1; i++) {
    for( int j = vps->vps_base_layer_internal_flag ? 0 : 1; j < i; j++ ) {
      if( direct_dependency_flag[ i ][ j ] ) {
        VpsInterLayerSamplePredictionEnabled[i][j] = (((direct_dependency_type[i][j] + 1) & 0x1) != 0); // F.7.4.3.1.1 (F-13)
        VpsInterLayerMotionPredictionEnabled[i][j] = (((direct_dependency_type[i][j] + 1) & 0x2) != 0); 
      }
      else {
        VpsInterLayerSamplePredictionEnabled[i][j] = false; // F.7.4.3.1.1 (F-13)
        VpsInterLayerMotionPredictionEnabled[i][j] = false;
      }
    }
  }

  vps_non_vui_extension_length = get_uvlc(reader);
  for (int i = 1; i <= vps_non_vui_extension_length; i++) {
    int vps_non_vui_extension_data_byte = get_bits(reader, 8);
  }
	vps_vui_present_flag = (get_bits(reader,1) != 0);

  if (vps_vui_present_flag) {
    // Byte alignment (vps_vui_alignment_bit_equal_to_one)
    for (int nrBits = bits_to_byte_boundary(reader); nrBits > 0; nrBits--) {
      if (get_bits(reader, 1) != 1) {
        return DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE;
      }
    }
    vui.read_vps_vui(reader, vps);
  }

  return DE265_OK;
}

de265_error decoded_picture_buffer_size_table::read(bitreader* reader,
                                                    video_parameter_set *vps)
{
  video_parameter_set_extension *vps_ext = &vps->vps_extension;

  for( int i = 1; i < vps_ext->NumOutputLayerSets; i++ ) {
    int currLsIdx = vps_ext->OlsIdxToLsIdx[ i ];
    sub_layer_flag_info_present_flag[ i ] = (get_bits(reader,1) != 0);
    for( int j = 0; j <= vps_ext->MaxSubLayersInLayerSetMinus1[ currLsIdx ]; j++ ) {
      if (j > 0 && sub_layer_flag_info_present_flag[i]) {
        sub_layer_dpb_info_present_flag[ i ][ j ] = (get_bits(reader,1) != 0);
      }
      sub_layer_dpb_info_present_flag[i][0] = true;
      if( sub_layer_dpb_info_present_flag[ i ][ j ] ) {
        for (int k = 0; k < vps_ext->NumLayersInIdList[currLsIdx]; k++) {
          if( vps_ext->NecessaryLayerFlag[ i ][ k ]  &&  ( vps->vps_base_layer_internal_flag ||
            ( vps_ext->LayerSetLayerIdList[ currLsIdx ][ k ] != 0 ) ) ) {
            max_vps_dec_pic_buffering_minus1[i][k][j] = get_uvlc(reader);
          }
        }
        max_vps_num_reorder_pics[ i ][ j ] = get_uvlc(reader);
        max_vps_latency_increase_plus1[ i ][ j ] = get_uvlc(reader);
      }
    }
  }

  return DE265_OK;
}

de265_error rep_format::read(bitreader* reader)
{
  pic_width_vps_in_luma_samples = get_bits(reader,16);
  pic_height_vps_in_luma_samples = get_bits(reader,16);
  chroma_and_bit_depth_vps_present_flag = (get_bits(reader,1) != 0);
  if( chroma_and_bit_depth_vps_present_flag ) {
    chroma_format_vps_idc = (de265_chroma)get_bits(reader,2);
    if (chroma_format_vps_idc == 3) {
      separate_colour_plane_vps_flag = (get_bits(reader,1) != 0);
    }
    bit_depth_vps_luma_minus8 = get_bits(reader,4);
    bit_depth_vps_chroma_minus8 = get_bits(reader,4);
  }
  conformance_window_vps_flag = (get_bits(reader,1) != 0);
  if( conformance_window_vps_flag ) {
    m_conformanceWindowVps.conf_win_vps_left_offset = get_uvlc(reader);
    m_conformanceWindowVps.conf_win_vps_right_offset = get_uvlc(reader);
    m_conformanceWindowVps.conf_win_vps_top_offset = get_uvlc(reader);
    m_conformanceWindowVps.conf_win_vps_bottom_offset = get_uvlc(reader);
  }

  return DE265_OK;
}
