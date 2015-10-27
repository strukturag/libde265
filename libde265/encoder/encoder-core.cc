/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
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


#include "libde265/encoder/encoder-core.h"
#include "libde265/encoder/encoder-context.h"
#include <assert.h>
#include <limits>
#include <math.h>
#include <iostream>
#include <fstream>



static int IntraPredModeCnt[7][35];
static int MPM_used[7][35];

static int IntraPredModeCnt_total[35];
static int MPM_used_total[35];

/*
void statistics_IntraPredMode(const encoder_context* ectx, int x,int y, const enc_cb* cb)
{
  if (cb->split_cu_flag) {
    for (int i=0;i<4;i++)
      if (cb->children[i]) {
        statistics_IntraPredMode(ectx, childX(x,i,cb->log2Size), childY(y,i,cb->log2Size), cb->children[i]);
      }
  }
  else {
    int cnt;
    int size = cb->log2Size;

    if (cb->PartMode == PART_NxN) { cnt=4; size--; } else cnt=1;

    for (int i=0;i<cnt;i++) {
      IntraPredModeCnt[size][ cb->intra.pred_mode[i] ]++;
      IntraPredModeCnt_total[ cb->intra.pred_mode[i] ]++;

      int xi = childX(x,i,cb->log2Size);
      int yi = childY(y,i,cb->log2Size);

      enum IntraPredMode candModeList[3];
      fillIntraPredModeCandidates(candModeList,xi,yi, xi>0, yi>0, ectx->img);

      int predmode = cb->intra.pred_mode[i];
      if (candModeList[0]==predmode ||
          candModeList[1]==predmode ||
          candModeList[2]==predmode) {
        MPM_used[size][predmode]++;
        MPM_used_total[predmode]++;
      }
    }
  }
}
*/

void statistics_print()
{
  for (int i=0;i<35;i++) {
    printf("%d",i);
    printf("  %d %d",IntraPredModeCnt_total[i], MPM_used_total[i]);

    for (int k=2;k<=6;k++) {
      printf("  %d %d",IntraPredModeCnt[k][i], MPM_used[k][i]);
    }

    printf("\n");
  }
}


void print_tb_tree_rates(const enc_tb* tb, int level)
{
  for (int i=0;i<level;i++)
    std::cout << "  ";

  std::cout << "TB rate=" << tb->rate << " (" << tb->rate_withoutCbfChroma << ")\n";
  if (tb->split_transform_flag) {
    for (int i=0;i<4;i++)
      print_tb_tree_rates(tb->children[i], level+1);
  }
}


void print_cb_tree_rates(const enc_cb* cb, int level)
{
  for (int i=0;i<level;i++)
    std::cout << "  ";

  std::cout << "CB rate=" << cb->rate << "\n";
  if (cb->split_cu_flag) {
    for (int i=0;i<4;i++)
      print_cb_tree_rates(cb->children[i], level+1);
  }
  else {
    print_tb_tree_rates(cb->transform_tree, level+1);
  }
}




void EncoderCore_Custom::setParams(encoder_params& params)
{
  // build algorithm tree

  mAlgo_CTB_QScale_Constant.setChildAlgo(&mAlgo_CB_Split_BruteForce);

  Algo_CB* algo_CB_skip = NULL;
  switch (params.mAlgo_CB_Skip()) {
  case ALGO_CB_Skip_BruteForce:
    mAlgo_CB_Skip_BruteForce.setSkipAlgo(&mAlgo_CB_MergeIndex_Fixed);
    mAlgo_CB_Skip_BruteForce.setNonSkipAlgo(&mAlgo_CB_IntraInter_BruteForce);
    algo_CB_skip = &mAlgo_CB_Skip_BruteForce;
    break;

  case ALGO_CB_Skip_ScreenFast:
    mAlgo_CB_Skip_ScreenFast.setNonSkipAlgo(&mAlgo_CB_IntraInter_BruteForce);
    algo_CB_skip = &mAlgo_CB_Skip_ScreenFast;
  }

  mAlgo_CB_Split_BruteForce.setChildAlgo(algo_CB_skip);


  Algo_CB_IntraPartMode* algo_CB_IntraPartMode = NULL;
  switch (params.mAlgo_CB_IntraPartMode()) {
  case ALGO_CB_IntraPartMode_BruteForce:
    algo_CB_IntraPartMode = &mAlgo_CB_IntraPartMode_BruteForce;
    break;
  case ALGO_CB_IntraPartMode_Fixed:
    algo_CB_IntraPartMode = &mAlgo_CB_IntraPartMode_Fixed;
    break;
  }

  mAlgo_CB_IntraInter_BruteForce.setIntraChildAlgo(algo_CB_IntraPartMode);
  mAlgo_CB_IntraInter_BruteForce.setInterChildAlgo(&mAlgo_CB_InterPartMode_Fixed);

  mAlgo_CB_MergeIndex_Fixed.setChildAlgo(&mAlgo_TB_Split_BruteForce);

  Algo_PB_MV* pbAlgo = NULL;
  switch (params.mAlgo_MEMode()) {
  case MEMode_Test:
    pbAlgo = &mAlgo_PB_MV_Test;
    break;
  case MEMode_Search:
    pbAlgo = &mAlgo_PB_MV_Search;
    break;
  }

  mAlgo_CB_InterPartMode_Fixed.setChildAlgo(pbAlgo);
  pbAlgo->setChildAlgo(&mAlgo_TB_Split_BruteForce);


  Algo_TB_IntraPredMode_ModeSubset* algo_TB_IntraPredMode = NULL;
  switch (params.mAlgo_TB_IntraPredMode()) {
  case ALGO_TB_IntraPredMode_BruteForce:
    algo_TB_IntraPredMode = &mAlgo_TB_IntraPredMode_BruteForce;
    break;
  case ALGO_TB_IntraPredMode_FastBrute:
    algo_TB_IntraPredMode = &mAlgo_TB_IntraPredMode_FastBrute;
    break;
  case ALGO_TB_IntraPredMode_MinResidual:
    algo_TB_IntraPredMode = &mAlgo_TB_IntraPredMode_MinResidual;
    break;
  }

  algo_CB_IntraPartMode->setChildAlgo(algo_TB_IntraPredMode);

  mAlgo_TB_Split_BruteForce.setAlgo_TB_IntraPredMode(algo_TB_IntraPredMode);
  mAlgo_TB_Split_BruteForce.setAlgo_TB_Residual(&mAlgo_TB_Transform);

  Algo_TB_RateEstimation* algo_TB_RateEstimation = NULL;
  switch (params.mAlgo_TB_RateEstimation()) {
  case ALGO_TB_RateEstimation_None:  algo_TB_RateEstimation = &mAlgo_TB_RateEstimation_None;  break;
  case ALGO_TB_RateEstimation_Exact: algo_TB_RateEstimation = &mAlgo_TB_RateEstimation_Exact; break;
  }
  mAlgo_TB_Transform.setAlgo_TB_RateEstimation(algo_TB_RateEstimation);
  //mAlgo_TB_Split_BruteForce.setParams(params.TB_Split_BruteForce);

  algo_TB_IntraPredMode->setChildAlgo(&mAlgo_TB_Split_BruteForce);


  // ===== set algorithm parameters ======

  //mAlgo_CB_IntraPartMode_Fixed.setParams(params.CB_IntraPartMode_Fixed);

  //mAlgo_TB_IntraPredMode_FastBrute.setParams(params.TB_IntraPredMode_FastBrute);
  //mAlgo_TB_IntraPredMode_MinResidual.setParams(params.TB_IntraPredMode_MinResidual);


  //mAlgo_CTB_QScale_Constant.setParams(params.CTB_QScale_Constant);


  algo_TB_IntraPredMode->enableIntraPredModeSubset( params.mAlgo_TB_IntraPredMode_Subset() );
}


void Logging::print_logging(const encoder_context* ectx, const char* id, const char* filename)
{
#if 000
  if (strcmp(id,logging_tb_split.name())==0) {
    logging_tb_split.print(ectx,filename);
  }
#endif
}


void en265_print_logging(const encoder_context* ectx, const char* id, const char* filename)
{
  Logging::print_logging(ectx,id,filename);
}
