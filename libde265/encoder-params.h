/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: Dirk Farin <farin@struktur.de>
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

#ifndef ENCODER_PARAMS_H
#define ENCODER_PARAMS_H

#include "libde265/encode.h"
#include "libde265/analyze.h"


enum RateControlMethod
  {
    RateControlMethod_ConstantQP,
    RateControlMethod_ConstantLambda
  };

enum IntraPredSearch
  {
    IntraPredSearch_Complete
  };



struct encoder_params
{
  encoder_params();

  void registerParams(config_parameters_NEW& config);


  // CB quad-tree

  option_int min_cb_size;
  option_int max_cb_size;

  option_int min_tb_size;
  option_int max_tb_size;

  option_int max_transform_hierarchy_depth_intra;


  // --- Algo_TB_IntraPredMode

  option_ALGO_TB_IntraPredMode        mAlgo_TB_IntraPredMode;
  option_ALGO_TB_IntraPredMode_Subset mAlgo_TB_IntraPredMode_Subset;

  Algo_TB_IntraPredMode_FastBrute::params TB_IntraPredMode_FastBrute;
  Algo_TB_IntraPredMode_MinResidual::params TB_IntraPredMode_MinResidual;


  // --- Algo_TB_Split_BruteForce

  Algo_TB_Split_BruteForce::params  TB_Split_BruteForce;


  // --- Algo_CB_IntraPartMode

  option_ALGO_CB_IntraPartMode mAlgo_CB_IntraPartMode;

  Algo_CB_IntraPartMode_Fixed::params CB_IntraPartMode_Fixed;

  // --- Algo_CB_Split

  // --- Algo_CTB_QScale

  Algo_CTB_QScale_Constant::params    CTB_QScale_Constant;


  // intra-prediction

  enum IntraPredSearch intraPredSearch;


  // rate-control

  enum RateControlMethod rateControlMethod;

  //int constant_QP;
  //int lambda;
};


#endif
