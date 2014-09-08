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

  // input

  int first_frame;
  int max_number_of_frames;

  const char* input_yuv;
  int input_width;
  int input_height;


  // output

  const char* output_filename;


  // debug

  const char* reconstruction_yuv;


  // CB quad-tree

  int min_cb_size;
  int max_cb_size;

  int min_tb_size;
  int max_tb_size;

  int max_transform_hierarchy_depth_intra;


  // --- Algo_TB_IntraPredMode

  choice_option mAlgo_TB_IntraPredMode;

  // --- Algo_TB_Split_BruteForce


  // --- Algo_CB_IntraPartMode

  choice_option mAlgo_CB_IntraPartMode;

  Algo_CB_IntraPartMode_Fixed::params CB_IntraPartMode_Fixed;
  Algo_CTB_QScale_Constant::params    CTB_QScale_Constant;


  // --- Algo_CB_Split

  // --- Algo_CTB_QScale


  // intra-prediction

  enum IntraPredSearch intraPredSearch;


  // rate-control

  enum RateControlMethod rateControlMethod;

  int constant_QP;
  int lambda;
};


#endif
