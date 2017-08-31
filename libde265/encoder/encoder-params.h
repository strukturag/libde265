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

#include "libde265/configparam.h"


enum RateControlMethod
  {
    RateControlMethod_ConstantQP,
    RateControlMethod_ConstantLambda
  };

enum IntraPredSearch
  {
    IntraPredSearch_Complete
  };


enum SOP_Structure
  {
    SOP_Intra,
    SOP_LowDelay
  };

class option_SOP_Structure : public choice_option<enum SOP_Structure>
{
 public:
  option_SOP_Structure() {
    add_choice("intra",     SOP_Intra);
    add_choice("low-delay", SOP_LowDelay, true);
  }
};


enum MEMode
  {
    MEMode_Test,
    MEMode_Search
  };

class option_MEMode : public choice_option<enum MEMode>
{
 public:
  option_MEMode() {
    add_choice("test",   MEMode_Test, true);
    add_choice("search", MEMode_Search);
  }
};


enum ALGO_CB_Skip {
  ALGO_CB_Skip_BruteForce,
  ALGO_CB_Skip_ScreenFast,
  ALGO_CB_Skip_ScreenRegion  // TODO: actually, this handles not only skip
};

class option_ALGO_CB_Skip : public choice_option<enum ALGO_CB_Skip>
{
 public:
  option_ALGO_CB_Skip() {
    add_choice("brute-force"   ,ALGO_CB_Skip_BruteForce, true);
    add_choice("screen-fast"   ,ALGO_CB_Skip_ScreenFast);
    add_choice("screen-region" ,ALGO_CB_Skip_ScreenRegion);
  }
};


// create a list with numbers [low, 2*low, 4*low, 8*low, ..., <=high]
std::vector<int> power2_range_list(int low,int high);

#endif
