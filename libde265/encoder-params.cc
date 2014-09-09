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

#include "encoder-params.h"



encoder_params::encoder_params()
{
  mAlgo_CB_IntraPartMode.addChoice("fixed",      ALGO_CB_IntraPartMode_Fixed);
  mAlgo_CB_IntraPartMode.addChoice("brute-force",ALGO_CB_IntraPartMode_BruteForce);

  mAlgo_TB_IntraPredMode.addChoice("minSSD"     ,ALGO_TB_IntraPredMode_MinSSD);
  mAlgo_TB_IntraPredMode.addChoice("brute-force",ALGO_TB_IntraPredMode_BruteForce);

  mAlgo_TB_IntraPredMode_Subset.addChoice("all"   ,ALGO_TB_IntraPredMode_Subset_All);
  mAlgo_TB_IntraPredMode_Subset.addChoice("HV+"   ,ALGO_TB_IntraPredMode_Subset_HVPlus);
  mAlgo_TB_IntraPredMode_Subset.addChoice("DC"    ,ALGO_TB_IntraPredMode_Subset_DC);
  mAlgo_TB_IntraPredMode_Subset.addChoice("planar",ALGO_TB_IntraPredMode_Subset_Planar);

  CB_IntraPartMode_Fixed_partMode.addChoice("NxN",   PART_NxN);
  CB_IntraPartMode_Fixed_partMode.addChoice("2Nx2N", PART_2Nx2N);

  rateControlMethod = RateControlMethod_ConstantQP;
}

