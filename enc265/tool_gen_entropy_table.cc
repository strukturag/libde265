/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "libde265/cabac.h"
#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>


void generate_entropy_table()
{
  const int nSymbols=1000*1000*10;
  const int oversample = 10;

  double tab[64][2];

  for (int i=0;i<64;i++)
    for (int k=0;k<2;k++)
      tab[i][k]=0;

  srand(time(0));
  //srand(123);

  int cnt=1;
  for (;;cnt++) {
    printf("-------------------- %d --------------------\n",cnt);

    for (int s=0;s<63;s++) {
      CABAC_encoder_bitstream cabac_mix0;
      CABAC_encoder_bitstream cabac_mix1;
      CABAC_encoder_bitstream cabac_ref;

      for (int i=0;i<nSymbols*oversample;i++) {
        int r = rand();
        int n = (r>>2) % 63;
        int m = (r>>1) & 1;
        int b = r & 1;

        context_model model;
        model.MPSbit = m;
        model.state  = n;
        cabac_ref.write_CABAC_bit(&model, b);

        model.MPSbit = m;
        model.state  = n;
        cabac_mix0.write_CABAC_bit(&model, b);

        model.MPSbit = m;
        model.state  = n;
        cabac_mix1.write_CABAC_bit(&model, b);

        if (i%oversample == oversample/2) {
          model.MPSbit = 1;
          model.state  = s;
          cabac_mix0.write_CABAC_bit(&model, 0);

          model.MPSbit = 1;
          model.state  = s;
          cabac_mix1.write_CABAC_bit(&model, 1);

          //b = rand() & 1;
          //cabac_mix.write_CABAC_bypass(1);
        }

      }

      cabac_ref.flush_CABAC();
      cabac_mix0.flush_CABAC();
      cabac_mix1.flush_CABAC();

      int bits_ref  = cabac_ref.size()*8;
      int bits_mix0 = cabac_mix0.size()*8;
      int bits_mix1 = cabac_mix1.size()*8;

      //printf("bits: %d %d\n",bits_ref,bits_mix);
      int bits_diff0 = bits_mix0-bits_ref;
      int bits_diff1 = bits_mix1-bits_ref;
      //printf("bits diff: %d\n",bits_diff);

      double bits_per_symbol0 = bits_diff0 / double(nSymbols);
      double bits_per_symbol1 = bits_diff1 / double(nSymbols);

      tab[s][0] += bits_per_symbol0;
      tab[s][1] += bits_per_symbol1;

      double bps0 = tab[s][0]/cnt;
      double bps1 = tab[s][1]/cnt;

      printf("/* state=%2d */  0x%05x /* %f */,  0x%05x /* %f */,\n", s,
             (int)(bps1*0x8000), bps1,
             (int)(bps0*0x8000), bps0);
    }

    printf("                0x0010c, 0x3bfbb /* dummy, should never be used */\n");
  }
}


int main(int argc, char** argv)
{
  generate_entropy_table();
  return 0;
}
