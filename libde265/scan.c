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

#include "scan.h"

static position scan1 = { 0,0 };
static position scan_h_1[ 2* 2], scan_v_1[ 2* 2], scan_d_1[ 2* 2];
static position scan_h_2[ 4* 4], scan_v_2[ 4* 4], scan_d_2[ 4* 4];
static position scan_h_3[ 8* 8], scan_v_3[ 8* 8], scan_d_3[ 8* 8];
static position scan_h_4[16*16], scan_v_4[16*16], scan_d_4[16*16];
static position scan_h_5[32*32], scan_v_5[32*32], scan_d_5[32*32];
static position scan_h_6[64*64], scan_v_6[64*64], scan_d_6[64*64];

static position* scan_h[7] = { &scan1,scan_h_1,scan_h_2,scan_h_3,scan_h_4,scan_h_5,scan_h_6 };
static position* scan_v[7] = { &scan1,scan_v_1,scan_v_2,scan_v_3,scan_v_4,scan_v_5,scan_v_6 };
static position* scan_d[7] = { &scan1,scan_d_1,scan_d_2,scan_d_3,scan_d_4,scan_d_5,scan_d_6 };

static void init_scan_h(position* scan, int blkSize)
{
  int i=0;
  for (int y=0;y<blkSize;y++)
    for (int x=0;x<blkSize;x++)
      {
        scan[i].x = x;
        scan[i].y = y;
        i++;
      }
}

static void init_scan_v(position* scan, int blkSize)
{
  int i=0;
  for (int x=0;x<blkSize;x++)
    for (int y=0;y<blkSize;y++)
      {
        scan[i].x = x;
        scan[i].y = y;
        i++;
      }
}

static void init_scan_d(position* scan, int blkSize)
{
  int i=0;
  int x=0,y=0;

  do {
    while (y>=0) {
      if (x<blkSize && y<blkSize) {
        scan[i].x = x;
        scan[i].y = y;
        i++;
      }
      y--;
      x++;
    }

    y=x;
    x=0;
  } while (i < blkSize*blkSize);
}


void init_scan_orders()
{
  for (int log2size=1;log2size<=6;log2size++)
    {
      init_scan_h(scan_h[log2size], 1<<log2size);
      init_scan_v(scan_v[log2size], 1<<log2size);
      init_scan_d(scan_d[log2size], 1<<log2size);
    }
}

const position* get_scan_order(int log2BlockSize, int scanIdx)
{
  switch (scanIdx) {
  case 0: return scan_d[log2BlockSize];
  case 1: return scan_h[log2BlockSize];
  case 2: return scan_v[log2BlockSize];
  default: return 0; // should never happen
  }
}

