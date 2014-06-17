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

#include "encode.h"
#include "slice.h"


static void encode_split_cu_flag(encoder_context* ectx,
                                 int x0, int y0, int ctDepth, int split_flag)
{
  // check if neighbors are available

  int availableL = check_CTB_available(ectx->img,ectx->shdr, x0,y0, x0-1,y0);
  int availableA = check_CTB_available(ectx->img,ectx->shdr, x0,y0, x0,y0-1);

  int condL = 0;
  int condA = 0;

  if (availableL && ectx->img->get_ctDepth(x0-1,y0) > ctDepth) condL=1;
  if (availableA && ectx->img->get_ctDepth(x0,y0-1) > ctDepth) condA=1;

  int contextOffset = condL + condA;
  int context = contextOffset;

  // decode bit

  ectx->cabac_encoder->write_CABAC_bit(&ectx->ctx_model[CONTEXT_MODEL_SPLIT_CU_FLAG + context], split_flag);
}



void encode_quadtree(encoder_context* ectx,
                     int x0,int y0, int log2CbSize, int ctDepth)
{
  de265_image* img = ectx->img;
  const seq_parameter_set* sps = &img->sps;

  int split_flag;

  /*
     CU split flag:

          | overlaps | minimum ||
     case | border   | size    ||  split
     -----+----------+---------++----------
       A  |    0     |     0   || optional
       B  |    0     |     1   ||    0
       C  |    1     |     0   ||    1
       D  |    1     |     1   ||    0
   */
  if (x0+(1<<log2CbSize) <= sps->pic_width_in_luma_samples &&
      y0+(1<<log2CbSize) <= sps->pic_height_in_luma_samples &&
      log2CbSize > sps->Log2MinCbSizeY) {

    // case A

    split_flag = (img->get_ctDepth(x0,y0) != ctDepth);

    encode_split_cu_flag(ectx, x0,y0, ctDepth, split_flag);
  } else {
    // case B/C/D

    if (log2CbSize > sps->Log2MinCbSizeY) { split_flag=1; }
    else                                  { split_flag=0; }
  }

}


void encode_image(encoder_context* ectx)
{
  de265_image* img = ectx->img;
  int log2ctbSize = img->sps.Log2CtbSizeY;

  for (int ctbY=0;ctbY<img->sps.PicHeightInCtbsY;ctbY++)
    for (int ctbX=0;ctbX<img->sps.PicWidthInCtbsY;ctbX++)
      {
        encode_quadtree(ectx, ctbX,ctbY, log2ctbSize, 0);
      }
}
