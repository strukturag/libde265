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


#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/encode.h"
#include <assert.h>

error_queue errqueue;

video_parameter_set vps;
seq_parameter_set   sps;
pic_parameter_set   pps;
slice_segment_header shdr;

CABAC_encoder writer;

de265_image img;
encoder_context ectx;


void draw_image()
{
  int w = sps.pic_width_in_luma_samples;
  int h = sps.pic_height_in_luma_samples;

  img.alloc_image(w,h, de265_chroma_420, &sps, NULL /* no decctx */);

  initialize_CABAC_models(ectx.ctx_model, shdr.initType, shdr.SliceQPY);

  int Log2CtbSize = sps.Log2CtbSizeY;

  for (int y=0;y<sps.PicHeightInCtbsY;y++)
    for (int x=0;x<sps.PicWidthInCtbsY;x++)
      {
        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;
        img.set_ctDepth(x0,y0, Log2CtbSize, ((x+y)&1)==0);
        img.set_pred_mode(x0,y0, Log2CtbSize, MODE_INTRA);
        img.set_PartMode(x0,y0, PART_2Nx2N);
        img.set_IntraPredMode(x0,y0, Log2CtbSize, (enum IntraPredMode)1);
      }
}


void write_stream_1()
{
  nal_header nal;

  // VPS

  vps.set_defaults(Profile_Main, 6,2);


  // SPS

  sps.set_defaults();
  sps.set_CB_log2size_range(4,5);
  sps.set_resolution(3840,2880);
  sps.compute_derived_values();

  // PPS

  pps.set_defaults();
  pps.set_derived_values(&sps);


  // slice

  shdr.set_defaults(&pps);

  img.vps  = vps;
  img.sps  = sps;
  img.pps  = pps;

  draw_image();

  ectx.img = &img;
  ectx.shdr = &shdr;
  ectx.cabac_encoder = &writer;

  //context_model ctx_model[CONTEXT_MODEL_TABLE_LENGTH];



  // write headers

  writer.write_startcode();
  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(&writer);
  vps.write(&errqueue, &writer);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(&writer);
  sps.write(&errqueue, &writer);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(&writer);
  pps.write(&errqueue, &writer, &sps);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_IDR_W_RADL);
  nal.write(&writer);
  shdr.write(&errqueue, &writer, &sps, &pps, nal.nal_unit_type);
  writer.skip_bits(1);
  writer.flush_VLC();

  encode_image(&ectx);
  writer.flush_CABAC();
}



int main(int argc, char** argv)
{
  de265_set_verbosity(3);

  write_stream_1();

  FILE* fh = fopen("out.bin","wb");
  fwrite(writer.data(), 1,writer.size(), fh);
  fclose(fh);

  return 0;
}
