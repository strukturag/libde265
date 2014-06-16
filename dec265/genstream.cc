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
#include <assert.h>

error_queue errqueue;

video_parameter_set vps;
seq_parameter_set   sps;
pic_parameter_set   pps;

CABAC_encoder writer;


void write_stream_1()
{
  nal_header nal;

  // VPS

  vps.set_defaults(Profile_Main, 6,2);


  // SPS

  sps.set_defaults();
  sps.set_resolution(384,288);


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
}



int main(int argc, char** argv)
{
  write_stream_1();

  FILE* fh = fopen("out.bin","wb");
  fwrite(writer.data(), 1,writer.size(), fh);
  fclose(fh);

  return 0;
}
