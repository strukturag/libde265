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

#include "libde265/encode.h"
#include "libde265/analyze.h"
#include "libde265/fallback.h"

#include <getopt.h>


int show_help=false;
int verbosity=0;

static struct option long_options[] = {
  {"help",       no_argument,       &show_help, 1 },
  {"verbose",    no_argument,       0, 'v' },
  {0,            0,                 0,  0 }
};



int main(int argc, char** argv)
{
  encoder_context ectx;


  // --- read encoder parameters ---

  bool cmdline_errors = false;
  config_parameters config_param;
  register_encoder_params(&config_param);

  FILE* reco_fh; // TODO

  if (!config_param.parse_command_line_params(&argc,argv, &ectx.params, true)) {
    cmdline_errors = true;
  }



  while (1) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "v"
                        , long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
    case 'v': verbosity++; break;
    }
  }


  // --- show usage information ---

  if (optind != argc || cmdline_errors || show_help) {
    fprintf(stderr," enc265  v%s\n", de265_get_version());
    fprintf(stderr,"--------------\n");
    fprintf(stderr,"usage: enc265 [options]\n");
    fprintf(stderr,"The video file must be a raw YUV file\n");
    fprintf(stderr,"\n");
    fprintf(stderr,"options:\n");
    fprintf(stderr,"      --help         show help\n");
    fprintf(stderr,"  -v, --verbose      increase verbosity level (up to 3 times)\n");

    config_param.show_params();

    exit(show_help ? 0 : 5);
  }



  // --- initialize encoder ---

  init_scan_orders();
  alloc_and_init_significant_coeff_ctxIdx_lookupTable();
  init_acceleration_functions_fallback(&ectx.accel);

  de265_set_verbosity(verbosity);

  ImageSink_YUV reconstruction_sink;
  if (strlen(ectx.params.reconstruction_yuv) != 0) {
    reconstruction_sink.set_filename(ectx.params.reconstruction_yuv);
    ectx.reconstruction_sink = &reconstruction_sink;
  }

  ImageSource_YUV image_source;
  image_source.set_input_file(ectx.params.input_yuv,
                              ectx.params.input_width,
                              ectx.params.input_height);
  ectx.img_source = &image_source;

  PacketSink_File packet_sink;
  packet_sink.set_filename(ectx.params.output_filename);
  ectx.packet_sink = &packet_sink;


  // --- run encoder ---

  encode_sequence(&ectx);

  return 0;
}
