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

#include "libde265/en265.h" //coder-context.h"

#include "libde265/configparam.h"
#include "libde265/image-io.h"
#include "libde265/analyze.h"

#include <getopt.h>


int show_help=false;
int verbosity=0;

static struct option long_options[] = {
  {"help",       no_argument,       &show_help, 1 },
  {"verbose",    no_argument,       0, 'v' },
  {0,            0,                 0,  0 }
};


struct inout_params
{
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
};


void register_params(config_parameters* config)
{
  const int NO_LIMIT = config_parameters::NO_LIMIT;

#define eoffset(name) offsetof(inout_params, name)

  config->register_config_string("input", 'i', eoffset(input_yuv), "paris_cif.yuv");
  config->register_config_int("width",  'w', eoffset(input_width),
                              352,      1,NO_LIMIT);
  config->register_config_int("height", 'h', eoffset(input_height),
                              288,      1,NO_LIMIT);

  config->register_config_string("output", 'o' , eoffset(output_filename), "out.bin");
  config->register_config_string("reconstruction", 'O' , eoffset(reconstruction_yuv), "recon.yuv");

  config->register_config_int("first-frame",  0 , eoffset(first_frame),
                              0,       0,NO_LIMIT);
  config->register_config_int("frames",      'f', eoffset(max_number_of_frames),
                              INT_MAX, 1,NO_LIMIT);
}


extern int skipTBSplit, noskipTBSplit;
extern int zeroBlockCorrelation[6][2][5];

int main(int argc, char** argv)
{
  en265_encoder_context* ectx = en265_new_encoder();


  bool cmdline_errors = false;

  // --- in/out parameters ---

  struct inout_params inout_params;
  config_parameters inout_param_config;
  register_params(&inout_param_config);

  if (!inout_param_config.parse_command_line_params(&argc,argv, &inout_params, true)) {
    cmdline_errors = true;
  }


  // --- read encoder parameters ---

  if (en265_parse_command_line_parameters(ectx, &argc, argv) != DE265_OK) {
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

    inout_param_config.show_params(&inout_params);
    fprintf(stderr,"\n");
    en265_show_params(ectx);

    exit(show_help ? 0 : 5);
  }



  de265_set_verbosity(verbosity);



  ImageSink_YUV reconstruction_sink;
  if (strlen(inout_params.reconstruction_yuv) != 0) {
    reconstruction_sink.set_filename(inout_params.reconstruction_yuv);
    //ectx.reconstruction_sink = &reconstruction_sink;
  }

  ImageSource_YUV image_source;
  image_source.set_input_file(inout_params.input_yuv,
                              inout_params.input_width,
                              inout_params.input_height);
  //ectx.img_source = &image_source;

  PacketSink_File packet_sink;
  packet_sink.set_filename(inout_params.output_filename);
  //ectx.packet_sink = &packet_sink;


  // --- run encoder ---

  //encode_sequence(&ectx);

  image_source.skip_frames( inout_params.first_frame );

  int maxPoc = inout_params.max_number_of_frames;
  bool eof = false;
  for (int poc=0; poc<maxPoc && !eof ;poc++)
    {
      // push one image into the encoder

      de265_image* input_image = image_source.get_image();
      if (input_image==NULL) {
        en265_push_eof(ectx);
        eof=true;
      }
      else {
        en265_push_image(ectx, input_image);
      }



      // encode images while more are available

      en265_encode(ectx);


      // write all pending packets

      for (;;) {
        en265_packet* pck = en265_get_packet(ectx,0);
        if (pck==NULL)
          break;

        packet_sink.send_packet(pck->data, pck->length);

        en265_free_packet(ectx,pck);
      }
    }



  // --- print statistics ---

  en265_print_logging((encoder_context*)ectx, "tb-split", NULL);


  en265_free_encoder(ectx);

  return 0;
}
