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
#include "libde265/encoder/analyze.h"

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
  inout_params();

  // input

  option_int first_frame;
  option_int max_number_of_frames;

  option_string input_yuv;
  option_int input_width;
  option_int input_height;

  // output

  option_string output_filename;

  // debug

  option_string reconstruction_yuv;


  void register_params(config_parameters& config);
};


inout_params::inout_params()
{
  input_yuv.set_ID("input"); input_yuv.set_short_option('i');
  input_yuv.set_default("paris_cif.yuv");

  output_filename.set_ID("output"); output_filename.set_short_option('o');
  output_filename.set_default("out.bin");

  reconstruction_yuv.set_ID("input");
  reconstruction_yuv.set_default("recon.yuv");

  first_frame.set_ID("first-frame");
  first_frame.set_default(0);
  first_frame.set_minimum(0);

  max_number_of_frames.set_ID("frames");
  max_number_of_frames.set_short_option('f');
  max_number_of_frames.set_minimum(1);
  //max_number_of_frames.set_default(INT_MAX);

  input_width.set_ID("width"); input_width.set_short_option('w');
  input_width.set_minimum(1);  input_width.set_default(352);

  input_height.set_ID("height"); input_height.set_short_option('h');
  input_height.set_minimum(1); input_height.set_default(288);
}


void inout_params::register_params(config_parameters& config)
{
  config.add_option(&input_yuv);
  config.add_option(&first_frame);
  config.add_option(&max_number_of_frames);
  config.add_option(&input_width);
  config.add_option(&input_height);
}


void test_parameters_API(en265_encoder_context* ectx)
{
  const char** param = en265_list_parameters(ectx);
  if (param) {
    for (int i=0; param[i]; i++) {
      printf("|%s| ",param[i]);

      enum en265_parameter_type type = en265_get_parameter_type(ectx, param[i]);
      const char* type_name="unknown";
      switch (type) {
      case en265_parameter_int: type_name="int"; break;
      case en265_parameter_bool: type_name="bool"; break;
      case en265_parameter_string: type_name="string"; break;
      case en265_parameter_choice: type_name="choice"; break;
      }

      printf("(%s)",type_name);

      if (type==en265_parameter_choice) {
        const char** choices = en265_list_parameter_choices(ectx, param[i]);
        if (choices) {
          for (int k=0; choices[k]; k++) {
            printf(" %s",choices[k]);
          }
        }
      }

      printf("\n");
    }
  }

  // en265_set_parameter_int(ectx, "min-tb-size", 8);
}


extern int skipTBSplit, noskipTBSplit;
extern int zeroBlockCorrelation[6][2][5];

int main(int argc, char** argv)
{
  de265_init();

  en265_encoder_context* ectx = en265_new_encoder();


  bool cmdline_errors = false;

  // --- in/out parameters ---

  struct inout_params inout_params;
  config_parameters inout_param_config;
  inout_params.register_params(inout_param_config);

  int first_idx=1;
  if (!inout_param_config.parse_command_line_params(&argc,argv, &first_idx, true)) {
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

    inout_param_config.print_params();
    fprintf(stderr,"\n");
    en265_show_params(ectx);

    exit(show_help ? 0 : 5);
  }



  de265_set_verbosity(verbosity);


  //test_parameters_API(ectx);


  ImageSink_YUV reconstruction_sink;
  if (strlen(inout_params.reconstruction_yuv.get().c_str()) != 0) {
    reconstruction_sink.set_filename(inout_params.reconstruction_yuv.get().c_str());
    //ectx.reconstruction_sink = &reconstruction_sink;
  }

  ImageSource_YUV image_source;
  image_source.set_input_file(inout_params.input_yuv.get().c_str(),
                              inout_params.input_width,
                              inout_params.input_height);

  PacketSink_File packet_sink;
  packet_sink.set_filename(inout_params.output_filename.get().c_str());


  // --- run encoder ---

  image_source.skip_frames( inout_params.first_frame );

  int maxPoc = INT_MAX;
  if (inout_params.max_number_of_frames.is_defined()) {
    maxPoc = inout_params.max_number_of_frames;
  }

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

  de265_free();

  return 0;
}
