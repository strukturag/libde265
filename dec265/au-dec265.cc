/*
  libde265 example application "au-dec265".

  MIT License

  Copyright (c) 2023 Dirk Farin <dirk.farin@gmail.com>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "libde265/de265.h"
#include "libde265/de265-multilayer.h"
#include <map>
#include <string>
#include <vector>
#include <cstring>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_LIBPNG
#include "encoder_png.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <sstream>
#include <getopt.h>
#include <iomanip>


#define BUFFER_SIZE 40960
#define NUM_THREADS 4

bool show_help = false;
#if HAVE_LIBPNG
bool output_as_yuv = false;
#else
bool output_as_yuv = true;
#endif


static struct option long_options[] = {
    {"help",   no_argument,       0, 'h'},
#if HAVE_LIBPNG
    {"yuv",    no_argument,       0, 'y'},
#endif
    {0, 0,                        0, 0}
};


static void output_yuv_picture(const de265_image* img, std::string layerType, bool first)
{
  std::string filename = layerType + ".yuv";

  FILE* fh = fopen(filename.c_str(), first ? "wb" : "ab");

  for (int c = 0; c < 3; c++) {
    int stride;
    const uint8_t* p = de265_get_image_plane(img, c, &stride);

    int width = de265_get_image_width(img, c);

    if (de265_get_bits_per_pixel(img, c) <= 8) {
      // --- save 8 bit YUV ---

      for (int y = 0; y < de265_get_image_height(img, c); y++) {
        fwrite(p + y * stride, width, 1, fh);
      }
    }
    else {
      // --- save 16 bit YUV ---

      int bpp = (de265_get_bits_per_pixel(img, c) + 7) / 8;
      int pixelsPerLine = stride / bpp;

      uint8_t* buf = new uint8_t[width * 2];
      uint16_t* p16 = (uint16_t*) p;

      for (int y = 0; y < de265_get_image_height(img, c); y++) {
        for (int x = 0; x < width; x++) {
          uint16_t pixel_value = (p16 + y * pixelsPerLine)[x];
          buf[2 * x + 0] = pixel_value & 0xFF;
          buf[2 * x + 1] = pixel_value >> 8;
        }

        fwrite(buf, width * 2, 1, fh);
      }

      delete[] buf;
    }
  }

  fclose(fh);
}


struct image
{
  int auxId;
  const de265_image* img;
};


void do_show_stream_info(const de265_access_unit* au)
{
  const de265_vps* vps = de265_access_unit_peek_vps(au);

  int layers = de265_vps_get_max_layers(vps);
  fprintf(stderr, "max number of layers: %d\n", layers);
  for (int l = 0; l < layers; l++) {
    std::string content;
    int auxId = de265_vps_get_layer_aux_id(vps, l);
    if (auxId == de265_aux_none) {
      content = "video";
    }
    else if (auxId == de265_aux_alpha) {
      content = "alpha";
    }
    else if (auxId == de265_aux_depth) {
      content = "depth";
    }
    else {
      content = std::to_string(auxId);
    }

    auto* img = de265_access_unit_peek_layer_picture(au, l);

    fprintf(stderr, "  layer[%d] content: %s (%dx%d)\n",
            l,
            content.c_str(),
            de265_get_image_width(img, 0),
            de265_get_image_height(img, 0));
  }
}

static uint32_t framecnt = 0;

void output_access_unit_in_yuv_files(const de265_access_unit* au)
{
  const de265_vps* vps = de265_access_unit_peek_vps(au);
  int num_layers = de265_vps_get_max_layers(vps);

  std::vector<image> images;

  for (int i = 0; i < num_layers; i++) {
    const de265_image* img;
    img = de265_access_unit_peek_layer_picture(au, i);

    if (img) {
      image pimg;
      pimg.img = img;
      pimg.auxId = de265_vps_get_layer_aux_id(vps, i);
      images.push_back(pimg);
    }
  }


  for (const auto& img : images) {
    std::string auxType;
    switch (img.auxId) {
      case 0:
        auxType = "video";
        break;
      case de265_aux_alpha:
        auxType = "alpha";
        break;
      case de265_aux_depth:
        auxType = "depth";
        break;
      default:
        auxType = "aux" + std::to_string(img.auxId);
        break;
    }

    output_yuv_picture(img.img, auxType, framecnt==1);
  }
}


void output_access_unit_as_png(const de265_access_unit* au)
{
#if HAVE_LIBPNG

  // --- get color and alpha layers (if available)

  const de265_vps* vps = de265_access_unit_peek_vps(au);

  const de265_image* img_color = nullptr;
  const de265_image* img_alpha = nullptr;

  int color_layer_id = de265_vps_get_layer_id_for_aux_id(vps, de265_aux_none);
  if (color_layer_id != -1) {
    img_color = de265_access_unit_peek_layer_picture(au, color_layer_id);
  }

  int alpha_layer_id = de265_vps_get_layer_id_for_aux_id(vps, de265_aux_alpha);
  if (alpha_layer_id != -1) {
    img_alpha = de265_access_unit_peek_layer_picture(au, alpha_layer_id);
  }


  // --- write to PNG file

  std::stringstream sstr;
  sstr << "output-" << std::setw(4) << std::setfill('0') << framecnt << ".png";

  if (img_color) {
    write_png(img_color, img_alpha, sstr.str());
  }
#endif
}


int main(int argc, char** argv)
{
  while (true) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "h"
#if HAVE_LIBPNG
                        "y"
#endif
                        , long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
      case 'h':
        show_help = true;
        break;
      case 'y':
        output_as_yuv = true;
        break;
    }
  }

  if (optind != argc - 1 || show_help) {
    fprintf(stderr, " au-dec265  v%s\n", de265_get_version());
    fprintf(stderr, "--------------\n");
    fprintf(stderr, "usage: au-dec265 [options] videofile.bin\n");
    fprintf(stderr, "The video file must be a raw bitstream.\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "options:\n");
    fprintf(stderr, "  -h, --help        show help\n");
#if HAVE_LIBPNG
    fprintf(stderr, "  -y, --yuv         output as YUV files\n");
#endif

    exit(show_help ? 0 : 5);
  }


  FILE* fh;
  if (strcmp(argv[optind], "-") == 0) {
    fh = stdin;
  }
  else {
    fh = fopen(argv[optind], "rb");
  }

  if (fh == nullptr) {
    fprintf(stderr, "cannot open file %s!\n", argv[optind]);
    exit(10);
  }


  de265_error err = DE265_OK;

  auto* decoder = de265_new_audecoder();

  bool stop = false;

  while (!stop) {
    // read a chunk of input data
    uint8_t buf[BUFFER_SIZE];
    int n = fread(buf, 1, BUFFER_SIZE, fh);

    // decode input data
    if (n) {
      err = de265_audecoder_push_data(decoder, buf, n, 0, nullptr);
      if (err != DE265_OK) {
        break;
      }
    }

    if (feof(fh)) {
      err = de265_audecoder_flush_data(decoder); // indicate end of stream
      stop = true;
    }


    // --- decoding / display loop

    int more = 1;
    while (more) {
      more = 0;

      // decode some more

      err = de265_audecoder_decode(decoder, &more);
      if (err != DE265_OK) {
        more = 0;
        break;
      }

      // write all images available at the decoder output

      for (;;) {
        const de265_access_unit* au = de265_audecoder_get_next_picture(decoder);
        if (au) {
          framecnt++;

          // --- show some stream info when we decoded the first frame

          static bool first = true;
          if (first) {
            do_show_stream_info(au);
            first = false;
          }

          printf("write frame %d\r", framecnt);
          fflush(stdout);

          if (output_as_yuv) {
            output_access_unit_in_yuv_files(au);
          }
          else {
            output_access_unit_as_png(au);
          }

          de265_access_unit_release(au);
        }
        else {
          break;
        }
      }
    }
  }

  fclose(fh);

  de265_audecoder_release(decoder);

  if (err != DE265_OK) {
    fprintf(stderr, "decoding error: %s (code=%d)\n", de265_get_error_text(err), err);
  }

  fprintf(stderr, "nFrames decoded: %d\n", framecnt);

  return err == DE265_OK ? 0 : 10;
}
