/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "de265.h"
#ifndef _MSC_VER
#include "config.h"
#endif
#include <stdio.h>
#include <stdlib.h>

#ifndef _MSC_VER
extern "C" {
#include "libde265/decctx.h"
}
#else
// VS2008 didn't support C99, compile all everything as C++
#include "libde265/decctx.h"
#endif

#if HAVE_VIDEOGFX
#include <libvideogfx.hh>
using namespace videogfx;
#endif

extern "C" {
#include "libde265/threads.h"
}


#if HAVE_VIDEOGFX
void display_image(const struct de265_image* img)
{
  static X11Win win;

  // display picture

  static bool first=true;

  if (first) {
    first=false;
    win.Create(de265_get_image_width(img,0),
               de265_get_image_height(img,0),
               "de265 output");
  }



  int width  = de265_get_image_width(img,0);
  int height = de265_get_image_height(img,0);

  Image<Pixel> visu;
  visu.Create(width, height, Colorspace_YUV, Chroma_420);

  for (int ch=0;ch<3;ch++) {
    const uint8_t* data;
    int stride;

    data   = de265_get_image_plane(img,ch,&stride);
    width  = de265_get_image_width(img,ch);
    height = de265_get_image_height(img,ch);

    for (int y=0;y<height;y++) {
      memcpy(visu.AskFrame((BitmapChannel)ch)[y], data + y*stride, width);
    }
  }

  win.Display(visu);
  //win.WaitForKeypress();
}
#endif


#define BUFFER_SIZE 4096
#define NUM_THREADS 4


int main(int argc, char** argv)
{
  if (argc < 2 || argc > 3) {
    fprintf(stderr,"usage: dec265 videofile.bin number-of-threads\n");
    fprintf(stderr,"The video file must be a raw h.265 bitstream (e.g. HM-10.0 output)\n");
    exit(5);
  }


  de265_error err =DE265_OK;

  de265_init();
  de265_decoder_context* ctx = de265_new_decoder();

  if (argc>=3) {
    int nThreads = atoi(argv[2]);
    if (nThreads>0) {
      err = de265_start_worker_threads(ctx, nThreads);
    }
  }

  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, false);

  FILE* fh = fopen(argv[1], "rb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s!\n", argv[1]);
    exit(10);
  }

  bool stop=false;
  while (!stop)
    {
      // read a chunk of input data
      uint8_t buf[BUFFER_SIZE];
      int n = fread(buf,1,BUFFER_SIZE,fh);

      // decode input data
      if (n) {
	err = de265_decode_data(ctx, buf, n);
	if (err != DE265_OK) {
	  break;
	}
      }

      if (feof(fh)) {
        err = de265_decode_data(ctx, NULL, 0); // indicate end of stream
        stop = true;
      }

      // show queued output images
      for (;;) {
        const de265_image* img = de265_get_next_picture(ctx);
        if (img==NULL) break;

        //fprintf(stderr,"SHOW POC: %d\n",img->PicOrderCntVal);

#if HAVE_VIDEOGFX
        display_image(img);
#else
        write_picture(img);
#endif
      }
    }

  fclose(fh);

  de265_free_decoder(ctx);

  if (err != DE265_OK) {
    fprintf(stderr,"decoding error: %s\n", de265_get_error_text(err));
  }

  return err==DE265_OK ? 0 : 10;
}
