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
#include <sys/time.h>
#include <unistd.h>
#include <getopt.h>

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

extern "C" {
void showMotionProfile();
void showIntraPredictionProfile();
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

int nThreads=0;
bool quiet=false;
bool check_hash=false;
bool show_help=false;

static struct option long_options[] = {
  {"quiet",      no_argument,       0, 'q' },
  {"threads",    required_argument, 0, 't' },
  {"check-hash", no_argument,       0, 'c' },
  {"help",       no_argument,       0, 'h' },
  //{"verbose",    no_argument,       0, 'v' },
  {0,         0,                 0,  0 }
};


int main(int argc, char** argv)
{
  while (1) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "qt:ch",
                        long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
    case 'q': quiet=true; break;
    case 't': nThreads=atoi(optarg); break;
    case 'c': check_hash=true; break;
    case 'h': show_help=true; break;
    }
  }

  if (optind != argc-1 || show_help) {
    fprintf(stderr,"usage: dec265 [options] videofile.bin\n");
    fprintf(stderr,"The video file must be a raw h.265 bitstream (e.g. HM-10.0 output)\n");
    fprintf(stderr,"\n");
    fprintf(stderr,"options:\n");
    fprintf(stderr,"  -q, --quiet       do not show decoded image\n");
    fprintf(stderr,"  -t, --threads N   set number of worker threads (0 - no threading)\n");
    fprintf(stderr,"  -c, --check-hash  perform hash check\n");
    fprintf(stderr,"  -h, --help        show help\n");

    exit(show_help ? 0 : 5);
  }


  de265_error err =DE265_OK;

  de265_init();
  de265_decoder_context* ctx = de265_new_decoder();

  if (argc>=3) {
    if (nThreads>0) {
      err = de265_start_worker_threads(ctx, nThreads);
    }
  }

  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, check_hash);

  FILE* fh = fopen(argv[optind], "rb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s!\n", argv[1]);
    exit(10);
  }

  bool stop=false;
  int framecnt=0;

  struct timeval tv_start;
  gettimeofday(&tv_start, NULL);

  int width,height;

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

      // printf("pending data: %d\n", de265_get_number_of_input_bytes_pending(ctx));

      if (feof(fh)) {
        err = de265_decode_data(ctx, NULL, 0); // indicate end of stream
        stop = true;
      }

      // show queued output images
      for (;;) {
        const de265_image* img = de265_get_next_picture(ctx);
        if (img==NULL) break;

        width  = img->width;
        height = img->height;

        framecnt++;
        //fprintf(stderr,"SHOW POC: %d\n",img->PicOrderCntVal);

        if (!quiet) {
#if HAVE_VIDEOGFX
          display_image(img);
#else
          //write_picture(img);
#endif
        }

        if ((framecnt%100)==0) {
          fprintf(stderr,"frame %d\r",framecnt);
        }
      }

      for (;;) {
        de265_error warning = de265_get_warning(ctx);
        if (warning==DE265_OK) {
          break;
        }

        fprintf(stderr,"WARNING: %s\n", de265_get_error_text(warning));
      }
    }

  fclose(fh);

  de265_free_decoder(ctx);

  struct timeval tv_end;
  gettimeofday(&tv_end, NULL);

  if (err != DE265_OK) {
    fprintf(stderr,"decoding error: %s\n", de265_get_error_text(err));
  }

  double secs = tv_end.tv_sec-tv_start.tv_sec;
  secs += (tv_end.tv_usec - tv_start.tv_usec)*0.001*0.001;

  fprintf(stderr,"nFrames decoded: %d (%dx%d @ %5.2f fps)\n",framecnt,
          width,height,framecnt/secs);


  showMotionProfile();
  showIntraPredictionProfile();

  return err==DE265_OK ? 0 : 10;
}
