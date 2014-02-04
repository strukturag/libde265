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

#define DO_MEMORY_LOGGING 0

#include "de265.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <getopt.h>
#include <malloc.h>
#include <signal.h>

#ifndef _MSC_VER
#include <sys/time.h>
#include <unistd.h>
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

#if HAVE_SDL
#include "sdl.hh"
#endif

extern "C" {
#include "libde265/threads.h"
}


#ifndef _MSC_VER
extern "C" {
void showMotionProfile();
void showIntraPredictionProfile();
void showTransformProfile();
}
#else
void showMotionProfile();
void showIntraPredictionProfile();
void showTransformProfile();
#endif



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

#if HAVE_SDL
SDL_YUV_Display sdlWin;
bool sdl_active=false;

bool display_sdl(const struct de265_image* img)
{
  if (!sdl_active) {
    int width  = de265_get_image_width(img,0);
    int height = de265_get_image_height(img,0);

    sdl_active=true;
    sdlWin.init(width,height);
  }

  sdlWin.display((uint8_t*)de265_get_image_plane(img,0,NULL),
                 (uint8_t*)de265_get_image_plane(img,1,NULL),
                 (uint8_t*)de265_get_image_plane(img,2,NULL));

  return sdlWin.doQuit();
}
#endif


#ifdef WIN32
#include <time.h>
#define WIN32_LEAN_AND_MEAN
#include <winsock.h>
int gettimeofday(struct timeval *tp, void *)
{
    time_t clock;
    struct tm tm;
    SYSTEMTIME wtm;

    GetLocalTime(&wtm);
    tm.tm_year      = wtm.wYear - 1900;
    tm.tm_mon       = wtm.wMonth - 1;
    tm.tm_mday      = wtm.wDay;
    tm.tm_hour      = wtm.wHour;
    tm.tm_min       = wtm.wMinute;
    tm.tm_sec       = wtm.wSecond;
    tm. tm_isdst    = -1;
    clock = mktime(&tm);
    tp->tv_sec = (long) clock;
    tp->tv_usec = wtm.wMilliseconds * 1000;

    return (0);
}
#endif

#define BUFFER_SIZE 4096
#define NUM_THREADS 4

int nThreads=0;
bool quiet=false;
bool check_hash=false;
bool show_profile=false;
bool show_help=false;
bool dump_headers=false;
bool write_yuv=false;
bool output_with_videogfx=false;
bool logging=true;
//std::string output_filename;
uint32_t max_frames=UINT32_MAX;

static struct option long_options[] = {
  {"quiet",      no_argument,       0, 'q' },
  {"threads",    required_argument, 0, 't' },
  {"check-hash", no_argument,       0, 'c' },
  {"profile",    no_argument,       0, 'p' },
  {"frames",     required_argument, 0, 'f' },
  {"output",     no_argument,       0, 'o' },
  {"dump",       no_argument,       0, 'd' },
  {"videogfx",   no_argument,       0, 'V' },
  {"no-logging", no_argument,       0, 'L' },
  {"help",       no_argument,       0, 'h' },
  //{"verbose",    no_argument,       0, 'v' },
  {0,         0,                 0,  0 }
};

#ifdef HAVE___MALLOC_HOOK
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
static void *(*old_malloc_hook)(size_t, const void *);

static void *new_malloc_hook(size_t size, const void *caller) {
  void *mem;

  /*
  if (size>1000000) {
    raise(SIGINT);
  }
  */

  __malloc_hook = old_malloc_hook;
  mem = malloc(size);
  fprintf(stderr, "%p: malloc(%zu) = %p\n", caller, size, mem);
  __malloc_hook = new_malloc_hook;

  return mem;
}

static void init_my_hooks(void) {
  old_malloc_hook = __malloc_hook;
  __malloc_hook = new_malloc_hook;
}

#if DO_MEMORY_LOGGING
void (*volatile __malloc_initialize_hook)(void) = init_my_hooks;
#endif
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#endif

int main(int argc, char** argv)
{
  while (1) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "qt:chpf:odL"
#if HAVE_VIDEOGFX && HAVE_SDL
                        "V"
#endif
                        , long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
    case 'q': quiet=true; break;
    case 't': nThreads=atoi(optarg); break;
    case 'c': check_hash=true; break;
    case 'p': show_profile=true; break;
    case 'f': max_frames=atoi(optarg); break;
    case 'o': write_yuv=true; /*output_filename=optarg;*/ break;
    case 'h': show_help=true; break;
    case 'd': dump_headers=true; break;
    case 'V': output_with_videogfx=true; break;
    case 'L': logging=false; break;
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
    fprintf(stderr,"  -p, --profile     show coding mode usage profile\n");
    fprintf(stderr,"  -f, --frames N    set number of frames to process\n");
    fprintf(stderr,"  -o, --output      write YUV reconstruction\n");
    fprintf(stderr,"  -d, --dump        dump headers\n");
#if HAVE_VIDEOGFX && HAVE_SDL
    fprintf(stderr,"  -V, --videogfx    output with videogfx instead of SDL\n");
#endif
    fprintf(stderr,"  -h, --help        show help\n");

    exit(show_help ? 0 : 5);
  }


  de265_error err =DE265_OK;

  de265_decoder_context* ctx = de265_new_decoder();

  if (argc>=3) {
    if (nThreads>0) {
      err = de265_start_worker_threads(ctx, nThreads);
    }
  }

  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, check_hash);

  if (dump_headers) {
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_SPS_HEADERS, 1);
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_VPS_HEADERS, 1);
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_PPS_HEADERS, 1);
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_SLICE_HEADERS, 1);
  }

  if (!logging) {
    de265_disable_logging();
  }

  FILE* fh = fopen(argv[optind], "rb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s!\n", argv[1]);
    exit(10);
  }

  bool stop=false;
  uint32_t framecnt=0;

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

        width  = de265_get_image_width(img,0);
        height = de265_get_image_height(img,0);

        framecnt++;
        //fprintf(stderr,"SHOW POC: %d\n",img->PicOrderCntVal);

        if (!quiet) {
#if HAVE_SDL && HAVE_VIDEOGFX
          if (output_with_videogfx) { 
            display_image(img);
          } else {
            stop = display_sdl(img);
          }
#elif HAVE_SDL
          stop = display_sdl(img);
#elif HAVE_VIDEOGFX
          display_image(img);
#endif

          if (write_yuv) {
            write_picture(img);
          }
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

      if (framecnt>=max_frames) {
        stop=true;
      }
    }

  fclose(fh);

  de265_free_decoder(ctx);

  struct timeval tv_end;
  gettimeofday(&tv_end, NULL);

  if (err != DE265_OK) {
    fprintf(stderr,"decoding error: %s (%d)\n", de265_get_error_text(err), err);
  }

  double secs = tv_end.tv_sec-tv_start.tv_sec;
  secs += (tv_end.tv_usec - tv_start.tv_usec)*0.001*0.001;

  fprintf(stderr,"nFrames decoded: %d (%dx%d @ %5.2f fps)\n",framecnt,
          width,height,framecnt/secs);


  if (show_profile) {
    showMotionProfile();
    showIntraPredictionProfile();
    showTransformProfile();
  }

  return err==DE265_OK ? 0 : 10;
}
