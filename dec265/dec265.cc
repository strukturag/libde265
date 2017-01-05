/*
 * libde265 example application "dec265".
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of dec265, an example application using libde265.
 *
 * dec265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dec265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with dec265.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DO_MEMORY_LOGGING 0

#include "de265.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <getopt.h>
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif
#include <signal.h>

#ifndef _MSC_VER
#include <sys/time.h>
#include <unistd.h>
#endif

#include "libde265/quality.h"
#include "libde265/util.h"

#ifdef WITH_FPS
#include "libde265/frame-dropper.h"
#endif

#if HAVE_VIDEOGFX
#include <libvideogfx.hh>
using namespace videogfx;
#endif

#if HAVE_SDL
#include "sdl.hh"
#endif


#define BUFFER_SIZE 40960
#define NUM_THREADS 4

int nThreads=0;
int nParallelFrames=10;
int max_latency=0; // off
bool nal_input=false;
int quiet=0;
bool check_hash=false;
bool show_help=false;
bool dump_headers=false;
bool write_yuv=false;
bool output_with_videogfx=false;
bool output_as_rgb=false;
bool no_acceleration=false;
const char *output_filename = "out.yuv";
uint32_t max_frames=UINT32_MAX;
bool write_bytestream=false;
const char *bytestream_filename;
bool measure_quality=false;
bool show_ssim_map=false;
bool show_psnr_map=false;
const char* reference_filename;
FILE* reference_file;
int highestTID = 100;
int decode_rate_percent = 100;
int verbosity=0;
int disable_deblocking=0;
int disable_sao=0;
int inexact_decoding_flags=0;

#define OPTION_MAX_LATENCY 1000

static struct option long_options[] = {
  {"quiet",      no_argument,       0, 'q' },
  {"threads",    required_argument, 0, 't' },
  {"parallel-frames", required_argument, 0, 'P' },
  {"check-hash", no_argument,       0, 'c' },
  {"profile",    no_argument,       0, 'p' },
  {"frames",     required_argument, 0, 'f' },
  {"output",     required_argument, 0, 'o' },
  {"dump",       no_argument,       0, 'd' },
  {"nal",        no_argument,       0, 'n' },
  {"videogfx",   no_argument,       0, 'V' },
  {"rgb",        no_argument,       0, 'R' },
  {"help",       no_argument,       0, 'h' },
  {"noaccel",    no_argument,       0, '0' },
  {"write-bytestream", required_argument,0, 'B' },
  {"measure",     required_argument, 0, 'm' },
  {"ssim",        no_argument,       0, 's' },
  {"errmap",      no_argument,       0, 'e' },
  {"highest-TID", required_argument, 0, 'T' },
  {"decoding-framerate-percent", required_argument, 0, 'D' },
  {"verbose",    no_argument,       0, 'v' },
  {"disable-deblocking", no_argument, &disable_deblocking, 1 },
  {"disable-sao",        no_argument, &disable_sao, 1 },
  {"enable-inexact-decoding", no_argument, 0, 'I' },
  {"max-latency",        required_argument, 0, OPTION_MAX_LATENCY },
  {0,         0,                 0,  0 }
};



static void write_picture(const de265_image* img)
{
  static FILE* fh = NULL;
  if (fh==NULL) { fh = fopen(output_filename, "wb"); }



  for (int c=0;c<3;c++) {
    int stride;
    const uint8_t* p = de265_get_image_plane(img, c, &stride);

    int width = de265_get_image_width(img,c);

    if (de265_get_bits_per_pixel(img,c)<=8) {
      // --- save 8 bit YUV ---

      for (int y=0;y<de265_get_image_height(img,c);y++) {
        fwrite(p + y*stride, width, 1, fh);
      }
    }
    else {
      // --- save 16 bit YUV ---

      int bpp = (de265_get_bits_per_pixel(img,c)+7)/8;
      int pixelsPerLine = stride/bpp;

      uint8_t* buf = new uint8_t[width*2];
      uint16_t* p16 = (uint16_t*)p;

      for (int y=0;y<de265_get_image_height(img,c);y++) {
        for (int x=0;x<width;x++) {
          uint16_t pixel_value = (p16+y*pixelsPerLine)[x];
          buf[2*x+0] = pixel_value & 0xFF;
          buf[2*x+1] = pixel_value >> 8;
        }

        fwrite(buf, width*2, 1, fh);
      }

      delete[] buf;
    }
  }

  fflush(fh);
}



#if HAVE_VIDEOGFX
//#include "libde265/image.h"
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
  de265_chroma chroma = de265_get_chroma_format(img);

  ChromaFormat vgfx_chroma;
  Colorspace   vgfx_cs;

  if (output_as_rgb)
    vgfx_cs = Colorspace_RGB;
  else
    vgfx_cs = Colorspace_YUV;

  switch (chroma) {
  case de265_chroma_420:  vgfx_chroma = Chroma_420; break;
  case de265_chroma_422:  vgfx_chroma = Chroma_422; break;
  case de265_chroma_444:  vgfx_chroma = Chroma_444; break;
  case de265_chroma_mono: vgfx_cs = Colorspace_Greyscale; break;
  }

  Image<Pixel> visu;
  visu.Create(width, height, vgfx_cs, vgfx_chroma);

  int nChannels = 3;
  if (chroma == de265_chroma_mono) {
    nChannels = 1;
  }

  const int channel_map_yuv[] = { 0,1,2 };
  const int channel_map_rgb[] = { 2,0,1 };
  const int* channel_map;

  if (output_as_rgb) {
    channel_map = channel_map_rgb;
  }
  else {
    channel_map = channel_map_yuv;
  }

  for (int ch=0;ch<nChannels;ch++) {
    const uint8_t* data;
    int stride;

    data   = de265_get_image_plane(img,channel_map[ch],&stride);
    width  = de265_get_image_width(img,channel_map[ch]);
    height = de265_get_image_height(img,channel_map[ch]);

    int bit_depth = de265_get_bits_per_pixel(img,ch);

    if (bit_depth==8) {
      for (int y=0;y<height;y++) {
        memcpy(visu.AskFrame((BitmapChannel)ch)[y], data + y*stride, width);
      }
    }
    else {
      const uint16_t* data16 = (const uint16_t*)data;
      for (int y=0;y<height;y++) {
        for (int x=0;x<width;x++) {
          visu.AskFrame((BitmapChannel)ch)[y][x] = *(data16 + y*stride +x) >> (bit_depth-8);
        }
      }
    }
  }

  //printf("displaying frame: %d\n", img->m_image->PicOrderCntVal);

  win.Display(visu);
  win.WaitForKeypress();
}
#endif

static uint8_t* convert_to_8bit(const uint8_t* data, int width, int height,
                                int pixelsPerLine, int bit_depth)
{
  const uint16_t* data16 = (const uint16_t*)data;
  uint8_t* out = new uint8_t[pixelsPerLine*height];

  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      out[y*pixelsPerLine + x] = *(data16 + y*pixelsPerLine +x) >> (bit_depth-8);
    }
  }

  return out;
}


#if HAVE_SDL
SDL_YUV_Display sdlWin;
bool sdl_active=false;

bool display_sdl(const struct de265_image* img)
{
  int width  = de265_get_image_width(img,0);
  int height = de265_get_image_height(img,0);

  int chroma_width  = de265_get_image_width(img,1);
  int chroma_height = de265_get_image_height(img,1);

  de265_chroma chroma = de265_get_chroma_format(img);

  if (!sdl_active) {
    sdl_active=true;
    enum SDL_YUV_Display::SDL_Chroma sdlChroma;
    switch (chroma) {
    case de265_chroma_420:  sdlChroma = SDL_YUV_Display::SDL_CHROMA_420;  break;
    case de265_chroma_422:  sdlChroma = SDL_YUV_Display::SDL_CHROMA_422;  break;
    case de265_chroma_444:  sdlChroma = SDL_YUV_Display::SDL_CHROMA_444;  break;
    case de265_chroma_mono: sdlChroma = SDL_YUV_Display::SDL_CHROMA_MONO; break;
    }

    sdlWin.init(width,height, sdlChroma);
  }

  int stride,chroma_stride;
  const uint8_t* y = de265_get_image_plane(img,0,&stride);
  const uint8_t* cb =de265_get_image_plane(img,1,&chroma_stride);
  const uint8_t* cr =de265_get_image_plane(img,2,NULL);

  int bpp_y = (de265_get_bits_per_pixel(img,0)+7)/8;
  int bpp_c = (de265_get_bits_per_pixel(img,1)+7)/8;
  int ppl_y = stride/bpp_y;
  int ppl_c = chroma_stride/bpp_c;

  uint8_t* y16  = NULL;
  uint8_t* cb16 = NULL;
  uint8_t* cr16 = NULL;
  int bd;

  if ((bd=de265_get_bits_per_pixel(img, 0)) > 8) {
    y16  = convert_to_8bit(y,  width,height,ppl_y,bd); y=y16;
  }

  if (chroma != de265_chroma_mono) {
    if ((bd=de265_get_bits_per_pixel(img, 1)) > 8) {
      cb16 = convert_to_8bit(cb, chroma_width,chroma_height,ppl_c,bd); cb=cb16;
    }
    if ((bd=de265_get_bits_per_pixel(img, 2)) > 8) {
      cr16 = convert_to_8bit(cr, chroma_width,chroma_height,ppl_c,bd); cr=cr16;
    }
  }

  sdlWin.display(y,cb,cr, ppl_y, ppl_c);

  delete[] y16;
  delete[] cb16;
  delete[] cr16;

  return sdlWin.doQuit();
}
#endif


static int width,height;

bool output_image(const de265_image* img)
{
  bool stop=false;

  width  = de265_get_image_width(img,0);
  height = de265_get_image_height(img,0);

  //printf("SHOW POC: %d / PTS: %ld / integrity: %d\n",img->PicOrderCntVal, img->pts, img->integrity);


  if (!quiet) {
#if HAVE_SDL && HAVE_VIDEOGFX
    if (output_with_videogfx) {
      display_image(img);
    } else {
      if (output_as_rgb) {
        fprintf(stderr,"RGB output requires libvideogfx output method\n");
        exit(10);
      }
      stop = display_sdl(img);
    }
#elif HAVE_SDL
    stop = display_sdl(img);
#elif HAVE_VIDEOGFX
    display_image(img);
#endif
  }
  if (write_yuv) {
    write_picture(img);
  }

  return stop;
}


static double mse_y=0.0, mse_cb=0.0, mse_cr=0.0;
static int    mse_frames=0;

static double ssim_y=0.0;
static int    ssim_frames=0;

void measure(const de265_image* img, int framecnt)
{
  // --- compute PSNR ---

  int width  = de265_get_image_width(img,0);
  int height = de265_get_image_height(img,0);

  uint8_t* p = (uint8_t*)malloc(width*height*3/2);
  if (p == NULL) {
    return;
  }

  size_t toread = width*height*3/2;
  if (fread(p,1,toread,reference_file) != toread) {
    free(p);
    return;
  }

  int stride, cstride;
  const uint8_t* yptr  = de265_get_image_plane(img,0, &stride);
  const uint8_t* cbptr = de265_get_image_plane(img,1, &cstride);
  const uint8_t* crptr = de265_get_image_plane(img,2, &cstride);

  double img_mse_y  = MSE( yptr,  stride, p, width,   width, height);
  double img_mse_cb = MSE(cbptr, cstride, p+width*height,      width/2, width/2,height/2);
  double img_mse_cr = MSE(crptr, cstride, p+width*height*5/4,  width/2, width/2,height/2);

  mse_frames++;

  mse_y  += img_mse_y;
  mse_cb += img_mse_cb;
  mse_cr += img_mse_cr;



  // --- compute SSIM ---

  double ssimSum = 0.0;

#if HAVE_VIDEOGFX
  Bitmap<Pixel> ref, coded;
  ref  .Create(width, height); // reference image
  coded.Create(width, height); // coded image

  const uint8_t* data;
  data = de265_get_image_plane(img,0,&stride);

  for (int y=0;y<height;y++) {
    memcpy(coded[y], data + y*stride, width);
    memcpy(ref[y],   p    + y*stride, width);
  }

  SSIM ssimAlgo;
  Bitmap<float> ssim = ssimAlgo.calcSSIM(ref,coded);

  Bitmap<Pixel> ssimMap;
  ssimMap.Create(width,height);

  for (int y=0;y<height;y++)
    for (int x=0;x<width;x++)
      {
        float v = ssim[y][x];
        ssimSum += v;
        v = v*v;
        v = 255*v; //pow(v, 20);

        //assert(v<=255.0);
        ssimMap[y][x] = v;
      }

  ssimSum /= width*height;


  Bitmap<Pixel> error_map = CalcErrorMap(ref, coded, TransferCurve_Sqrt);


  // display PSNR error map

  if (show_psnr_map) {
    static X11Win win;
    static bool first=true;

    if (first) {
      first=false;
      win.Create(de265_get_image_width(img,0),
                 de265_get_image_height(img,0),
                 "psnr output");
    }

    win.Display(MakeImage(error_map));
  }


  // display SSIM error map

  if (show_ssim_map) {
    static X11Win win;
    static bool first=true;

    if (first) {
      first=false;
      win.Create(de265_get_image_width(img,0),
                 de265_get_image_height(img,0),
                 "ssim output");
    }

    win.Display(MakeImage(ssimMap));
  }
#endif

  ssim_frames++;
  ssim_y += ssimSum;

  printf("%5d   %6f %6f %6f %6f\n",
         framecnt,
         PSNR(img_mse_y), PSNR(img_mse_cb), PSNR(img_mse_cr),
         ssimSum);

  free(p);
}


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

    int c = getopt_long(argc, argv, "qt:chf:o:dB:n0vT:D:m:seRP:I"
#if HAVE_VIDEOGFX && HAVE_SDL
                        "V"
#endif
                        , long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
    case 'q': quiet++; break;
    case 't': nThreads=atoi(optarg); break;
    case 'P': nParallelFrames=atoi(optarg); break;
    case 'c': check_hash=true; break;
    case 'f': max_frames=atoi(optarg); break;
    case 'o': write_yuv=true; output_filename=optarg; break;
    case 'h': show_help=true; break;
    case 'd': dump_headers=true; break;
    case 'n': nal_input=true; break;
    case 'V': output_with_videogfx=true; break;
    case 'R': output_as_rgb=true; break;
    case '0': no_acceleration=true; break;
    case 'B': write_bytestream=true; bytestream_filename=optarg; break;
    case 'm': measure_quality=true; reference_filename=optarg; break;
    case 's': show_ssim_map=true; break;
    case 'e': show_psnr_map=true; break;
    case 'T': highestTID=atoi(optarg); break;
    case 'D': decode_rate_percent=atoi(optarg); break;
    case 'v': verbosity++; break;
    case 'I': inexact_decoding_flags=de265_inexact_decoding_mask_numeric; break;
    case OPTION_MAX_LATENCY: max_latency=atoi(optarg); break;
    }
  }

  if (optind != argc-1 || show_help) {
    fprintf(stderr," dec265  v%s\n", de265_get_version());
    fprintf(stderr,"--------------\n");
    fprintf(stderr,"usage: dec265 [options] videofile.bin\n");
    fprintf(stderr,"The video file must be a raw bitstream, or a stream with NAL units (option -n).\n");
    fprintf(stderr,"\n");
    fprintf(stderr,"options:\n");
    fprintf(stderr,"  -q, --quiet       do not show decoded image\n");
    fprintf(stderr,"  -t, --threads N   set number of worker threads (0 - no threading)\n");
    fprintf(stderr,"  -P, --parallel-frames N   number of frames to decode in parallel (default=%d)\n", nParallelFrames);
    fprintf(stderr,"  -c, --check-hash  perform hash check\n");
    fprintf(stderr,"  -n, --nal         input is a stream with 4-byte length prefixed NAL units\n");
    fprintf(stderr,"  -f, --frames N    set number of frames to process\n");
    fprintf(stderr,"  -o, --output      write YUV reconstruction\n");
    fprintf(stderr,"  -d, --dump        dump headers\n");
#if HAVE_VIDEOGFX && HAVE_SDL
    fprintf(stderr,"  -V, --videogfx    output with videogfx instead of SDL\n");
#endif
    fprintf(stderr,"  -R, --rgb         show h.265 files coded in RGB colorspace\n");
    fprintf(stderr,"  -0, --noaccel     do not use any accelerated code (SSE)\n");
    fprintf(stderr,"  -v, --verbose     increase verbosity level (up to 3 times)\n");
    fprintf(stderr,"  -B, --write-bytestream FILENAME  write raw bytestream (from NAL input)\n");
    fprintf(stderr,"  -m, --measure YUV compute PSNRs relative to reference YUV\n");
#if HAVE_VIDEOGFX
    fprintf(stderr,"  -s, --ssim        show SSIM-map (only when -m active)\n");
    fprintf(stderr,"  -e, --errmap      show error-map (only when -m active)\n");
#endif
    fprintf(stderr,"  -T, --highest-TID select highest temporal sublayer to decode\n");
    fprintf(stderr,"  -D, --decoding-framerate-percent    percentage of frames to decode (others will be dropped)\n");
    fprintf(stderr,"      --disable-deblocking   disable deblocking filter\n");
    fprintf(stderr,"      --disable-sao          disable sample-adaptive offset filter\n");
    fprintf(stderr,"  -I, --enable-inexact-decoding  enable optimizations that may lead to slightly wrong output\n");
    fprintf(stderr,"      --max-latency          maximum picture latency in reorder buffer\n");
    fprintf(stderr,"  -h, --help        show help\n");

    exit(show_help ? 0 : 5);
  }


  de265_set_verbosity(verbosity);

  de265_error err =DE265_OK;

  de265_decoder_context* ctx = de265_new_decoder();

  // TODO de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, check_hash);
  de265_suppress_faulty_pictures(ctx, false);

  if (disable_deblocking) inexact_decoding_flags |= de265_inexact_decoding_no_deblocking;
  if (disable_sao)        inexact_decoding_flags |= de265_inexact_decoding_no_SAO;
  de265_allow_inexact_decoding(ctx, inexact_decoding_flags);

  if (dump_headers) {
    /* TODO
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_SPS_HEADERS, 1);
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_VPS_HEADERS, 1);
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_PPS_HEADERS, 1);
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_SLICE_HEADERS, 1);
    */
  }

  de265_set_CPU_capabilities(ctx, no_acceleration ? 0 : de265_cpu_capability_all);

  if (nThreads==0) {
    nThreads = 1;
  }

  if (nParallelFrames <= 0) {
    nParallelFrames = 1;
  }

  de265_set_max_decode_frames_parallel(ctx, nParallelFrames);
  err = de265_start_worker_threads(ctx, nThreads);

  de265_set_limit_TID(ctx, highestTID);
  de265_set_framerate_ratio(ctx, decode_rate_percent);
  de265_set_max_reorder_buffer_latency(ctx, max_latency);

  if (measure_quality) {
    reference_file = fopen(reference_filename, "rb");
  }


  FILE* fh = fopen(argv[optind], "rb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s (%s)\n", argv[optind], strerror(errno));
    exit(10);
  }

  //fseek(fh, 15500000,SEEK_CUR);


  FILE* bytestream_fh = NULL;

  if (write_bytestream) {
    bytestream_fh = fopen(bytestream_filename, "wb");
  }

  bool stop=false;

  struct timeval tv_start;
  gettimeofday(&tv_start, NULL);

#ifdef WITH_FPS
  fps_estimator fps_estim;
  fps_estim.set_fps_estimator_timespan(5.0f);

  frame_drop_ratio_calculator drop_ratio;
  drop_ratio.set_target_fps(50.0);
#endif

  uint32_t framecnt=0;

  int pos=0;

  while (!stop)
    {
      //tid = (framecnt/1000) & 1;
      //de265_set_limit_TID(ctx, tid);


      int actions = de265_get_action(ctx, true);


      loginfo(LogThreading,"actions: push-input:%d get-image:%d eos:%d\n",
              !!(actions & de265_action_push_more_input),
              !!(actions & de265_action_get_image),
              !!(actions & de265_action_end_of_stream));


      // --- push more input ---

      if (actions & de265_action_push_more_input) {

        if (nal_input) {
          uint8_t len[4];
          int n = fread(len,1,4,fh);
          int length = (len[0]<<24) + (len[1]<<16) + (len[2]<<8) + len[3];

          uint8_t* buf = (uint8_t*)malloc(length);
          n = fread(buf,1,length,fh);
          err = de265_push_NAL(ctx, buf,n,  pos, (void*)1);

          if (write_bytestream) {
            uint8_t sc[3] = { 0,0,1 };
            fwrite(sc ,1,3,bytestream_fh);
            fwrite(buf,1,n,bytestream_fh);
          }

          free(buf);
          pos+=n;
        }
        else {
          // read a chunk of input data
          uint8_t buf[BUFFER_SIZE];
          int n = fread(buf,1,BUFFER_SIZE,fh);

          // decode input data
          if (n) {
            err = de265_push_data(ctx, buf, n, pos, (void*)2);
            if (err != DE265_OK) {
              break;
            }
          }

          pos+=n;

          static int skippos=0;
          if (false && pos>skippos+1000000) { // fake skipping
            de265_reset(ctx);

            int skip = 4000000;

            pos += skip;
            skippos=pos;

            fseek(fh, skip,SEEK_CUR);
          }

          // printf("pending data: %d\n", de265_get_number_of_input_bytes_pending(ctx));

          if (feof(fh)) {
            err = de265_push_end_of_stream(ctx); // indicate end of stream
          }
        }
      }

      // --- get images ---

      if (actions & de265_action_get_image) {

        // show available images

        const de265_image* img = de265_get_next_picture(ctx);
        if (img) {

          if (measure_quality) {
            measure(img, framecnt);
          }

#if WITH_FPS
          fps_estim.on_frame_decoded( de265_get_time() );
#endif

          framecnt++;
          if ((framecnt%100)==0) {
            fprintf(stderr,"frame %d  fps=%.2f\r",framecnt,
#if WITH_FPS
                    fps_estim.fps_measurement_available() ? fps_estim.get_fps_measurement() : 0.0
#else
                    0.0
#endif
                    );

#if WITH_FPS
            if (false && fps_estim.fps_measurement_available()) {
              printf("\n");
              float ratio = drop_ratio.update_decoding_ratio(fps_estim.get_fps_measurement());
              printf("ratio = %f\n",ratio);

              //drop_ratio.set_decoding_ratio(ratio);
              de265_set_framerate_ratio(ctx, ratio*100);

              fps_estim.reset_fps_estimator();
            }
#endif
          }

          stop = output_image(img);

          if (framecnt>=max_frames) {
            stop=true;
          }


          de265_release_picture(img);
        }

        // show warnings

        for (;;) {
          de265_error warning = de265_get_warning(ctx);
          if (warning==DE265_OK) {
            break;
          }

          if (quiet<=1) fprintf(stderr,"WARNING: %s\n", de265_get_error_text(warning));
        }
      }

      else if (actions & de265_action_end_of_stream) {
        stop = true;
      }
    }

  fclose(fh);

  if (write_bytestream) {
    fclose(bytestream_fh);
  }

  if (measure_quality) {
    printf("#total  %6f %6f %6f %6f\n",
           PSNR(mse_y /mse_frames),
           PSNR(mse_cb/mse_frames),
           PSNR(mse_cr/mse_frames),
           ssim_y/ssim_frames);

    fclose(reference_file);
  }

  de265_free_decoder(ctx);

  struct timeval tv_end;
  gettimeofday(&tv_end, NULL);

  if (err != DE265_OK) {
    if (quiet<=1) fprintf(stderr,"decoding error: %s (code=%d)\n", de265_get_error_text(err), err);
  }

  double secs = tv_end.tv_sec-tv_start.tv_sec;
  secs += (tv_end.tv_usec - tv_start.tv_usec)*0.001*0.001;

  if (quiet<=1) fprintf(stderr,"nFrames decoded: %d (%dx%d @ %5.2f fps)\n",framecnt,
                        width,height,framecnt/secs);


  return err==DE265_OK ? 0 : 10;
}
