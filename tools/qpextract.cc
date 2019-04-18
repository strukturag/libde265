/*
  libde265-based parser.

  MIT License

  Copyright (c) 2019 Facebook, Chema Gonzalez <chemag@gmail.com>

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

#define DO_MEMORY_LOGGING 0

#include "de265.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
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


#define BUFFER_SIZE 40960

bool nal_input=false;
bool check_hash=false;
bool show_help=false;
bool logging=true;
bool no_acceleration=false;
const char *bytestream_filename;
const char* reference_filename;
int highestTID = 100;
int verbosity=0;
int disable_deblocking=0;
int disable_sao=0;

static struct option long_options[] = {
  {"check-hash", no_argument,       0, 'c' },
  {"profile",    no_argument,       0, 'p' },
  {"frames",     required_argument, 0, 'f' },
  {"output",     required_argument, 0, 'o' },
  {"dump",       no_argument,       0, 'd' },
  {"dump-image-data", no_argument,  0, 'I' },
  {"nal",        no_argument,       0, 'n' },
  {"no-logging", no_argument,       0, 'L' },
  {"help",       no_argument,       0, 'h' },
  {"noaccel",    no_argument,       0, '0' },
  {"highest-TID", required_argument, 0, 'T' },
  {"verbose",    no_argument,       0, 'v' },
  {"disable-deblocking", no_argument, &disable_deblocking, 1 },
  {"disable-sao",        no_argument, &disable_sao, 1 },
  {0,         0,                 0,  0 }
};



static int width,height;
static uint32_t framecnt=0;




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


void dump_vps(video_parameter_set* vps)
{
  vps->dump(STDOUT_FILENO);
}

void dump_sps(seq_parameter_set* sps)
{
  sps->dump(STDOUT_FILENO);
}

void dump_pps(pic_parameter_set* pps)
{
  pps->dump(STDOUT_FILENO);
}

void dump_image(de265_image* img)
{
  const seq_parameter_set& sps = img->get_sps();
  int minCbSize = sps.MinCbSizeY;

  // calculate QP distro
  int qp_distro[100];
  for (int q=0;q<100;q++)
    qp_distro[q] = 0;

  for (int y0=0;y0<sps.PicHeightInMinCbsY;y0++)
    for (int x0=0;x0<sps.PicWidthInMinCbsY;x0++)
      {
        int log2CbSize = img->get_log2CbSize_cbUnits(x0,y0);
        if (log2CbSize==0) {
          continue;
        }

        int xb = x0*minCbSize;
        int yb = y0*minCbSize;

        int CbSize = 1<<log2CbSize;
        //printf("----qp_distro value---- x0: %i, y0: %i, xb: %i, yb: %i, log2CbSize: %i, CbSize: %i\n", x0, y0, xb, yb, log2CbSize, CbSize); // XXX

        int q = img->get_QPY(xb,yb);
        if (q < 0 || q >= 100) {
          fprintf(stderr, "error: q: %d\n",q);
        } else {
          //qp_distro[q] += (CbSize*CbSize);
          qp_distro[q] += 1;
        }
      }

  // dump QP distro
  bool first_nonzero = false;
  int lowest_nonzero = 0;
  int highest_nonzero = 0;
#define BUFSIZE 1024
  char buffer[BUFSIZE] = {};
  int bi = 0;
  for (int q=0;q<100;q++)
    {
    if (qp_distro[q] != 0 && !first_nonzero)
      {
      first_nonzero = true;
      lowest_nonzero = q;
      }
    if (qp_distro[q] != 0)
      highest_nonzero = q;
    }
  bi += snprintf(buffer+bi, BUFSIZE-bi, "qp_distro[%i:%i] { ", lowest_nonzero, highest_nonzero);
  for (int q=0;q<100;q++)
    {
    if (q < lowest_nonzero || q > highest_nonzero)
      continue;
    bi += snprintf(buffer+bi, BUFSIZE-bi, "%d ", qp_distro[q]);
    }
  bi += snprintf(buffer+bi, BUFSIZE-bi, "}\n");
  fprintf(stdout, buffer);
}


int main(int argc, char** argv)
{
  while (1) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "qt:chf:o:dILB:n0vT:m:se"
                        , long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
    case 'c': check_hash=true; break;
    case 'h': show_help=true; break;
    case 'n': nal_input=true; break;
    case 'L': logging=false; break;
    case '0': no_acceleration=true; break;
    case 'T': highestTID=atoi(optarg); break;
    case 'v': verbosity++; break;
    }
  }

  if (optind != argc-1 || show_help) {
    fprintf(stderr," qpextract  v%s\n", de265_get_version());
    fprintf(stderr,"--------------\n");
    fprintf(stderr,"usage: qpextract [options] videofile.bin\n");
    fprintf(stderr,"The video file must be a raw bitstream, or a stream with NAL units (option -n).\n");
    fprintf(stderr,"\n");
    fprintf(stderr,"options:\n");
    fprintf(stderr,"  -c, --check-hash  perform hash check\n");
    fprintf(stderr,"  -n, --nal         input is a stream with 4-byte length prefixed NAL units\n");
    fprintf(stderr,"  -d, --dump        dump headers\n");
    fprintf(stderr,"  -T, --highest-TID select highest temporal sublayer to decode\n");
    fprintf(stderr,"      --disable-deblocking   disable deblocking filter\n");
    fprintf(stderr,"      --disable-sao          disable sample-adaptive offset filter\n");
    fprintf(stderr,"  -h, --help        show help\n");

    exit(show_help ? 0 : 5);
  }


  // create and configure decoder
  de265_error err = DE265_OK;
  de265_decoder_context* ctx = de265_new_decoder();
  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, check_hash);
  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_SUPPRESS_FAULTY_PICTURES, false);
  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_DISABLE_DEBLOCKING, disable_deblocking);
  de265_set_parameter_bool(ctx, DE265_DECODER_PARAM_DISABLE_SAO, disable_sao);

  //if (dump_headers) {
  //  de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_SEI, 1);
  //  de265_set_parameter_int(ctx, DE265_DECODER_PARAM_DUMP_SLICE_HEADERS, 1);
  //}

  if (no_acceleration) {
    de265_set_parameter_int(ctx, DE265_DECODER_PARAM_ACCELERATION_CODE, de265_acceleration_SCALAR);
  }

  if (!logging) {
    de265_disable_logging();
  }

  de265_set_verbosity(verbosity);


  de265_set_limit_TID(ctx, highestTID);

  // set callback
  struct de265_callback_block cb;
#if 0
  cb.get_vps = dump_vps;
  cb.get_sps = dump_sps;
  cb.get_pps = dump_pps;
#endif
  cb.get_image = dump_image;
  de265_callback_register(ctx, &cb);

  FILE* fh = fopen(argv[optind], "rb");
  if (fh==NULL) {
    fprintf(stderr,"cannot open file %s!\n", argv[1]);
    exit(10);
  }

  bool stop=false;

  int pos=0;

  while (!stop)
    {
      if (nal_input) {
        uint8_t len[4];
        int n = fread(len,1,4,fh);
        int length = (len[0]<<24) + (len[1]<<16) + (len[2]<<8) + len[3];

        uint8_t* buf = (uint8_t*)malloc(length);
        n = fread(buf,1,length,fh);
        err = de265_push_NAL(ctx, buf,n,  pos, (void*)1);
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
      }

      if (feof(fh)) {
        err = de265_flush_data(ctx); // indicate end of stream
        stop = true;
      }

      // decoding loop
      int more=1;
      while (more)
        {
          more = 0;

          // decode some more
          err = de265_decode(ctx, &more);
          if (err != DE265_OK) {
            if (check_hash && err == DE265_ERROR_CHECKSUM_MISMATCH)
              stop = 1;
            more = 0;
            break;
          }

          // show available images
          const de265_image* img = de265_get_next_picture(ctx);

          // show warnings
          for (;;) {
            de265_error warning = de265_get_warning(ctx);
            if (warning==DE265_OK) {
              break;
            }
          }
        }
    }

  fclose(fh);

  de265_free_decoder(ctx);

  return err==DE265_OK ? 0 : 10;
}
