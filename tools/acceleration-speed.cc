/*
 * H.265 video codec.
 * Copyright (c) 2015 struktur AG, Dirk Farin <farin@struktur.de>
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>

#include <string>

#include "libde265/image.h"
#include "libde265/fallback-dct.h"
#include "libde265/image-io.h"


bool show_help=false;
bool do_check=false;
bool do_time=false;
bool do_eval=false;
int  img_width=0;
int  img_height=0;
int  nframes=1000;
int  repeat=10;
std::string function;
std::string input_file;

static struct option long_options[] = {
  {"help",    no_argument,       0, 'H' },
  {"input",   required_argument, 0, 'i' },
  {"width",   required_argument, 0, 'w' },
  {"height",  required_argument, 0, 'h' },
  {"nframes", required_argument, 0, 'n' },
  {"function",required_argument, 0, 'f' },
  {"check",   no_argument,       0, 'c' },
  {"time",    no_argument,       0, 't' },
  {"eval",    no_argument,       0, 'e' },
  {"repeat",  required_argument, 0, 'r' },
  {0,            0,              0,  0  }
};



class DSPFunc
{
public:
  virtual ~DSPFunc() { }

  virtual const char* name() const = 0;

  virtual int getBlkWidth() const = 0;
  virtual int getBlkHeight() const = 0;

  virtual void runOnBlock(int x,int y) = 0;
  virtual DSPFunc* referenceImplementation() const { return NULL; }

  virtual bool prepareNextImage(const de265_image*) = 0;

  void runOnImage(const de265_image* img);
  bool compareToReferenceImplementation() { return false; }
};


void DSPFunc::runOnImage(const de265_image* img)
{
  int w = img->get_width(0);
  int h = img->get_height(0);

  int blkWidth  = getBlkWidth();
  int blkHeight = getBlkHeight();

  for (int y=0;y<=h-blkHeight;y+=blkHeight)
    for (int x=0;x<=w-blkWidth;x+=blkWidth) {
      runOnBlock(x,y);
    }
}



class DSPFunc_FDCT8x8 : public DSPFunc
{
public:
  DSPFunc_FDCT8x8() { prev_image=NULL; curr_image=NULL; residuals=NULL; }

  virtual const char* name() const { return "FDCT"; }

  virtual int getBlkWidth() const { return 8; }
  virtual int getBlkHeight() const { return 8; }

  virtual void runOnBlock(int x,int y) {
    bool D = false;

    if (D) {
      printf("-> ");
      for (int yy=0;yy<8;yy++)
        for (int xx=0;xx<8;xx++)
          printf("%d ", *(residuals+x+xx+(y+yy)*stride));
      printf("\n");
    }

    fdct_8x8_8_fallback(coeffs, residuals+x+y*stride, stride);

    if (D) {
      printf("   ");
      for (int x=0;x<8*8;x++)
        printf("%d ", coeffs[x]);
      printf("\n");

      int32_t out[8*8];
      transform_idct_8x8_fallback(out, coeffs, 20-8, 15);

      printf("   ");
      for (int x=0;x<8*8;x++)
        printf("%d ", out[x]);
      printf("\n");
    }
  }

  virtual DSPFunc* referenceImplementation() const { return NULL; }

  virtual bool prepareNextImage(const de265_image* img)
  {
    if (curr_image==NULL) {
      curr_image = img;
      return false;
    }

    delete prev_image;
    prev_image = curr_image;
    curr_image = img;

    int w = curr_image->get_width(0);
    int h = curr_image->get_height(0);

    if (residuals==NULL) {
      int align=16;
      stride = (w+align-1)/align*align;
      residuals = new int16_t[stride*h];
    }

    int cstride = curr_image->get_luma_stride();
    int pstride = prev_image->get_luma_stride();
    const uint8_t* curr = curr_image->get_image_plane_at_pos(0,0,0);
    const uint8_t* prev = prev_image->get_image_plane_at_pos(0,0,0);

    for (int y=0;y<h;y++)
      for (int x=0;x<w;x++) {
        residuals[y*stride+x] = curr[y*cstride+x] - prev[y*pstride+x];
        residuals[y*stride+x] = curr[y*cstride+x] - prev[y*pstride+x];
      }

    return true;
  }

private:
  const de265_image* prev_image;
  const de265_image* curr_image;

  int16_t* residuals;
  int      stride;
  int16_t  coeffs[8*8];
};


int main(int argc, char** argv)
{
  while (1) {
    int option_index = 0;

    int c = getopt_long(argc, argv, "Hci:w:h:n:f:ter:", long_options, &option_index);
    if (c == -1)
      break;

    switch (c) {
    case 'H': show_help=true; break;
    case 'c': do_check=true; break;
    case 'w': img_width =atoi(optarg); break;
    case 'h': img_height=atoi(optarg); break;
    case 'n': nframes=atoi(optarg); break;
    case 'f': function=optarg; break;
    case 'i': input_file=optarg; break;
    case 't': do_time=true; break;
    case 'e': do_eval=true; break;
    case 'r': repeat=atoi(optarg); break;
    }
  }



  ImageSource_YUV image_source;
  image_source.set_input_file(input_file.c_str(), img_width, img_height);


  DSPFunc_FDCT8x8 fdct_algo;
  DSPFunc* algo = &fdct_algo;
  int img_counter=0;


  bool eof = false;
  for (int f=0; f<nframes ; f++)
    {
      de265_image* image = image_source.get_image();
      if (image==NULL) {
        eof=true;
        break;
      }

      img_counter++;

      if (algo->prepareNextImage(image)) {
        printf("run %d times on image %d\n",repeat,img_counter);
        for (int r=0;r<repeat;r++) {
          algo->runOnImage(image);
          //bool compareToReferenceImplementation(const de265_image*);
        }
      }
    }

  return 0;
}
