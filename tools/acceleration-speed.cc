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
#include <stack>

#include "libde265/image.h"
#include "libde265/fallback-dct.h"
#include "libde265/x86/sse-dct.h"
#include "libde265/image-io.h"


bool show_help=false;
bool do_check=false;
bool do_time=false;
bool do_eval=false;
int  img_width=352;
int  img_height=288;
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
  DSPFunc() { next = first; first = this; }
  virtual ~DSPFunc() { }

  virtual const char* name() const = 0;

  virtual int getBlkWidth() const = 0;
  virtual int getBlkHeight() const = 0;

  virtual void runOnBlock(int x,int y) = 0;
  virtual DSPFunc* referenceImplementation() const { return NULL; }

  virtual bool prepareNextImage(const de265_image*) = 0;

  void runOnImage(const de265_image* img);
  bool compareToReferenceImplementation() { return false; }

  static DSPFunc* first;
  DSPFunc* next;
};


DSPFunc* DSPFunc::first = NULL;


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



class DSPFunc_FDCT_Base : public DSPFunc
{
public:
  DSPFunc_FDCT_Base(int size) { prev_image=NULL; curr_image=NULL; residuals=NULL; blkSize=size; }

  virtual const char* name() const { return "FDCT-Base"; }

  virtual int getBlkWidth() const { return blkSize; }
  virtual int getBlkHeight() const { return blkSize; }

  virtual void runOnBlock(int x,int y) {
    bool D = false;

    fdct_8x8_8_fallback(coeffs, residuals+x+y*stride, stride);

    if (D) { dump(x,y); }
  }

  void dump(int x,int y) {
    printf("-> ");
    for (int yy=0;yy<blkSize;yy++)
      for (int xx=0;xx<blkSize;xx++)
        printf("%d ", *(residuals+x+xx+(y+yy)*stride));
    printf("\n");

    printf("   ");
    for (int x=0;x<blkSize*blkSize;x++)
      printf("%d ", coeffs[x]);
    printf("\n");

    int32_t out[32*32];
    transform_idct_8x8_fallback(out, coeffs, 20-8, 15);

    printf("   ");
    for (int x=0;x<blkSize*blkSize;x++)
      printf("%d ", out[x]);
    printf("\n");
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
      }

    return true;
  }

private:
  const de265_image* prev_image;
  const de265_image* curr_image;

protected:
  int blkSize;

  int16_t* residuals;
  int      stride;
  int16_t  coeffs[32*32];
};


class DSPFunc_FDCT_Scalar_4x4 : public DSPFunc_FDCT_Base
{
public:
  DSPFunc_FDCT_Scalar_4x4() : DSPFunc_FDCT_Base(8) { }

  virtual const char* name() const { return "FDCT-Scalar-4x4"; }

  virtual void runOnBlock(int x,int y) {
    bool D = false;

    fdct_4x4_8_fallback(coeffs, residuals+(x+y*stride)*16, stride);

    if (D) { dump(x,y); }
  }
};


class DSPFunc_FDCT_Scalar_8x8 : public DSPFunc_FDCT_Base
{
public:
  DSPFunc_FDCT_Scalar_8x8() : DSPFunc_FDCT_Base(8) { }

  virtual const char* name() const { return "FDCT-Scalar-8x8"; }

  virtual void runOnBlock(int x,int y) {
    bool D = false;

    fdct_8x8_8_fallback(coeffs, residuals+(x+y*stride)*16, stride);

    if (D) { dump(x,y); }
  }
};


class DSPFunc_FDCT_Scalar_16x16 : public DSPFunc_FDCT_Base
{
public:
  DSPFunc_FDCT_Scalar_16x16() : DSPFunc_FDCT_Base(16) { }

  virtual const char* name() const { return "FDCT-Scalar-16x16"; }

  virtual void runOnBlock(int x,int y) {
    bool D = false;

    fdct_16x16_8_fallback(coeffs, residuals+x+y*stride, stride);

    if (D) { dump(x,y); }
  }
};

class DSPFunc_FDCT_Scalar_32x32 : public DSPFunc_FDCT_Base
{
public:
  DSPFunc_FDCT_Scalar_32x32() : DSPFunc_FDCT_Base(32) { }

  virtual const char* name() const { return "FDCT-Scalar-32x32"; }

  virtual void runOnBlock(int x,int y) {
    bool D = false;

    fdct_32x32_8_fallback(coeffs, residuals+x+y*stride, stride);

    if (D) { dump(x,y); }
  }
};




class DSPFunc_IDCT_Base : public DSPFunc
{
public:
  DSPFunc_IDCT_Base(int size) {
    prev_image=NULL; curr_image=NULL; coeffs=NULL; blkSize=size;
    out = new uint8_t[size*size]; // allocate it to ensure alignment
  }

  virtual const char* name() const { return "IDCT-Base"; }

  virtual int getBlkWidth() const { return blkSize; }
  virtual int getBlkHeight() const { return blkSize; }

  virtual void runOnBlock(int x,int y) {
    // note: x+y*width does not make any sense, but gives us some random data
    //transform_idct_8x8_fallback(out, coeffs + x+y*width);
  }

  virtual DSPFunc* referenceImplementation() const { return NULL; }

  virtual bool prepareNextImage(const de265_image* img)
  {
    // --- generate fake coefficients ---
    // difference between two frames

    if (curr_image==NULL) {
      curr_image = img;
      return false;
    }

    delete prev_image;
    prev_image = curr_image;
    curr_image = img;

    int w = curr_image->get_width(0);
    int h = curr_image->get_height(0);

    int align = blkSize;
    width = (w+align-1) / align * align;

    blksPerRow = w/blkSize;

    if (coeffs==NULL) {
      coeffs = new int16_t[width*h];
    }

    int cstride = curr_image->get_luma_stride();
    int pstride = prev_image->get_luma_stride();
    const uint8_t* curr = curr_image->get_image_plane_at_pos(0,0,0);
    const uint8_t* prev = prev_image->get_image_plane_at_pos(0,0,0);

    for (int y=0;y<h;y++)
      for (int x=0;x<w;x++) {
        coeffs[y*w+x] = curr[y*cstride+x] - prev[y*pstride+x];
      }

    return true;
  }

  inline int16_t* xy2coeff(int x,int y) const {
    //int xb = x/blkSize;
    //int yb = y/blkSize;
    int offset = (x+y*blksPerRow)*blkSize; //(xb+yb*blksPerRow)*blkSize*blkSize;
    //printf("%d %d -> %d %p\n", x,y, offset, coeffs+offset);
    return coeffs + offset;
  }

private:
  const de265_image* prev_image;
  const de265_image* curr_image;

protected:
  int blkSize;

  int16_t* coeffs;
  int      width;
  int      blksPerRow;
  uint8_t* out; // [32*32];
};


class DSPFunc_IDCT_Scalar_4x4 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_Scalar_4x4() : DSPFunc_IDCT_Base(4) { }

  virtual const char* name() const { return "IDCT-Scalar-4x4"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,4*4);
    transform_4x4_add_8_fallback(out, xy2coeff(x,y), 4);
  }
};

class DSPFunc_IDCT_Scalar_8x8 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_Scalar_8x8() : DSPFunc_IDCT_Base(8) { }

  virtual const char* name() const { return "IDCT-Scalar-8x8"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,8*8);
    transform_8x8_add_8_fallback(out, xy2coeff(x,y), 8);
  }
};

class DSPFunc_IDCT_Scalar_16x16 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_Scalar_16x16() : DSPFunc_IDCT_Base(16) { }

  virtual const char* name() const { return "IDCT-Scalar-16x16"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,16*16);
    transform_16x16_add_8_fallback(out, xy2coeff(x,y), 16);
  }
};

class DSPFunc_IDCT_Scalar_32x32 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_Scalar_32x32() : DSPFunc_IDCT_Base(32) { }

  virtual const char* name() const { return "IDCT-Scalar-32x32"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,32*32);
    transform_32x32_add_8_fallback(out, xy2coeff(x,y), 32);
  }
};


class DSPFunc_IDCT_SSE_4x4 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_SSE_4x4() : DSPFunc_IDCT_Base(4) { }

  virtual const char* name() const { return "IDCT-SSE-4x4"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,4*4);
    ff_hevc_transform_4x4_add_8_sse4(out, xy2coeff(x,y), 4);
  }
};

class DSPFunc_IDCT_SSE_8x8 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_SSE_8x8() : DSPFunc_IDCT_Base(8) { }

  virtual const char* name() const { return "IDCT-SSE-8x8"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,8*8);
    ff_hevc_transform_8x8_add_8_sse4(out, xy2coeff(x,y), 8);
  }
};

class DSPFunc_IDCT_SSE_16x16 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_SSE_16x16() : DSPFunc_IDCT_Base(16) { }

  virtual const char* name() const { return "IDCT-SSE-16x16"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,16*16);
    ff_hevc_transform_16x16_add_8_sse4(out, xy2coeff(x,y), 16);
  }
};

class DSPFunc_IDCT_SSE_32x32 : public DSPFunc_IDCT_Base
{
public:
  DSPFunc_IDCT_SSE_32x32() : DSPFunc_IDCT_Base(32) { }

  virtual const char* name() const { return "IDCT-SSE-32x32"; }

  virtual void runOnBlock(int x,int y) {
    memset(out,0,32*32);
    ff_hevc_transform_32x32_add_8_sse4(out, xy2coeff(x,y), 32);
  }
};


DSPFunc_FDCT_Scalar_4x4   fdct_scalar_4x4;
DSPFunc_FDCT_Scalar_8x8   fdct_scalar_8x8;
DSPFunc_FDCT_Scalar_16x16 fdct_scalar_16x16;
DSPFunc_FDCT_Scalar_32x32 fdct_scalar_32x32;

DSPFunc_IDCT_Scalar_4x4   idct_scalar_4x4;
DSPFunc_IDCT_Scalar_8x8   idct_scalar_8x8;
DSPFunc_IDCT_Scalar_16x16 idct_scalar_16x16;
DSPFunc_IDCT_Scalar_32x32 idct_scalar_32x32;

DSPFunc_IDCT_SSE_4x4   idct_sse_4x4;
DSPFunc_IDCT_SSE_8x8   idct_sse_8x8;
DSPFunc_IDCT_SSE_16x16 idct_sse_16x16;
DSPFunc_IDCT_SSE_32x32 idct_sse_32x32;


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


  if (show_help) {
    fprintf(stderr,
            "acceleration-speed  SIMD DSP function testing tool\n"
            "--------------------------------------------------\n"
            "      --help      show help\n"
            "  -i, --input     input YUV file\n"
            "  -w, --width     input width (default: 352)\n"
            "  -h, --height    input height (default: 288)\n"
            "  -n, --nframes   number of frames to process (defualt: 1000)\n"
            "  -f, --function  which function to test (see below)\n"
            "  -r, --repeat    number of repetitions for each image (default: 10)\n"
            "\n"
            "these functions are known:\n"
            );

    std::stack<const char*> funcnames;

    for (DSPFunc* func = DSPFunc::first;
         func ;
         func=func->next) {
      funcnames.push(func->name());
    }

    while (!funcnames.empty()) {
      fprintf(stderr,
              "  %s\n", funcnames.top());
      funcnames.pop();
    }

    return 0;
  }


  // --- find DSP function with the given name ---

  if (function.empty()) {
    fprintf(stderr,"No function specified. Use option '--function'.\n");
    exit(10);
  }

  DSPFunc* algo = NULL;
  for (DSPFunc* f = DSPFunc::first; f ; f=f->next) {
    if (strcasecmp(f->name(), function.c_str())==0) {
      algo = f;
      break;
    }
  }

  if (algo==NULL) {
    fprintf(stderr,"Argument to '--function' invalid. No function with that name.\n");
    exit(10);
  }



  ImageSource_YUV image_source;
  image_source.set_input_file(input_file.c_str(), img_width, img_height);

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
