/*
  libde265 example application "enc265".

  MIT License

  Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>

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

#include "image-io-png.h"
#include <assert.h>

#if HAVE_VIDEOGFX
#include <libvideogfx.hh>
using namespace videogfx;


ImageSource_PNG::ImageSource_PNG()
{
  mFilenameTemplate = NULL;
  mNextImageNumber = 1;

  mReachedEndOfStream = false;

  mWidth=mHeight=0;
}

ImageSource_PNG::~ImageSource_PNG()
{
}

bool ImageSource_PNG::set_input_file(const char* filename)
{
  mFilenameTemplate = filename;
  return true;
}

de265_image* ImageSource_PNG::get_image(bool block)
{
  if (mReachedEndOfStream) return NULL;


  // --- construct image filename ---

  char filename[1000];
  sprintf(filename,mFilenameTemplate,mNextImageNumber);
  mNextImageNumber++;


  // --- load image ---

  Image<Pixel> input;
  bool success = videogfx::ReadImage_PNG(input, filename);
  if (!success) {
    mReachedEndOfStream = true;
    return NULL;
  }


  mWidth = input.AskWidth();
  mHeight= input.AskHeight();

  de265_image* img = new de265_image;
  img->alloc_image(mWidth,mHeight,de265_chroma_444, NULL, false,
                   NULL, /*NULL,*/ 0, NULL, false);
  assert(img); // TODO: error handling


  uint8_t* p;
  int stride;

  for (int c=0;c<3;c++) {
    int h265channel;
    switch (c) {
    case 0: h265channel=2; break; // R
    case 1: h265channel=0; break; // G
    case 2: h265channel=1; break; // B
    }

    p = img->get_image_plane(h265channel);
    stride = img->get_image_stride(h265channel);

    for (int y=0;y<mHeight;y++) {
      memcpy(p, input.AskFrame((BitmapChannel(c)))[y], mWidth);
      p += stride;
    }
  }

  return img;
}

void ImageSource_PNG::skip_frames(int n)
{
  mNextImageNumber += n;
}

#endif
