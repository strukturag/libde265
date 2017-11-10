/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of libde265.
 *
 * libde265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * libde265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libde265.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "image-io-png.h"
#include <assert.h>

#if HAVE_VIDEOGFX
#include <libvideogfx.hh>
using namespace videogfx;


ImageSource_PNG::ImageSource_PNG()
{
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

image* ImageSource_PNG::get_image(bool block)
{
  if (mReachedEndOfStream) return NULL;


  // --- construct image filename ---

  char filename[1000];
  sprintf(filename,mFilenameTemplate.c_str(),mNextImageNumber);
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

  image::supplementary_data supp;

  if (mTargetColorspace == de265_colorspace_GBR) {
    supp.colorspace = de265_colorspace_GBR;

    image* img = new image;
    img->alloc_image(mWidth,mHeight,de265_chroma_444, 8,8,
                     0, // PTS
                     supp,
                     NULL);
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
  else {
    supp.colorspace = de265_colorspace_YCbCr;

    image* img = new image;
    img->alloc_image(mWidth,mHeight,de265_chroma_420, 8,8,
                     0, // PTS
                     supp,
                     NULL);
    assert(img); // TODO: error handling


    uint8_t* dst_y    = img->get_image_plane(0);
    int dst_y_stride  = img->get_image_stride(0);
    uint8_t* dst_cb   = img->get_image_plane(1);
    int dst_cb_stride = img->get_image_stride(1);
    uint8_t* dst_cr   = img->get_image_plane(2);
    int dst_cr_stride = img->get_image_stride(2);

    const Pixel*const* src_r = input.AskFrameR();
    const Pixel*const* src_g = input.AskFrameG();
    const Pixel*const* src_b = input.AskFrameB();

    for (int y=0;y<mHeight;y++) {
      for (int x=0;x<mWidth;x++) {
        int col_y  = 16  +  65.738*src_r[y][x]/256 + 129.057*src_g[y][x]/256 +  25.064*src_b[y][x]/256;

        dst_y[y*dst_y_stride + x] = Clip1_8bit(col_y);

        if ((x&1)==0 && (y&1)==0) {
          int col_cb = 128 -  37.945*src_r[y][x]/256 - 74.494*src_g[y][x]/256 + 112.439*src_b[y][x]/256;
          int col_cr = 128 + 112.439*src_r[y][x]/256 - 94.154*src_g[y][x]/256 -  18.285*src_b[y][x]/256;

          dst_cb[y/2 * dst_cb_stride + x/2] = Clip1_8bit(col_cb);
          dst_cr[y/2 * dst_cr_stride + x/2] = Clip1_8bit(col_cr);
        }
      }
    }

    return img;
  }
}

void ImageSource_PNG::skip_frames(int n)
{
  mNextImageNumber += n;
}

#endif
