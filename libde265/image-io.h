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

#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include "libde265/image.h"
#include <deque>


class ImageSource
{
 public:
  ImageSource() { }
  virtual ~ImageSource() { }

  enum ImageStatus { Available, Waiting, EndOfVideo };

  virtual ImageStatus  get_status(int offset=0) = 0;
  virtual de265_image* get_image(int offset=0, bool block=true) = 0;
  virtual void release_next_image(int n=1) = 0;

  int get_width();
  int get_height();
};



class ImageSource_YUV : public ImageSource
{
 public:
 ImageSource_YUV() : mFH(NULL) { }
  virtual ~ImageSource_YUV();

  bool set_input_file(const char* filename, int w,int h);

  virtual ImageStatus  get_status(int offset=0);
  virtual de265_image* get_image(int offset=0, bool block=true);
  virtual void release_next_image(int n=1);

 private:
  FILE* mFH;
  int width,height;

  std::deque<de265_image*> mQueue;

  //int  mNextFrame;
  bool mReachedEndOfFile;

  void preload_next_image();
};

#endif
