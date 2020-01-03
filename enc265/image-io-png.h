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

#ifndef IMAGE_IO_PNG_H
#define IMAGE_IO_PNG_H

#include "libde265/image-io.h"
#include <deque>


#if HAVE_VIDEOGFX
class ImageSource_PNG : public ImageSource
{
 public:
  LIBDE265_API ImageSource_PNG();
  virtual LIBDE265_API ~ImageSource_PNG();

  bool LIBDE265_API set_input_file(const char* filename);

  //virtual ImageStatus  get_status();
  virtual LIBDE265_API de265_image* get_image(bool block=true);
  virtual LIBDE265_API void skip_frames(int n);

  virtual LIBDE265_API int get_width() const { return mWidth; }
  virtual LIBDE265_API int get_height() const { return mHeight; }

 private:
  const char* mFilenameTemplate;
  int mNextImageNumber;

  bool mReachedEndOfStream;

  int mWidth,mHeight;
};
#endif

#endif
