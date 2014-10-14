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

  //enum ImageStatus { Available, Waiting, EndOfVideo };

  //virtual ImageStatus  get_status() = 0;
  virtual de265_image* get_image(bool block=true) = 0;
  virtual void skip_frames(int n) = 0;

  virtual int get_width() const = 0;
  virtual int get_height() const = 0;
};



class ImageSource_YUV : public ImageSource
{
 public:
 ImageSource_YUV() : mFH(NULL) { }
  virtual ~ImageSource_YUV();

  bool set_input_file(const char* filename, int w,int h);

  //virtual ImageStatus  get_status();
  virtual de265_image* get_image(bool block=true);
  virtual void skip_frames(int n);

  virtual int get_width() const { return width; }
  virtual int get_height() const { return height; }

 private:
  FILE* mFH;
  bool mReachedEndOfFile;

  int width,height;

  de265_image* read_next_image();
};




class ImageSink
{
 public:
  virtual ~ImageSink() { }

  virtual void send_image(const de265_image* img) = 0;
};

class ImageSink_YUV : public ImageSink
{
 public:
 ImageSink_YUV() : mFH(NULL) { }
  ~ImageSink_YUV();

  bool set_filename(const char* filename);

  virtual void send_image(const de265_image* img);

 private:
  FILE* mFH;
};



class PacketSink
{
 public:
  virtual ~PacketSink() { }

  virtual void send_packet(uint8_t* data, int n) = 0;
};


class PacketSink_File : public PacketSink
{
 public:
 PacketSink_File() : mFH(NULL) { }
  virtual ~PacketSink_File();

  void set_filename(const char* filename);

  virtual void send_packet(uint8_t* data, int n);

 private:
  FILE* mFH;
};

#endif
