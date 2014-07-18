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

#include "libde265/image-io.h"
#include <assert.h>


int ImageSource::get_width()
{
  de265_image* img = get_image();
  if (img==NULL) return 0;
  else return img->get_width();
}

int ImageSource::get_height()
{
  de265_image* img = get_image();
  if (img==NULL) return 0;
  else return img->get_height();
}


ImageSource_YUV::~ImageSource_YUV()
{
  while (!mQueue.empty()) {
    release_next_image();
  }

  if (mFH) {
    fclose(mFH);
  }
}


bool ImageSource_YUV::set_input_file(const char* filename, int w,int h)
{
  assert(mFH==NULL);

  mFH = fopen(filename,"rb");
  if (mFH==NULL) {
    return false;
  }

  width =w;
  height=h;
  //mNextFrame = 0;
  mReachedEndOfFile = false;

  return true;
}


void ImageSource_YUV::preload_next_image()
{
  if (mReachedEndOfFile) return;

  de265_image* img = new de265_image;
  img->alloc_image(width,height,de265_chroma_420, NULL, false, NULL);
  assert(img); // TODO: error handling

  // --- load image ---

  uint8_t* p;
  int stride;

  p = img->get_image_plane(0);  stride = img->get_image_stride(0);
  for (int y=0;y<height;y++) {
    fread(p+y*stride,1,width,mFH);
  }

  p = img->get_image_plane(1);  stride = img->get_image_stride(1);
  for (int y=0;y<height/2;y++) {
    fread(p+y*stride,1,width/2,mFH);
  }

  p = img->get_image_plane(2);  stride = img->get_image_stride(2);
  for (int y=0;y<height/2;y++) {
    fread(p+y*stride,1,width/2,mFH);
  }

  // --- check for EOF ---

  if (feof(mFH)) {
    delete img;
    mReachedEndOfFile = true;
  }

  // --- put image into queue ---

  mQueue.push_back(img);
}


ImageSource::ImageStatus  ImageSource_YUV::get_status(int offset)
{
  while (offset >= mQueue.size()) {
    preload_next_image();
    if (mReachedEndOfFile) {
      return EndOfVideo;
    }
  }

  return Available;
}


de265_image* ImageSource_YUV::get_image(int offset, bool block)
{
  if (get_status(offset)==Available) {
    return mQueue[offset];
  }
  else {
    return NULL;
  }
}


void ImageSource_YUV::release_next_image(int n)
{
  while (n>0) {
    
    if (mQueue.empty()) {
      break;
    }
    else {
      delete mQueue[0];
      mQueue.pop_front();
    }

    n--;
  }

  if (n>0) {
    if (mReachedEndOfFile) { return; }

    fseek(mFH, width*height*3/2*n, SEEK_CUR);
  }
}



ImageSink_YUV::~ImageSink_YUV()
{
  if (mFH) {
    fclose(mFH);
  }
}

bool ImageSink_YUV::set_filename(const char* filename)
{
  assert(mFH==NULL);

  mFH = fopen(filename,"wb");
}

void ImageSink_YUV::send_image(const de265_image* img)
{
  // --- write image ---

  const uint8_t* p;
  int stride;

  int width = img->get_width();
  int height= img->get_height();

  p = img->get_image_plane(0);  stride = img->get_image_stride(0);
  for (int y=0;y<height;y++) {
    fwrite(p+y*stride,1,width,mFH);
  }

  p = img->get_image_plane(1);  stride = img->get_image_stride(1);
  for (int y=0;y<height/2;y++) {
    fwrite(p+y*stride,1,width/2,mFH);
  }

  p = img->get_image_plane(2);  stride = img->get_image_stride(2);
  for (int y=0;y<height/2;y++) {
    fwrite(p+y*stride,1,width/2,mFH);
  }
}



PacketSink_File::~PacketSink_File()
{
  if (mFH) {
    fclose(mFH);
  }
}


void PacketSink_File::set_filename(const char* filename)
{
  assert(mFH==NULL);

  mFH = fopen(filename,"wb");
}


void PacketSink_File::send_packet(uint8_t* data, int n)
{
  uint8_t startCode[3];
  startCode[0] = 0;
  startCode[1] = 0;
  startCode[2] = 1;

  fwrite(startCode,1,3,mFH);
  fwrite(data,1,n,mFH);
}

