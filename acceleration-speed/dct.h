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

#ifndef ACCELERATION_SPEED_DCT_H
#define ACCELERATION_SPEED_DCT_H

#include "acceleration-speed.h"
#include "libde265/fallback-dct.h"


class DSPFunc_FDCT_Base : public DSPFunc
{
public:
  DSPFunc_FDCT_Base(int size) {
    residuals=NULL;
    blkSize=size;
    coeffs = new int16_t[size*size];
  }

  virtual const char* name() const { return "FDCT-Base"; }

  virtual int getBlkWidth() const { return blkSize; }
  virtual int getBlkHeight() const { return blkSize; }

  virtual void runOnBlock(int x,int y) = 0;

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

  bool compareToReferenceImplementation()
  {
    DSPFunc_FDCT_Base* refImpl = dynamic_cast<DSPFunc_FDCT_Base*>(referenceImplementation());

    for (int i=0;i<blkSize*blkSize;i++)
      if (coeffs[i] != refImpl->coeffs[i])
        return false;

    return true;
  }

  virtual bool prepareNextImage(std::shared_ptr<const de265_image> img)
  {
    if (!curr_image) {
      curr_image = img;
      return false;
    }

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
  std::shared_ptr<const de265_image> prev_image;
  std::shared_ptr<const de265_image> curr_image;

protected:
  int blkSize;

  int16_t* residuals;
  int      stride;
  int16_t* coeffs;
};





class DSPFunc_IDCT_Base : public DSPFunc
{
public:
  DSPFunc_IDCT_Base(int size) {
    prev_image=NULL; curr_image=NULL; coeffs=NULL; blkSize=size;
    out = new uint8_t[size*size]; // allocate it to ensure alignment
  }

  virtual const char* name() const { return "IDCT-Base"; }

  virtual int getBlkWidth()  const { return blkSize; }
  virtual int getBlkHeight() const { return blkSize; }

  virtual void runOnBlock(int x,int y) = 0;

  virtual DSPFunc* referenceImplementation() const { return NULL; }

  virtual bool compareToReferenceImplementation()
  {
    DSPFunc_IDCT_Base* refImpl = dynamic_cast<DSPFunc_IDCT_Base*>(referenceImplementation());

    for (int i=0;i<blkSize*blkSize;i++)
      if (out[i] != refImpl->out[i])
        return false;

    return true;
  }

  virtual bool prepareNextImage(std::shared_ptr<const de265_image> img)
  {
    // --- generate fake coefficients ---
    // difference between two frames

    if (!curr_image) {
      curr_image = img;
      return false;
    }

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
    // note: x+y*width does not make any sense, but gives us some random data

    //int xb = x/blkSize;
    //int yb = y/blkSize;
    int offset = (x+y*blksPerRow)*blkSize; //(xb+yb*blksPerRow)*blkSize*blkSize;
    //printf("%d %d -> %d %p\n", x,y, offset, coeffs+offset);
    return coeffs + offset;
  }

private:
  std::shared_ptr<const de265_image> prev_image;
  std::shared_ptr<const de265_image> curr_image;

protected:
  int blkSize;

  int16_t* coeffs;
  int      width;
  int      blksPerRow;
  uint8_t* out; // [32*32];
};


#endif
