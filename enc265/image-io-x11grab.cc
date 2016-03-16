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

#include "image-io-x11grab.h"
#include <assert.h>


#include <sys/shm.h>
#include <X11/Xutil.h>


ImageSource_X11Grab::ImageSource_X11Grab()
{
  mX=mY=0;
  mWidth=mHeight=0;

  mInitialized=false;
  mUseShm=false;
  mDisplay=NULL;
  mImage=NULL;
}

ImageSource_X11Grab::~ImageSource_X11Grab()
{
  if (mImage) {
    XDestroyImage(mImage);
  }

  if (mUseShm) {
    XShmDetach(mDisplay, &mShminfo);
    shmdt(mShminfo.shmaddr);
    shmctl(mShminfo.shmid, IPC_RMID, NULL);
  }

  XCloseDisplay(mDisplay);
}


bool ImageSource_X11Grab::init()
{
  mDisplay = XOpenDisplay(NULL); // use $DISPLAY
  if (!mDisplay) {
    fprintf(stderr,"cannot open X11 display\n");
    return false;
  }

  int screen = DefaultScreen(mDisplay);

  mUseShm = XShmQueryExtension(mDisplay);
  if (!mUseShm) {
    fprintf(stderr,"cannot use XShm extension\n");
    return false;
  }

  mImage = XShmCreateImage(mDisplay,
                           DefaultVisual(mDisplay, screen),
                           DefaultDepth(mDisplay, screen),
                           ZPixmap,
                           NULL,
                           &mShminfo,
                           mWidth, mHeight);
  mShminfo.shmid = shmget(IPC_PRIVATE,
                          mImage->bytes_per_line * mImage->height,
                          IPC_CREAT|0777);
  if (mShminfo.shmid == -1) {
    fprintf(stderr,"cannot allocate shared memory\n");
    return false;
  }

  mShminfo.shmaddr = (char*)shmat(mShminfo.shmid, 0, 0);
  mImage->data = mShminfo.shmaddr;
  mShminfo.readOnly = False;

  if (!XShmAttach(mDisplay, &mShminfo)) {
    fprintf(stderr,"cannot attach shared memory\n");
    return false;
  }

  mUseShm=true;
  mInitialized=true;

  printf("color masks:\n"
         "R: %04lx\n"
         "G: %04lx\n"
         "B: %04lx\n"
         "BPP: %d\n\n",
         mImage->red_mask,
         mImage->green_mask,
         mImage->blue_mask,
         mImage->bits_per_pixel);

  return true;
}


de265_image* ImageSource_X11Grab::get_image(bool block)
{
  if (!mInitialized) { init(); }

  int screen = DefaultScreen(mDisplay);
  int root  = RootWindow(mDisplay, screen);

  if (!XShmGetImage(mDisplay, root, mImage, mX,mY, AllPlanes)) {
    fprintf(stderr,"XShmGetImage failed\n");
    return NULL;
  }


  de265_image* img = new de265_image;
  img->alloc_image(mWidth,mHeight,de265_chroma_444, NULL, false,
                   NULL, NULL, 0, NULL, false);
  assert(img); // TODO: error handling


  uint8_t* p[3];
  int stride[3];

  for (int c=0;c<3;c++) {
    p[c] = img->get_image_plane(c);
    stride[c] = img->get_image_stride(c);
  }

  for (int y=0;y<mHeight;y++) {
    for (int x=0;x<mWidth;x++) {
      p[2][x+y*stride[2]] = mImage->data[y*mImage->bytes_per_line+4*x+2]; // R
      p[0][x+y*stride[0]] = mImage->data[y*mImage->bytes_per_line+4*x+1]; // G
      p[1][x+y*stride[1]] = mImage->data[y*mImage->bytes_per_line+4*x+0]; // B
    }
  }

  return img;
}
