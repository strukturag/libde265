/*
 * libde265 example application "dec265".
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * This file is part of dec265, an example application using libde265.
 *
 * dec265 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * dec265 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with dec265.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sdl.hh"


bool SDL_YUV_Display::init(int frame_width, int frame_height)
{
  // reduce image size to a multiple of 8 (apparently required by YUV overlay)

  frame_width  &= ~7;
  frame_height &= ~7;


  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_NOPARACHUTE) < 0 ) {
    printf("SDL_Init() failed: %s\n", SDL_GetError( ) );
    SDL_Quit();
    return false;
  }

  const SDL_VideoInfo* info = SDL_GetVideoInfo();
  if( !info ) {
    printf("SDL_GetVideoInfo() failed: %s\n", SDL_GetError() );
    SDL_Quit();
    return false;
  }

  Uint8 bpp = info->vfmt->BitsPerPixel;

  Uint32 vflags;
  if (info->hw_available)
    vflags = SDL_HWSURFACE;
  else
    vflags = SDL_SWSURFACE;

  // set window title
  const char *window_title = "SDL YUV display";
  SDL_WM_SetCaption(window_title, NULL);

  mScreen = SDL_SetVideoMode(frame_width, frame_height, bpp, vflags);
  if (mScreen == NULL) {
    printf("SDL: Couldn't set video mode to %dx%d,%d bpp: %s",
           frame_width, frame_height, bpp, SDL_GetError());
    SDL_Quit();
    return false;
  }

  mYUVOverlay = SDL_CreateYUVOverlay(frame_width, frame_height, SDL_YV12_OVERLAY, mScreen);
  if (mYUVOverlay == NULL ) {
    printf("SDL: Couldn't create SDL YUV overlay: %s",SDL_GetError());
    SDL_Quit();
    return false;
  }

  rect.x = 0;
  rect.y = 0;
  rect.w = frame_width;
  rect.h = frame_height;

  mWindowOpen=true;

  return true;
}

void SDL_YUV_Display::display(const unsigned char *Y,
                              const unsigned char *U,
                              const unsigned char *V,
                              int stride, int chroma_stride)
{
  if (!mWindowOpen) return;
  if (SDL_LockYUVOverlay(mYUVOverlay) < 0) return;

  if (stride == rect.w && chroma_stride == rect.w/2) {

    // fast copy

    memcpy(mYUVOverlay->pixels[0], Y, rect.w * rect.h);
    memcpy(mYUVOverlay->pixels[1], V, rect.w * rect.h / 4);
    memcpy(mYUVOverlay->pixels[2], U, rect.w * rect.h / 4);
  }
  else {
    // copy line by line, because sizes are different

    for (int y=0;y<rect.h;y++)
      {
        memcpy(mYUVOverlay->pixels[0]+y*rect.w, Y+stride*y, rect.w);
      }

    for (int y=0;y<rect.h/2;y++)
      {
        memcpy(mYUVOverlay->pixels[2]+y*rect.w/2, U+chroma_stride*y, rect.w/2);
        memcpy(mYUVOverlay->pixels[1]+y*rect.w/2, V+chroma_stride*y, rect.w/2);
      }
  }

  SDL_UnlockYUVOverlay(mYUVOverlay);

  SDL_DisplayYUVOverlay(mYUVOverlay, &rect);
}

bool SDL_YUV_Display::doQuit() const
{
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT) {
      return true;
    }
  }

  return false;
}

void SDL_YUV_Display::close()
{
  SDL_FreeYUVOverlay(mYUVOverlay);
  SDL_Quit();

  mWindowOpen=false;
}
