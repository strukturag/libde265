
#include "sdl.hh"


bool SDL_YUV_Display::init(int frame_width, int frame_height)
{
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

void SDL_YUV_Display::display(unsigned char *Y, unsigned char *U, unsigned char *V)
{
  if (!mWindowOpen) return;
  if (SDL_LockYUVOverlay(mYUVOverlay) < 0) return;

  memcpy(mYUVOverlay->pixels[0], Y, rect.w * rect.h);
  memcpy(mYUVOverlay->pixels[1], V, rect.w * rect.h / 4);
  memcpy(mYUVOverlay->pixels[2], U, rect.w * rect.h / 4);

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
