
#include <SDL.h>


class SDL_YUV_Display
{
public:

  bool init(int frame_width, int frame_height);
  void display(unsigned char *Y, unsigned char *U, unsigned char *V);
  void close();

  bool doQuit() const;

  bool isOpen() const { return mWindowOpen; }

private:  
  SDL_Surface *mScreen;
  SDL_Overlay *mYUVOverlay;
  SDL_Rect     rect;
  bool         mWindowOpen;
};
