
#include <SDL.h>


class SDL_YUV_Display
{
public:

  bool init(int frame_width, int frame_height);
  void display(const unsigned char *Y, const unsigned char *U, const unsigned char *V,
               int stride, int chroma_stride);
  void close();

  bool doQuit() const;

  bool isOpen() const { return mWindowOpen; }

private:  
  SDL_Surface *mScreen;
  SDL_Overlay *mYUVOverlay;
  SDL_Rect     rect;
  bool         mWindowOpen;
};
