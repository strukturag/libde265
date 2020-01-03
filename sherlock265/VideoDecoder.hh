/*
  libde265 example application "sherlock265".

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

#ifndef VIDEODECODER_HH
#define VIDEODECODER_HH

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <QtGui>
#ifdef HAVE_SWSCALE
#ifdef __cplusplus
extern "C" {
#endif
#include <libswscale/swscale.h>
#ifdef __cplusplus
}
#endif
#endif

#include "VideoWidget.hh"
#include "de265.h"


class VideoDecoder : public QThread
{
  Q_OBJECT

public:
  VideoDecoder();
  ~VideoDecoder();

  void init(const char* filename);

protected:
  void run();  // thread entry point

public slots:
  void startDecoder();
  void stopDecoder();
  void singleStepDecoder();

  void showCBPartitioning(bool flag);
  void showTBPartitioning(bool flag);
  void showPBPartitioning(bool flag);
  void showIntraPredMode(bool flag);
  void showPBPredMode(bool flag);
  void showQuantPY(bool flag);
  void showMotionVec(bool flag);
  void showTiles(bool flag);
  void showSlices(bool flag);
  void showDecodedImage(bool flag);

signals:
  void displayImage(QImage*);

private:
  // de265 decoder

  FILE* mFH;
  //input_context_FILE inputctx;
  //rbsp_buffer buf;
  de265_decoder_context* ctx;
  const de265_image* img;

  QMutex mutex;

  QImage mImgBuffers[2];
  int    mNextBuffer;
  int    mFrameCount;

  bool   mPlayingVideo;
  bool   mVideoEnded;
  bool   mSingleStep;


  bool   mShowDecodedImage;
  bool   mShowQuantPY;
  bool   mCBShowPartitioning;
  bool   mTBShowPartitioning;
  bool   mPBShowPartitioning;
  bool   mShowIntraPredMode;
  bool   mShowPBPredMode;
  bool   mShowMotionVec;
  bool   mShowTiles;
  bool   mShowSlices;

  void decoder_loop();

  void init_decoder(const char* filename);
  void free_decoder();

  void show_frame(const de265_image* img);
#ifdef HAVE_VIDEOGFX
  void convert_frame_libvideogfx(const de265_image* img, QImage & qimg);
#endif
#ifdef HAVE_SWSCALE
  SwsContext* sws;
  int width;
  int height;
  void convert_frame_swscale(const de265_image* img, QImage & qimg);
#endif
};

#endif
