/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "VideoDecoder.hh"
#include <libvideogfx.hh>


using namespace videogfx;

extern "C" {
#include "decctx.h"
}


VideoDecoder::VideoDecoder()
  : mNextBuffer(0),
    mFrameCount(0),
    mPlayingVideo(false),
    mVideoEnded(false),
    mSingleStep(false),
    mShowDecodedImage(true),
    mCBShowPartitioning(false),
    mTBShowPartitioning(false),
    mPBShowPartitioning(false),
    mShowPBPredMode(false),
    mShowIntraPredMode(false),
    mFH(NULL)
{
}


VideoDecoder::~VideoDecoder()
{
  free_decoder();
}

void VideoDecoder::run()
{
  decoder_loop();
}


void VideoDecoder::init(const char* filename)
{
  init_decoder(filename);
}


void VideoDecoder::startDecoder()
{
  if (mPlayingVideo || mVideoEnded) { return; }

  mPlayingVideo=true;
  exit();
}

void VideoDecoder::stopDecoder()
{
  if (!mPlayingVideo) { return; }

  mPlayingVideo=false;
}

void VideoDecoder::singleStepDecoder()
{
  if (mPlayingVideo || mVideoEnded) { return; }

  mPlayingVideo=true;
  mSingleStep=true;
  exit();
}

void VideoDecoder::decoder_loop()
{
  for (;;)
    {
      if (mPlayingVideo) {
        de265_release_next_picture(ctx);

        const de265_image* img = de265_peek_next_picture(ctx);
        while (img==NULL)
          {
            /*
            int err;
            err = read_nal_unit(&inputctx.ctx, &buf);
            if (err!=DE265_OK)
              {
                mVideoEnded=true;
                mPlayingVideo=false; // TODO: send signal back
                break;
              }
            
            err = de265_decode_NAL(&ctx, &buf);
            if (err!=DE265_OK)
              {
                mVideoEnded=true;
                mPlayingVideo=false; // TODO: send signal back
                break;
              }
            */

            uint8_t buf[4096];
            fread(buf,4096,1,mFH);
            int err = de265_decode_data(ctx,buf,4096);
            if (err!=DE265_OK)
              {
                mVideoEnded=true;
                mPlayingVideo=false; // TODO: send signal back
                break;
              }

            // try again to get picture

            img = de265_peek_next_picture(ctx);
          }


        // show one decoded picture

        if (img) {
          show_frame(img);

          if (mSingleStep) {
            mSingleStep=false;
            mPlayingVideo=false;
          }
        }


        // process events

        QCoreApplication::processEvents();
      }
      else {
        exec();
      }
    }
}


void VideoDecoder::show_frame(const de265_image* img)
{
  Image<Pixel> debugvisu;

  if (mShowDecodedImage) {
    Image<Pixel> visu;
    visu.Create(img->width, img->height, Colorspace_YUV, Chroma_420);

    for (int y=0;y<img->height;y++) {
      memcpy(visu.AskFrameY()[y], img->y + y*img->stride, img->width);
    }

    for (int y=0;y<img->chroma_height;y++) {
      memcpy(visu.AskFrameU()[y], img->cb + y*img->chroma_stride, img->chroma_width);
    }

    for (int y=0;y<img->chroma_height;y++) {
      memcpy(visu.AskFrameV()[y], img->cr + y*img->chroma_stride, img->chroma_width);
    }


    ChangeColorspace(debugvisu, visu, Colorspace_RGB);
  }
  else {
    debugvisu.Create(img->width,img->height, Colorspace_RGB);
    Clear(debugvisu, Color<Pixel>(0,0,0));
  }


  const decoder_context* cx = (const decoder_context*)ctx;

  if (1) {
    if (mShowPBPredMode)
      {
        draw_PB_pred_modes(cx,
                           debugvisu.AskFrameR()[0],
                           debugvisu.AskFrameG()[0],
                           debugvisu.AskFrameB()[0],
                           debugvisu.AskBitmapB().AskStride());
      }

    if (mShowIntraPredMode)
      {
        draw_intra_pred_modes(cx,debugvisu.AskFrameR()[0],debugvisu.AskBitmapR().AskStride(),140);
        draw_intra_pred_modes(cx,debugvisu.AskFrameG()[0],debugvisu.AskBitmapG().AskStride(),140);
        draw_intra_pred_modes(cx,debugvisu.AskFrameB()[0],debugvisu.AskBitmapB().AskStride(),255);
      }

    if (mTBShowPartitioning)
      {
        draw_TB_grid(cx, debugvisu.AskFrameR()[0], debugvisu.AskBitmapR().AskStride(),255);
        draw_TB_grid(cx, debugvisu.AskFrameG()[0], debugvisu.AskBitmapG().AskStride(), 80);
        draw_TB_grid(cx, debugvisu.AskFrameB()[0], debugvisu.AskBitmapB().AskStride(),  0);
      }

    if (mPBShowPartitioning)
      {
        draw_PB_grid(cx, debugvisu.AskFrameR()[0], debugvisu.AskBitmapR().AskStride(),  0);
        draw_PB_grid(cx, debugvisu.AskFrameG()[0], debugvisu.AskBitmapG().AskStride(),200);
        draw_PB_grid(cx, debugvisu.AskFrameB()[0], debugvisu.AskBitmapB().AskStride(),  0);
      }

    if (mCBShowPartitioning)
      {
        draw_CB_grid(cx, debugvisu.AskFrameR()[0], debugvisu.AskBitmapR().AskStride(),255);
        draw_CB_grid(cx, debugvisu.AskFrameG()[0], debugvisu.AskBitmapG().AskStride(),255);
        draw_CB_grid(cx, debugvisu.AskFrameB()[0], debugvisu.AskBitmapB().AskStride(),255);
      }
  }


  // --- convert to QImage and show ---

  if (mFrameCount==0) {
    mImgBuffers[0] = QImage(QSize(img->width,img->height), QImage::Format_RGB32);
    mImgBuffers[1] = QImage(QSize(img->width,img->height), QImage::Format_RGB32);
  }

  QImage* qimg = &mImgBuffers[mNextBuffer];
  uchar* ptr = qimg->bits();
  int bpl = qimg->bytesPerLine();

  for (int y=0;y<img->height;y++)
    {
      for (int x=0;x<img->width;x++)
        {
          *(uint32_t*)(ptr+x*4) = ((debugvisu.AskFrameR()[y][x] << 16) |
                                   (debugvisu.AskFrameG()[y][x] <<  8) |
                                   (debugvisu.AskFrameB()[y][x] <<  0));
        }

      ptr += bpl;
    }


  if (0) {
    if (mTBShowPartitioning)
      {
        draw_TB_grid(cx, qimg->bits(), bpl, 0xff9000);
      }

    if (mCBShowPartitioning)
      {
        draw_CB_grid(cx, qimg->bits(), bpl, 0xffffff);
      }
  }


  emit displayImage(qimg);
  mNextBuffer = 1-mNextBuffer;
  mFrameCount++;
}


void VideoDecoder::showCBPartitioning(bool flag)
{
  mCBShowPartitioning=flag;

  const de265_image* img = de265_peek_next_picture(ctx);
  if (img != NULL) { show_frame(img); }
}


void VideoDecoder::showTBPartitioning(bool flag)
{
  mTBShowPartitioning=flag;

  const de265_image* img = de265_peek_next_picture(ctx);
  if (img != NULL) { show_frame(img); }
}

void VideoDecoder::showPBPartitioning(bool flag)
{
  mPBShowPartitioning=flag;

  const de265_image* img = de265_peek_next_picture(ctx);
  if (img != NULL) { show_frame(img); }
}

void VideoDecoder::showIntraPredMode(bool flag)
{
  mShowIntraPredMode=flag;

  const de265_image* img = de265_peek_next_picture(ctx);
  if (img != NULL) { show_frame(img); }
}

void VideoDecoder::showPBPredMode(bool flag)
{
  mShowPBPredMode=flag;

  const de265_image* img = de265_peek_next_picture(ctx);
  if (img != NULL) { show_frame(img); }
}

void VideoDecoder::showDecodedImage(bool flag)
{
  mShowDecodedImage=flag;

  const de265_image* img = de265_peek_next_picture(ctx);
  if (img != NULL) { show_frame(img); }
}




void VideoDecoder::init_decoder(const char* filename)
{
  de265_init();

  mFH = fopen(filename,"rb");
  //init_file_context(&inputctx, filename);
  //rbsp_buffer_init(&buf);

  ctx = de265_new_decoder();
}

void VideoDecoder::free_decoder()
{
  if (mFH) { fclose(mFH); }

  de265_free_decoder(ctx);
}
