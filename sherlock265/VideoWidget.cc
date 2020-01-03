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

#include "VideoWidget.hh"
#include <QtGui>


VideoWidget::VideoWidget(QWidget *parent)
  : QWidget(parent), mImg(NULL)
{
  setAutoFillBackground(false);
  setAttribute(Qt::WA_NoSystemBackground, true);

  QPalette palette = this->palette();
  palette.setColor(QPalette::Background, Qt::black);
  setPalette(palette);

  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  setUpdatesEnabled(true);
}

VideoWidget::~VideoWidget()
{
}


QSize VideoWidget::sizeHint() const
{
  return QSize(352,288);
}


void VideoWidget::paintEvent(QPaintEvent *event)
{
  QPainter painter(this);

  if (mImg) {
    QRect videoRect = mImg->rect();
    videoRect.moveCenter(this->rect().center());

    QRect erect = event->rect();

    if (!videoRect.contains(event->rect())) {
      QRegion region = event->region();
      region = region.subtracted(videoRect);

      QBrush brush = palette().background();

      foreach (const QRect &rect, region.rects()) {
        painter.fillRect(rect, brush);
      }
    }

    painter.drawImage(videoRect, *mImg);
  } else {
    painter.fillRect(event->rect(), palette().background());
  }
}

void VideoWidget::resizeEvent(QResizeEvent *event)
{
  QWidget::resizeEvent(event);
}
