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

#include "VideoPlayer.hh"


VideoPlayer::VideoPlayer(const char* filename)
{
  mDecoder = new VideoDecoder;
  mDecoder->init(filename);

  videoWidget = new VideoWidget;

  stopButton = new QPushButton("Stop");
  //QObject::connect(stopButton, SIGNAL(clicked()), qApp, SLOT(stop()));
  QObject::connect(stopButton, SIGNAL(clicked()), mDecoder, SLOT(stopDecoder()));

  startButton = new QPushButton("&Start");
  QObject::connect(startButton, SIGNAL(clicked()), mDecoder, SLOT(startDecoder()));

  QPushButton* stepButton = new QPushButton("Step");
  QObject::connect(stepButton, SIGNAL(clicked()), mDecoder, SLOT(singleStepDecoder()));


  QObject::connect(mDecoder,    SIGNAL(displayImage(QImage*)),
                   videoWidget, SLOT(setImage(QImage*)), Qt::QueuedConnection);



  QPushButton* showCBPartitioningButton = new QPushButton("CB-tree");
  showCBPartitioningButton->setCheckable(true);
  QObject::connect(showCBPartitioningButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showCBPartitioning(bool)));

  QPushButton* showTBPartitioningButton = new QPushButton("TB-tree");
  showTBPartitioningButton->setCheckable(true);
  QObject::connect(showTBPartitioningButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showTBPartitioning(bool)));

  QPushButton* showPBPartitioningButton = new QPushButton("PB-tree");
  showPBPartitioningButton->setCheckable(true);
  QObject::connect(showPBPartitioningButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showPBPartitioning(bool)));

  QPushButton* showIntraPredModeButton = new QPushButton("intra-pred");
  showIntraPredModeButton->setCheckable(true);
  QObject::connect(showIntraPredModeButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showIntraPredMode(bool)));

  QPushButton* showPBPredModeButton = new QPushButton("PB-mode");
  showPBPredModeButton->setCheckable(true);
  QObject::connect(showPBPredModeButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showPBPredMode(bool)));

  QPushButton* showQuantPYButton = new QPushButton("Quant");
  showQuantPYButton->setCheckable(true);
  QObject::connect(showQuantPYButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showQuantPY(bool)));

  QPushButton* showMotionVecButton = new QPushButton("MotionVec");
  showMotionVecButton->setCheckable(true);
  QObject::connect(showMotionVecButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showMotionVec(bool)));

  QPushButton* showTilesButton = new QPushButton("Tiles");
  showTilesButton->setCheckable(true);
  QObject::connect(showTilesButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showTiles(bool)));

  QPushButton* showSlicesButton = new QPushButton("Slices");
  showSlicesButton->setCheckable(true);
  QObject::connect(showSlicesButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showSlices(bool)));

  QPushButton* showDecodedImageButton = new QPushButton("image");
  showDecodedImageButton->setCheckable(true);
  showDecodedImageButton->setChecked(true);
  QObject::connect(showDecodedImageButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showDecodedImage(bool)));

  QGridLayout *layout = new QGridLayout;
  layout->addWidget(videoWidget, 0,0,1,7);
  layout->addWidget(startButton, 1,0,1,1);
  layout->addWidget(stopButton,  1,1,1,1);
  layout->addWidget(stepButton,  1,2,1,1);
  layout->addWidget(showDecodedImageButton,  1,6,1,1);
  layout->addWidget(showTilesButton,         1,5,1,1);
  layout->addWidget(showSlicesButton,        1,4,1,1);
  layout->addWidget(showCBPartitioningButton,2,0,1,1);
  layout->addWidget(showTBPartitioningButton,2,1,1,1);
  layout->addWidget(showPBPartitioningButton,2,2,1,1);
  layout->addWidget(showIntraPredModeButton, 2,3,1,1);
  layout->addWidget(showPBPredModeButton,    2,4,1,1);
  layout->addWidget(showQuantPYButton,       2,5,1,1);
  layout->addWidget(showMotionVecButton,     2,6,1,1);
  setLayout(layout);


  mDecoder->start();
}
