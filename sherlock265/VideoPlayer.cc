/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
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

  QPushButton* showDecodedImageButton = new QPushButton("image");
  showDecodedImageButton->setCheckable(true);
  showDecodedImageButton->setChecked(true);
  QObject::connect(showDecodedImageButton, SIGNAL(toggled(bool)),
                   mDecoder, SLOT(showDecodedImage(bool)));

  QGridLayout *layout = new QGridLayout;
  layout->addWidget(videoWidget, 0,0,1,6);
  layout->addWidget(startButton, 1,0,1,1);
  layout->addWidget(stopButton,  1,1,1,1);
  layout->addWidget(stepButton,  1,2,1,1);
  layout->addWidget(showCBPartitioningButton,2,0,1,1);
  layout->addWidget(showTBPartitioningButton,2,1,1,1);
  layout->addWidget(showPBPartitioningButton,2,2,1,1);
  layout->addWidget(showIntraPredModeButton, 2,3,1,1);
  layout->addWidget(showPBPredModeButton,    2,4,1,1);
  layout->addWidget(showDecodedImageButton,  2,5,1,1);
  setLayout(layout);


  mDecoder->start();
}
