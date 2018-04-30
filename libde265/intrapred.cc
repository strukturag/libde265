/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
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

#include "intrapred.h"
#include "transform.h"
#include "util.h"
#include "encoder/encoder-types.h"
#include <assert.h>


#include <sys/types.h>
#include <string.h>



void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3],
                                 enum IntraPredMode candIntraPredModeA,
                                 enum IntraPredMode candIntraPredModeB)
{
  // build candidate list

  if (candIntraPredModeA == candIntraPredModeB) {
    if (candIntraPredModeA < 2) {
      candModeList[0] = INTRA_PLANAR;
      candModeList[1] = INTRA_DC;
      candModeList[2] = INTRA_ANGULAR_26;
    }
    else {
      candModeList[0] = candIntraPredModeA;
      candModeList[1] = (enum IntraPredMode)(2 + ((candIntraPredModeA-2 -1 +32) % 32));
      candModeList[2] = (enum IntraPredMode)(2 + ((candIntraPredModeA-2 +1    ) % 32));
    }
  }
  else {
    candModeList[0] = candIntraPredModeA;
    candModeList[1] = candIntraPredModeB;

    if (candIntraPredModeA != INTRA_PLANAR &&
        candIntraPredModeB != INTRA_PLANAR) {
      candModeList[2] = INTRA_PLANAR;
    }
    else if (candIntraPredModeA != INTRA_DC &&
             candIntraPredModeB != INTRA_DC) {
      candModeList[2] = INTRA_DC;
    }
    else {
      candModeList[2] = INTRA_ANGULAR_26;
    }
  }

  /*
    printf("candModeList: %d %d %d\n",
    candModeList[0],
    candModeList[1],
    candModeList[2]
    );
  */
}


void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3], int x,int y, int PUidx,
                                 bool availableA, // left
                                 bool availableB, // top
                                 const de265_image* img)
{
  const seq_parameter_set* sps = &img->get_sps();

  // block on left side

  enum IntraPredMode candIntraPredModeA, candIntraPredModeB;
  if (availableA==false) {
    candIntraPredModeA=INTRA_DC;
  }
  else if (img->get_pred_mode(x-1,y) != MODE_INTRA ||
           img->get_pcm_flag (x-1,y)) {
    candIntraPredModeA=INTRA_DC;
 }
  else {
    candIntraPredModeA = img->get_IntraPredMode_atIndex(PUidx-1);
  }

  // block above

  if (availableB==false) {
    candIntraPredModeB=INTRA_DC;
  }
  else if (img->get_pred_mode(x,y-1) != MODE_INTRA ||
           img->get_pcm_flag (x,y-1)) {
    candIntraPredModeB=INTRA_DC;
  }
  else if (y-1 < ((y >> sps->Log2CtbSizeY) << sps->Log2CtbSizeY)) {
    candIntraPredModeB=INTRA_DC;
  }
  else {
    candIntraPredModeB = img->get_IntraPredMode_atIndex(PUidx-sps->PicWidthInMinPUs);
  }


  logtrace(LogSlice,"%d;%d candA:%d / candB:%d\n", x,y,
           availableA ? candIntraPredModeA : -999,
           availableB ? candIntraPredModeB : -999);


  fillIntraPredModeCandidates(candModeList,
                              candIntraPredModeA,
                              candIntraPredModeB);
}


int find_intra_pred_mode(enum IntraPredMode mode,
                         enum IntraPredMode candModeList[3])
{
  // check whether the mode is in the candidate list

  for (int i=0;i<3;i++) {
    if (candModeList[i] == mode) {
      return i;
    }
  }

  // sort candModeList

  if (candModeList[0] > candModeList[1]) {
    std::swap(candModeList[0],candModeList[1]);
  }
  if (candModeList[0] > candModeList[2]) {
    std::swap(candModeList[0],candModeList[2]);
  }
  if (candModeList[1] > candModeList[2]) {
    std::swap(candModeList[1],candModeList[2]);
  }

  // skip modes already in the candidate list

  int intraMode = mode;

  for (int i=2;i>=0;i--) {
    if (intraMode >= candModeList[i]) { intraMode--; }
  }

  return -intraMode-1;
}


void list_chroma_pred_candidates(enum IntraPredMode chroma_mode[5],
                                 enum IntraPredMode luma_mode)
{
  enum IntraPredMode chroma_cand[5];
  chroma_cand[0] = INTRA_PLANAR;
  chroma_cand[1] = INTRA_ANGULAR_26;
  chroma_cand[2] = INTRA_ANGULAR_10;
  chroma_cand[3] = INTRA_DC;
  chroma_cand[4] = luma_mode;

  switch (luma_mode) {
  case INTRA_PLANAR:     chroma_cand[0] = INTRA_ANGULAR_34; break;
  case INTRA_ANGULAR_26: chroma_cand[1] = INTRA_ANGULAR_34; break;
  case INTRA_ANGULAR_10: chroma_cand[2] = INTRA_ANGULAR_34; break;
  case INTRA_DC:         chroma_cand[3] = INTRA_ANGULAR_34; break;
  default:
    // use defaults from above
    break;
  }
}


int get_intra_scan_idx(int log2TrafoSize, enum IntraPredMode intraPredMode, int cIdx,
                       const seq_parameter_set* sps)
{
  if (log2TrafoSize==2 ||
      (log2TrafoSize==3 && (cIdx==0 ||
                            sps->ChromaArrayType==CHROMA_444))) {
    /**/ if (intraPredMode >=  6 && intraPredMode <= 14) return 2;
    else if (intraPredMode >= 22 && intraPredMode <= 30) return 1;
    else return 0;
  }
  else { return 0; }
}


int get_intra_scan_idx_luma(int log2TrafoSize, enum IntraPredMode intraPredMode)
{
  if (log2TrafoSize==2 || log2TrafoSize==3) {
    /**/ if (intraPredMode >=  6 && intraPredMode <= 14) return 2;
    else if (intraPredMode >= 22 && intraPredMode <= 30) return 1;
    else return 0;
  }
  else { return 0; }
}

int get_intra_scan_idx_chroma(int log2TrafoSize, enum IntraPredMode intraPredMode)
{
  if (log2TrafoSize==1 || log2TrafoSize==2) {
    /**/ if (intraPredMode >=  6 && intraPredMode <= 14) return 2;
    else if (intraPredMode >= 22 && intraPredMode <= 30) return 1;
    else return 0;
  }
  else { return 0; }
}


enum IntraPredMode lumaPredMode_to_chromaPredMode(enum IntraPredMode luma,
                                                  enum IntraChromaPredMode chroma)
{
  switch (chroma) {
  case INTRA_CHROMA_LIKE_LUMA:
    return luma;

  case INTRA_CHROMA_PLANAR_OR_34:
    if (luma==INTRA_PLANAR) return INTRA_ANGULAR_34;
    else                    return INTRA_PLANAR;

  case INTRA_CHROMA_ANGULAR_26_OR_34:
    if (luma==INTRA_ANGULAR_26) return INTRA_ANGULAR_34;
    else                        return INTRA_ANGULAR_26;

  case INTRA_CHROMA_ANGULAR_10_OR_34:
    if (luma==INTRA_ANGULAR_10) return INTRA_ANGULAR_34;
    else                        return INTRA_ANGULAR_10;

  case INTRA_CHROMA_DC_OR_34:
    if (luma==INTRA_DC)         return INTRA_ANGULAR_34;
    else                        return INTRA_DC;
  }


  assert(false);
  return INTRA_DC;
}


template <class pixel_t>
void intra_border_computer<pixel_t>::reference_sample_substitution()
{
  // reference sample substitution

  const int bit_depth = img->get_bit_depth(cIdx);

  if (nAvail!=4*nT+1) {
    if (nAvail==0) {
      if (sizeof(pixel_t)==1) {
        memset(out_border-2*nT, 1<<(bit_depth-1), 4*nT+1);
      }
      else {
        for (int i = -2*nT; i <= 2*nT ; i++) {
          out_border[i] = 1<<(bit_depth-1);
        }
      }
    }
    else {
      if (!available[-2*nT]) {
        out_border[-2*nT] = firstValue;
      }

      for (int i=-2*nT+1; i<=2*nT; i++)
        if (!available[i]) {
          out_border[i]=out_border[i-1];
        }
    }
  }

  logtrace(LogIntraPred,"availableN: ");
  print_border(available,NULL,nT);
  logtrace(LogIntraPred,"\n");

  logtrace(LogIntraPred,"output:     ");
  print_border(out_border,NULL,nT);
  logtrace(LogIntraPred,"\n");
}




template <class pixel_t>
void intra_border_computer<pixel_t>::preproc()
{
  sps = &img->get_sps();
  pps = &img->get_pps();

  SubWidth  = (cIdx==0) ? 1 : sps->SubWidthC;
  SubHeight = (cIdx==0) ? 1 : sps->SubHeightC;

  // --- check for CTB boundaries ---

  int xBLuma = xB * SubWidth;
  int yBLuma = yB * SubHeight;

  int log2CtbSize = sps->Log2CtbSizeY;
  int picWidthInCtbs = sps->PicWidthInCtbsY;


  //printf("xB/yB: %d %d\n",xB,yB);

  // are we at left image border

  if (xBLuma == 0) {
    availableLeft = false;
    availableTopLeft = false;
    xBLuma = 0; // fake value, available flags are already set to false
  }


  // are we at top image border

  if (yBLuma == 0) {
    availableTop = false;
    availableTopLeft = false;
    availableTopRight = false;
    yBLuma = 0; // fake value, available flags are already set to false
  }

  if (xBLuma+nT*SubWidth >= sps->pic_width_in_luma_samples) {
    availableTopRight=false;
  }

  // check for tile and slice boundaries

  int xCurrCtb = xBLuma >> log2CtbSize;
  int yCurrCtb = yBLuma >> log2CtbSize;
  int xLeftCtb = (xBLuma-1) >> log2CtbSize;
  int xRightCtb = (xBLuma+nT*SubWidth) >> log2CtbSize;
  int yTopCtb   = (yBLuma-1) >> log2CtbSize;

  int currCTBSlice = img->get_SliceAddrRS(xCurrCtb,yCurrCtb);
  int leftCTBSlice = availableLeft ? img->get_SliceAddrRS(xLeftCtb, yCurrCtb) : -1;
  int topCTBSlice  = availableTop ? img->get_SliceAddrRS(xCurrCtb, yTopCtb) : -1;
  int toprightCTBSlice = availableTopRight ? img->get_SliceAddrRS(xRightCtb, yTopCtb) : -1;
  int topleftCTBSlice  = availableTopLeft  ? img->get_SliceAddrRS(xLeftCtb, yTopCtb) : -1;

  /*
  printf("size: %d\n",pps->TileIdRS.size());
  printf("curr: %d left: %d top: %d\n",
         xCurrCtb+yCurrCtb*picWidthInCtbs,
         availableLeft ? xLeftCtb+yCurrCtb*picWidthInCtbs : 9999,
         availableTop  ? xCurrCtb+yTopCtb*picWidthInCtbs  : 9999);
  */
  int currCTBTileID = pps->TileIdRS[xCurrCtb+yCurrCtb*picWidthInCtbs];
  int leftCTBTileID = availableLeft ? pps->TileIdRS[xLeftCtb+yCurrCtb*picWidthInCtbs] : -1;
  int topCTBTileID  = availableTop ? pps->TileIdRS[xCurrCtb+yTopCtb*picWidthInCtbs] : -1;
  int topleftCTBTileID = availableTopLeft ? pps->TileIdRS[xLeftCtb+yTopCtb*picWidthInCtbs] : -1;
  int toprightCTBTileID= availableTopRight? pps->TileIdRS[xRightCtb+yTopCtb*picWidthInCtbs] : -1;

  if (leftCTBSlice != currCTBSlice  || leftCTBTileID != currCTBTileID ) availableLeft    = false;
  if (topCTBSlice  != currCTBSlice  || topCTBTileID  != currCTBTileID ) availableTop     = false;
  if (topleftCTBSlice !=currCTBSlice||topleftCTBTileID!=currCTBTileID ) availableTopLeft = false;
  if (toprightCTBSlice!=currCTBSlice||toprightCTBTileID!=currCTBTileID) availableTopRight= false;


  // number of pixels that are in the valid image area to the right and to the bottom

  nBottom = sps->pic_height_in_luma_samples - yB*SubHeight;
  nBottom=(nBottom+SubHeight-1)/SubHeight;
  if (nBottom>2*nT) nBottom=2*nT;

  nRight  = sps->pic_width_in_luma_samples  - xB*SubWidth;
  nRight =(nRight +SubWidth-1)/SubWidth;
  if (nRight >2*nT) nRight=2*nT;

  nAvail=0;

  available = &available_data[2*MAX_INTRA_PRED_BLOCK_SIZE];

  memset(available-2*nT, 0, 4*nT+1);
}



// (8.4.4.2.2)
template <class pixel_t>
void fill_border_samples(de265_image* img,
                         int xB,int yB,  // in component specific resolution
                         int nT, int cIdx,
                         pixel_t* out_border)
{
  intra_border_computer<pixel_t> c;
  c.init(out_border, img, nT, cIdx, xB, yB);
  c.preproc();
  c.fill_from_image();
  c.reference_sample_substitution();
}


template <class pixel_t>
void intra_border_computer<pixel_t>::fill_from_image()
{
  assert(nT<=32);

  pixel_t* image;
  int stride;
  image  = (pixel_t*)img->get_image_plane(cIdx);
  stride = img->get_image_stride(cIdx);

  int xBLuma = xB * SubWidth;
  int yBLuma = yB * SubHeight;

  int currBlockAddr = pps->MinTbAddrZS[ (xBLuma>>sps->Log2MinTrafoSize) +
                                        (yBLuma>>sps->Log2MinTrafoSize) * sps->PicWidthInTbsY ];


  // copy pixels at left column

  for (int y=nBottom-1 ; y>=0 ; y-=4)
    if (availableLeft)
      {
        int NBlockAddr = pps->MinTbAddrZS[ (((xB-1)*SubWidth )>>sps->Log2MinTrafoSize) +
                                           (((yB+y)*SubHeight)>>sps->Log2MinTrafoSize)
                                           * sps->PicWidthInTbsY ];

        bool availableN = NBlockAddr <= currBlockAddr;

        if (pps->constrained_intra_pred_flag) {
          if (img->get_pred_mode((xB-1)*SubWidth,(yB+y)*SubHeight)!=MODE_INTRA)
            availableN = false;
        }

        if (availableN) {
          if (!nAvail) firstValue = image[xB-1 + (yB+y)*stride];

          for (int i=0;i<4;i++) {
            available[-y+i-1] = availableN;
            out_border[-y+i-1] = image[xB-1 + (yB+y-i)*stride];
          }

          nAvail+=4;
        }
      }

  // copy pixel at top-left position

  if (availableTopLeft)
    {
      int NBlockAddr = pps->MinTbAddrZS[ (((xB-1)*SubWidth )>>sps->Log2MinTrafoSize) +
                                         (((yB-1)*SubHeight)>>sps->Log2MinTrafoSize)
                                         * sps->PicWidthInTbsY ];

      bool availableN = NBlockAddr <= currBlockAddr;

      if (pps->constrained_intra_pred_flag) {
        if (img->get_pred_mode((xB-1)*SubWidth,(yB-1)*SubHeight)!=MODE_INTRA) {
          availableN = false;
        }
      }

      if (availableN) {
        if (!nAvail) firstValue = image[xB-1 + (yB-1)*stride];

        out_border[0] = image[xB-1 + (yB-1)*stride];
        available[0] = availableN;
        nAvail++;
      }
    }

  // copy pixels at top row

  for (int x=0 ; x<nRight ; x+=4) {
    bool borderAvailable;
    if (x<nT) borderAvailable=availableTop;
    else      borderAvailable=availableTopRight;

    if (borderAvailable)
      {
        int NBlockAddr = pps->MinTbAddrZS[ (((xB+x)*SubWidth )>>sps->Log2MinTrafoSize) +
                                           (((yB-1)*SubHeight)>>sps->Log2MinTrafoSize)
                                           * sps->PicWidthInTbsY ];

        bool availableN = NBlockAddr <= currBlockAddr;

        if (pps->constrained_intra_pred_flag) {
          if (img->get_pred_mode((xB+x)*SubWidth,(yB-1)*SubHeight)!=MODE_INTRA) {
            availableN = false;
          }
        }


        if (availableN) {
          if (!nAvail) firstValue = image[xB+x + (yB-1)*stride];

          for (int i=0;i<4;i++) {
            out_border[x+i+1] = image[xB+x+i + (yB-1)*stride];
            available[x+i+1] = availableN;
          }

          nAvail+=4;
        }
      }
  }
}


const int intraPredAngle_table[1+34] =
  { 0, 0,32,26,21,17,13, 9, 5, 2, 0,-2,-5,-9,-13,-17,-21,-26,
    -32,-26,-21,-17,-13,-9,-5,-2,0,2,5,9,13,17,21,26,32 };

const int invAngle_table[25-10] =
  { -4096,-1638,-910,-630,-482,-390,-315,-256,
    -315,-390,-482,-630,-910,-1638,-4096 };


template <class pixel_t>
void decode_intra_prediction_internal(de265_image* img,
                                      int xB0,int yB0,
                                      enum IntraPredMode intraPredMode,
                                      pixel_t* dst, int dstStride,
                                      int nT, int cIdx)
{
  pixel_t  border_pixels_mem[4*MAX_INTRA_PRED_BLOCK_SIZE+1];
  pixel_t* border_pixels = &border_pixels_mem[2*MAX_INTRA_PRED_BLOCK_SIZE];

  fill_border_samples(img, xB0,yB0, nT, cIdx, border_pixels);

  if (img->get_sps().range_extension.intra_smoothing_disabled_flag == 0 &&
      (cIdx==0 || img->get_sps().ChromaArrayType==CHROMA_444))
    {
      intra_prediction_sample_filtering(img->get_sps(), border_pixels, nT, cIdx, intraPredMode);
    }


  switch (intraPredMode) {
  case INTRA_PLANAR:
    intra_prediction_planar(dst,dstStride, nT,cIdx, border_pixels);
    break;
  case INTRA_DC:
    intra_prediction_DC(dst,dstStride, nT,cIdx, border_pixels);
    break;
  default:
    {
      int bit_depth = img->get_bit_depth(cIdx);
      bool disableIntraBoundaryFilter =
        (img->get_sps().range_extension.implicit_rdpcm_enabled_flag &&
         img->get_cu_transquant_bypass(xB0,yB0));

      intra_prediction_angular(dst,dstStride, bit_depth,disableIntraBoundaryFilter,
                               xB0,yB0,intraPredMode,nT,cIdx, border_pixels);
    }
    break;
  }
}


// (8.4.4.2.1)
void decode_intra_prediction(de265_image* img,
                             int xB0,int yB0,
                             enum IntraPredMode intraPredMode,
                             int nT, int cIdx)
{
  logtrace(LogIntraPred,"decode_intra_prediction xy0:%d/%d mode=%d nT=%d, cIdx=%d\n",
           xB0,yB0, intraPredMode, nT,cIdx);
  /*
    printf("decode_intra_prediction xy0:%d/%d mode=%d nT=%d, cIdx=%d\n",
    xB0,yB0, intraPredMode, nT,cIdx);
  */

  if (img->high_bit_depth(cIdx)) {
    decode_intra_prediction_internal<uint16_t>(img,xB0,yB0, intraPredMode,
                                               img->get_image_plane_at_pos_NEW<uint16_t>(cIdx,xB0,yB0),
                                               img->get_image_stride(cIdx),
                                               nT,cIdx);
  }
  else {
    decode_intra_prediction_internal<uint8_t>(img,xB0,yB0, intraPredMode,
                                              img->get_image_plane_at_pos_NEW<uint8_t>(cIdx,xB0,yB0),
                                              img->get_image_stride(cIdx),
                                              nT,cIdx);
  }
}


// TODO: remove this
template <> void decode_intra_prediction<uint8_t>(de265_image* img,
                                                  int xB0,int yB0,
                                                  enum IntraPredMode intraPredMode,
                                                  uint8_t* dst, int nT, int cIdx)
{
    decode_intra_prediction_internal<uint8_t>(img,xB0,yB0, intraPredMode,
                                              dst,nT,
                                              nT,cIdx);
}


// TODO: remove this
template <> void decode_intra_prediction<uint16_t>(de265_image* img,
                                                   int xB0,int yB0,
                                                   enum IntraPredMode intraPredMode,
                                                   uint16_t* dst, int nT, int cIdx)
{
  decode_intra_prediction_internal<uint16_t>(img,xB0,yB0, intraPredMode,
                                             dst,nT,
                                             nT,cIdx);
}
