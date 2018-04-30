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

#ifndef DE265_INTRAPRED_H
#define DE265_INTRAPRED_H

#include "libde265/decctx.h"

extern const int intraPredAngle_table[1+34];


/* Fill the three intra-pred-mode candidates into candModeList.
   Block position is (x,y) and you also have to give the PUidx for this
   block (which is (x>>Log2MinPUSize) + (y>>Log2MinPUSize)*PicWidthInMinPUs).
   availableA/B is the output of check_CTB_available().
 */
void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3],
                                 int x,int y, int PUidx,
                                 bool availableA, // left
                                 bool availableB, // top
                                 const de265_image* img);


inline void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3], int x,int y,
                                 bool availableA, // left
                                 bool availableB, // top
                                 const de265_image* img)
{
  int PUidx = img->get_sps().getPUIndexRS(x,y);
  fillIntraPredModeCandidates(candModeList, x,y, PUidx, availableA,availableB, img);
}

void fillIntraPredModeCandidates(enum IntraPredMode candModeList[3],
                                 enum IntraPredMode candIntraPredModeA,
                                 enum IntraPredMode candIntraPredModeB);


/* Return value >= 0 -> use mpm_idx(return value)
   else              -> use rem_intra(-return value-1)

   This function may modify the candModeList !
 */
int find_intra_pred_mode(enum IntraPredMode mode,
                         enum IntraPredMode candModeList[3]);

void list_chroma_pred_candidates(enum IntraPredMode chroma_mode[5],
                                 enum IntraPredMode luma_mode);

int get_intra_scan_idx(int log2TrafoSize, enum IntraPredMode intraPredMode, int cIdx,
                       const seq_parameter_set* sps);

int get_intra_scan_idx_luma  (int log2TrafoSize, enum IntraPredMode intraPredMode); // DEPRECATED
int get_intra_scan_idx_chroma(int log2TrafoSize, enum IntraPredMode intraPredMode); // DEPRECATED

enum IntraPredMode lumaPredMode_to_chromaPredMode(enum IntraPredMode luma,
                                                  enum IntraChromaPredMode chroma);

/*
void decode_intra_block(decoder_context* ctx,
                        thread_context* tctx,
                        int cIdx,
                        int xB0,int yB0, // position of TU in frame (chroma adapted)
                        int x0,int y0,   // position of CU in frame (chroma adapted)
                        int log2TrafoSize, int trafoDepth,
                        enum IntraPredMode intraPredMode,
                        bool transform_skip_flag);
*/

//void fill_border_samples(decoder_context* ctx, int xB,int yB,
//                         int nT, int cIdx, uint8_t* out_border);

void decode_intra_prediction(de265_image* img,
                             int xB0,int yB0,
                             enum IntraPredMode intraPredMode,
                             int nT, int cIdx);

// TODO: remove this
template <class pixel_t> void decode_intra_prediction(de265_image* img,
                                                      int xB0,int yB0,
                                                      enum IntraPredMode intraPredMode,
                                                      pixel_t* dst, int nT, int cIdx);




// --- internal use only ---

// Actually, the largest TB block can only be 32, but in some intra-pred-mode algorithms
// (e.g. min-residual), we may call intra prediction on the maximum CTB size (64).
static const int MAX_INTRA_PRED_BLOCK_SIZE = 64;


template <class pixel_t>
class intra_border_computer
{
 public:
  pixel_t* out_border;

  const de265_image* img;
  int nT;
  int cIdx;

  int xB,yB;

  const seq_parameter_set* sps;
  const pic_parameter_set* pps;

  uint8_t available_data[4*MAX_INTRA_PRED_BLOCK_SIZE + 1];
  uint8_t* available;

  int SubWidth;
  int SubHeight;

  bool availableLeft;    // is CTB at left side available?
  bool availableTop;     // is CTB at top side available?
  bool availableTopRight; // is CTB at top-right side available?
  bool availableTopLeft;  // if CTB at top-left pixel available?

  int nBottom;
  int nRight;
  int nAvail;
  pixel_t firstValue;

  void init(pixel_t* _out_border,
            const de265_image* _img, int _nT, int _cIdx, int _xB, int _yB) {
    img=_img; nT=_nT; cIdx=_cIdx;
    out_border=_out_border; xB=_xB; yB=_yB;

    assert(nT <= MAX_INTRA_PRED_BLOCK_SIZE);

    availableLeft=true;
    availableTop=true;
    availableTopRight=true;
    availableTopLeft=true;
  }
  void preproc();
  void fill_from_image();

  void reference_sample_substitution();
};


#ifdef DE265_LOG_TRACE
template <class pixel_t>
void print_border(pixel_t* data, uint8_t* available, int nT)
{
  for (int i=-2*nT ; i<=2*nT ; i++) {
    if (i==0 || i==1 || i==-nT || i==nT+1) {
      logtrace(LogIntraPred,"|");
    } else {
      logtrace(LogIntraPred," ");
    }

    if (available==NULL || available[i]) {
      logtrace(LogIntraPred,"%02x",data[i]);
    }
    else {
      logtrace(LogIntraPred,"--");
    }
  }
}
#else
#define print_border(data, available, nT)
#endif


// (8.4.4.2.3)
template <class pixel_t>
void intra_prediction_sample_filtering(const seq_parameter_set& sps,
                                       pixel_t* p,
                                       int nT, int cIdx,
                                       enum IntraPredMode intraPredMode)
{
  int filterFlag;

  //printf("filtering, mode: %d\n",intraPredMode);

  if (intraPredMode==INTRA_DC || nT==4) {
    filterFlag = 0;
  } else {
    // int-cast below prevents a typing problem that leads to wrong results when abs_value is a macro
    int minDistVerHor = libde265_min( abs_value((int)intraPredMode-26),
                                      abs_value((int)intraPredMode-10) );

    //printf("mindist: %d\n",minDistVerHor);

    switch (nT) {
    case 8:  filterFlag = (minDistVerHor>7) ? 1 : 0; break;
    case 16: filterFlag = (minDistVerHor>1) ? 1 : 0; break;
    case 32: filterFlag = (minDistVerHor>0) ? 1 : 0; break;
      // there is no official 64x64 TB block, but we call this for some intra-pred mode algorithms
      // on the whole CB (2Nx2N mode for the whole CTB)
    case 64: filterFlag = 0; break;
    default: filterFlag = -1; assert(false); break; // should never happen
    }
  }


  if (filterFlag) {
    int biIntFlag = (sps.strong_intra_smoothing_enable_flag &&
                     cIdx==0 &&
                     nT==32 &&
                     abs_value(p[0]+p[ 64]-2*p[ 32]) < (1<<(sps.bit_depth_luma-5)) &&
                     abs_value(p[0]+p[-64]-2*p[-32]) < (1<<(sps.bit_depth_luma-5)))
      ? 1 : 0;

    pixel_t  pF_mem[4*32+1];
    pixel_t* pF = &pF_mem[2*32];

    if (biIntFlag) {
      pF[-2*nT] = p[-2*nT];
      pF[ 2*nT] = p[ 2*nT];
      pF[    0] = p[    0];

      for (int i=1;i<=63;i++) {
        pF[-i] = p[0] + ((i*(p[-64]-p[0])+32)>>6);
        pF[ i] = p[0] + ((i*(p[ 64]-p[0])+32)>>6);
      }
    } else {
      pF[-2*nT] = p[-2*nT];
      pF[ 2*nT] = p[ 2*nT];

      for (int i=-(2*nT-1) ; i<=2*nT-1 ; i++)
        {
          pF[i] = (p[i+1] + 2*p[i] + p[i-1] + 2) >> 2;
        }
    }


    // copy back to original array

    memcpy(p-2*nT, pF-2*nT, (4*nT+1) * sizeof(pixel_t));
  }
  else {
    // do nothing ?
  }


  logtrace(LogIntraPred,"post filtering: ");
  print_border(p,NULL,nT);
  logtrace(LogIntraPred,"\n");
}


template <class pixel_t>
void intra_prediction_planar(pixel_t* dst, int dstStride,
                             int nT,int cIdx,
                             pixel_t* border)
{
  int Log2_nT = Log2(nT);

  for (int y=0;y<nT;y++)
    for (int x=0;x<nT;x++)
      {
        dst[x+y*dstStride] = ((nT-1-x)*border[-1-y] + (x+1)*border[ 1+nT] +
                              (nT-1-y)*border[ 1+x] + (y+1)*border[-1-nT] + nT) >> (Log2_nT+1);
      }


  logtrace(LogIntraPred,"result of planar prediction\n");

  for (int y=0;y<nT;y++)
    {
      for (int x=0;x<nT;x++)
        logtrace(LogIntraPred,"%02x ", dst[x+y*dstStride]);

      logtrace(LogIntraPred,"\n");
    }
}


template <class pixel_t>
void intra_prediction_DC(pixel_t* dst, int dstStride,
                         int nT,int cIdx,
                         pixel_t* border)
{
  int Log2_nT = Log2(nT);

  int dcVal = 0;
  for (int i=0;i<nT;i++)
    {
      dcVal += border[ i+1];
      dcVal += border[-i-1];
    }

  dcVal += nT;
  dcVal >>= Log2_nT+1;

  if (cIdx==0 && nT<32) {
    dst[0] = (border[-1] + 2*dcVal + border[1] +2) >> 2;

    for (int x=1;x<nT;x++) { dst[x]           = (border[ x+1] + 3*dcVal+2)>>2; }
    for (int y=1;y<nT;y++) { dst[y*dstStride] = (border[-y-1] + 3*dcVal+2)>>2; }
    for (int y=1;y<nT;y++)
      for (int x=1;x<nT;x++)
        {
          dst[x+y*dstStride] = dcVal;
        }
  } else {
    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++)
        {
          dst[x+y*dstStride] = dcVal;
        }
  }
}


extern const int intraPredAngle_table[1+34];
extern const int invAngle_table[25-10];


// (8.4.4.2.6)
template <class pixel_t>
void intra_prediction_angular(pixel_t* dst, int dstStride,
                              int bit_depth, bool disableIntraBoundaryFilter,
                              int xB0,int yB0,
                              enum IntraPredMode intraPredMode,
                              int nT,int cIdx,
                              pixel_t* border)
{
  pixel_t  ref_mem[4*MAX_INTRA_PRED_BLOCK_SIZE+1]; // TODO: what is the required range here ?
  pixel_t* ref=&ref_mem[2*MAX_INTRA_PRED_BLOCK_SIZE];

  assert(intraPredMode<35);
  assert(intraPredMode>=2);

  int intraPredAngle = intraPredAngle_table[intraPredMode];

  if (intraPredMode >= 18) {

    for (int x=0;x<=nT;x++)
      { ref[x] = border[x]; }

    if (intraPredAngle<0) {
      int invAngle = invAngle_table[intraPredMode-11];

      if ((nT*intraPredAngle)>>5 < -1) {
        for (int x=(nT*intraPredAngle)>>5; x<=-1; x++) {
          ref[x] = border[0-((x*invAngle+128)>>8)];
        }
      }
    } else {
      for (int x=nT+1; x<=2*nT;x++) {
        ref[x] = border[x];
      }
    }

    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++)
        {
          int iIdx = ((y+1)*intraPredAngle)>>5;
          int iFact= ((y+1)*intraPredAngle)&31;

          if (iFact != 0) {
            dst[x+y*dstStride] = ((32-iFact)*ref[x+iIdx+1] + iFact*ref[x+iIdx+2] + 16)>>5;
          } else {
            dst[x+y*dstStride] = ref[x+iIdx+1];
          }
        }

    if (intraPredMode==26 && cIdx==0 && nT<32 && !disableIntraBoundaryFilter) {
      for (int y=0;y<nT;y++) {
        dst[0+y*dstStride] = Clip_BitDepth(border[1] + ((border[-1-y] - border[0])>>1), bit_depth);
      }
    }
  }
  else { // intraPredAngle < 18

    for (int x=0;x<=nT;x++)
      { ref[x] = border[-x]; }  // DIFF (neg)

    if (intraPredAngle<0) {
      int invAngle = invAngle_table[intraPredMode-11];

      if ((nT*intraPredAngle)>>5 < -1) {
        for (int x=(nT*intraPredAngle)>>5; x<=-1; x++) {
          ref[x] = border[((x*invAngle+128)>>8)]; // DIFF (neg)
        }
      }
    } else {
      for (int x=nT+1; x<=2*nT;x++) {
        ref[x] = border[-x]; // DIFF (neg)
      }
    }

    for (int y=0;y<nT;y++)
      for (int x=0;x<nT;x++)
        {
          int iIdx = ((x+1)*intraPredAngle)>>5;  // DIFF (x<->y)
          int iFact= ((x+1)*intraPredAngle)&31;  // DIFF (x<->y)

          if (iFact != 0) {
            dst[x+y*dstStride] = ((32-iFact)*ref[y+iIdx+1] + iFact*ref[y+iIdx+2] + 16)>>5; // DIFF (x<->y)
          } else {
            dst[x+y*dstStride] = ref[y+iIdx+1]; // DIFF (x<->y)
          }
        }

    if (intraPredMode==10 && cIdx==0 && nT<32 && !disableIntraBoundaryFilter) {  // DIFF 26->10
      for (int x=0;x<nT;x++) { // DIFF (x<->y)
        dst[x] = Clip_BitDepth(border[-1] + ((border[1+x] - border[0])>>1), bit_depth); // DIFF (x<->y && neg)
      }
    }
  }


  logtrace(LogIntraPred,"result of angular intra prediction (mode=%d):\n",intraPredMode);

  for (int y=0;y<nT;y++)
    {
      for (int x=0;x<nT;x++)
        logtrace(LogIntraPred,"%02x ", dst[x+y*dstStride]);

      logtrace(LogIntraPred,"\n");
    }
}


#endif
