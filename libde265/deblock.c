/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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

#include "deblock.h"
#include "util.h"
#include "transform.h"

#include <assert.h>



// 8.7.2.1 for both EDGE_HOR and EDGE_VER at the same time
void markTransformBlockBoundary(decoder_context* ctx, int x0,int y0,
                                int log2TrafoSize,int trafoDepth,
                                int filterLeftCbEdge, int filterTopCbEdge)
{
  logtrace(LogDeblock,"markTransformBlockBoundary(%d,%d, %d,%d, %d,%d)\n",x0,y0,
           log2TrafoSize,trafoDepth, filterLeftCbEdge,filterTopCbEdge);

  if (get_split_transform_flag(ctx,x0,y0,trafoDepth)) {
    int x1 = x0 + ((1<<log2TrafoSize)>>1);
    int y1 = y0 + ((1<<log2TrafoSize)>>1);

    markTransformBlockBoundary(ctx,x0,y0,log2TrafoSize-1,trafoDepth+1, filterLeftCbEdge,   filterTopCbEdge);
    markTransformBlockBoundary(ctx,x1,y0,log2TrafoSize-1,trafoDepth+1, DEBLOCK_FLAG_VERTI, filterTopCbEdge);
    markTransformBlockBoundary(ctx,x0,y1,log2TrafoSize-1,trafoDepth+1, filterLeftCbEdge,   DEBLOCK_FLAG_HORIZ);
    markTransformBlockBoundary(ctx,x1,y1,log2TrafoSize-1,trafoDepth+1, DEBLOCK_FLAG_VERTI, DEBLOCK_FLAG_HORIZ);
  }
  else {
    // VER

    for (int k=0;k<(1<<log2TrafoSize);k+=4) {
      set_deblk_flags(ctx, x0,y0+k, filterLeftCbEdge);
    }

    // HOR

    for (int k=0;k<(1<<log2TrafoSize);k+=4) {
      set_deblk_flags(ctx, x0+k,y0, filterTopCbEdge);
    }
  }
}



// 8.7.2.2 for both EDGE_HOR and EDGE_VER at the same time
void markPredictionBlockBoundary(decoder_context* ctx, int x0,int y0,
                                 int log2CbSize,
                                 int filterLeftCbEdge, int filterTopCbEdge)
{
  logtrace(LogDeblock,"markPredictionBlockBoundary(%d,%d, %d, %d,%d)\n",x0,y0,
           log2CbSize, filterLeftCbEdge,filterTopCbEdge);

  enum PartMode partMode = get_PartMode(ctx,x0,y0);

  int cbSize = 1<<log2CbSize;
  int cbSize2 = 1<<(log2CbSize-1);
  int cbSize4 = 1<<(log2CbSize-2);

  switch (partMode) {
  case PART_NxN:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+cbSize2,y0+k, DEBLOCK_PB_EDGE_VERTI);
      set_deblk_flags(ctx, x0+k,y0+cbSize2, DEBLOCK_PB_EDGE_HORIZ);
    }
    break;

  case PART_Nx2N:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+cbSize2,y0+k, DEBLOCK_PB_EDGE_VERTI);
    }
    break;

  case PART_2NxN:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+k,y0+cbSize2, DEBLOCK_PB_EDGE_HORIZ);
    }
    break;

  case PART_nLx2N:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+cbSize4,y0+k, DEBLOCK_PB_EDGE_VERTI);
    }
    break;

  case PART_nRx2N:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+cbSize2+cbSize4,y0+k, DEBLOCK_PB_EDGE_VERTI);
    }
    break;

  case PART_2NxnU:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+k,y0+cbSize4, DEBLOCK_PB_EDGE_HORIZ);
    }
    break;

  case PART_2NxnD:
    for (int k=0;k<cbSize;k++) {
      set_deblk_flags(ctx, x0+k,y0+cbSize2+cbSize4, DEBLOCK_PB_EDGE_HORIZ);
    }
    break;

  case PART_2Nx2N:
    // NOP
    break;
  }
}


char derive_edgeFlags(decoder_context* ctx)
{
  const int minCbSize = ctx->current_sps->MinCbSizeY;
  char deblocking_enabled=0; // whether deblocking is enabled in some part of the image

  for (int cb_y=0;cb_y<ctx->current_sps->PicHeightInMinCbsY;cb_y++)
    for (int cb_x=0;cb_x<ctx->current_sps->PicWidthInMinCbsY;cb_x++)
      {
        int log2CbSize = get_log2CbSize_cbUnits(ctx,cb_x,cb_y);
        if (log2CbSize==0) {
          continue;
        }

        // we are now at the top corner of a CB

        int x0 = cb_x * minCbSize;
        int y0 = cb_y * minCbSize;


        // check whether we should filter this slice

        slice_segment_header* shdr = get_SliceHeader(ctx,x0,y0);

        // check whether to filter left and top edge

        uint8_t filterLeftCbEdge = DEBLOCK_FLAG_VERTI;
        uint8_t filterTopCbEdge  = DEBLOCK_FLAG_HORIZ;
        if (x0 == 0) filterLeftCbEdge = 0;
        if (y0 == 0) filterTopCbEdge  = 0;

        // ... TODO: check for slice and tile boundaries (8.7.2, step 2 in both processes)

        if (shdr->slice_deblocking_filter_disabled_flag==0) {
          deblocking_enabled=1;

          markTransformBlockBoundary(ctx, x0,y0, log2CbSize,0,
                                     filterLeftCbEdge, filterTopCbEdge);

          markPredictionBlockBoundary(ctx, x0,y0, log2CbSize,
                                      filterLeftCbEdge, filterTopCbEdge);
        }
      }

  return deblocking_enabled;
}


void derive_boundaryStrength_CTB(decoder_context* ctx, bool vertical, int xCtb,int yCtb)
{
  int ctbSize = ctx->current_sps->CtbSizeY;
  int deblkSize = ctbSize/4;

  derive_boundaryStrength(ctx,vertical,
                          yCtb*deblkSize, (yCtb+1)*deblkSize,
                          xCtb*deblkSize, (xCtb+1)*deblkSize);
}

// 8.7.2.3 (both, EDGE_VER and EDGE_HOR)
void derive_boundaryStrength(decoder_context* ctx, bool vertical, int yStart,int yEnd,
                             int xStart,int xEnd)
{
  //int stride = ctx->img.stride; TODO: UNUSED
  int xIncr = vertical ? 2 : 1;
  int yIncr = vertical ? 1 : 2;
  int xOffs = vertical ? 1 : 0;
  int yOffs = vertical ? 0 : 1;
  int edgeMask = vertical ?
    (DEBLOCK_FLAG_VERTI | DEBLOCK_PB_EDGE_VERTI) :
    (DEBLOCK_FLAG_HORIZ | DEBLOCK_PB_EDGE_HORIZ);
  int transformEdgeMask = vertical ? DEBLOCK_FLAG_VERTI : DEBLOCK_FLAG_HORIZ;

  xEnd = min(xEnd,ctx->deblk_width);
  yEnd = min(yEnd,ctx->deblk_height);

  int TUShift = ctx->current_sps->Log2MinTrafoSize;
  int TUStride= ctx->current_sps->PicWidthInTbsY;

  for (int y=yStart;y<yEnd;y+=yIncr)
    for (int x=xStart;x<xEnd;x+=xIncr) {
      int xDi = x*4;
      int yDi = y*4;

      logtrace(LogDeblock,"%d %d %s = %s\n",xDi,yDi, vertical?"Vertical":"Horizontal",
               (get_deblk_flags(ctx,xDi,yDi) & edgeMask) ? "edge" : "...");

      uint8_t edgeFlags = get_deblk_flags(ctx,xDi,yDi);

      if (edgeFlags & edgeMask) {
        //int p0 = ctx->img.y[(xDi-xOffs)+(yDi-yOffs)*stride]; TODO: UNUSED
        //int q0 = ctx->img.y[xDi+yDi*stride];                 TODO: UNUSED

        bool p_is_intra_pred = (get_pred_mode(ctx, xDi-xOffs, yDi-yOffs) == MODE_INTRA);
        bool q_is_intra_pred = (get_pred_mode(ctx, xDi,       yDi      ) == MODE_INTRA);

        int bS;

        if (p_is_intra_pred || q_is_intra_pred) {
          bS = 2;
        }
        else {
          // opposing site
          int xDiOpp = xDi-xOffs;
          int yDiOpp = yDi-yOffs;
          //uint8_t edgeFlagsOpp = get_deblk_flags(ctx,xDiOpp,yDiOpp);

          /*
          if ((edgeFlags & transformEdgeMask) &&
              (get_nonzero_coefficient(ctx,xDi,yDi) ||
               get_nonzero_coefficient(ctx,xDiOpp,yDiOpp))) {
          */
          if ((edgeFlags & transformEdgeMask) &&
              (ctx->tu_info[(xDi   >>TUShift) + (yDi   >>TUShift)*TUStride].flags & TU_FLAG_NONZERO_COEFF ||
               ctx->tu_info[(xDiOpp>>TUShift) + (yDiOpp>>TUShift)*TUStride].flags & TU_FLAG_NONZERO_COEFF)) {
            bS = 1;
          }
          else {

            bS = 0;

            slice_segment_header* shdrP = get_SliceHeader(ctx,xDiOpp,yDiOpp);
            slice_segment_header* shdrQ = get_SliceHeader(ctx,xDi   ,yDi);

            const PredVectorInfo* mviP = get_mv_info(ctx,xDiOpp,yDiOpp);
            const PredVectorInfo* mviQ = get_mv_info(ctx,xDi   ,yDi);

            int refPicP0 = mviP->predFlag[0] ? shdrP->RefPicList[0][ mviP->refIdx[0] ] : -1;
            int refPicP1 = mviP->predFlag[1] ? shdrP->RefPicList[1][ mviP->refIdx[1] ] : -1;
            int refPicQ0 = mviQ->predFlag[0] ? shdrQ->RefPicList[0][ mviQ->refIdx[0] ] : -1;
            int refPicQ1 = mviQ->predFlag[1] ? shdrQ->RefPicList[1][ mviQ->refIdx[1] ] : -1;

            MotionVector mvP0 = mviP->mv[0]; if (!mviP->predFlag[0]) { mvP0.x=mvP0.y=0; }
            MotionVector mvP1 = mviP->mv[1]; if (!mviP->predFlag[1]) { mvP1.x=mvP1.y=0; }
            MotionVector mvQ0 = mviQ->mv[0]; if (!mviQ->predFlag[0]) { mvQ0.x=mvQ0.y=0; }
            MotionVector mvQ1 = mviQ->mv[1]; if (!mviQ->predFlag[1]) { mvQ1.x=mvQ1.y=0; }

            bool samePics = ((refPicP0==refPicQ0 && refPicP1==refPicQ1) ||
                             (refPicP0==refPicQ1 && refPicP1==refPicQ0));

            if (!samePics) {
              bS = 1;
            }
            else {
              int numMV_P = mviP->predFlag[0] + mviP->predFlag[1];
              int numMV_Q = mviQ->predFlag[0] + mviQ->predFlag[1];

              assert(numMV_P==numMV_Q);

              // two different reference pictures or only one reference picture
              if (refPicP0 != refPicP1) {

                if (refPicP0 == refPicQ0) {
                  if (abs_value(mvP0.x-mvQ0.x) >= 4 ||
                      abs_value(mvP0.y-mvQ0.y) >= 4 ||
                      abs_value(mvP1.x-mvQ1.x) >= 4 ||
                      abs_value(mvP1.y-mvQ1.y) >= 4) {
                    bS = 1;
                  }
                }
                else {
                  if (abs_value(mvP0.x-mvQ1.x) >= 4 ||
                      abs_value(mvP0.y-mvQ1.y) >= 4 ||
                      abs_value(mvP1.x-mvQ0.x) >= 4 ||
                      abs_value(mvP1.y-mvQ0.y) >= 4) {
                    bS = 1;
                  }
                }
              }
              else {
                assert(refPicQ0==refPicQ1);

                if ((abs_value(mvP0.x-mvQ0.x) >= 4 ||
                     abs_value(mvP0.y-mvQ0.y) >= 4 ||
                     abs_value(mvP1.x-mvQ1.x) >= 4 ||
                     abs_value(mvP1.y-mvQ1.y) >= 4)
                    &&
                    (abs_value(mvP0.x-mvQ1.x) >= 4 ||
                     abs_value(mvP0.y-mvQ1.y) >= 4 ||
                     abs_value(mvP1.x-mvQ0.x) >= 4 ||
                     abs_value(mvP1.y-mvQ0.y) >= 4)) {
                  bS = 1;
                }
              }
            }

            /*
            printf("unimplemented deblocking code for CU at %d;%d\n",xDi,yDi);

            logerror(LogDeblock, "unimplemented code reached (file %s, line %d)\n",
                     __FILE__, __LINE__);
            */
          }
        }

        set_deblk_bS(ctx,xDi,yDi, bS);
      }
      else {
        set_deblk_bS(ctx,xDi,yDi, 0);
      }
    }
}


static uint8_t table_8_23_beta[52] = {
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 7, 8,
   9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,
  38,40,42,44,46,48,50,52,54,56,58,60,62,64
};

static uint8_t table_8_23_tc[54] = {
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
   1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
   5, 5, 6, 6, 7, 8, 9,10,11,13,14,16,18,20,22,24
};


// 8.7.2.4
void edge_filtering_luma_CTB(decoder_context* ctx, bool vertical, int xCtb,int yCtb)
{
  int ctbSize = ctx->current_sps->CtbSizeY;
  int deblkSize = ctbSize/4;

  edge_filtering_luma(ctx,vertical,
                      yCtb*deblkSize, (yCtb+1)*deblkSize,
                      xCtb*deblkSize, (xCtb+1)*deblkSize);
}


// 8.7.2.4
void edge_filtering_luma(decoder_context* ctx, bool vertical,
                         int yStart,int yEnd, int xStart,int xEnd)
{
  //int minCbSize = ctx->current_sps->MinCbSizeY;
  int xIncr = vertical ? 2 : 1;
  int yIncr = vertical ? 1 : 2;

  const int stride = ctx->img->stride;

  //printf("-> %d %d\n",yStart,yEnd);

  int bitDepth_Y = ctx->current_sps->BitDepth_Y;

  xEnd = min(xEnd,ctx->deblk_width);
  yEnd = min(yEnd,ctx->deblk_height);

  for (int y=yStart;y<yEnd;y+=yIncr)
    for (int x=xStart;x<xEnd;x+=xIncr) {
      int xDi = x*4;
      int yDi = y*4;
      int bS = get_deblk_bS(ctx, xDi,yDi);

      logtrace(LogDeblock,"--- x:%d y:%d bS:%d---\n",xDi,yDi,bS);

      if (bS>0) {

        // 8.7.2.4.3

        uint8_t* ptr = ctx->img->y + stride*yDi + xDi;

        uint8_t q[4][4], p[4][4];
        for (int k=0;k<4;k++)
          for (int i=0;i<4;i++)
            {
              if (vertical) {
                q[k][i] = ptr[ i  +k*stride];
                p[k][i] = ptr[-i-1+k*stride];
              }
              else {
                q[k][i] = ptr[k + i   *stride];
                p[k][i] = ptr[k -(i+1)*stride];
              }
            }


#if 0
        for (int k=0;k<4;k++)
          {
            for (int i=0;i<4;i++)
              {
                printf("%02x ", p[k][3-i]);
              }

            printf("| ");

            for (int i=0;i<4;i++)
              {
                printf("%02x ", q[k][i]);
              }
            printf("\n");
          }
#endif


        int QP_Q = get_QPY(ctx, xDi,yDi);
        int QP_P = (vertical ? get_QPY(ctx, xDi-1,yDi) : get_QPY(ctx,xDi,yDi-1));
        int qP_L = (QP_Q+QP_P+1)>>1;

        logtrace(LogDeblock,"QP: %d & %d -> %d\n",QP_Q,QP_P,qP_L);

        int sliceIndexQ00 = get_SliceHeaderIndex(ctx,xDi,yDi);
        int beta_offset = ctx->slice[sliceIndexQ00].slice_beta_offset;
        int tc_offset   = ctx->slice[sliceIndexQ00].slice_tc_offset;

        int Q_beta = Clip3(0,51, qP_L + beta_offset);
        int betaPrime = table_8_23_beta[Q_beta];
        int beta = betaPrime * (1<<(bitDepth_Y - 8));

        int Q_tc = Clip3(0,53, qP_L + 2*(bS-1) + tc_offset);
        int tcPrime = table_8_23_tc[Q_tc];
        int tc = tcPrime * (1<<(bitDepth_Y - 8));

        logtrace(LogDeblock,"beta: %d (%d)  tc: %d (%d)\n",beta,beta_offset, tc,tc_offset);

        int dE=0, dEp=0, dEq=0;

        if (vertical || !vertical) {
          int dp0 = abs_value(p[0][2] - 2*p[0][1] + p[0][0]);
          int dp3 = abs_value(p[3][2] - 2*p[3][1] + p[3][0]);
          int dq0 = abs_value(q[0][2] - 2*q[0][1] + q[0][0]);
          int dq3 = abs_value(q[3][2] - 2*q[3][1] + q[3][0]);

          int dpq0 = dp0 + dq0;
          int dpq3 = dp3 + dq3;

          int dp = dp0 + dp3;
          int dq = dq0 + dq3;
          int d  = dpq0+ dpq3;

          if (d<beta) {
            //int dpq = 2*dpq0;
            bool dSam0 = (2*dpq0 < (beta>>2) &&
                          abs_value(p[0][3]-p[0][0])+abs_value(q[0][0]-q[0][3]) < (beta>>3) &&
                          abs_value(p[0][0]-q[0][0]) < ((5*tc+1)>>1));

            bool dSam3 = (2*dpq3 < (beta>>2) &&
                          abs_value(p[3][3]-p[3][0])+abs_value(q[3][0]-q[3][3]) < (beta>>3) &&
                          abs_value(p[3][0]-q[3][0]) < ((5*tc+1)>>1));

            if (dSam0 && dSam3) {
              dE=2;
            }
            else {
              dE=1;
            }

            if (dp < ((beta + (beta>>1))>>3)) { dEp=1; }
            if (dq < ((beta + (beta>>1))>>3)) { dEq=1; }

            logtrace(LogDeblock,"dE:%d dEp:%d dEq:%d\n",dE,dEp,dEq);
          }
        }
        else {
          // TODO
          assert(0);
        }


        // 8.7.2.4.4

        if (dE != 0) {
          for (int k=0;k<4;k++) {
            int nDp,nDq;

            logtrace(LogDeblock,"line:%d\n",k);

            const uint8_t p0 = p[k][0];
            const uint8_t p1 = p[k][1];
            const uint8_t p2 = p[k][2];
            const uint8_t p3 = p[k][3];
            const uint8_t q0 = q[k][0];
            const uint8_t q1 = q[k][1];
            const uint8_t q2 = q[k][2];
            const uint8_t q3 = q[k][3];

            if (dE==2) {
              // strong filtering

              nDp=nDq=3;

              uint8_t pnew[3],qnew[3];
              pnew[0] = Clip3(p0-2*tc,p0+2*tc, (p2 + 2*p1 + 2*p0 + 2*q0 + q1 +4)>>3);
              pnew[1] = Clip3(p1-2*tc,p1+2*tc, (p2 + p1 + p0 + q0+2)>>2);
              pnew[2] = Clip3(p2-2*tc,p2+2*tc, (2*p3 + 3*p2 + p1 + p0 + q0 + 4)>>3);
              qnew[0] = Clip3(q0-2*tc,q0+2*tc, (p1+2*p0+2*q0+2*q1+q2+4)>>3);
              qnew[1] = Clip3(q1-2*tc,q1+2*tc, (p0+q0+q1+q2+2)>>2);
              qnew[2] = Clip3(q2-2*tc,q2+2*tc, (p0+q0+q1+3*q2+2*q3+4)>>3);

              logtrace(LogDeblock,"strong filtering\n");

              if (vertical) {
                for (int i=0;i<3;i++) {
                  ptr[-i-1+k*stride] = pnew[i];
                  ptr[ i + k*stride] = qnew[i];
                }

                // ptr[-1+k*stride] = ptr[ 0+k*stride] = 200;
              }
              else {
                for (int i=0;i<3;i++) {
                  ptr[ k -(i+1)*stride] = pnew[i];
                  ptr[ k + i   *stride] = qnew[i];
                }
              }
            }
            else {
              // weak filtering

              nDp=nDq=0;

              int delta = (9*(q0-p0) - 3*(q1-p1) + 8)>>4;

              if (abs_value(delta) < tc*10) {

                delta = Clip3(-tc,tc,delta);
                logtrace(LogDeblock," delta:%d\n",delta);

                if (vertical) {
                  ptr[-0-1+k*stride] = Clip1_8bit(p0+delta);
                  ptr[ 0  +k*stride] = Clip1_8bit(q0-delta);
                }
                else {
                  ptr[ k -1*stride] = Clip1_8bit(p0+delta);
                  ptr[ k +0*stride] = Clip1_8bit(q0-delta);
                }

                //ptr[ 0+k*stride] = 200;

                if (dEp==1) {
                  int delta_p = Clip3(-(tc>>1), tc>>1, (((p2+p0+1)>>1)-p1+delta)>>1);
                  if (vertical) { ptr[-1-1+k*stride] = Clip1_8bit(p1+delta_p); }
                  else          { ptr[ k  -2*stride] = Clip1_8bit(p1+delta_p); }
                }

                if (dEq==1) {
                  int delta_q = Clip3(-(tc>>1), tc>>1, (((q2+q0+1)>>1)-q1-delta)>>1);
                  if (vertical) { ptr[ 1  +k*stride] = Clip1_8bit(q1+delta_q); }
                  else          { ptr[ k  +1*stride] = Clip1_8bit(q1+delta_q); }
                }

                nDp = dEp+1;
                nDq = dEq+1;

                logtrace(LogDeblock,"weak filtering (%d:%d)\n",nDp,nDq);
              }
            }
          }
        }
      }
    }
}


// 8.7.2.4
void edge_filtering_chroma_CTB(decoder_context* ctx, bool vertical, int xCtb,int yCtb)
{
  int ctbSize = ctx->current_sps->CtbSizeY;
  int deblkSize = ctbSize/4;

  edge_filtering_chroma(ctx,vertical,
                        yCtb*deblkSize, (yCtb+1)*deblkSize,
                        xCtb*deblkSize, (xCtb+1)*deblkSize);
}


void edge_filtering_chroma(decoder_context* ctx, bool vertical, int yStart,int yEnd,
                           int xStart,int xEnd)
{
  //int minCbSize = ctx->current_sps->MinCbSizeY;
  int xIncr = vertical ? 4 : 2;
  int yIncr = vertical ? 2 : 4;

  const int stride = ctx->img->chroma_stride;

  xEnd = min(xEnd,ctx->deblk_width);
  yEnd = min(yEnd,ctx->deblk_height);

  for (int y=yStart;y<yEnd;y+=yIncr)
    for (int x=xStart;x<xEnd;x+=xIncr) {
      int xDi = x*2;
      int yDi = y*2;
      int bS = get_deblk_bS(ctx, 2*xDi,2*yDi);

      if (bS>1) {
        // 8.7.2.4.5

        for (int cplane=0;cplane<2;cplane++) {
          int cQpPicOffset = (cplane==0 ?
                              ctx->current_pps->pic_cb_qp_offset :
                              ctx->current_pps->pic_cr_qp_offset);

          uint8_t* ptr = (cplane==0 ?
                          ctx->img->cb + stride*yDi + xDi :
                          ctx->img->cr + stride*yDi + xDi);

          uint8_t p[2][4];
          uint8_t q[2][4];

          logtrace(LogDeblock,"-%s- %d %d\n",cplane==0 ? "Cb" : "Cr",xDi,yDi);

          for (int i=0;i<2;i++)
            for (int k=0;k<4;k++)
              {
                if (vertical) {
                  q[i][k] = ptr[ i  +k*stride];
                  p[i][k] = ptr[-i-1+k*stride];
                }
                else {
                  q[i][k] = ptr[k + i   *stride];
                  p[i][k] = ptr[k -(i+1)*stride];
                }
              }

#if 0
        for (int k=0;k<4;k++)
          {
            for (int i=0;i<2;i++)
              {
                printf("%02x ", p[1-i][k]);
              }

            printf("| ");

            for (int i=0;i<2;i++)
              {
                printf("%02x ", q[i][k]);
              }
            printf("\n");
          }
#endif

          int QP_Q = get_QPY(ctx, 2*xDi,2*yDi);
          int QP_P = (vertical ? get_QPY(ctx, 2*xDi-1,2*yDi) : get_QPY(ctx,2*xDi,2*yDi-1));
          int qP_i = ((QP_Q+QP_P+1)>>1) + cQpPicOffset;
          int QP_C = table8_22(qP_i);

          logtrace(LogDeblock,"QP: %d & %d -> %d\n",QP_Q,QP_P,QP_C);

          int sliceIndexQ00 = get_SliceHeaderIndex(ctx,2*xDi,2*yDi);
          int tc_offset   = ctx->slice[sliceIndexQ00].slice_tc_offset;

          int Q = Clip3(0,53, QP_C + 2*(bS-1) + tc_offset);

          int tcPrime = table_8_23_tc[Q];
          int tc = tcPrime * (1<<(ctx->current_sps->BitDepth_C - 8));

          if (vertical) {
            for (int k=0;k<4;k++) {
              int delta = Clip3(-tc,tc, ((((q[0][k]-p[0][k])<<2)+p[1][k]-q[1][k]+4)>>3));
              logtrace(LogDeblock,"delta=%d\n",delta);
              ptr[-1+k*stride] = Clip1_8bit(p[0][k]+delta);
              ptr[ 0+k*stride] = Clip1_8bit(q[0][k]-delta);
            }
          }
          else {
            for (int k=0;k<4;k++) {
              int delta = Clip3(-tc,tc, ((((q[0][k]-p[0][k])<<2)+p[1][k]-q[1][k]+4)>>3));
              ptr[ k-1*stride] = Clip1_8bit(p[0][k]+delta);
              ptr[ k+0*stride] = Clip1_8bit(q[0][k]-delta);
            }
          }
        }
      }
    }
}


static void thread_deblock(void* d)
{
  struct thread_task_deblock* data = (struct thread_task_deblock*)d;
  struct decoder_context* ctx = data->ctx;

  int xStart=0;
  int xEnd = ctx->deblk_width;

  derive_boundaryStrength(ctx, data->vertical, data->first,data->last, xStart,xEnd);
  edge_filtering_luma    (ctx, data->vertical, data->first,data->last, xStart,xEnd);
  edge_filtering_chroma  (ctx, data->vertical, data->first,data->last, xStart,xEnd);
}


static void thread_deblock_ctb(void* d)
{
  struct thread_task_deblock* data = (struct thread_task_deblock*)d;
  struct decoder_context* ctx = data->ctx;

  derive_boundaryStrength_CTB(ctx, data->vertical, data->ctb_x,data->ctb_y);
  edge_filtering_luma_CTB    (ctx, data->vertical, data->ctb_x,data->ctb_y);
  edge_filtering_chroma_CTB  (ctx, data->vertical, data->ctb_x,data->ctb_y);
}

static void thread_deblock_ctb_row(void* d)
{
  struct thread_task_deblock* data = (struct thread_task_deblock*)d;
  struct decoder_context* ctx = data->ctx;

  for (int x=0;x<ctx->current_sps->PicWidthInCtbsY;x++) {
    derive_boundaryStrength_CTB(ctx, data->vertical, x,data->ctb_y);
    edge_filtering_luma_CTB    (ctx, data->vertical, x,data->ctb_y);
    edge_filtering_chroma_CTB  (ctx, data->vertical, x,data->ctb_y);
  }
}

static void thread_deblock_full_ctb_row(void* d)
{
  struct thread_task_deblock* data = (struct thread_task_deblock*)d;
  struct decoder_context* ctx = data->ctx;

  int ctbSize = ctx->current_sps->CtbSizeY;
  int deblkSize = ctbSize/4;

  int xStart=0;
  int xEnd = ctx->deblk_width;

  int yStart =  data->ctb_y   *deblkSize;
  int yEnd   = (data->ctb_y+1)*deblkSize;

  derive_boundaryStrength(ctx, data->vertical, yStart,yEnd, xStart,xEnd);
  edge_filtering_luma    (ctx, data->vertical, yStart,yEnd, xStart,xEnd);
  edge_filtering_chroma  (ctx, data->vertical, yStart,yEnd, xStart,xEnd);
}


void apply_deblocking_filter(decoder_context* ctx)
{
  //return;

  char enabled_deblocking = derive_edgeFlags(ctx);

  if (enabled_deblocking)
    {
      if (ctx->num_worker_threads==0) {

        // vertical filtering

        logtrace(LogDeblock,"VERTICAL\n");
        derive_boundaryStrength(ctx, true ,0,ctx->deblk_height,0,ctx->deblk_width);
        edge_filtering_luma    (ctx, true ,0,ctx->deblk_height,0,ctx->deblk_width);
        edge_filtering_chroma  (ctx, true ,0,ctx->deblk_height,0,ctx->deblk_width);

        // horizontal filtering

        logtrace(LogDeblock,"HORIZONTAL\n");
        derive_boundaryStrength(ctx, false ,0,ctx->deblk_height,0,ctx->deblk_width);
        edge_filtering_luma    (ctx, false ,0,ctx->deblk_height,0,ctx->deblk_width);
        edge_filtering_chroma  (ctx, false ,0,ctx->deblk_height,0,ctx->deblk_width);
      }
      else {

        flush_thread_pool(&ctx->thread_pool);

#if 1
        for (int pass=0;pass<2;pass++) {

          thread_task task;

          task.task_id = -1;
          task.task_cmd = THREAD_TASK_DEBLOCK;
          task.work_routine = thread_deblock;

          int numStripes= ctx->num_worker_threads * 4; // TODO: what is a good number of stripes?
          ctx->thread_pool.tasks_pending = numStripes;

          for (int i=0;i<numStripes;i++)
            {
              int ys = i*ctx->deblk_height/numStripes;
              int ye = (i+1)*ctx->deblk_height/numStripes;

              // required because multi-threading might cut odd strips
              ys &= ~3;
              if (i != numStripes-1) ye &= ~3;


              task.data.task_deblock.ctx   = ctx;
              task.data.task_deblock.first = ys;
              task.data.task_deblock.last  = ye;
              task.data.task_deblock.vertical = (pass==0);

              add_task(&ctx->thread_pool, &task);
            }

          flush_thread_pool(&ctx->thread_pool);
        }
#endif
#if 0
        for (int pass=0;pass<2;pass++)
          {
            thread_task task;

            task.task_id = -1;
            task.task_cmd = THREAD_TASK_DEBLOCK;
            task.work_routine = thread_deblock_ctb;

            ctx->thread_pool.tasks_pending = ctx->current_sps->PicSizeInCtbsY;

            for (int y=0;y<ctx->current_sps->PicHeightInCtbsY;y++)
              for (int x=0;x<ctx->current_sps->PicWidthInCtbsY;x++)
                {
                  task.data.task_deblock.ctx   = ctx;
                  task.data.task_deblock.ctb_x = x;
                  task.data.task_deblock.ctb_y = y;
                  task.data.task_deblock.vertical = (pass==0);
                
                  add_task(&ctx->thread_pool, &task);
                }
          
            flush_thread_pool(&ctx->thread_pool);
          }
#endif
#if 0
        for (int pass=0;pass<2;pass++)
          {
            thread_task task;

            task.task_id = -1;
            task.task_cmd = THREAD_TASK_DEBLOCK;
            task.work_routine = thread_deblock_ctb_row;

            ctx->thread_pool.tasks_pending = ctx->current_sps->PicHeightInCtbsY;

            for (int y=0;y<ctx->current_sps->PicHeightInCtbsY;y++)
              //for (int x=0;x<ctx->current_sps->PicWidthInCtbsY;x++)
                {
                  task.data.task_deblock.ctx   = ctx;
                  //task.data.task_deblock.ctb_x = x;
                  task.data.task_deblock.ctb_y = y;
                  task.data.task_deblock.vertical = (pass==0);
                
                  add_task(&ctx->thread_pool, &task);
                }
          
            flush_thread_pool(&ctx->thread_pool);
          }
#endif
#if 0
        for (int pass=0;pass<2;pass++)
          {
            thread_task task;

            task.task_id = -1;
            task.task_cmd = THREAD_TASK_DEBLOCK;
            task.work_routine = thread_deblock_full_ctb_row;

            ctx->thread_pool.tasks_pending = ctx->current_sps->PicHeightInCtbsY;

            for (int y=0;y<ctx->current_sps->PicHeightInCtbsY;y++)
              //for (int x=0;x<ctx->current_sps->PicWidthInCtbsY;x++)
                {
                  task.data.task_deblock.ctx   = ctx;
                  //task.data.task_deblock.ctb_x = x;
                  task.data.task_deblock.ctb_y = y;
                  task.data.task_deblock.vertical = (pass==0);
                
                  add_task(&ctx->thread_pool, &task);
                }
          
            flush_thread_pool(&ctx->thread_pool);
          }
#endif
      }
    }
}
