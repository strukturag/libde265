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
        }
      }

  return deblocking_enabled;
}


// 8.7.2.3 (both, EDGE_VER and EDGE_HOR)
void derive_boundaryStrength(decoder_context* ctx, bool vertical)
{
  //int stride = ctx->img.stride; TODO: UNUSED
  int xIncr = vertical ? 2 : 1;
  int yIncr = vertical ? 1 : 2;
  int xOffs = vertical ? 1 : 0;
  int yOffs = vertical ? 0 : 1;
  int edgeFlag = vertical ? DEBLOCK_FLAG_VERTI : DEBLOCK_FLAG_HORIZ;

  for (int y=0;y<ctx->deblk_height;y+=yIncr)
    for (int x=0;x<ctx->deblk_width; x+=xIncr) {
      int xDi = x*4;
      int yDi = y*4;

      logtrace(LogDeblock,"%d %d %s = %s\n",xDi,yDi, vertical?"Vertical":"Horizontal",
             (get_deblk_flags(ctx,xDi,yDi) & edgeFlag) ? "edge" : "...");

      if (get_deblk_flags(ctx,xDi,yDi) & edgeFlag) {
        //int p0 = ctx->img.y[(xDi-xOffs)+(yDi-yOffs)*stride]; TODO: UNUSED
        //int q0 = ctx->img.y[xDi+yDi*stride];                 TODO: UNUSED

        bool p_is_intra_pred = (get_pred_mode(ctx, xDi-xOffs, yDi-yOffs) == MODE_INTRA);
        bool q_is_intra_pred = (get_pred_mode(ctx, xDi,       yDi      ) == MODE_INTRA);

        int bS;

        if (p_is_intra_pred || q_is_intra_pred) {
          bS = 2;
        }
        else {
          assert(false); // TODO
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
void edge_filtering_luma(decoder_context* ctx, bool vertical)
{
  //int minCbSize = ctx->current_sps->MinCbSizeY;
  int xIncr = vertical ? 2 : 1;
  int yIncr = vertical ? 1 : 2;

  const int stride = ctx->img.stride;


  for (int y=0;y<ctx->deblk_height;y+=yIncr)
    for (int x=0;x<ctx->deblk_width; x+=xIncr) {
      int xDi = x*4;
      int yDi = y*4;
      int bS = get_deblk_bS(ctx, xDi,yDi);

      logtrace(LogDeblock,"--- x:%d y:%d bS:%d---\n",xDi,yDi,bS);

      if (bS>0) {

        // 8.7.2.4.3

        uint8_t* ptr = ctx->img.y + stride*yDi + xDi;

        uint8_t q[4][4], p[4][4];
        for (int i=0;i<4;i++)
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
            for (int i=0;i<4;i++)
              {
                printf("%02x ", p[3-i][k]);
              }

            printf("| ");

            for (int i=0;i<4;i++)
              {
                printf("%02x ", q[i][k]);
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
        int beta = betaPrime * (1<<(ctx->current_sps->BitDepth_Y - 8));

        int Q_tc = Clip3(0,53, qP_L + 2*(bS-1) + tc_offset);
        int tcPrime = table_8_23_tc[Q_tc];
        int tc = tcPrime * (1<<(ctx->current_sps->BitDepth_Y - 8));

        logtrace(LogDeblock,"beta: %d (%d)  tc: %d (%d)\n",beta,beta_offset, tc,tc_offset);

        int dE=0, dEp=0, dEq=0;

        if (vertical || !vertical) {
          int dp0 = abs_value(p[2][0] - 2*p[1][0] + p[0][0]);
          int dp3 = abs_value(p[2][3] - 2*p[1][3] + p[0][3]);
          int dq0 = abs_value(q[2][0] - 2*q[1][0] + q[0][0]);
          int dq3 = abs_value(q[2][3] - 2*q[1][3] + q[0][3]);

          int dpq0 = dp0 + dq0;
          int dpq3 = dp3 + dq3;

          int dp = dp0 + dp3;
          int dq = dq0 + dq3;
          int d  = dpq0+ dpq3;

          if (d<beta) {
            //int dpq = 2*dpq0;
            bool dSam0 = (2*dpq0 < (beta>>2) &&
                          abs_value(p[3][0]-p[0][0])+abs_value(q[0][0]-q[3][0]) < (beta>>3) &&
                          abs_value(p[0][0]-q[0][0]) < ((5*tc+1)>>1));

            bool dSam3 = (2*dpq3 < (beta>>2) &&
                          abs_value(p[3][3]-p[0][3])+abs_value(q[0][3]-q[3][3]) < (beta>>3) &&
                          abs_value(p[0][3]-q[0][3]) < ((5*tc+1)>>1));

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

            const uint8_t p0 = p[0][k];
            const uint8_t p1 = p[1][k];
            const uint8_t p2 = p[2][k];
            const uint8_t p3 = p[3][k];
            const uint8_t q0 = q[0][k];
            const uint8_t q1 = q[1][k];
            const uint8_t q2 = q[2][k];
            const uint8_t q3 = q[3][k];

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


void edge_filtering_chroma(decoder_context* ctx, bool vertical)
{
  //int minCbSize = ctx->current_sps->MinCbSizeY;
  int xIncr = vertical ? 4 : 2;
  int yIncr = vertical ? 2 : 4;

  const int stride = ctx->img.chroma_stride;


  for (int y=0;y<ctx->deblk_height;y+=yIncr)
    for (int x=0;x<ctx->deblk_width; x+=xIncr) {
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
                          ctx->img.cb + stride*yDi + xDi :
                          ctx->img.cr + stride*yDi + xDi);

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


void apply_deblocking_filter(decoder_context* ctx)
{
  //return;

  char enabled_deblocking = derive_edgeFlags(ctx);

  if (enabled_deblocking)
    {
      // vertical filtering

      logtrace(LogDeblock,"VERTICAL\n");
      derive_boundaryStrength(ctx, true);
      edge_filtering_luma(ctx, true);
      edge_filtering_chroma(ctx, true);

      // horizontal filtering

      logtrace(LogDeblock,"HORIZONTAL\n");
      derive_boundaryStrength(ctx, false);
      edge_filtering_luma(ctx, false);
      edge_filtering_chroma(ctx, false);
    }
}
