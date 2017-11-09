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

#include "sao.h"
#include "util.h"
#include "image-unit.h"

#include <stdlib.h>
#include <string.h>


/* Maximum SAO offsets:
      8 bit:  +/-  7
      9 bit:  +/- 15
   >=10 bit:  +/- 31
 */

template <class pixel_t>
void apply_sao_internal(image* img, int xCtb,int yCtb,
                        const slice_segment_header* shdr, int cIdx, int nSW,int nSH,
                        const pixel_t* in_img,  int in_stride,
                        /* */ pixel_t* out_img, int out_stride)
{
  const acceleration_functions& acceleration = img->decctx->acceleration;

  const sao_info* saoinfo = img->get_sao_info(xCtb,yCtb);

  int SaoTypeIdx = (saoinfo->SaoTypeIdx >> (2*cIdx)) & 0x3;

  logtrace(LogSAO,"apply_sao CTB %d;%d cIdx:%d type=%d (%dx%d)\n",xCtb,yCtb,cIdx, SaoTypeIdx, nSW,nSH);

  if (SaoTypeIdx==0) {
    return;
  }

  const seq_parameter_set* sps = &img->get_sps();
  const pic_parameter_set* pps = &img->get_pps();
  const int bitDepth = (cIdx==0 ? sps->BitDepth_Y : sps->BitDepth_C);
  const int maxPixelValue = (1<<bitDepth)-1;

  // top left position of CTB in pixels
  const int xC = xCtb*nSW;
  const int yC = yCtb*nSH;

  const int width  = img->get_width(cIdx);
  const int height = img->get_height(cIdx);

  const int ctbSliceAddrRS = img->get_SliceHeader(xC,yC)->SliceAddrRS;

  const int picWidthInCtbs = sps->PicWidthInCtbsY;
  const int chromashiftW = sps->get_chroma_shift_W(cIdx);
  const int chromashiftH = sps->get_chroma_shift_H(cIdx);
  const int ctbshiftW = sps->Log2CtbSizeY - chromashiftW;
  const int ctbshiftH = sps->Log2CtbSizeY - chromashiftH;


  for (int i=0;i<5;i++)
    {
      logtrace(LogSAO,"offset[%d] = %d\n", i, i==0 ? 0 : saoinfo->saoOffsetVal[cIdx][i-1]);
    }


  // actual size of CTB to be processed (can be smaller when partially outside of image)
  const int ctbW = (xC+nSW>width)  ? width -xC : nSW;
  const int ctbH = (yC+nSH>height) ? height-yC : nSH;


  const bool extendedTests = img->get_CTB_has_pcm_or_cu_transquant_bypass(xCtb,yCtb);

  if (SaoTypeIdx==2) {
    int hPos[2], vPos[2];
    int vPosStride[2]; // vPos[] multiplied by image stride
    int SaoEoClass = (saoinfo->SaoEoClass >> (2*cIdx)) & 0x3;

    switch (SaoEoClass) {
    case 0: hPos[0]=-1; hPos[1]= 1; vPos[0]= 0; vPos[1]=0; break;
    case 1: hPos[0]= 0; hPos[1]= 0; vPos[0]=-1; vPos[1]=1; break;
    case 2: hPos[0]=-1; hPos[1]= 1; vPos[0]=-1; vPos[1]=1; break;
    case 3: hPos[0]= 1; hPos[1]=-1; vPos[0]=-1; vPos[1]=1; break;
    }

    vPosStride[0] = vPos[0] * in_stride;
    vPosStride[1] = vPos[1] * in_stride;

    /* Reorder sao_info.saoOffsetVal[] array, so that we can index it
       directly with the sum of the two pixel-difference signs. */
    int8_t  saoOffsetVal[5]; // [2] unused
    saoOffsetVal[0] = saoinfo->saoOffsetVal[cIdx][1-1];
    saoOffsetVal[1] = saoinfo->saoOffsetVal[cIdx][2-1];
    saoOffsetVal[2] = 0;
    saoOffsetVal[3] = saoinfo->saoOffsetVal[cIdx][3-1];
    saoOffsetVal[4] = saoinfo->saoOffsetVal[cIdx][4-1];


    for (int j=0;j<ctbH;j++) {
      const pixel_t* in_ptr  = &in_img [xC+(yC+j)*in_stride];
      /* */ pixel_t* out_ptr = &out_img[xC+(yC+j)*out_stride];

      for (int i=0;i<ctbW;i++) {
        int edgeIdx = -1;

        logtrace(LogSAO, "pos %d,%d\n",xC+i,yC+j);

        if ((extendedTests &&
             (sps->pcm_loop_filter_disable_flag &&
              img->get_pcm_flag((xC+i)<<chromashiftW,(yC+j)<<chromashiftH))) ||
            img->get_cu_transquant_bypass((xC+i)<<chromashiftW,(yC+j)<<chromashiftH)) {
          continue;
        }

        // do the expensive test for boundaries only at the boundaries
        bool testBoundary = (i==0 || j==0 || i==ctbW-1 || j==ctbH-1);

        if (testBoundary)
          for (int k=0;k<2;k++) {
            int xS = xC+i+hPos[k];
            int yS = yC+j+vPos[k];

            if (xS<0 || yS<0 || xS>=width || yS>=height) {
              edgeIdx=0;
              break;
            }


            // This part seems inefficient with all the get_SliceHeaderIndex() calls,
            // but removing this part (because the input was known to have only a single
            // slice anyway) reduced computation time only by 1.3%.
            // TODO: however, this may still be a big part of SAO itself.

            slice_segment_header* sliceHeader = img->get_SliceHeader(xS<<chromashiftW,
                                                                     yS<<chromashiftH);
            if (sliceHeader==NULL) { return; }

            int sliceAddrRS = sliceHeader->SliceAddrRS;
            if (sliceAddrRS <  ctbSliceAddrRS &&
                img->get_SliceHeader((xC+i)<<chromashiftW,
                                     (yC+j)<<chromashiftH)->slice_loop_filter_across_slices_enabled_flag==0) {
              edgeIdx=0;
              break;
            }

            if (sliceAddrRS >  ctbSliceAddrRS &&
                img->get_SliceHeader(xS<<chromashiftW,
                                     yS<<chromashiftH)->slice_loop_filter_across_slices_enabled_flag==0) {
              edgeIdx=0;
              break;
            }


            if (pps->loop_filter_across_tiles_enabled_flag==0 &&
                pps->TileIdRS[(xS>>ctbshiftW) + (yS>>ctbshiftH)*picWidthInCtbs] !=
                pps->TileIdRS[(xC>>ctbshiftW) + (yC>>ctbshiftH)*picWidthInCtbs]) {
              edgeIdx=0;
              break;
            }
          }

        if (edgeIdx != 0) {

          edgeIdx = ( Sign(in_ptr[i] - in_ptr[i+hPos[0]+vPosStride[0]]) +
                      Sign(in_ptr[i] - in_ptr[i+hPos[1]+vPosStride[1]])   );

          int offset = saoOffsetVal[edgeIdx+2];

          out_ptr[i] = Clip3(0,maxPixelValue,
                             in_ptr[i] + offset);
        }
      }
    }
  }
  else {
    int bandShift = bitDepth-5;
    int saoLeftClass = saoinfo->sao_band_position[cIdx];
    logtrace(LogSAO,"saoLeftClass: %d\n",saoLeftClass);

    char bandOffset[32];
    memset(bandOffset, 0, sizeof(char)*32);

    for (int k=0;k<4;k++) {
      bandOffset[ (k+saoLeftClass)&31 ] = saoinfo->saoOffsetVal[cIdx][k];
    }


    /* If PCM or transquant_bypass is used in this CTB, we have to
       run all checks (A).
       Otherwise, we run a simplified version of the code (B).

       NOTE: this whole part of SAO does not seem to be a significant part of the time spent
    */

    if (extendedTests) {

      // (A) full version with all checks

      for (int j=0;j<ctbH;j++)
        for (int i=0;i<ctbW;i++) {

          if ((sps->pcm_loop_filter_disable_flag &&
               img->get_pcm_flag((xC+i)<<chromashiftW,(yC+j)<<chromashiftH)) ||
              img->get_cu_transquant_bypass((xC+i)<<chromashiftW,(yC+j)<<chromashiftH)) {
            continue;
          }

          int band = in_img[xC+i+(yC+j)*in_stride]>>bandShift;

          // Shifts are a strange thing. On x86, >>x actually computes >>(x%64).
          // So we have to take care of large bandShifts.
          //if (bandShift>=8) { band=0; }

          int offset = bandOffset[band];
          if (offset) {
            logtrace(LogSAO,"%d %d (%d) offset %d  %x -> %x\n",xC+i,yC+j,band,
                     offset,
                     in_img[xC+i+(yC+j)*in_stride],
                     in_img[xC+i+(yC+j)*in_stride]+offset);

            out_img[xC+i+(yC+j)*out_stride] = Clip3(0,maxPixelValue,
                                                    in_img[xC+i+(yC+j)*in_stride] + offset);
          }
        }
    }
    else
      {
        // (B) simplified version (only works if no PCM and transquant_bypass is active)

        if (sizeof(pixel_t)==1) {
          acceleration.sao_band_8((uint8_t*)&out_img[xC+yC*out_stride],out_stride,
                                  (uint8_t*)&in_img[xC+yC*in_stride],in_stride,
                                  ctbW,ctbH,
                                  saoLeftClass,
                                  saoinfo->saoOffsetVal[cIdx][0],
                                  saoinfo->saoOffsetVal[cIdx][1],
                                  saoinfo->saoOffsetVal[cIdx][2],
                                  saoinfo->saoOffsetVal[cIdx][3]);
        }
        else {

        for (int j=0;j<ctbH;j++)
          for (int i=0;i<ctbW;i++) {

            int band = in_img[xC+i+(yC+j)*in_stride]>>bandShift;
            // see above
            //if (bandShift>=8) { band=0; }

            int offset = bandOffset[band];
            if (offset) {
              out_img[xC+i+(yC+j)*out_stride] = Clip3(0,maxPixelValue,
                                                      in_img[xC+i+(yC+j)*in_stride] + offset);
            }
          }
        }
      }
  }
}


template <class pixel_t>
void apply_sao(image* img, int xCtb,int yCtb,
               const slice_segment_header* shdr, int cIdx, int nSW,int nSH,
               const pixel_t* in_img,  int in_stride,
               /* */ pixel_t* out_img, int out_stride)
{
  if (img->high_bit_depth(cIdx)) {
    apply_sao_internal<uint16_t>(img,xCtb,yCtb, shdr,cIdx,nSW,nSH,
                                 (uint16_t*)in_img, in_stride,
                                 (uint16_t*)out_img,out_stride);
  }
  else {
    apply_sao_internal<uint8_t>(img,xCtb,yCtb, shdr,cIdx,nSW,nSH,
                                in_img, in_stride,
                                out_img,out_stride);
  }
}


void apply_sample_adaptive_offset(image* img)
{
  const seq_parameter_set& sps = img->get_sps();

  if (sps.sample_adaptive_offset_enabled_flag==0) {
    return;
  }

  image inputCopy;
  de265_error err = inputCopy.copy_image(img);
  if (err != DE265_OK) {
    img->decctx->add_warning(DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY,false);
    return;
  }

  for (int yCtb=0; yCtb<sps.PicHeightInCtbsY; yCtb++)
    for (int xCtb=0; xCtb<sps.PicWidthInCtbsY; xCtb++)
      {
        const slice_segment_header* shdr = img->get_SliceHeaderCtb(xCtb,yCtb);

        if (shdr->slice_sao_luma_flag) {
          apply_sao(img, xCtb,yCtb, shdr, 0, 1<<sps.Log2CtbSizeY, 1<<sps.Log2CtbSizeY,
                    inputCopy.get_image_plane(0), inputCopy.get_image_stride(0),
                    img->get_image_plane(0), img->get_image_stride(0));
        }

        if (shdr->slice_sao_chroma_flag) {
          int nSW = (1<<sps.Log2CtbSizeY) / sps.SubWidthC;
          int nSH = (1<<sps.Log2CtbSizeY) / sps.SubHeightC;

          apply_sao(img, xCtb,yCtb, shdr, 1, nSW,nSH,
                    inputCopy.get_image_plane(1), inputCopy.get_image_stride(1),
                    img->get_image_plane(1), img->get_image_stride(1));

          apply_sao(img, xCtb,yCtb, shdr, 2, nSW,nSH,
                    inputCopy.get_image_plane(2), inputCopy.get_image_stride(2),
                    img->get_image_plane(2), img->get_image_stride(2));
        }
      }
}


void apply_sample_adaptive_offset_sequential(image* img)
{
  const seq_parameter_set& sps = img->get_sps();

  if (sps.sample_adaptive_offset_enabled_flag==0) {
    return;
  }

  int lumaImageSize   = img->get_image_stride(0) * img->get_height(0) * img->get_bytes_per_pixel(0);
  int chromaImageSize = img->get_image_stride(1) * img->get_height(1) * img->get_bytes_per_pixel(1);

  uint8_t* inputCopy = new uint8_t[ libde265_max(lumaImageSize, chromaImageSize) ];
  if (inputCopy == NULL) {
    img->decctx->add_warning(DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY,false);
    return;
  }


  int nChannels = 3;
  if (sps.ChromaArrayType == de265_chroma_mono) { nChannels=1; }

  for (int cIdx=0;cIdx<nChannels;cIdx++) {

    int stride = img->get_image_stride(cIdx);
    int height = img->get_height(cIdx);

    memcpy(inputCopy, img->get_image_plane(cIdx), stride * height * img->get_bytes_per_pixel(cIdx));

    for (int yCtb=0; yCtb<sps.PicHeightInCtbsY; yCtb++)
      for (int xCtb=0; xCtb<sps.PicWidthInCtbsY; xCtb++)
        {
          const slice_segment_header* shdr = img->get_SliceHeaderCtb(xCtb,yCtb);
          if (shdr==NULL) { return; }

          if (cIdx==0 && shdr->slice_sao_luma_flag) {
            apply_sao(img, xCtb,yCtb, shdr, 0, 1<<sps.Log2CtbSizeY, 1<<sps.Log2CtbSizeY,
                      inputCopy, stride,
                      img->get_image_plane(0), img->get_image_stride(0));
          }

          if (cIdx!=0 && shdr->slice_sao_chroma_flag) {
            int nSW = (1<<sps.Log2CtbSizeY) / sps.SubWidthC;
            int nSH = (1<<sps.Log2CtbSizeY) / sps.SubHeightC;

            apply_sao(img, xCtb,yCtb, shdr, cIdx, nSW,nSH,
                      inputCopy, stride,
                      img->get_image_plane(cIdx), img->get_image_stride(cIdx));
          }
        }
  }

  delete[] inputCopy;
}




class thread_task_sao : public thread_task
{
public:
  int  ctb_y;
  image* img; /* this is where we get the SPS from
                 (either inputImg or tmpImg can be a dummy image)
              */

  image* inputImg;
  image* tmpImg;
  int inputProgress;

  virtual void work();
  virtual std::string name() const {
    char buf[100];
    sprintf(buf,"sao-%d",ctb_y);
    return buf;
  }
};


void thread_task_sao::work()
{
  //img->thread_run(this);

  const seq_parameter_set& sps = img->get_sps();

  const int rightCtb = sps.PicWidthInCtbsY-1;
  const int ctbSize  = (1<<sps.Log2CtbSizeY);


#if D_MT
  printf("========================================================== SAO %d\n",ctb_y);
#endif

  // copy input image to output for this CTB-row

  img->progress().wait_for_progress(rightCtb,ctb_y,  inputProgress);
  tmpImg->copy_lines_from(inputImg, ctb_y * ctbSize, (ctb_y+1) * ctbSize);

  // mark SAO progress

  for (int x=0;x<=rightCtb;x++) {
    const int CtbWidth = sps.PicWidthInCtbsY;
    img->progress().set_CTB_progress(x,ctb_y,CTB_PROGRESS_SAO_INTERNAL);
  }


  // wait until also the CTB-rows below and above are ready

  if (ctb_y>0) {
    img->progress().wait_for_progress(rightCtb,ctb_y-1, CTB_PROGRESS_SAO_INTERNAL);
  }

  if (ctb_y+1<sps.PicHeightInCtbsY) {
    img->progress().wait_for_progress(rightCtb,ctb_y+1, CTB_PROGRESS_SAO_INTERNAL);
  }



  // process SAO in the CTB-row

  for (int xCtb=0; xCtb<sps.PicWidthInCtbsY; xCtb++)
    {
      const slice_segment_header* shdr = img->get_SliceHeaderCtb(xCtb,ctb_y);
      if (shdr==NULL) {
        break;
      }

      if (shdr->slice_sao_luma_flag) {
        apply_sao(img, xCtb,ctb_y, shdr, 0, ctbSize, ctbSize,
                  tmpImg->get_image_plane(0), tmpImg->get_image_stride(0),
                  inputImg ->get_image_plane(0), inputImg ->get_image_stride(0));
      }

      if (shdr->slice_sao_chroma_flag) {
        int nSW = ctbSize / sps.SubWidthC;
        int nSH = ctbSize / sps.SubHeightC;

        apply_sao(img, xCtb,ctb_y, shdr, 1, nSW,nSH,
                  tmpImg->get_image_plane(1), tmpImg->get_image_stride(1),
                  inputImg ->get_image_plane(1), inputImg ->get_image_stride(1));

        apply_sao(img, xCtb,ctb_y, shdr, 2, nSW,nSH,
                  tmpImg->get_image_plane(2), tmpImg->get_image_stride(2),
                  inputImg ->get_image_plane(2), inputImg ->get_image_stride(2));
      }
    }


  // mark SAO progress

  for (int x=0;x<=rightCtb;x++) {
    img->progress().set_CTB_progress(x,ctb_y,CTB_PROGRESS_SAO);
  }
}



class thread_task_sao_image : public thread_task
{
public:
  image* img; /* this is where we get the SPS from
                 (either inputImg or tmpImg can be a dummy image)
              */

  image* inputImg;
  image* tmpImg;
  int inputProgress;

  virtual void work();
  virtual std::string name() const {
    char buf[100];
    sprintf(buf,"sao-image");
    return buf;
  }
};


void thread_task_sao_image::work()
{
#if D_TIMER
  debug_timer timer;
  timer.start();
#endif

  const seq_parameter_set& sps = img->get_sps();

  const int rightCtb = sps.PicWidthInCtbsY-1;
  const int ctbSize  = (1<<sps.Log2CtbSizeY);


  // wait until also the CTB-rows below and above are ready

  img->progress().wait_for_progress(rightCtb,0,  inputProgress);
  tmpImg->copy_lines_from(inputImg, 0 * ctbSize, (0+1) * ctbSize);

  for (int ctb_y=0; ctb_y < sps.PicHeightInCtbsY; ctb_y++)
    {
      if (ctb_y+1<sps.PicHeightInCtbsY) {
        img->progress().wait_for_progress(rightCtb,ctb_y+1, inputProgress);
        tmpImg->copy_lines_from(inputImg, (ctb_y+1) * ctbSize, (ctb_y+1+1) * ctbSize);
      }

#if D_MT
      printf("========================================================== SAO POC=%d %d\n",
             img->PicOrderCntVal, ctb_y);
#endif

      // copy input image to output for this CTB-row

      //tmpImg->copy_lines_from(inputImg, ctb_y * ctbSize, (ctb_y+1) * ctbSize);


      // process SAO in the CTB-row

      for (int xCtb=0; xCtb<sps.PicWidthInCtbsY; xCtb++)
        {
          const slice_segment_header* shdr = img->get_SliceHeaderCtb(xCtb,ctb_y);
          if (shdr==NULL) {
            break;
          }

          if (shdr->slice_sao_luma_flag) {
            apply_sao(img, xCtb,ctb_y, shdr, 0, ctbSize, ctbSize,
                      tmpImg->get_image_plane(0), tmpImg->get_image_stride(0),
                      inputImg ->get_image_plane(0), inputImg ->get_image_stride(0));
          }

          if (shdr->slice_sao_chroma_flag) {
            int nSW = ctbSize / sps.SubWidthC;
            int nSH = ctbSize / sps.SubHeightC;

            apply_sao(img, xCtb,ctb_y, shdr, 1, nSW,nSH,
                      tmpImg->get_image_plane(1), tmpImg->get_image_stride(1),
                      inputImg ->get_image_plane(1), inputImg ->get_image_stride(1));

            apply_sao(img, xCtb,ctb_y, shdr, 2, nSW,nSH,
                      tmpImg->get_image_plane(2), tmpImg->get_image_stride(2),
                      inputImg ->get_image_plane(2), inputImg ->get_image_stride(2));
          }
        }


      // mark SAO progress

      for (int x=0;x<=rightCtb;x++) {
        const int CtbWidth = sps.PicWidthInCtbsY;
        img->progress().set_CTB_progress(x,ctb_y,CTB_PROGRESS_SAO);
      }
    }

#if D_TIMER
  timer.stop();
  printf("sao: %f\n", timer.get_usecs());
#endif
}




bool add_sao_tasks(image_unit* imgunit, int saoInputProgress)
{
  image* img = imgunit->img.get();
  const seq_parameter_set& sps = img->get_sps();

  if (sps.sample_adaptive_offset_enabled_flag==0) {
    return false;
  }


  decoder_context* ctx = img->decctx;

  de265_error err = imgunit->sao_tmp_img.alloc_image(img->get_width(), img->get_height(),
                                                     img->get_chroma_format(),
                                                     img->get_bit_depth(0),
                                                     img->get_bit_depth(1),
                                                     img->pts,
                                                     img->get_supplementary_data(),
                                                     img->user_data,
                                                     nullptr);
  if (err != DE265_OK) {
    img->decctx->add_warning(DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY,false);
    return false;
  }

  imgunit->sao_tmp_img.set_decoder_context(img->decctx);
  imgunit->sao_tmp_img.set_encoder_context(img->encctx);

  bool row_parallel = false;

  if (row_parallel) {
    for (int y=0;y<sps.PicHeightInCtbsY;y++)
      {
        auto task = std::make_shared<thread_task_sao>();

        task->inputImg  = img;
        task->tmpImg = &imgunit->sao_tmp_img;
        task->img = img;
        task->ctb_y = y;
        task->inputProgress = saoInputProgress;

        imgunit->tasks.push_back(task);
        ctx->get_thread_pool().add_task(task);
      }
  }
  else {
    auto task = std::make_shared<thread_task_sao_image>();

    task->inputImg  = img;
    task->tmpImg = &imgunit->sao_tmp_img;
    task->img = img;
    task->inputProgress = saoInputProgress;

    imgunit->tasks.push_back(task);
    ctx->get_thread_pool().add_task(task);
  }

  return true;
}
