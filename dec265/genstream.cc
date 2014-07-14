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


#include "libde265/nal-parser.h"
#include "libde265/decctx.h"
#include "libde265/encode.h"
#include "libde265/slice.h"
#include "libde265/scan.h"
#include "libde265/intrapred.h"
#include "libde265/transform.h"
#include "libde265/fallback-dct.h"
#include "libde265/quality.h"
#include "libde265/fallback.h"
#include <assert.h>

error_queue errqueue;
acceleration_functions accel;

video_parameter_set vps;
seq_parameter_set   sps;
pic_parameter_set   pps;
slice_segment_header shdr;

CABAC_encoder_bitstream writer;

de265_image img;
encoder_context ectx;


void encode_image_coeffTest_1()
{
  int w = sps.pic_width_in_luma_samples;
  int h = sps.pic_height_in_luma_samples;

  img.alloc_image(w,h, de265_chroma_420, &sps, true, NULL /* no decctx */);
  img.alloc_encoder_data(&sps);

  initialize_CABAC_models(ectx.ctx_model, shdr.initType, shdr.SliceQPY);

  int Log2CtbSize = sps.Log2CtbSizeY;


  // encode CTB by CTB

  for (int y=0;y<sps.PicHeightInCtbsY;y++)
    for (int x=0;x<sps.PicWidthInCtbsY;x++)
      {
        enc_cb* cb = ectx.enc_cb_pool.get_new();
        cb->split_cu_flag = false;

        cb->cu_transquant_bypass_flag = false;
        cb->PredMode = MODE_INTRA;
        cb->PartMode = PART_2Nx2N;

        enc_pb_intra* pb = ectx.enc_pb_intra_pool.get_new();
        cb->intra_pb[0] = pb;
        pb->pred_mode = INTRA_DC;
        pb->pred_mode_chroma = INTRA_DC;


        enc_tb* tb = ectx.enc_tb_pool.get_new();
        cb->transform_tree = tb;

        tb->parent = NULL;
        tb->split_transform_flag = false;
        tb->cbf_luma = true;
        tb->cbf_cb   = false;
        tb->cbf_cr   = false;

        int16_t coeff[16*16];
        memset(coeff,0,16*16*sizeof(int16_t));
        coeff[0] = 1; //((x+y)/*&1*/) * 5 - 101;
        //coeff[2+1*16] = 25;
        //coeff[3+4*16] = 30;
        if (x%2==0 && y%2==0)
          coeff[((x/2)%16) + ((y/2)%16)*16] = 100;

        //if (x+y==0) coeff[0]=-80;
        tb->coeff[0] = coeff;

        cb->write_to_image(&img, x<<Log2CtbSize, y<<Log2CtbSize, Log2CtbSize, true);
        encode_ctb(&ectx, cb, x,y);

        int last = (y==sps.PicHeightInCtbsY-1 &&
                    x==sps.PicWidthInCtbsY-1);

        //printf("wrote CTB at %d;%d\n",x*16,y*16);
        //printf("write term bit: %d\n",last);
        writer.write_CABAC_term_bit(last);


        ectx.enc_cb_pool.free_all();
        ectx.enc_tb_pool.free_all();
        ectx.enc_pb_intra_pool.free_all();
      }
}


void encode_image_FDCT_1()
{
  FILE* fh = fopen("paris_cif.yuv","rb");
  uint8_t* input[3];
  input[0] = (uint8_t*)malloc(352*288);
  input[1] = (uint8_t*)malloc(352*288/4);
  input[2] = (uint8_t*)malloc(352*288/4);
  fread(input[0],352,288,fh);
  fread(input[1],352,288/4,fh);
  fread(input[2],352,288/4,fh);
  fclose(fh);
  int stride = 352;

  int w = sps.pic_width_in_luma_samples;
  int h = sps.pic_height_in_luma_samples;

  img.alloc_image(w,h, de265_chroma_420, &sps, true, NULL /* no decctx */);
  img.alloc_encoder_data(&sps);

  initialize_CABAC_models(ectx.ctx_model, shdr.initType, shdr.SliceQPY);

  int Log2CtbSize = sps.Log2CtbSizeY;


  // encode CTB by CTB

  for (int y=0;y<sps.PicHeightInCtbsY;y++)
    for (int x=0;x<sps.PicWidthInCtbsY;x++)
      {
        enc_cb* cb = ectx.enc_cb_pool.get_new();
        cb->split_cu_flag = false;

        cb->cu_transquant_bypass_flag = false;
        cb->PredMode = MODE_INTRA;
        cb->PartMode = PART_2Nx2N;

        enc_pb_intra* pb = ectx.enc_pb_intra_pool.get_new();
        cb->intra_pb[0] = pb;
        pb->pred_mode = INTRA_DC;
        pb->pred_mode_chroma = INTRA_DC;


        enc_tb* tb = ectx.enc_tb_pool.get_new();
        cb->transform_tree = tb;

        tb->parent = NULL;
        tb->split_transform_flag = false;
        tb->cbf_luma = true;
        tb->cbf_cb   = true;
        tb->cbf_cr   = true;

        int16_t coeff_luma[16*16], coeff_cb[8*8], coeff_cr[8*8];
        /*
        fdct_16x16_8_fallback(coeff_luma, &input[0][(y<<Log2CtbSize)*stride + (x<<Log2CtbSize)], stride);
        fdct_8x8_8_fallback(coeff_cb, &input[1][(y<<Log2CtbSize)*stride/4 + (x<<Log2CtbSize)/2], stride/2);
        fdct_8x8_8_fallback(coeff_cr, &input[2][(y<<Log2CtbSize)*stride/4 + (x<<Log2CtbSize)/2], stride/2);
        */
        coeff_luma[0]=-1;
        coeff_cb[0] -= 16;
        coeff_cr[0] -= 16;
        if (coeff_cb[0]==0) coeff_cb[0]=((x+y)&1 ? -1 : 1);
        if (coeff_cr[0]==0) coeff_cr[0]=((x+y)&1 ? -1 : 1);
        //if (x==0 && y==0) coeff_cb[0]=coeff_cr[0]=-16;
        tb->coeff[0] = coeff_luma;
        tb->coeff[1] = coeff_cb;
        tb->coeff[2] = coeff_cr;

        cb->write_to_image(&img, x<<Log2CtbSize, y<<Log2CtbSize, Log2CtbSize, true);
        encode_ctb(&ectx, cb, x,y);

        int last = (y==sps.PicHeightInCtbsY-1 &&
                    x==sps.PicWidthInCtbsY-1);

        //printf("wrote CTB at %d;%d\n",x*16,y*16);
        //printf("write term bit: %d\n",last);
        writer.write_CABAC_term_bit(last);


        ectx.enc_cb_pool.free_all();
        ectx.enc_tb_pool.free_all();
        ectx.enc_pb_intra_pool.free_all();
      }
}


bool coeffzero(const int16_t* c,int n)
{
  for (int i=0;i<n;i++)
    if (c[i]) return false;

  return true;
}


void printBlk(int16_t* data, int blksize, int stride)
{
  for (int y=0;y<blksize;y++) {
    logtrace(LogTransform,"  ");
    for (int x=0;x<blksize;x++) {
      logtrace(LogTransform,"*%3d ", data[x+y*stride]);
    }
    logtrace(LogTransform,"*\n");
  }
}


enum IntraPredMode find_best_intra_mode(de265_image& img,int x0,int y0, int blkSize, int cIdx,
                                        const uint8_t* ref, int stride)
{
  enum IntraPredMode best_mode;
  int min_sad=-1;

  int candidates[3];

  const seq_parameter_set* sps = &img.sps;


  fillIntraPredModeCandidates(candidates, x0,y0,
                              sps->getPUIndexRS(x0,y0),
                              x0>0, y0>0, &img);

  // --- test candidates first ---

  for (int idx=0;idx<3;idx++) {
    enum IntraPredMode mode = (enum IntraPredMode)candidates[idx];
    decode_intra_prediction(&img, x0,y0, (enum IntraPredMode)mode, blkSize, cIdx);

    // measure SAD

    int sad=0;
    int imgStride = img.get_image_stride(cIdx);
    uint8_t* pred = img.get_image_plane(cIdx) + x0 + y0*imgStride;
    for (int y=0;y<blkSize;y++)
      for (int x=0;x<blkSize;x++)
        {
          int diff = ref[x + y*stride] - pred[x + y*imgStride];
          sad += abs_value(diff);
        }

    sad *= 0.5;

    if (mode==0 || sad<min_sad) {
      min_sad = sad;
      best_mode = (enum IntraPredMode)mode;
    }
  }


  // --- test all modes ---

  for (int idx=0;idx<35;idx++) {
    enum IntraPredMode mode = (enum IntraPredMode)idx; //candidates[idx];
    decode_intra_prediction(&img, x0,y0, (enum IntraPredMode)mode, blkSize, cIdx);

    // measure SAD

    int sad=0;
    int imgStride = img.get_image_stride(cIdx);
    uint8_t* pred = img.get_image_plane(cIdx) + x0 + y0*imgStride;
    for (int y=0;y<blkSize;y++)
      for (int x=0;x<blkSize;x++)
        {
          int diff = ref[x + y*stride] - pred[x + y*imgStride];
          sad += abs_value(diff);
        }

    if (min_sad<0 || sad<min_sad) {
      min_sad = sad;
      best_mode = (enum IntraPredMode)mode;
    }
  }

  // printf("%d;%d -> %d\n",x0,y0,best_mode);

  return best_mode;
}


void diff_blk(int16_t* out,int out_stride,
              const uint8_t* a_ptr, int a_stride,
              const uint8_t* b_ptr, int b_stride,
              int blkSize)
{
  for (int by=0;by<blkSize;by++)
    for (int bx=0;bx<blkSize;bx++)
      {
        out[by*out_stride+bx] = a_ptr[by*a_stride+bx] - b_ptr[by*b_stride+bx];
      }
}


double encode_image_FDCT_2(uint8_t const*const input[3],int width,int height, int qp)
{
  int stride=width;

  int w = sps.pic_width_in_luma_samples;
  int h = sps.pic_height_in_luma_samples;

  img.alloc_image(w,h, de265_chroma_420, &sps, true, NULL /* no decctx */);
  img.alloc_encoder_data(&sps);

  initialize_CABAC_models(ectx.ctx_model, shdr.initType, shdr.SliceQPY);

  int Log2CtbSize = sps.Log2CtbSizeY;

  uint8_t* luma_plane = img.get_image_plane(0);
  uint8_t* cb_plane = img.get_image_plane(1);
  uint8_t* cr_plane = img.get_image_plane(2);


  // encode CTB by CTB

  for (int y=0;y<sps.PicHeightInCtbsY;y++)
    for (int x=0;x<sps.PicWidthInCtbsY;x++)
      {
        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;

        logtrace(LogSlice,"encode CTB at %d %d\n",x0,y0);

        enc_cb* cb = ectx.enc_cb_pool.get_new();
        cb->split_cu_flag = false;

        cb->cu_transquant_bypass_flag = false;
        cb->PredMode = MODE_INTRA;
        cb->PartMode = PART_2Nx2N;

        enc_pb_intra* pb = ectx.enc_pb_intra_pool.get_new();
        cb->intra_pb[0] = pb;

        enum IntraPredMode intraMode = find_best_intra_mode(img,x0,y0, 16,0,
                                                            &input[0][y0*stride+x0], stride);

        pb->pred_mode = INTRA_PLANAR;
        pb->pred_mode_chroma = INTRA_PLANAR;

        pb->pred_mode = intraMode;
        pb->pred_mode_chroma = intraMode;


        enc_tb* tb = ectx.enc_tb_pool.get_new();
        cb->transform_tree = tb;

        tb->parent = NULL;
        tb->split_transform_flag = false;


        pb->do_intra_prediction(&img,x0,y0, 4 /* log2blksize */);

        // subtract intra-prediction from input

        int16_t blk[3][16*16];
        diff_blk(blk[0],16,
                 &input[0][y0*stride+x0],stride,
                 &luma_plane[y0*stride+x0],stride, 16);
        diff_blk(blk[1],8,
                 &input[1][y0/2*stride/2+x0/2],stride/2,
                 &cb_plane[y0/2*stride/2+x0/2],stride/2, 8);
        diff_blk(blk[2],8,
                 &input[2][y0/2*stride/2+x0/2],stride/2,
                 &cr_plane[y0/2*stride/2+x0/2],stride/2, 8);

        int16_t coeff_luma[16*16], coeff_cb[8*8], coeff_cr[8*8];
        tb->coeff[0] = coeff_luma;
        tb->coeff[1] = coeff_cb;
        tb->coeff[2] = coeff_cr;

        fdct_16x16_8_fallback(coeff_luma, blk[0],16);
        fdct_8x8_8_fallback(coeff_cb, blk[1],8);
        fdct_8x8_8_fallback(coeff_cr, blk[2],8);

        logtrace(LogTransform,"raw DCT coefficients:\n");
        printBlk(coeff_luma,16,16);

        quant_coefficients(coeff_luma, coeff_luma, 4, qp, true);
        quant_coefficients(coeff_cb,   coeff_cb,   3, qp, true);
        quant_coefficients(coeff_cr,   coeff_cr,   3, qp, true);

        logtrace(LogTransform,"quantized DCT coefficients:\n");
        printBlk(coeff_luma,16,16);

        tb->set_cbf_flags_from_coefficients(4 /* log2BlkSize */);

        cb->write_to_image(&img, x<<Log2CtbSize, y<<Log2CtbSize, Log2CtbSize, true);
        encode_ctb(&ectx, cb, x,y);


        // decode into image

        //printf("reconstruct %d/%d\n",x0,y0);

        tb->dequant_and_add_transform(&accel, &img, x0,y0, 4 /* blksize */, qp);

        if (0) {
          printf("dequant luma:\n");
          printBlk(coeff_luma, 16,16);
          printf("dequant cb:\n");
          printBlk(coeff_cb, 8,8);
          printf("dequant cr:\n");
          printBlk(coeff_cr, 8,8);
        }

#if 0
        printf("decoded pixels:\n");
        for (int y=0;y<16;y++,printf("\n"))
          for (int x=0;x<16;x++) {
            printf("%02x ",luma_plane[(y0+y)*stride+x0+x]);
          }
#endif

        int last = (y==sps.PicHeightInCtbsY-1 &&
                    x==sps.PicWidthInCtbsY-1);

        //printf("wrote CTB at %d;%d\n",x*16,y*16);
        //printf("write term bit: %d\n",last);
        writer.write_CABAC_term_bit(last);


        ectx.enc_cb_pool.free_all();
        ectx.enc_tb_pool.free_all();
        ectx.enc_pb_intra_pool.free_all();
      }


  double psnr = PSNR(MSE(input[0], width,
                         luma_plane, img.get_image_stride(0),
                         width, height));
  return psnr;
}


enc_cb* encode_cb_no_split(uint8_t const*const input[3],int stride,
                           int x0,int y0, int log2CbSize, int qp)
{
  printf("encode at %d %d, size %d\n",x0,y0,1<<log2CbSize);

  uint8_t* luma_plane = img.get_image_plane(0);
  uint8_t* cb_plane = img.get_image_plane(1);
  uint8_t* cr_plane = img.get_image_plane(2);

  int cbSize = 1<<log2CbSize;
  int cbSizeChroma = cbSize>>1;

  enc_cb* cb = ectx.enc_cb_pool.get_new();

  cb->split_cu_flag = false;
  cb->log2CbSize = log2CbSize;

  cb->cu_transquant_bypass_flag = false;


  // --- set intra prediction mode ---

  cb->PredMode = MODE_INTRA;
  cb->PartMode = PART_2Nx2N;

  enc_pb_intra* pb = ectx.enc_pb_intra_pool.get_new();
  cb->intra_pb[0] = pb;

  enum IntraPredMode intraMode = find_best_intra_mode(img,x0,y0, cbSize,0,
                                                      &input[0][y0*stride+x0], stride);

  pb->pred_mode = INTRA_PLANAR;
  pb->pred_mode_chroma = INTRA_PLANAR;

  pb->pred_mode = intraMode;
  pb->pred_mode_chroma = intraMode;


  // --- compute transform coefficients ---

  enc_tb* tb = ectx.enc_tb_pool.get_new();
  cb->transform_tree = tb;

  tb->parent = NULL;
  tb->split_transform_flag = false;


  cb->intra_pb[0]->do_intra_prediction(&img, x0,y0, log2CbSize);

  // subtract intra-prediction from input

  int16_t blk[3][16*16];
  diff_blk(blk[0],cbSize,
           &input[0][y0*stride+x0],stride,
           &luma_plane[y0*stride+x0],stride, cbSize);
  diff_blk(blk[1],cbSizeChroma,
           &input[1][y0/2*stride/2+x0/2],stride/2,
           &cb_plane[y0/2*stride/2+x0/2],stride/2, cbSizeChroma);
  diff_blk(blk[2],cbSizeChroma,
           &input[2][y0/2*stride/2+x0/2],stride/2,
           &cr_plane[y0/2*stride/2+x0/2],stride/2, cbSizeChroma);

  tb->coeff[0] = ectx.get_coeff_mem(cbSize*cbSize);
  tb->coeff[1] = ectx.get_coeff_mem(cbSizeChroma*cbSizeChroma);
  tb->coeff[2] = ectx.get_coeff_mem(cbSizeChroma*cbSizeChroma);

  int trType = 0;
  if (log2CbSize==2) trType=1; // TODO: inter mode

  fwd_transform(&accel, tb->coeff[0], cbSize, log2CbSize, 0,  blk[0], cbSize);
  fwd_transform(&accel, tb->coeff[1], cbSizeChroma,
                log2CbSize-1, trType,  blk[1], cbSizeChroma);
  fwd_transform(&accel, tb->coeff[2], cbSizeChroma,
                log2CbSize-1, trType,  blk[2], cbSizeChroma);

  quant_coefficients(tb->coeff[0], tb->coeff[0], log2CbSize,   qp, true);
  quant_coefficients(tb->coeff[1], tb->coeff[1], log2CbSize-1, qp, true);
  quant_coefficients(tb->coeff[2], tb->coeff[2], log2CbSize-1, qp, true);

  tb->set_cbf_flags_from_coefficients(log2CbSize);

  //tb->dequant_and_add_transform(&img, x0,y0, 4 /* blksize */, qp);


  // estimate bits

#if 0
  cb->write_to_image(&img, x0,y0, log2CbSize, true);

  CABAC_encoder_estim estim;
  encoder_output out;
  out = ectx.bitstream_output;
  out.cabac_encoder = &estim;

  ectx.set_output(&out);
  encode_ctb(&ectx, cb, x0,y0);
  ectx.set_output(&ectx.bitstream_output);

  totalSize += estim.size();
  //printf("bytes: %d\n", estim.size());
#endif

  cb->rd_cost=1;

  return cb;
}


enc_cb* encode_cb_may_split(uint8_t const*const input[3],int stride,
                            int x0,int y0, int Log2CtbSize, int qp);

enc_cb* encode_cb_split(uint8_t const*const input[3],int stride,
                        int x0,int y0, int Log2CbSize, int qp)
{
  enc_cb* cb = ectx.enc_cb_pool.get_new();

  cb->split_cu_flag = true;

  cb->cu_transquant_bypass_flag = false;
  cb->log2CbSize = Log2CbSize;

  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (Log2CbSize-1);
    int dy = (i>>1) << (Log2CbSize-1);

    cb->children[i] = encode_cb_may_split(input, stride, x0+dx, y0+dy, Log2CbSize-1, qp);
  }

  if (Log2CbSize==4 && (((x0>>Log2CbSize) + (y0>>Log2CbSize)) & 1)==1)
    cb->rd_cost=0;

  return cb;
}


enc_cb* encode_cb_may_split(uint8_t const*const input[3],int stride,
                            int x0,int y0, int Log2CtbSize, int qp)
{
  enc_cb* cb_no_split = encode_cb_no_split(input,stride,x0,y0, Log2CtbSize, qp);
  enc_cb* cb_split = NULL;
  enc_cb* cb = cb_no_split;

  if (Log2CtbSize>3) {
    cb_split = encode_cb_split(input,stride,x0,y0, Log2CtbSize, qp);

    printf("a\n");

    if (cb_split != NULL && cb_split->rd_cost < cb_no_split->rd_cost) {
      cb = cb_split;
      printf("-> b\n");
    }
  }

  return cb;
}


double encode_image_FDCT_3(uint8_t const*const input[3],int width,int height, int qp)
{
  int stride=width;

  int w = sps.pic_width_in_luma_samples;
  int h = sps.pic_height_in_luma_samples;

  img.alloc_image(w,h, de265_chroma_420, &sps, true, NULL /* no decctx */);
  img.alloc_encoder_data(&sps);

  initialize_CABAC_models(ectx.ctx_model, shdr.initType, shdr.SliceQPY);

  int Log2CtbSize = sps.Log2CtbSizeY;

  uint8_t* luma_plane = img.get_image_plane(0);
  uint8_t* cb_plane = img.get_image_plane(1);
  uint8_t* cr_plane = img.get_image_plane(2);


  // encode CTB by CTB

  for (int y=0;y<sps.PicHeightInCtbsY;y++)
    for (int x=0;x<sps.PicWidthInCtbsY;x++)
      {
        ectx.reset_coeff_mem();

        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;

        logtrace(LogSlice,"encode CTB at %d %d\n",x0,y0);

        enc_cb* cb = encode_cb_may_split(input,stride, x0,y0, Log2CtbSize, qp);


        cb->write_to_image(&img, x<<Log2CtbSize, y<<Log2CtbSize, Log2CtbSize, true);

#if 0
        CABAC_encoder_estim estim;
        encoder_output out;
        out = ectx.bitstream_output;
        out.cabac_encoder = &estim;

        ectx.set_output(&out);
        //printf("--- estim ---\n");
        encode_ctb(&ectx, cb, x,y);

        CABAC_encoder_bitstream bs;
        bs.range = writer.range;
        bs.low   = writer.low;

        encoder_output outbs;
        outbs = ectx.bitstream_output;
        outbs.cabac_encoder = &bs;

        ectx.set_output(&outbs);
        //printf("--- bitstream ---\n");
        encode_ctb(&ectx, cb, x,y);
        bs.flush_CABAC();

        printf("real: %d  estim: %d\n",bs.size(),estim.size());

        ectx.set_output(&ectx.bitstream_output);
#endif

        //printf("--- real ---\n");
        encode_ctb(&ectx, cb, x,y);

        // decode into image

        cb->do_intra_prediction(&img, x0,y0);
        cb->dequant_and_add_transform(&accel, &img, x0,y0, qp);

        //cb->intra_pb[0]->do_intra_prediction(&img, x0,y0, 4);
        //cb->transform_tree->dequant_and_add_transform(&img, x0,y0, 4 /* blksize */, qp);


        int last = (y==sps.PicHeightInCtbsY-1 &&
                    x==sps.PicWidthInCtbsY-1);
        writer.write_CABAC_term_bit(last);


        ectx.enc_cb_pool.free_all();
        ectx.enc_tb_pool.free_all();
        ectx.enc_pb_intra_pool.free_all();
      }


  double psnr = PSNR(MSE(input[0], width,
                         luma_plane, img.get_image_stride(0),
                         width, height));
  return psnr;
}


template <class T> void printblk(T* data, int w)
{
  for (int y=0;y<w;y++,printf("\n"))
    for (int x=0;x<w;x++) {
      printf("%3d ",data[x+y*w]);
    }
}


void DCT_test()
{
  int16_t input[16*16];
  for (int i=0;i<16*16;i++)
    input[i] = 120; //(i%16)*10;

  printf("input\n");
  printblk(input,16);

  int16_t coeff[16*16];
  fdct_16x16_8_fallback(coeff, input,16);
  //fdct_8x8_8_fallback(coeff, input,16);
  //fdct_4x4_8_fallback(coeff, input,16);

  printf("coeff\n");
  printblk(coeff,16);

  for (int i=0;i<16*16;i++) {
    //coeff[i] *= 256;
  }

  uint8_t output[16*16];
  memset(output,0,16*16);
  transform_16x16_add_8_fallback(output, coeff, 16);

  printf("output\n");
  printblk(output,16);
}


extern void split_last_significant_position(int pos, int* prefix, int* suffix, int* nSuffixBits);

static void test_last_significant_coeff_pos()
{
  for (int i=0;i<64;i++)
    {
      int prefix,suffix,bits;
      split_last_significant_position(i,&prefix,&suffix,&bits);

      int v;
      if (prefix>3) {
        int b = (prefix>>1)-1;
        assert(b==bits);
        v = ((2+(prefix&1))<<b) + suffix;
      }
      else {
        v = prefix;
      }

      printf("%d : %d %d %d  -> %d\n",i,prefix,suffix,bits,v);
      assert(v==i);
    }
}


void write_stream_1()
{
  nal_header nal;

  // VPS

  vps.set_defaults(Profile_Main, 6,2);


  // SPS

  sps.set_defaults();
  sps.set_CB_log2size_range(4,4);
  sps.set_TB_log2size_range(4,4);
  sps.set_resolution(352,288);
  sps.compute_derived_values();

  // PPS

  pps.set_defaults();

  // turn off deblocking filter
  pps.deblocking_filter_control_present_flag = true;
  pps.deblocking_filter_override_enabled_flag = false;
  pps.pic_disable_deblocking_filter_flag = true;
  pps.pps_loop_filter_across_slices_enabled_flag = false;

  pps.set_derived_values(&sps);


  // slice

  shdr.set_defaults(&pps);
  shdr.slice_deblocking_filter_disabled_flag = true;
  shdr.slice_loop_filter_across_slices_enabled_flag = false;

  img.vps  = vps;
  img.sps  = sps;
  img.pps  = pps;

  ectx.img = &img;
  ectx.shdr = &shdr;
  ectx.bitstream_output.cabac_encoder = &writer;
  ectx.set_output(&ectx.bitstream_output);

  //context_model ctx_model[CONTEXT_MODEL_TABLE_LENGTH];



  // write headers

  writer.write_startcode();
  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(&writer);
  vps.write(&errqueue, &writer);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(&writer);
  sps.write(&errqueue, &writer);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(&writer);
  pps.write(&errqueue, &writer, &sps);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_IDR_W_RADL);
  nal.write(&writer);
  shdr.write(&errqueue, &writer, &sps, &pps, nal.nal_unit_type);
  writer.skip_bits(1);
  writer.flush_VLC();

  //encode_image_coeffTest_1();
  encode_image_FDCT_1();
  //encode_image_FDCT_2();

  //encode_image(&ectx);
  writer.flush_CABAC();
}


void encode_stream_intra_1(const char* yuv_filename, int width, int height)
{
  int qp = 15;


  FILE* fh = fopen(yuv_filename,"rb");
  uint8_t* input[3];
  input[0] = (uint8_t*)malloc(width*height);
  input[1] = (uint8_t*)malloc(width*height/4);
  input[2] = (uint8_t*)malloc(width*height/4);


  nal_header nal;

  // VPS

  vps.set_defaults(Profile_Main, 6,2);


  // SPS

  sps.set_defaults();
  sps.set_CB_log2size_range(3,3);
  sps.set_TB_log2size_range(3,3);
  sps.set_resolution(352,288);
  sps.compute_derived_values();

  // PPS

  pps.set_defaults();
  pps.pic_init_qp = qp;

  // turn off deblocking filter
  pps.deblocking_filter_control_present_flag = true;
  pps.deblocking_filter_override_enabled_flag = false;
  pps.pic_disable_deblocking_filter_flag = true;
  pps.pps_loop_filter_across_slices_enabled_flag = false;

  pps.set_derived_values(&sps);


  // slice

  shdr.set_defaults(&pps);
  shdr.slice_deblocking_filter_disabled_flag = true;
  shdr.slice_loop_filter_across_slices_enabled_flag = false;

  img.vps  = vps;
  img.sps  = sps;
  img.pps  = pps;

  ectx.img = &img;
  ectx.shdr = &shdr;
  ectx.bitstream_output.cabac_encoder = &writer;
  ectx.set_output(&ectx.bitstream_output);

  //context_model ctx_model[CONTEXT_MODEL_TABLE_LENGTH];



  // write headers

  writer.write_startcode();
  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(&writer);
  vps.write(&errqueue, &writer);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(&writer);
  sps.write(&errqueue, &writer);
  writer.flush_VLC();

  writer.write_startcode();
  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(&writer);
  pps.write(&errqueue, &writer, &sps);
  writer.flush_VLC();

  int maxPoc = 20;//99100;
  for (int poc=0; poc<maxPoc ;poc++)
    {
      fprintf(stderr,"encoding frame %d\n",poc);

      fread(input[0],1,width*height,fh);
      fread(input[1],1,width*height/4,fh);
      fread(input[2],1,width*height/4,fh);
      
      if (feof(fh)) { break; }


      // write slice header

      //shdr.slice_pic_order_cnt_lsb = poc & 0xFF;

      writer.write_startcode();
      //nal.set(poc==0 ? NAL_UNIT_IDR_W_RADL : NAL_UNIT_TRAIL_N);
      nal.set(NAL_UNIT_IDR_W_RADL);
      nal.write(&writer);
      shdr.write(&errqueue, &writer, &sps, &pps, nal.nal_unit_type);
      writer.skip_bits(1);
      writer.flush_VLC();

      //encode_image_coeffTest_1();
      //encode_image_FDCT_1();
      writer.init_CABAC();
      //double psnr = encode_image_FDCT_2(input,width,height, qp);
      double psnr = encode_image_FDCT_3(input,width,height, qp);

      //encode_image(&ectx);
      writer.flush_CABAC();

      fprintf(stderr,"  PSNR-Y: %f\n", psnr);
    }

  fclose(fh);
}


int main(int argc, char** argv)
{
  //de265_set_verbosity(3);

  init_scan_orders();
  alloc_and_init_significant_coeff_ctxIdx_lookupTable();
  init_acceleration_functions_fallback(&accel);

  encode_stream_intra_1("paris_cif.yuv",352,288);


  FILE* fh = fopen("out.bin","wb");
  fwrite(writer.data(), 1,writer.size(), fh);
  fclose(fh);

  return 0;
}
