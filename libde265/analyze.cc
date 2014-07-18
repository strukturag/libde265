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


#include "libde265/analyze.h"
#include <assert.h>


enum IntraPredMode find_best_intra_mode(de265_image& img,int x0,int y0, int log2BlkSize, int cIdx,
                                        const uint8_t* ref, int stride)
{
  //return INTRA_DC;
  //return INTRA_ANGULAR_14;

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
    decode_intra_prediction(&img, x0,y0, (enum IntraPredMode)mode, 1<<log2BlkSize, cIdx);

    uint32_t distortion = SSD(ref,stride,
      img.get_image_plane_at_pos(cIdx, x0,y0), img.get_image_stride(cIdx),
      1<<log2BlkSize, 1<<log2BlkSize);

    int sad=distortion;

    sad *= 0.5;
    //sad *= 0.9;

    if (mode==0 || sad<min_sad) {
      min_sad = sad;
      best_mode = (enum IntraPredMode)mode;
    }
  }


  // --- test all modes ---

  for (int idx=0;idx<35;idx++) {
    enum IntraPredMode mode = (enum IntraPredMode)idx; //candidates[idx];
    decode_intra_prediction(&img, x0,y0, (enum IntraPredMode)mode, 1<<log2BlkSize, cIdx);


    uint32_t distortion = SSD(ref,stride,
      img.get_image_plane_at_pos(cIdx, x0,y0), img.get_image_stride(cIdx),
      1<<log2BlkSize, 1<<log2BlkSize);

    int sad=distortion;

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


enc_cb* encode_cb_no_split(encoder_context* ectx,
                           const de265_image* input,
                           int x0,int y0, int log2CbSize, int ctDepth, int qp)
{
  //printf("encode at %d %d, size %d\n",x0,y0,1<<log2CbSize);

  int stride = ectx->img.get_image_stride(0);

  uint8_t* luma_plane = ectx->img.get_image_plane(0);
  uint8_t* cb_plane = ectx->img.get_image_plane(1);
  uint8_t* cr_plane = ectx->img.get_image_plane(2);

  int cbSize = 1<<log2CbSize;
  int cbSizeChroma = cbSize>>1;

  enc_cb* cb = ectx->enc_cb_pool.get_new();

  cb->split_cu_flag = false;
  cb->log2CbSize = log2CbSize;
  cb->ctDepth = ctDepth;

  cb->cu_transquant_bypass_flag = false;


  //printf("input\n");
  //printblk(input[0],stride,x0,y0,cbSize);

  // --- set intra prediction mode ---

  cb->PredMode = MODE_INTRA;
  cb->PartMode = PART_2Nx2N;

  enc_pb_intra* pb = ectx->enc_pb_intra_pool.get_new();
  cb->intra_pb[0] = pb;

  enum IntraPredMode intraMode = find_best_intra_mode(ectx->img,x0,y0, log2CbSize,0,
                                                      input->get_image_plane_at_pos(0,x0,y0),
                                                      input->get_image_stride(0));

  pb->pred_mode = intraMode;
  pb->pred_mode_chroma = intraMode;


  // --- compute transform coefficients ---

  enc_tb* tb = ectx->enc_tb_pool.get_new();
  cb->transform_tree = tb;

  tb->parent = NULL;
  tb->split_transform_flag = false;
  tb->log2TbSize = log2CbSize;

  cb->intra_pb[0]->do_intra_prediction(&ectx->img, x0,y0, log2CbSize);

  //printf("result of intra prediction\n");
  //printblk(luma_plane,stride,x0,y0,cbSize);
  //printblk(cb_plane,stride/2,x0/2,y0/2,cbSize/2);

  // subtract intra-prediction from input

  int16_t blk[3][32*32];
  diff_blk(blk[0],cbSize,
           input->get_image_plane_at_pos(0,x0,y0), input->get_image_stride(0),
           &luma_plane[y0*stride+x0],stride, cbSize);
  diff_blk(blk[1],cbSizeChroma,
           input->get_image_plane_at_pos(1,x0/2,y0/2), input->get_image_stride(1),
           &cb_plane[y0/2*stride/2+x0/2],stride/2, cbSizeChroma);
  diff_blk(blk[2],cbSizeChroma,
           input->get_image_plane_at_pos(2,x0/2,y0/2), input->get_image_stride(2),
           &cr_plane[y0/2*stride/2+x0/2],stride/2, cbSizeChroma);

  tb->coeff[0] = ectx->enc_coeff_pool.get_new(cbSize*cbSize);
  tb->coeff[1] = ectx->enc_coeff_pool.get_new(cbSizeChroma*cbSizeChroma);
  tb->coeff[2] = ectx->enc_coeff_pool.get_new(cbSizeChroma*cbSizeChroma);

  int trType = 0;
  if (log2CbSize==2) trType=1; // TODO: inter mode

  fwd_transform(&ectx->accel, tb->coeff[0], cbSize, log2CbSize, trType,  blk[0], cbSize);
  fwd_transform(&ectx->accel, tb->coeff[1], cbSizeChroma,
                log2CbSize-1, 0,  blk[1], cbSizeChroma);
  fwd_transform(&ectx->accel, tb->coeff[2], cbSizeChroma,
                log2CbSize-1, 0,  blk[2], cbSizeChroma);

  //printf("raw coeffs\n");
  //printcoeff(tb->coeff[0],cbSize);
  //printcoeff(tb->coeff[1],cbSize/2);

  quant_coefficients(tb->coeff[0], tb->coeff[0], log2CbSize,   qp, true);
  quant_coefficients(tb->coeff[1], tb->coeff[1], log2CbSize-1, qp, true);
  quant_coefficients(tb->coeff[2], tb->coeff[2], log2CbSize-1, qp, true);

  tb->set_cbf_flags_from_coefficients();

  //printf("quantized coeffs\n");
  //printcoeff(tb->coeff[0],cbSize);
  //printcoeff(tb->coeff[1],cbSize/2);


  // estimate bits

  cb->write_to_image(&ectx->img, x0,y0, log2CbSize, true);

  cb->reconstruct(&ectx->accel, &ectx->img, x0,y0, qp);
  cb->distortion = compute_distortion_ssd(&ectx->img, input, x0,y0, log2CbSize, 0);

  //printf("reconstruction: add transform\n");
  //printblk(luma_plane,stride,x0,y0,cbSize);



  ectx->switch_to_CABAC_estim();
  encode_quadtree(ectx, cb, x0,y0,log2CbSize,cb->ctDepth);
  cb->rate = ectx->cabac_estim.size();
  ectx->switch_to_CABAC_stream();

  return cb;
}


enc_cb* encode_cb_split(encoder_context* ectx,
                        const de265_image* input,
                        int x0,int y0, int Log2CbSize, int ctDepth, int qp)
{
  enc_cb* cb = ectx->enc_cb_pool.get_new();

  cb->split_cu_flag = true;

  cb->cu_transquant_bypass_flag = false;
  cb->log2CbSize = Log2CbSize;
  cb->ctDepth = ctDepth;

  cb->distortion = 0;
  cb->rate       = 0;

  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (Log2CbSize-1);
    int dy = (i>>1) << (Log2CbSize-1);

    cb->children[i] = encode_cb_may_split(ectx, input, x0+dx, y0+dy,
                                          Log2CbSize-1, ctDepth+1, qp);

    cb->distortion += cb->children[i]->distortion;
    cb->rate       += cb->children[i]->rate;
  }

  return cb;
}


enc_cb* encode_cb_may_split(encoder_context* ectx,
                            const de265_image* input,
                            int x0,int y0, int Log2CbSize, int ctDepth, int qp)
{
  enc_cb* cb_no_split = encode_cb_no_split(ectx, input,x0,y0, Log2CbSize, ctDepth, qp);
  enc_cb* cb_split = NULL;
  enc_cb* cb = cb_no_split;

  if (Log2CbSize > ectx->sps.Log2MinCbSizeY) {
    cb_split = encode_cb_split(ectx, input,x0,y0, Log2CbSize, ctDepth, qp);

    float lambda = 250.0;

    float rd_cost_split    = cb_split->distortion    + lambda * cb_split->rate;
    float rd_cost_no_split = cb_no_split->distortion + lambda * cb_no_split->rate;

    bool split =  (rd_cost_split < rd_cost_no_split);
    //bool split = (Log2CbSize==4 && (((x0>>Log2CbSize) + (y0>>Log2CbSize)) & 1)==1);

    if (split) {
      cb = cb_split;
    }
    else {
      cb->write_to_image(&ectx->img, x0,y0, Log2CbSize, true);
      cb->reconstruct(&ectx->accel, &ectx->img, x0,y0, qp);
    }
  }

  return cb;
}


double encode_image(encoder_context* ectx,
                    const de265_image* input, int qp)
{
  int stride=input->get_image_stride(0);

  int w = ectx->sps.pic_width_in_luma_samples;
  int h = ectx->sps.pic_height_in_luma_samples;

  ectx->img.alloc_image(w,h, de265_chroma_420, &ectx->sps, true, NULL /* no decctx */);
  ectx->img.alloc_encoder_data(&ectx->sps);
  ectx->img.clear_metadata();

  initialize_CABAC_models(ectx->ctx_model, ectx->shdr.initType, ectx->shdr.SliceQPY);

  int Log2CtbSize = ectx->sps.Log2CtbSizeY;

  uint8_t* luma_plane = ectx->img.get_image_plane(0);
  uint8_t* cb_plane   = ectx->img.get_image_plane(1);
  uint8_t* cr_plane   = ectx->img.get_image_plane(2);


  // encode CTB by CTB

  for (int y=0;y<ectx->sps.PicHeightInCtbsY;y++)
    for (int x=0;x<ectx->sps.PicWidthInCtbsY;x++)
      {
        ectx->img.set_SliceAddrRS(x, y, ectx->shdr.SliceAddrRS);

        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;

        logtrace(LogSlice,"encode CTB at %d %d\n",x0,y0);

        enc_cb* cb = encode_cb_may_split(ectx, input, x0,y0, Log2CtbSize, 0, qp);


        cb->write_to_image(&ectx->img, x<<Log2CtbSize, y<<Log2CtbSize, Log2CtbSize, true);

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
        encode_ctb(ectx, cb, x,y);

        // decode into image

        /* TMP
        cb->do_intra_prediction(&img, x0,y0);
        cb->dequant_and_add_transform(&accel, &img, x0,y0, qp);
        */

#if 0
        for (int dy=0;dy<(1<<Log2CtbSize);dy++, printf("\n"))
          for (int dx=0;dx<(1<<Log2CtbSize);dx++)
            {
              printf("%02x/%02x ",
                     input[0][x0+dx+(y0+dy)*width],
                     luma_plane[x0+dx+(y0+dy)*width]);

              if (dx==7) printf(" ");
              if (dx==15 && dy==7) printf("\n");
            }
#endif

        int last = (y==ectx->sps.PicHeightInCtbsY-1 &&
                    x==ectx->sps.PicWidthInCtbsY-1);
        ectx->cabac_bitstream.write_CABAC_term_bit(last);


        ectx->free_all_pools();
      }


  double psnr = PSNR(MSE(input->get_image_plane(0), input->get_image_stride(0),
                         luma_plane, ectx->img.get_image_stride(0),
                         input->get_width(), input->get_height()));
  return psnr;
}


void encode_sequence(encoder_context* ectx)
{
  // TODO: must be <30, because Y->C mapping (tab8_22) is not implemented yet
  int qp = ectx->params.constant_QP;


  nal_header nal;

  // VPS

  ectx->vps.set_defaults(Profile_Main, 6,2);


  // SPS

  ectx->sps.set_defaults();
  ectx->sps.set_CB_log2size_range( Log2(ectx->params.min_cb_size), Log2(ectx->params.max_cb_size));
  ectx->sps.set_TB_log2size_range( Log2(ectx->params.min_cb_size), Log2(ectx->params.max_cb_size));
  ectx->sps.set_resolution(ectx->img_source->get_width(),
                           ectx->img_source->get_height());
  ectx->sps.compute_derived_values();

  // PPS

  ectx->pps.set_defaults();
  ectx->pps.pic_init_qp = qp;

  // turn off deblocking filter
  ectx->pps.deblocking_filter_control_present_flag = true;
  ectx->pps.deblocking_filter_override_enabled_flag = false;
  ectx->pps.pic_disable_deblocking_filter_flag = true;
  ectx->pps.pps_loop_filter_across_slices_enabled_flag = false;

  ectx->pps.set_derived_values(&ectx->sps);


  // slice

  ectx->shdr.set_defaults(&ectx->pps);
  ectx->shdr.slice_deblocking_filter_disabled_flag = true;
  ectx->shdr.slice_loop_filter_across_slices_enabled_flag = false;

  ectx->img.vps  = ectx->vps;
  ectx->img.sps  = ectx->sps;
  ectx->img.pps  = ectx->pps;



  // write headers

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(ectx->cabac);
  ectx->vps.write(&ectx->errqueue, ectx->cabac);
  ectx->cabac->flush_VLC();
  ectx->write_packet();

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(ectx->cabac);
  ectx->sps.write(&ectx->errqueue, ectx->cabac);
  ectx->cabac->flush_VLC();
  ectx->write_packet();

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(ectx->cabac);
  ectx->pps.write(&ectx->errqueue, ectx->cabac, &ectx->sps);
  ectx->cabac->flush_VLC();
  ectx->write_packet();

  ectx->img_source->release_next_image( ectx->params.first_frame );

  int maxPoc = ectx->params.max_number_of_frames;
  for (int poc=0; poc<maxPoc ;poc++)
    {
      fprintf(stderr,"encoding frame %d\n",poc);

      de265_image* input_image = ectx->img_source->get_image();
      if (input_image==NULL) { break; } // EOF


      // write slice header

      //shdr.slice_pic_order_cnt_lsb = poc & 0xFF;

      nal.set(NAL_UNIT_IDR_W_RADL);
      nal.write(ectx->cabac);
      ectx->shdr.write(&ectx->errqueue, ectx->cabac, &ectx->sps, &ectx->pps, nal.nal_unit_type);
      ectx->cabac->skip_bits(1);
      ectx->cabac->flush_VLC();

      ectx->cabac->init_CABAC();
      double psnr = encode_image(ectx,input_image, qp);
      fprintf(stderr,"  PSNR-Y: %f\n", psnr);
      ectx->cabac->flush_CABAC();
      ectx->write_packet();


      // --- write reconstruction ---

      if (ectx->reconstruction_sink) {
        ectx->reconstruction_sink->send_image(&ectx->img);
      }


      // --- release input image ---

      ectx->img_source->release_next_image();
    }
}

