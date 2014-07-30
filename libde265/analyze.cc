/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
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


#include "libde265/analyze.h"
#include <assert.h>
#include <limits>


float lambda = 50.0;

const enc_tb* encode_transform_tree_may_split(encoder_context* ectx,
                                              context_model_table ctxModel,
                                              const de265_image* input,
                                              const enc_tb* parent,
                                              enc_cb* cb,
                                              int x0,int y0, int xBase,int yBase, int log2TbSize,
                                              int blkIdx,
                                              int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                              int qp);


enum IntraPredMode find_best_intra_mode(de265_image& img,int x0,int y0, int log2BlkSize, int cIdx,
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

  return best_mode;
}


static void diff_blk(int16_t* out,int out_stride,
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


static bool has_nonzero_value(const int16_t* data, int n)
{
  for (int i=0;i<n;i++)
    if (data[i]) return true;

  return false;
}


void encode_transform_unit(encoder_context* ectx,
                           enc_tb* tb,
                           const de265_image* input,
                           int x0,int y0, // luma position
                           int log2TbSize, // chroma adapted
                           const enc_cb* cb,
                           int qp, int cIdx)
{
  int xC = x0;
  int yC = y0;
  int tbSize = 1<<log2TbSize;


  // --- do intra prediction ---

  enum IntraPredMode intraPredMode  = ectx->img.get_IntraPredMode(x0,y0);

  if (cIdx>0) {
    intraPredMode = lumaPredMode_to_chromaPredMode(intraPredMode,
                                                   cb->intra.chroma_mode);

    xC >>= 1;
    yC >>= 1;
  }
  
  decode_intra_prediction(&ectx->img, xC,  yC,   intraPredMode,  tbSize  , cIdx);



  // --- subtract prediction from input ---

  int16_t blk[32*32];
  uint8_t* pred = ectx->img.get_image_plane(cIdx);
  int stride = ectx->img.get_image_stride(cIdx);

  diff_blk(blk,tbSize,
           input->get_image_plane_at_pos(cIdx,xC,yC), input->get_image_stride(cIdx),
           &pred[yC*stride+xC],stride, tbSize);


  // --- forward transform ---

  tb->coeff[cIdx] = ectx->enc_coeff_pool.get_new(tbSize*tbSize);

  int trType = 0;
  if (cIdx==0 && log2TbSize==2) trType=1; // TODO: inter mode

  fwd_transform(&ectx->accel, tb->coeff[cIdx], tbSize, log2TbSize, trType,  blk, tbSize);


  // --- quantization ---

  quant_coefficients(tb->coeff[cIdx], tb->coeff[cIdx], log2TbSize,   qp, true);

  tb->cbf[cIdx] = has_nonzero_value(tb->coeff[cIdx], 1<<(log2TbSize<<1));
}


const enc_tb* encode_transform_tree_no_split(encoder_context* ectx,
                                             context_model_table ctxModel,
                                             const de265_image* input,
                                             const enc_tb* parent,
                                             enc_cb* cb,
                                             int x0,int y0, int xBase,int yBase, int log2TbSize,
                                             int blkIdx,
                                             int trafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                             int qp)
{
  printf("--- TT at %d %d, size %d, trafoDepth %d\n",x0,y0,1<<log2TbSize,trafoDepth);

  de265_image* img = &ectx->img;

  int stride = ectx->img.get_image_stride(0);

  uint8_t* luma_plane = ectx->img.get_image_plane(0);
  uint8_t* cb_plane = ectx->img.get_image_plane(1);
  uint8_t* cr_plane = ectx->img.get_image_plane(2);

  // --- compute transform coefficients ---

  enc_tb* tb = ectx->enc_tb_pool.get_new();

  tb->parent = parent;
  tb->split_transform_flag = false;
  tb->log2TbSize = log2TbSize;
  tb->cbf[0] = tb->cbf[1] = tb->cbf[2] = 0;


  // luma block

  encode_transform_unit(ectx, tb, input, x0,y0, log2TbSize, cb, qp, 0 /* Y */);


  // chroma blocks

  if (log2TbSize > 2) {
    encode_transform_unit(ectx, tb, input, x0,y0, log2TbSize-1, cb, qp, 1 /* Cb */);
    encode_transform_unit(ectx, tb, input, x0,y0, log2TbSize-1, cb, qp, 2 /* Cr */);
  }
  else if (blkIdx==3) {
    encode_transform_unit(ectx, tb, input, xBase,yBase, log2TbSize, cb, qp, 1 /* Cb */);
    encode_transform_unit(ectx, tb, input, xBase,yBase, log2TbSize, cb, qp, 2 /* Cr */);
  }

  tb->reconstruct(&ectx->accel, &ectx->img, x0,y0, xBase,yBase, cb, qp, blkIdx);



  // measure rate

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);

  /* TODO: cannot do this currently, because tb->parent->cbf[1] is not defined at this point
  encode_transform_tree(ectx, tb, cb, x0,y0, xBase,yBase,
                        log2TbSize, trafoDepth, blkIdx, MaxTrafoDepth, IntraSplitFlag, true);
  */

  tb->rate = estim.getRDBits();


  // measure distortion

  int tbSize = 1<<log2TbSize;
  tb->distortion = SSD(input->get_image_plane_at_pos(0, x0,y0), input->get_image_stride(0),
                       img  ->get_image_plane_at_pos(0, x0,y0), img  ->get_image_stride(0),
                       tbSize, tbSize);

  return tb;
}


const enc_tb* encode_transform_tree_split(encoder_context* ectx,
                                          context_model_table ctxModel,
                                          const de265_image* input,
                                          const enc_tb* parent,
                                          enc_cb* cb,
                                          int x0,int y0, int log2TbSize,
                                          int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                          int qp)
{
  const de265_image* img = &ectx->img;

  enc_tb* tb = ectx->enc_tb_pool.get_new();

  tb->parent = parent;
  tb->split_transform_flag = true;
  tb->log2TbSize = log2TbSize;


  // --- estimate rate for this tree level ---

  /* We cannot do this entirely correctly here, because we do not know yet the
     cbf flags. Hence, we only measure the bits for the split_transform_flag.
  */

  /* TODO: currently ignore split_transform_flag, too.
  ectx->switch_to_CABAC_estim();
  encode_split_transform_flag(ectx, tb, cb, x0,y0, x0,y0, log2TbSize,
                        TrafoDepth, 0, MaxTrafoDepth, IntraSplitFlag, false);
  tb->rate = ectx->cabac_estim.size();
  */
  tb->rate = 0;
  tb->distortion = 0;


  // --- encode all child nodes ---

  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (log2TbSize-1);
    int dy = (i>>1) << (log2TbSize-1);

    tb->children[i] = encode_transform_tree_may_split(ectx, ctxModel, input, tb, cb,
                                                      x0+dx, y0+dy, x0,y0,
                                                      log2TbSize-1, i,
                                                      TrafoDepth+1, MaxTrafoDepth, IntraSplitFlag,
                                                      qp);

    tb->distortion += tb->children[i]->distortion;
    tb->rate       += tb->children[i]->rate;
  }  

  tb->set_cbf_flags_from_children();

  return tb;
}


const enc_tb* encode_transform_tree_may_split(encoder_context* ectx,
                                              context_model_table ctxModel,
                                              const de265_image* input,
                                              const enc_tb* parent,
                                              enc_cb* cb,
                                              int x0,int y0, int xBase,int yBase, int log2TbSize,
                                              int blkIdx,
                                              int TrafoDepth, int MaxTrafoDepth, int IntraSplitFlag,
                                              int qp)
{
  bool selectIntraPredMode = false;
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_2Nx2N && TrafoDepth==0);
  selectIntraPredMode |= (cb->PredMode==MODE_INTRA && cb->PartMode==PART_NxN   && TrafoDepth==1);

  if (selectIntraPredMode) {
    enum IntraPredMode intraMode = find_best_intra_mode(ectx->img,x0,y0, log2TbSize, 0,
                                                        input->get_image_plane_at_pos(0,x0,y0),
                                                        input->get_image_stride(0));

    cb->intra.pred_mode[blkIdx] = intraMode;
    cb->intra.chroma_mode  = INTRA_CHROMA_LIKE_LUMA;

    // TODO: it's probably better to have more fine-grained writing to the image (only pred-mode)
    cb->write_to_image(&ectx->img, x0,y0, true);
  }



  bool test_split = (log2TbSize > 2 &&
                     TrafoDepth < MaxTrafoDepth &&
                     log2TbSize > ectx->sps.Log2MinTrafoSize);

  bool test_no_split = (IntraSplitFlag==0 || TrafoDepth>0);

  context_model_table ctxSplit;
  if (test_split) {
    copy_context_model_table(ctxSplit, ctxModel);
  }


  printf("log2TbSize:%d TrafoDepth:%d MaxTrafoDepth:%d log2TbSize:%d MinTrafoSize:%d\n",
         log2TbSize,
         TrafoDepth,
         MaxTrafoDepth,
         log2TbSize,
         ectx->sps.Log2MinTrafoSize);
  printf("  intra split flag: %d\n",IntraSplitFlag);

  const enc_tb* tb_no_split = NULL;
  const enc_tb* tb_split    = NULL;
  float rd_cost_no_split = std::numeric_limits<float>::max();
  float rd_cost_split    = std::numeric_limits<float>::max();

  if (test_no_split) {
    printf("test no split\n");
    tb_no_split = encode_transform_tree_no_split(ectx, ctxModel, input, parent,
                                                 cb, x0,y0, xBase,yBase, log2TbSize,
                                                 blkIdx,
                                                 TrafoDepth,MaxTrafoDepth,IntraSplitFlag,
                                                 qp);

    rd_cost_no_split = tb_no_split->distortion + lambda * tb_no_split->rate;
    printf("-\n");
  }


  if (test_split) {
    printf("test split\n");
    tb_split = encode_transform_tree_split(ectx, ctxSplit, input, parent, cb,
                                           x0,y0, log2TbSize,
                                           TrafoDepth, MaxTrafoDepth, IntraSplitFlag,
                                           qp);
    
    rd_cost_split    = tb_split->distortion    + lambda * tb_split->rate;
    printf("-\n");
  }


  bool split = (rd_cost_split < rd_cost_no_split);

  if (split) {
    assert(tb_split);
    return tb_split;
  }
  else {
    assert(tb_no_split);
    tb_no_split->reconstruct(&ectx->accel,
                             &ectx->img, x0,y0, xBase,yBase,
                             cb, qp, blkIdx);

    return tb_no_split;
  }
}


enc_cb* encode_cb_no_split(encoder_context* ectx,
                           context_model_table ctxModel,
                           const de265_image* input,
                           int x0,int y0, int log2CbSize, int ctDepth, int qp)
{
  //printf("--- encode at %d %d, size %d\n",x0,y0,1<<log2CbSize);

  int cbSize = 1<<log2CbSize;

  enc_cb* cb = ectx->enc_cb_pool.get_new();

  cb->split_cu_flag = false;
  cb->log2CbSize = log2CbSize;
  cb->ctDepth = ctDepth;

  cb->cu_transquant_bypass_flag = false;


  // --- set intra prediction mode ---

  cb->PredMode = MODE_INTRA;
  cb->PartMode = PART_2Nx2N;

  if (cb->log2CbSize == ectx->sps.Log2MinCbSizeY) {
    cb->PartMode = PART_NxN;
  }

  // rate for split_cu_flag (=false)

  /* TODO: TMP DISABLE
  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);
  encode_coding_unit(ectx,cb,x0,y0,log2CbSize, false);
  cb->rate = estim.getRDBits();
  */
  cb->rate = 0;

  // encode transform tree

  int IntraSplitFlag= (cb->PredMode == MODE_INTRA && cb->PartMode == PART_NxN);
  int MaxTrafoDepth = ectx->sps.max_transform_hierarchy_depth_intra + IntraSplitFlag;

  cb->transform_tree = encode_transform_tree_may_split(ectx, ctxModel, input, NULL, cb,
                                                       x0,y0, x0,y0, log2CbSize,
                                                       0,
                                                       0, MaxTrafoDepth, IntraSplitFlag,
                                                       qp);

  cb->distortion  = cb->transform_tree->distortion;
  cb->rate       += cb->transform_tree->rate;


  // estimate bits

  cb->write_to_image(&ectx->img, x0,y0, true);

  cb->distortion = compute_distortion_ssd(&ectx->img, input, x0,y0, log2CbSize, 0);

  return cb;
}


enc_cb* encode_cb_split(encoder_context* ectx,
                        context_model_table ctxModel,
                        const de265_image* input,
                        int x0,int y0, int Log2CbSize, int ctDepth, int qp)
{
  enc_cb* cb = ectx->enc_cb_pool.get_new();

  cb->split_cu_flag = true;

  cb->cu_transquant_bypass_flag = false;
  cb->log2CbSize = Log2CbSize;
  cb->ctDepth = ctDepth;


  // rate for split_cu_flag (=true)

  CABAC_encoder_estim estim;
  ectx->switch_CABAC(ctxModel, &estim);
  encode_quadtree(ectx,cb,x0,y0,Log2CbSize,ctDepth, false);

  cb->distortion = 0;
  cb->rate       = estim.getRDBits();


  for (int i=0;i<4;i++) {
    int dx = (i&1)  << (Log2CbSize-1);
    int dy = (i>>1) << (Log2CbSize-1);

    cb->children[i] = encode_cb_may_split(ectx, ctxModel,
                                          input, x0+dx, y0+dy,
                                          Log2CbSize-1, ctDepth+1, qp);

    cb->distortion += cb->children[i]->distortion;
    cb->rate       += cb->children[i]->rate;
  }

  return cb;
}


enc_cb* encode_cb_may_split(encoder_context* ectx,
                            context_model_table ctxModel,
                            const de265_image* input,
                            int x0,int y0, int Log2CbSize,
                            int ctDepth,
                            int qp)
{
  context_model_table ctxSplit;


  // if we will try splitting the CB, make a copy of the initial ctxModel

  if (Log2CbSize > ectx->sps.Log2MinCbSizeY) {
    copy_context_model_table(ctxSplit, ctxModel);
  }

  enc_cb* cb_no_split = encode_cb_no_split(ectx, ctxModel,
                                           input,x0,y0, Log2CbSize, ctDepth, qp);
  enc_cb* cb_split = NULL;
  enc_cb* cb = cb_no_split;

  if (Log2CbSize > ectx->sps.Log2MinCbSizeY) {
    cb_split = encode_cb_split(ectx, ctxSplit,
                               input,x0,y0, Log2CbSize, ctDepth, qp);

    float rd_cost_split    = cb_split->distortion    + lambda * cb_split->rate;
    float rd_cost_no_split = cb_no_split->distortion + lambda * cb_no_split->rate;

    bool split =  (rd_cost_split < rd_cost_no_split);

    if (split) {
      cb = cb_split;
    }
    else {
      cb->write_to_image(&ectx->img, x0,y0, true);
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

        // make a copy of the context model that we can modify for testing alternatives

        context_model_table ctxModel;
        copy_context_model_table(ctxModel, ectx->ctx_model_bitstream);

        enc_cb* cb = encode_cb_may_split(ectx, ctxModel,
                                         input, x0,y0, Log2CtbSize, 0, qp);

        cb->write_to_image(&ectx->img, x<<Log2CtbSize, y<<Log2CtbSize, true);


        // --- write bitstream ---

        ectx->switch_to_CABAC_stream();
        //int preSize = ectx->cabac->size();
        encode_ctb(ectx, cb, x,y);
        //int postSize = ectx->cabac->size();
        //printf("real: %d  estim: %f\n",postSize-preSize, cb->rate/8);


        int last = (y==ectx->sps.PicHeightInCtbsY-1 &&
                    x==ectx->sps.PicWidthInCtbsY-1);
        ectx->cabac_bitstream.write_CABAC_term_bit(last);


        ectx->free_all_pools();
      }


  // frame PSNR

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
  ectx->sps.set_TB_log2size_range( Log2(ectx->params.min_tb_size), Log2(ectx->params.max_tb_size));
  ectx->sps.max_transform_hierarchy_depth_intra = ectx->params.max_transform_hierarchy_depth_intra;

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

