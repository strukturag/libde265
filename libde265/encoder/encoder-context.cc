/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: Dirk Farin <farin@struktur.de>
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

#include "encoder/encoder-context.h"
#include "libde265/encoder/encoder-syntax.h"
#include "libde265/util.h"
#include "libde265/image.h"

#include <math.h>
#include <algorithm>

#define ENCODER_DEVELOPMENT 0
#define COMPARE_ESTIMATED_RATE_TO_REAL_RATE 0


double encode_image(encoder_context*, std::shared_ptr<const image> input, EncoderCore&);


encoder_context::encoder_context()
  : m_input_image_history(this)
{
  encoder_started=false;

  //img_source = NULL;
  //reconstruction_sink = NULL;
  //packet_sink = NULL;

  parameters_have_been_set = false;

  image_allocation_functions = image::default_image_allocation;

  use_adaptive_context = true; //false;

  //enc_coeff_pool.set_blk_size(64*64*20); // TODO: this a guess

  //switch_CABAC_to_bitstream();


  param_CPU_capabilities = de265_get_CPU_capabilites_all_autodetected();

  acceleration.init(param_CPU_capabilities,
                    param_inexact_decoding_flags);
}


encoder_context::~encoder_context()
{
  while (!output_packets.empty()) {
    en265_free_packet(this, output_packets.front());
    output_packets.pop_front();
  }
}


void encoder_context::set_encoder_core(std::shared_ptr<EncoderCore> core)
{
  algocore=core;

  params_config.reset();
  algocore->registerParams(params_config);
}


void encoder_context::push_picture(image_ptr img)
{
  algocore->push_picture(img);
}


void encoder_context::push_end_of_input()
{
  algocore->push_end_of_input();
}


void encoder_context::start_encoder()
{
  if (encoder_started) {
    return;
  }

  algocore->initialize(&picbuf, this);

  encoder_started=true;
}


void encoder_context::fill_headers(std::shared_ptr<video_parameter_set> vps,
                                   std::shared_ptr<seq_parameter_set> sps,
                                   std::shared_ptr<pic_parameter_set> pps,
                                   image_ptr img) const
{
  algocore->fill_headers(vps,sps,pps, img);
}


en265_packet* encoder_context::create_packet(en265_packet_content_type t,
                                             CABAC_encoder_bitstream& cabac)
{
  en265_packet* pck = new en265_packet;

  uint8_t* data = new uint8_t[cabac.size()];
  memcpy(data, cabac.data(), cabac.size());

  pck->version = 1;

  pck->data = data;
  pck->length = cabac.size();

  pck->frame_number = -1;
  pck->content_type = t;
  pck->complete_picture = 0;
  pck->final_slice = 0;
  pck->dependent_slice = 0;
  //pck->pts = 0;
  //pck->user_data = NULL;
  pck->nuh_layer_id = 0;
  pck->nuh_temporal_id = 0;

  pck->encoder_context = this;

  pck->input_image    = NULL;
  pck->reconstruction = NULL;

  cabac_encoder.reset();

  return pck;
}


de265_error encoder_context::encode_picture_from_input_buffer()
{
  if (!picbuf.have_more_frames_to_encode()) {
    return DE265_OK;
  }


  imgdata = picbuf.get_next_picture_to_encode();


  if (!parameters_have_been_set) {

    // TODO: must be <30, because Y->C mapping (tab8_22) is not implemented yet
    int qp = algocore->getPPS_QP();

    //lambda = ectx->params.lambda;
    lambda = 0.0242 * pow(1.27245, qp);

    parameters_have_been_set = true;
  }


  imgdata->mark_encoding_started();

  this->shdr    = imgdata->ctbs.get_slice_header(0,0).get(); // TODO: HACK

  loginfo(LogEncoder,"encoding frame %d\n",imgdata->frame_number);




  // --- send the image headers if not sent already

  if (m_vps[ imgdata->vps->video_parameter_set_id ] != imgdata->vps) {
    m_vps[ imgdata->vps->video_parameter_set_id ] = imgdata->vps;

    // send VPS

    en265_packet* pck;
    nal_header nal;
    CABAC_encoder_bitstream cabac_encoder;

    nal.set(NAL_UNIT_VPS_NUT);
    nal.write(cabac_encoder);
    imgdata->vps->write(cabac_encoder);
    cabac_encoder.add_trailing_bits();
    cabac_encoder.flush_VLC();
    pck = create_packet(EN265_PACKET_VPS, cabac_encoder);
    pck->nal_unit_type = EN265_NUT_VPS;
    push_output_packet(pck);
  }

  if (m_sps[ imgdata->sps->seq_parameter_set_id ] != imgdata->sps) {
    m_sps[ imgdata->sps->seq_parameter_set_id ] = imgdata->sps;

    // send SPS

    en265_packet* pck;
    nal_header nal;
    CABAC_encoder_bitstream cabac_encoder;

    nal.set(NAL_UNIT_SPS_NUT);
    nal.write(cabac_encoder);
    imgdata->sps->write(cabac_encoder);
    cabac_encoder.add_trailing_bits();
    cabac_encoder.flush_VLC();
    pck = create_packet(EN265_PACKET_SPS, cabac_encoder);
    pck->nal_unit_type = EN265_NUT_SPS;
    push_output_packet(pck);
  }

  if (m_pps[ imgdata->pps->pic_parameter_set_id ] != imgdata->pps) {
    m_pps[ imgdata->pps->pic_parameter_set_id ] = imgdata->pps;

    // send PPS

    en265_packet* pck;
    nal_header nal;
    CABAC_encoder_bitstream cabac_encoder;

    nal.set(NAL_UNIT_PPS_NUT);
    nal.write(cabac_encoder);
    imgdata->pps->write(cabac_encoder, imgdata->sps.get());
    cabac_encoder.add_trailing_bits();
    cabac_encoder.flush_VLC();
    pck = create_packet(EN265_PACKET_PPS, cabac_encoder);
    pck->nal_unit_type = EN265_NUT_PPS;
    push_output_packet(pck);
  }


  // write slice header

  // slice

  shdr->slice_deblocking_filter_disabled_flag = true;
  shdr->slice_loop_filter_across_slices_enabled_flag = false;
  shdr->compute_derived_values(get_pps().get());

  shdr->set_pps(get_pps()); //get_pps_ptr() );

  //shdr.slice_pic_order_cnt_lsb = poc & 0xFF;

  imgdata->nal.write(cabac_encoder);
  shdr->write(cabac_encoder, get_sps().get(), get_pps().get(), imgdata->nal.nal_unit_type);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();


  // encode image

  cabac_encoder.init_CABAC();
  double psnr = encode_image();
  loginfo(LogEncoder,"  PSNR-Y: %f\n", psnr);
  cabac_encoder.flush_CABAC();
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();


  // set reconstruction image

  /*
  imgdata->set_reconstruction_image(img);
  //picbuf.set_prediction_image(imgdata->frame_number, prediction);
  img.reset();
  this->imgdata = NULL;
  this->shdr = NULL;
  */

  // build output packet

  en265_packet* pck = create_packet(EN265_PACKET_SLICE, cabac_encoder);
  //pck->input_image    = imgdata->input;
  //pck->reconstruction = imgdata->reconstruction;
  pck->frame_number   = imgdata->frame_number;
  pck->nal_unit_type  = (enum en265_nal_unit_type)imgdata->nal.nal_unit_type;
  pck->nuh_layer_id   = imgdata->nal.nuh_layer_id;
  pck->nuh_temporal_id= imgdata->nal.nuh_temporal_id;
  output_packets.push_back(pck);


  imgdata->mark_encoding_finished();

  return DE265_OK;
}


// /*LIBDE265_API*/ ImageSink_YUV reconstruction_sink;

double encoder_context::encode_image()
{
#if 1
  /*
  int stride=input->get_image_stride(0);

  int w = ectx->get_sps()->pic_width_in_luma_samples;
  int h = ectx->get_sps()->pic_height_in_luma_samples;

  // --- create reconstruction image ---
  ectx->img = std::make_shared<image>();
  ectx->img->set_headers(ectx->get_vps(), ectx->get_sps(), ectx->get_pps());
  ectx->img->PicOrderCntVal = input->PicOrderCntVal;

  ectx->img->alloc_image(w,h, input->get_chroma_format(), 8,8,
                         0, // PTS
                         image::supplementary_data(),
                         NULL, // user data
                         nullptr); // alloc_funcs
  ectx->img->set_encoder_context(ectx);

  ectx->img->alloc_metadata(ectx->get_sps());
  ectx->img->clear_metadata();
  */

  img = imgdata->reconstruction;

#if 0
  if (1) {
    ectx->prediction = new image;
    ectx->prediction->alloc_image(w,h, input->get_chroma_format(), &ectx->sps, false /* no metadata */,
                                  NULL /* no decctx */, NULL /* no encctx */, 0,NULL,false);
    ectx->prediction->vps = ectx->vps;
    ectx->prediction->sps = ectx->sps;
    ectx->prediction->pps = ectx->pps;
  }
#endif

  active_qp = get_pps()->pic_init_qp; // TODO take current qp from slice


  cabac_ctx_models.init(shdr->initType, shdr->SliceQPY);
  cabac_encoder.set_context_models(&cabac_ctx_models);


  context_model_table modelEstim;
  CABAC_encoder_estim cabacEstim;

  modelEstim.init(shdr->initType, shdr->SliceQPY);
  cabacEstim.set_context_models(&modelEstim);


  int Log2CtbSize = get_sps()->Log2CtbSizeY;

  uint8_t* luma_plane = img->get_image_plane(0);
  uint8_t* cb_plane   = img->get_image_plane(1);
  uint8_t* cr_plane   = img->get_image_plane(2);

  double mse=0;


  // --- run image preprocessing algorithms

  algocore->preprocess_image(this, imgdata);


  // encode CTB by CTB

  //imgdata->ctbs.clear();
  imgdata->ctbs.set_pps(get_pps());

  for (int y=0;y<get_sps()->PicHeightInCtbsY;y++)
    for (int x=0;x<get_sps()->PicWidthInCtbsY;x++)
      {
        img->set_SliceAddrRS(x, y, shdr->SliceAddrRS);

        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;

        loginfo(LogSlice,"encode CTB at %d %d\n",x0,y0);

        // make a copy of the context model that we can modify for testing alternatives

        context_model_table ctxModel;
        //copy_context_model_table(ctxModel, ectx->ctx_model_bitstream);
        ctxModel = cabac_ctx_models.copy();
        ctxModel = modelEstim.copy(); // TODO TMP

        disable_logging(LogSymbols);
        enable_logging(LogSymbols);  // TODO TMP

        //printf("================================================== ANALYZE\n");

#if 1
        /*
          enc_cb* cb = encode_cb_may_split(ectx, ctxModel,
          input, x0,y0, Log2CtbSize, 0, qp);
        */

        enc_cb* cb = algocore->getCTBAlgo()->analyze(this,ctxModel, x0,y0);
#else
        float minCost = std::numeric_limits<float>::max();
        int bestQ = 0;
        int qp = ectx->params.constant_QP;

        enc_cb* cb;
        for (int q=1;q<51;q++) {
          copy_context_model_table(ctxModel, ectx->ctx_model_bitstream);

          enc_cb* cbq = encode_cb_may_split(ectx, ctxModel,
                                            input, x0,y0, Log2CtbSize, 0, q);

          float cost = cbq->distortion + ectx->lambda * cbq->rate;
          if (cost<minCost) { minCost=cost; bestQ=q; }

          if (q==qp) { cb=cbq; }
        }

        printf("Q %d\n",bestQ);
        fflush(stdout);
#endif

        //print_cb_tree_rates(cb,0);

        //statistics_IntraPredMode(ectx, x0,y0, cb);


        // --- write bitstream ---

        //ectx->switch_CABAC_to_bitstream();

        enable_logging(LogSymbols);

        logdebug(LogEncoder,"write CTB %d;%d\n",x,y);

        if (logdebug_enabled(LogEncoder)) {
          cb->debug_dumpTree(enc_tb::DUMPTREE_ALL);
        }

        /*
        cb->debug_assertTreeConsistency(ectx->img);

        //cb->invalidateMetadataInSubTree(ectx->img);
        cb->writeMetadata(ectx, ectx->img,
                          enc_node::METADATA_INTRA_MODES |
                          enc_node::METADATA_RECONSTRUCTION |
                          enc_node::METADATA_CT_DEPTH);

        cb->debug_assertTreeConsistency(ectx->img);
        */

        imgdata->ctbs.encode_ctb(&cabac_encoder, x,y);

        imgdata->ctbs.getCTB(x,y)->writeReconstructionToImage(img.get(), get_sps().get());

        //printf("================================================== WRITE\n");


        if (COMPARE_ESTIMATED_RATE_TO_REAL_RATE) {
          float realPre = cabacEstim.getRDBits();
          imgdata->ctbs.encode_ctb(&cabacEstim, x,y);
          float realPost = cabacEstim.getRDBits();

          printf("estim: %f  real: %f  diff: %f\n",
                 cb->rate,
                 realPost-realPre,
                 cb->rate - (realPost-realPre));
        }


        int last = (y==get_sps()->PicHeightInCtbsY-1 &&
                    x==get_sps()->PicWidthInCtbsY-1);
        cabac_encoder.write_CABAC_term_bit(last);

        //imgdata->ctbs.set_slice_header_id(x,y, sliceHeaderID);

        //ectx->free_all_pools();

        mse += cb->distortion;
      }

  mse /= img->get_width() * img->get_height();


  //reconstruction_sink.send_image(ectx->img);


  //statistics_print();


  //delete ectx->prediction;


  // we write the reconstruction after each CTB
  //ectx->ctbs.writeReconstructionToImage(ectx->img, &ectx->get_sps());

#if 0
  std::ofstream ostr("out.pgm");
  ostr << "P5\n" << ectx->img->get_width() << " " << ectx->img->get_height() << "\n255\n";
  for (int y=0;y<ectx->img->get_height();y++) {
    ostr.write( (char*)ectx->img->get_image_plane_at_pos(0,0,y), ectx->img->get_width() );
  }
#endif

  // --- frame PSNR

  double psnr = 10*log10(255.0*255.0 / mse);

#if ENABLE_TEST_MODE
  double psnr2 = PSNR(MSE(imgdata->input->get_image_plane(0),
                          imgdata->input->get_image_stride(0),
                          luma_plane,
                          img->get_image_stride(0),
                          imgdata->input->get_width(),
                          imgdata->input->get_height()));

  printf("rate-estim PSNR: %f vs %f\n",psnr,psnr2);
#endif

  return psnr;
#endif
  return 0.0; // TMP
}


encoder_context_scc::encoder_context_scc()
{
  vps = std::make_shared<video_parameter_set>();
  sps = std::make_shared<seq_parameter_set>();
  pps = std::make_shared<pic_parameter_set>();

  vps->set_defaults(Profile_Main, 6,2);

  sps->set_CB_size_range(8,8);
  sps->set_TB_size_range(8,8);
  sps->set_PCM_size_range(8,8);

  sps->pcm_enabled_flag = true;
  sps->pcm_sample_bit_depth_luma = 6;
  sps->pcm_sample_bit_depth_chroma = 5;
  sps->max_transform_hierarchy_depth_inter = 0;
  sps->max_transform_hierarchy_depth_intra = 0;

  pps->set_defaults();
  pps->pic_disable_deblocking_filter_flag = 1;
  pps->pps_loop_filter_across_slices_enabled_flag = false;
  pps->transquant_bypass_enable_flag = true;
  pps->sps = sps;
}


void encoder_context_scc::push_image(image_ptr img)
{
  if (state == Uninitialized) {
    set_image_parameters(img);
    send_headers();
    state=Encoding;
  }


  // encode slice header

  // TODO: multi-slice pictures
  std::shared_ptr<slice_segment_header> shdr = std::make_shared<slice_segment_header>();
  shdr->slice_type = SLICE_TYPE_I;
  shdr->slice_pic_order_cnt_lsb = 0; // TODO get_pic_order_count_lsb();
  shdr->slice_deblocking_filter_disabled_flag = true;
  shdr->slice_loop_filter_across_slices_enabled_flag = false;
  shdr->first_slice_segment_in_pic_flag = true;
  shdr->five_minus_max_num_merge_cand = 5-1;
  shdr->set_pps(pps); //get_pps_ptr() );

  shdr->compute_derived_values(pps.get());


  nal_header nal(NAL_UNIT_IDR_N_LP);
  nal.write(cabac_encoder);
  shdr->write(cabac_encoder, sps.get(), pps.get(), nal.nal_unit_type);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();


  // initialize CABAC models

  cabac_ctx_models.init(shdr->initType, shdr->SliceQPY);
  cabac_encoder.set_context_models(&cabac_ctx_models);


  // encode image

  ctbs.alloc(img->get_width(), img->get_height(), sps->Log2CtbSizeY);
  uint16_t sliceHeaderID = ctbs.add_slice_header(shdr);
  ctbs.set_pps(pps);
  ctbs.set_input_image(img);



  Algo_CB_IntraPartMode_Fixed algo_intra_part_mode(PART_2Nx2N);
  Algo_TB_IntraPredMode_MinResidual algo_intra_pred_mode;
  Algo_TB_Split_BruteForce algo_tb_split;
  Algo_TB_Transform algo_tb_transform;

  algo_intra_part_mode.setChildAlgo(&algo_intra_pred_mode);

  algo_intra_pred_mode.setChildAlgo(&algo_tb_split);
  algo_intra_pred_mode.enableIntraPredModeSubset(ALGO_TB_IntraPredMode_Subset_HVPlus);

  algo_tb_split.setAlgo_TB_Split(&algo_intra_pred_mode);
  algo_tb_split.setAlgo_TB_NoSplit(&algo_tb_transform);


  for (int y=0;y<sps->PicHeightInCtbsY;y++)
    for (int x=0;x<sps->PicWidthInCtbsY;x++) {
      //ectx->img->set_SliceAddrRS(x, y, ectx->shdr->SliceAddrRS);

      int Log2CtbSize = sps->Log2CtbSizeY;
      int x0 = x<<Log2CtbSize;
      int y0 = y<<Log2CtbSize;

      enc_cb* cb = new enc_cb();
      cb->log2Size = sps->Log2CtbSizeY;
      cb->x = x0;
      cb->y = y0;

      cb->split_cu_flag = 0;
      cb->ctDepth = 0;

      cb->qp = 0; // TODO ectx->active_qp;
      cb->PredMode = MODE_INTRA;
      cb->PartMode = PART_2Nx2N;

      if (false) {
        // PCM

        cb->cu_transquant_bypass_flag = false;
        cb->pcm_flag = true;

        cb->intra.pcm_data_ptr[0] = img->get_image_plane_at_pos(0, x0,y0);
        cb->intra.pcm_data_ptr[1] = img->get_image_plane_at_pos(1, x0,y0);
        cb->intra.pcm_data_ptr[2] = img->get_image_plane_at_pos(2, x0,y0);
      }
      else {
        // Transquant

        cb->cu_transquant_bypass_flag = false;
        cb->pcm_flag = false;

        //cb = algo_intra_part_mode.analyze(ectx,ctxModel, x0,y0);
      }

      ctbs.setCTB(x,y, cb);
      ctbs.set_slice_header_id(x,y, sliceHeaderID);

      ctbs.encode_ctb(&cabac_encoder, x,y);


      int last = (y==sps->PicHeightInCtbsY-1 &&
                  x==sps->PicWidthInCtbsY-1);
      cabac_encoder.write_CABAC_term_bit(last);
    }


  en265_packet* pck = copy_encoded_data_into_packet(EN265_PACKET_SLICE);
  pck->nal_unit_type = EN265_NUT_IDR_N_LP;
  output_packets.push_back(pck);
}


en265_packet* encoder_context_scc::get_next_packet()
{
  if (output_packets.empty()) {
    return nullptr;
  }
  else {
    en265_packet* pck = output_packets.front();
    output_packets.pop_front();
    return pck;
  }
}


void encoder_context_scc::set_image_parameters(image_ptr img)
{
  sps->chroma_format_idc = img->get_chroma_format();

  sps->set_coded_resolution(img->get_width(),
                            img->get_height());

  /*
  de265_error err = sps->compute_derived_values(false);
  if (err != DE265_OK) {
    // TODO: handle error
    printf("ERR: %d\n",err);
    exit(10);
  }
  */

  pps->set_derived_values(sps.get());
}


void encoder_context_scc::send_headers()
{
  nal_header nal;

  // write headers

  en265_packet* pck;

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(cabac_encoder);
  vps->write(cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = copy_encoded_data_into_packet(EN265_PACKET_VPS);
  pck->nal_unit_type = EN265_NUT_VPS;
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(cabac_encoder);
  sps->write(cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = copy_encoded_data_into_packet(EN265_PACKET_SPS);
  pck->nal_unit_type = EN265_NUT_SPS;
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(cabac_encoder);
  pps->write(cabac_encoder, sps.get());
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = copy_encoded_data_into_packet(EN265_PACKET_PPS);
  pck->nal_unit_type = EN265_NUT_PPS;
  output_packets.push_back(pck);
}


en265_packet* encoder_context_scc::copy_encoded_data_into_packet(en265_packet_content_type type)
{
  en265_packet* pck = new en265_packet;

  pck->version = 1;
  pck->content_type = type;

  pck->length = cabac_encoder.size();
  pck->data = cabac_encoder.detach_data();
  cabac_encoder.reset();


  pck->frame_number = -1;
  pck->complete_picture = 0;
  pck->final_slice = 0;
  pck->dependent_slice = 0;
  pck->nuh_layer_id = 0;
  pck->nuh_temporal_id = 0;

  //pck->encoder_context = this;

  //pck->pts = 0;
  //pck->user_data = NULL;
//pck->input_image = NULL;
//  pck->reconstruction = NULL;

  return pck;
}
