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

#define ENCODER_DEVELOPMENT 0
#define COMPARE_ESTIMATED_RATE_TO_REAL_RATE 0


double encode_image(encoder_context*, std::shared_ptr<const image> input, EncoderCore&);


encoder_context::encoder_context()
  : m_input_image_history(this)
{
  encoder_started=false;

  vps = std::make_shared<video_parameter_set>();
  sps = std::make_shared<seq_parameter_set>();
  pps = std::make_shared<pic_parameter_set>();

  //img_source = NULL;
  //reconstruction_sink = NULL;
  //packet_sink = NULL;

  image_spec_is_defined = false;
  parameters_have_been_set = false;
  headers_have_been_sent = false;

  image_allocation_functions = image::default_image_allocation;

  use_adaptive_context = true; //false;

  //enc_coeff_pool.set_blk_size(64*64*20); // TODO: this a guess

  //switch_CABAC_to_bitstream();


  params.registerParams(params_config);
  algo.registerParams(params_config);
}


encoder_context::~encoder_context()
{
  while (!output_packets.empty()) {
    en265_free_packet(this, output_packets.front());
    output_packets.pop_front();
  }
}


void encoder_context::start_encoder()
{
  if (encoder_started) {
    return;
  }


  if (params.sop_structure() == SOP_Intra) {
    sop = std::shared_ptr<sop_creator_intra_only>(new sop_creator_intra_only());
  }
  else {
    auto s = std::shared_ptr<sop_creator_trivial_low_delay>(new sop_creator_trivial_low_delay());
    s->setParams(params.mSOP_LowDelay);
    sop = s;
  }

  sop->set_encoder_context(this);
  sop->set_encoder_picture_buffer(&picbuf);


  encoder_started=true;
}


en265_packet* encoder_context::create_packet(en265_packet_content_type t)
{
  en265_packet* pck = new en265_packet;

  uint8_t* data = new uint8_t[cabac_encoder.size()];
  memcpy(data, cabac_encoder.data(), cabac_encoder.size());

  pck->version = 1;

  pck->data = data;
  pck->length = cabac_encoder.size();

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

//pck->input_image = NULL;
//  pck->reconstruction = NULL;

  cabac_encoder.reset();

  return pck;
}


de265_error encoder_context::encode_headers()
{
  nal_header nal;

  // VPS

  vps->set_defaults(Profile_Main, 6,2);


  // SPS

  sps->set_defaults();
  sps->set_CB_log2size_range( Log2(params.min_cb_size), Log2(params.max_cb_size));
  sps->set_TB_log2size_range( Log2(params.min_tb_size), Log2(params.max_tb_size));
  sps->max_transform_hierarchy_depth_intra = params.max_transform_hierarchy_depth_intra;
  sps->max_transform_hierarchy_depth_inter = params.max_transform_hierarchy_depth_inter;

  if (imgdata->input->get_chroma_format() == de265_chroma_444) {
    sps->chroma_format_idc = CHROMA_444;
  }

  sps->set_resolution(image_width, image_height);
  sop->set_SPS_header_values();
  de265_error err = sps->compute_derived_values(true);
  if (err != DE265_OK) {
    fprintf(stderr,"invalid SPS parameters\n");
    exit(10);
  }


  // PPS

  pps->set_defaults();
  pps->sps = sps;
  pps->pic_init_qp = algo.getPPS_QP();

  // turn off deblocking filter
  pps->deblocking_filter_control_present_flag = true;
  pps->deblocking_filter_override_enabled_flag = false;
  pps->pic_disable_deblocking_filter_flag = true;
  pps->pps_loop_filter_across_slices_enabled_flag = false;

  pps->set_derived_values(sps.get());



  // write headers

  en265_packet* pck;

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(cabac_encoder);
  vps->write(this, cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = create_packet(EN265_PACKET_VPS);
  pck->nal_unit_type = EN265_NUT_VPS;
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(cabac_encoder);
  sps->write(this, cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = create_packet(EN265_PACKET_SPS);
  pck->nal_unit_type = EN265_NUT_SPS;
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(cabac_encoder);
  pps->write(this, cabac_encoder, sps.get());
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = create_packet(EN265_PACKET_PPS);
  pck->nal_unit_type = EN265_NUT_PPS;
  output_packets.push_back(pck);



  headers_have_been_sent = true;

  return DE265_OK;
}


de265_error encoder_context::encode_picture_from_input_buffer()
{
  if (!picbuf.have_more_frames_to_encode()) {
    return DE265_OK;
  }


  if (!image_spec_is_defined) {
    const image_data* id = picbuf.peek_next_picture_to_encode();
    image_width  = id->input->get_width();
    image_height = id->input->get_height();
    image_spec_is_defined = true;

    ctbs.alloc(image_width, image_height, Log2(params.max_cb_size));
  }


  if (!parameters_have_been_set) {
    algo.setParams(params);


    // TODO: must be <30, because Y->C mapping (tab8_22) is not implemented yet
    int qp = algo.getPPS_QP();

    //lambda = ectx->params.lambda;
    lambda = 0.0242 * pow(1.27245, qp);

    parameters_have_been_set = true;
  }





  image_data* imgdata;
  imgdata = picbuf.get_next_picture_to_encode();
  assert(imgdata);
  picbuf.mark_encoding_started(imgdata->frame_number);

  this->imgdata = imgdata;
  this->shdr    = &imgdata->shdr;
  loginfo(LogEncoder,"encoding frame %d\n",imgdata->frame_number);


  // write headers if not written yet

  if (!headers_have_been_sent) {
    encode_headers();
  }


  // write slice header

  // slice

  imgdata->shdr.slice_deblocking_filter_disabled_flag = true;
  imgdata->shdr.slice_loop_filter_across_slices_enabled_flag = false;
  imgdata->shdr.compute_derived_values(pps.get());

  imgdata->shdr.set_pps(pps); //get_pps_ptr() );

  //shdr.slice_pic_order_cnt_lsb = poc & 0xFF;

  imgdata->nal.write(cabac_encoder);
  imgdata->shdr.write(this, cabac_encoder, sps.get(), pps.get(), imgdata->nal.nal_unit_type);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();


  // encode image

  cabac_encoder.init_CABAC();
  double psnr = encode_image(this,imgdata->input, algo);
  loginfo(LogEncoder,"  PSNR-Y: %f\n", psnr);
  cabac_encoder.flush_CABAC();
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();


  // set reconstruction image

  picbuf.set_reconstruction_image(imgdata->frame_number, img);
  //picbuf.set_prediction_image(imgdata->frame_number, prediction);
  img.reset();
  this->imgdata = NULL;
  this->shdr = NULL;

  // build output packet

  en265_packet* pck = create_packet(EN265_PACKET_SLICE);
  //pck->input_image    = imgdata->input;
  //pck->reconstruction = imgdata->reconstruction;
  pck->frame_number   = imgdata->frame_number;
  pck->nal_unit_type  = (enum en265_nal_unit_type)imgdata->nal.nal_unit_type;
  pck->nuh_layer_id   = imgdata->nal.nuh_layer_id;
  pck->nuh_temporal_id= imgdata->nal.nuh_temporal_id;
  output_packets.push_back(pck);


  picbuf.mark_encoding_finished(imgdata->frame_number);

  return DE265_OK;
}


// /*LIBDE265_API*/ ImageSink_YUV reconstruction_sink;

double encode_image(encoder_context* ectx,
                    std::shared_ptr<const image> input,
                    EncoderCore& algo)
{
  int stride=input->get_image_stride(0);

  int w = ectx->get_sps().pic_width_in_luma_samples;
  int h = ectx->get_sps().pic_height_in_luma_samples;

  // --- create reconstruction image ---
  ectx->img = std::make_shared<image>();
  ectx->img->set_headers(ectx->get_shared_vps(), ectx->get_shared_sps(), ectx->get_shared_pps());
  ectx->img->PicOrderCntVal = input->PicOrderCntVal;

  ectx->img->alloc_image(w,h, input->get_chroma_format(), 8,8,
                         0, // PTS
                         image::supplementary_data(),
                         NULL, // user data
                         nullptr); // alloc_funcs
  ectx->img->set_encoder_context(ectx);

  ectx->img->alloc_metadata(ectx->get_shared_sps());
  ectx->img->clear_metadata();

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

  ectx->active_qp = ectx->get_pps().pic_init_qp; // TODO take current qp from slice


  ectx->cabac_ctx_models.init(ectx->shdr->initType, ectx->shdr->SliceQPY);
  ectx->cabac_encoder.set_context_models(&ectx->cabac_ctx_models);


  context_model_table modelEstim;
  CABAC_encoder_estim cabacEstim;

  modelEstim.init(ectx->shdr->initType, ectx->shdr->SliceQPY);
  cabacEstim.set_context_models(&modelEstim);


  int Log2CtbSize = ectx->get_sps().Log2CtbSizeY;

  uint8_t* luma_plane = ectx->img->get_image_plane(0);
  uint8_t* cb_plane   = ectx->img->get_image_plane(1);
  uint8_t* cr_plane   = ectx->img->get_image_plane(2);

  double mse=0;


  // encode CTB by CTB

  ectx->ctbs.clear();

  for (int y=0;y<ectx->get_sps().PicHeightInCtbsY;y++)
    for (int x=0;x<ectx->get_sps().PicWidthInCtbsY;x++)
      {
        ectx->img->set_SliceAddrRS(x, y, ectx->shdr->SliceAddrRS);

        int x0 = x<<Log2CtbSize;
        int y0 = y<<Log2CtbSize;

        logtrace(LogSlice,"encode CTB at %d %d\n",x0,y0);

        // make a copy of the context model that we can modify for testing alternatives

        context_model_table ctxModel;
        //copy_context_model_table(ctxModel, ectx->ctx_model_bitstream);
        ctxModel = ectx->cabac_ctx_models.copy();
        ctxModel = modelEstim.copy(); // TODO TMP

        disable_logging(LogSymbols);
        enable_logging(LogSymbols);  // TODO TMP

        //printf("================================================== ANALYZE\n");

#if 1
        /*
          enc_cb* cb = encode_cb_may_split(ectx, ctxModel,
          input, x0,y0, Log2CtbSize, 0, qp);
        */

        enc_cb* cb = algo.getAlgoCTBQScale()->analyze(ectx,ctxModel, x0,y0);
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

        encode_ctb(ectx, &ectx->cabac_encoder, cb, x,y);

        ectx->ctbs.getCTB(x,y)->writeReconstructionToImage(ectx->img.get(), &ectx->get_sps());

        //printf("================================================== WRITE\n");


        if (COMPARE_ESTIMATED_RATE_TO_REAL_RATE) {
          float realPre = cabacEstim.getRDBits();
          encode_ctb(ectx, &cabacEstim, cb, x,y);
          float realPost = cabacEstim.getRDBits();

          printf("estim: %f  real: %f  diff: %f\n",
                 cb->rate,
                 realPost-realPre,
                 cb->rate - (realPost-realPre));
        }


        int last = (y==ectx->get_sps().PicHeightInCtbsY-1 &&
                    x==ectx->get_sps().PicWidthInCtbsY-1);
        ectx->cabac_encoder.write_CABAC_term_bit(last);

        //delete cb;

        //ectx->free_all_pools();

        mse += cb->distortion;
      }

  mse /= ectx->img->get_width() * ectx->img->get_height();


  //reconstruction_sink.send_image(ectx->img);


  //statistics_print();


  //delete ectx->prediction;


  // frame PSNR

  // we write the reconstruction after each CTB
  //ectx->ctbs.writeReconstructionToImage(ectx->img, &ectx->get_sps());

#if 0
  std::ofstream ostr("out.pgm");
  ostr << "P5\n" << ectx->img->get_width() << " " << ectx->img->get_height() << "\n255\n";
  for (int y=0;y<ectx->img->get_height();y++) {
    ostr.write( (char*)ectx->img->get_image_plane_at_pos(0,0,y), ectx->img->get_width() );
  }
#endif

  double psnr = 10*log10(255.0*255.0 / mse);

#if 0
  double psnr2 = PSNR(MSE(input->get_image_plane(0), input->get_image_stride(0),
                          luma_plane, ectx->img->get_image_stride(0),
                          input->get_width(), input->get_height()));

  printf("rate-estim PSNR: %f vs %f\n",psnr,psnr2);
#endif

  return psnr;
}


encoder_context_scc::encoder_context_scc()
{
  vps = std::make_shared<video_parameter_set>();
  sps = std::make_shared<seq_parameter_set>();
  pps = std::make_shared<pic_parameter_set>();

  vps->set_defaults(Profile_Main, 6,2);

  sps->set_CB_size_range(16,16);
  sps->set_PCM_size_range(16,16);

  pps->pic_disable_deblocking_filter_flag = 1;
  pps->pps_loop_filter_across_slices_enabled_flag = false;
}


void encoder_context_scc::push_image(image_ptr img)
{
  printf("%d %d\n",img->get_width(), img->get_height());

  if (state == Uninitialized) {
    set_image_parameters(img);
    send_headers();
    state=Encoding;
  }


  // encode slice header

  slice_segment_header shdr; // TODO: multi-slice pictures
  shdr.slice_type = SLICE_TYPE_I;
  shdr.slice_pic_order_cnt_lsb = 0; // TODO get_pic_order_count_lsb();
  shdr.slice_deblocking_filter_disabled_flag = true;
  shdr.slice_loop_filter_across_slices_enabled_flag = false;
  shdr.compute_derived_values(pps.get());

  shdr.set_pps(pps); //get_pps_ptr() );

  nal_header nal(NAL_UNIT_IDR_N_LP);
  nal.write(cabac_encoder);
  shdr.write(nullptr, cabac_encoder, sps.get(), pps.get(), nal.nal_unit_type);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();

  en265_packet* pck = copy_encoded_data_into_packet(EN265_PACKET_SLICE);
  pck->nal_unit_type = EN265_NUT_IDR_N_LP;
  output_packets.push_back(pck);



  // initialize CABAC models

  cabac_ctx_models.init(shdr.initType, shdr.SliceQPY);
  cabac_encoder.set_context_models(&cabac_ctx_models);


  // encoder image

  ctbs.alloc(img->get_width(), img->get_height(), Log2(sps->Log2CtbSizeY));

  for (int y=0;y<sps->PicHeightInCtbsY;y++)
    for (int x=0;x<sps->PicWidthInCtbsY;x++) {
      //ectx->img->set_SliceAddrRS(x, y, ectx->shdr->SliceAddrRS);

      int Log2CtbSize = sps->Log2CtbSizeY;
      int x0 = x<<Log2CtbSize;
      int y0 = y<<Log2CtbSize;

      enc_cb* cb = new enc_cb();
      cb->log2Size = sps->Log2CtbSizeY;
      cb->ctDepth = 0;
      cb->x = x0;
      cb->y = y0;
      cb->qp = 0; // TODO ectx->active_qp;
      cb->cu_transquant_bypass_flag = false;
      cb->pcm_flag = true;
    }
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
  switch (img->get_chroma_format()) {
  case de265_chroma_mono: sps->chroma_format_idc = CHROMA_MONO; break;
  case de265_chroma_420:  sps->chroma_format_idc = CHROMA_420;  break;
  case de265_chroma_422:  sps->chroma_format_idc = CHROMA_422;  break;
  case de265_chroma_444:  sps->chroma_format_idc = CHROMA_444;  break;
    // TODO: CHROMA_444_SEPARATE
  }

  sps->pic_width_in_luma_samples = img->get_width();
  sps->pic_height_in_luma_samples = img->get_height();

  de265_error err = sps->compute_derived_values(false);
  if (err != DE265_OK) {
    // TODO: handle error
    printf("ERR: %d\n",err);
    exit(10);
  }
}


void encoder_context_scc::send_headers()
{
  nal_header nal;

  // write headers

  en265_packet* pck;

  nal.set(NAL_UNIT_VPS_NUT);
  nal.write(cabac_encoder);
  vps->write(nullptr, cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = copy_encoded_data_into_packet(EN265_PACKET_VPS);
  pck->nal_unit_type = EN265_NUT_VPS;
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_SPS_NUT);
  nal.write(cabac_encoder);
  sps->write(nullptr, cabac_encoder);
  cabac_encoder.add_trailing_bits();
  cabac_encoder.flush_VLC();
  pck = copy_encoded_data_into_packet(EN265_PACKET_SPS);
  pck->nal_unit_type = EN265_NUT_SPS;
  output_packets.push_back(pck);

  nal.set(NAL_UNIT_PPS_NUT);
  nal.write(cabac_encoder);
  pps->write(nullptr, cabac_encoder, sps.get());
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
