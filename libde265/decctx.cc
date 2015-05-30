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

#include "decctx.h"
#include "decctx-multilayer.h"
#include "util.h"
#include "sao.h"
#include "sei.h"
#include "deblock.h"

#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "fallback.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_SSE4_1
#include "x86/sse.h"
#endif

#ifdef HAVE_ARM
#include "arm/arm.h"
#endif

#define SAVE_INTERMEDIATE_IMAGES 0

#if SAVE_INTERMEDIATE_IMAGES
#include "visualize.h"
#endif

extern void thread_decode_CTB_row(void* d);
extern void thread_decode_slice_segment(void* d);


thread_context::thread_context()
{
  /*
  CtbAddrInRS = 0;
  CtbAddrInTS = 0;

  CtbX = 0;
  CtbY = 0;
  */

  /*
  refIdx[0] = refIdx[1] = 0;
  mvd[0][0] = mvd[0][1] = mvd[1][0] = mvd[1][1] = 0;
  merge_flag = 0;
  merge_idx = 0;
  mvp_lX_flag[0] = mvp_lX_flag[1] = 0;
  inter_pred_idc = 0;
  */

  /*
  enum IntraPredMode IntraPredModeC; // chroma intra-prediction mode for current CB
  */

  /*
  cu_transquant_bypass_flag = false;
  memset(transform_skip_flag,0, 3*sizeof(uint8_t));
  */


  //memset(coeffList,0,sizeof(int16_t)*3*32*32);
  //memset(coeffPos,0,sizeof(int16_t)*3*32*32);
  //memset(nCoeff,0,sizeof(int16_t)*3);



  IsCuQpDeltaCoded = false;
  CuQpDelta = 0;

  /*
  currentQPY = 0;
  currentQG_x = 0;
  currentQG_y = 0;
  lastQPYinPreviousQG = 0;
  */

  /*
  qPYPrime = 0;
  qPCbPrime = 0;
  qPCrPrime = 0;
  */

  /*
  memset(&cabac_decoder, 0, sizeof(CABAC_decoder));
  memset(&ctx_model, 0, sizeof(ctx_model));
  */

  decctx = NULL;
  img = NULL;
  shdr = NULL;

  imgunit = NULL;
  sliceunit = NULL;


  //memset(this,0,sizeof(thread_context));

  // some compilers/linkers don't align struct members correctly,
  // adjust if necessary
  int offset = (uintptr_t)_coeffBuf & 0x0f;

  if (offset == 0) {
    coeffBuf = _coeffBuf;  // correctly aligned already
  }
  else {
    coeffBuf = (int16_t *) (((uint8_t *)_coeffBuf) + (16-offset));
  }

  memset(coeffBuf, 0, 32*32*sizeof(int16_t));
}


slice_unit::slice_unit(decoder_context* decctx)
  : ctx(decctx),
    nal(NULL),
    shdr(NULL),
    flush_reorder_buffer(false),
    thread_contexts(NULL),
    imgunit(NULL),
    nThreads(0),
    first_decoded_CTB_RS(-1),
    last_decoded_CTB_RS(-1)
{
  state = Unprocessed;
  nThreadContexts = 0;
}

slice_unit::~slice_unit()
{
  ctx->nal_parser->free_NAL_unit(nal);

  if (thread_contexts) {
    delete[] thread_contexts;
  }
}


void slice_unit::allocate_thread_contexts(int n)
{
  assert(thread_contexts==NULL);

  thread_contexts = new thread_context[n];
  nThreadContexts = n;
}


image_unit::image_unit()
{
  img=NULL;
  role=Invalid;
  state=Unprocessed;
}


image_unit::~image_unit()
{
  for (int i=0;i<slice_units.size();i++) {
    delete slice_units[i];
  }

  for (int i=0;i<tasks.size();i++) {
    delete tasks[i];
  }
}


base_context::base_context()
{
  set_acceleration_functions(de265_acceleration_AUTO);
}


decoder_context::decoder_context()
{
  //memset(ctx, 0, sizeof(decoder_context));

  // --- parameters ---

  param_sei_check_hash = false;
  param_conceal_stream_errors = true;
  param_suppress_faulty_pictures = false;

  param_disable_deblocking = false;
  param_disable_sao = false;
  //param_disable_mc_residual_idct = false;
  //param_disable_intra_residual_idct = false;

  // --- processing ---

  param_sps_headers_fd = -1;
  param_vps_headers_fd = -1;
  param_pps_headers_fd = -1;
  param_slice_headers_fd = -1;

  param_image_allocation_functions = de265_image::default_image_allocation;
  param_image_allocation_userdata  = NULL;

  /*
  memset(&vps, 0, sizeof(video_parameter_set)*DE265_MAX_VPS_SETS);
  memset(&sps, 0, sizeof(seq_parameter_set)  *DE265_MAX_SPS_SETS);
  memset(&pps, 0, sizeof(pic_parameter_set)  *DE265_MAX_PPS_SETS);
  memset(&slice,0,sizeof(slice_segment_header)*DE265_MAX_SLICES);
  */

  current_vps = NULL;
  current_sps = NULL;
  current_pps = NULL;

  //memset(&thread_pool,0,sizeof(struct thread_pool));
  num_worker_threads = 0;


  // frame-rate

  limit_HighestTid = 6;   // decode all temporal layers (up to layer 6)
  framerate_ratio = 100;  // decode all 100%

  goal_HighestTid = 6;
  current_HighestTid = 6;
  layer_framerate_ratio = 100;

  compute_framedrop_table();


  //

  current_image_poc_lsb = 0;
  first_decoded_picture = 0;
  NoRaslOutputFlag = 0;
  HandleCraAsBlaFlag = 0;
  FirstAfterEndOfSequenceNAL = 0;
  PicOrderCntMsb = 0;
  prevPicOrderCntLsb = 0;
  prevPicOrderCntMsb = 0;
  img = NULL;

  /*
  int PocLsbLt[MAX_NUM_REF_PICS];
  int UsedByCurrPicLt[MAX_NUM_REF_PICS];
  int DeltaPocMsbCycleLt[MAX_NUM_REF_PICS];
  int CurrDeltaPocMsbPresentFlag[MAX_NUM_REF_PICS];
  int FollDeltaPocMsbPresentFlag[MAX_NUM_REF_PICS];

  int NumPocStCurrBefore;
  int NumPocStCurrAfter;
  int NumPocStFoll;
  int NumPocLtCurr;
  int NumPocLtFoll;

  // These lists contain absolute POC values.
  int PocStCurrBefore[MAX_NUM_REF_PICS]; // used for reference in current picture, smaller POC
  int PocStCurrAfter[MAX_NUM_REF_PICS];  // used for reference in current picture, larger POC
  int PocStFoll[MAX_NUM_REF_PICS]; // not used for reference in current picture, but in future picture
  int PocLtCurr[MAX_NUM_REF_PICS]; // used in current picture
  int PocLtFoll[MAX_NUM_REF_PICS]; // used in some future picture

  // These lists contain indices into the DPB.
  int RefPicSetStCurrBefore[DE265_DPB_SIZE];
  int RefPicSetStCurrAfter[DE265_DPB_SIZE];
  int RefPicSetStFoll[DE265_DPB_SIZE];
  int RefPicSetLtCurr[DE265_DPB_SIZE];
  int RefPicSetLtFoll[DE265_DPB_SIZE];


  uint8_t nal_unit_type;

  char IdrPicFlag;
  char RapPicFlag;
  */



  // --- internal data ---

  first_decoded_picture = true;
  //ctx->FirstAfterEndOfSequenceNAL = true;
  //ctx->last_RAP_picture_NAL_type = NAL_UNIT_UNDEFINED;

  //de265_init_image(&ctx->coeff);

  // --- decoded picture buffer ---

  current_image_poc_lsb = -1; // any invalid number

  // Multilayer extensions
  NumActiveRefLayerPics0 = 0;
  NumActiveRefLayerPics1 = 0;

  for (int i=0; i<MAX_REF_LAYERS; i++) {
    ilRefPic[i] = NULL;
  }
  ilRefPic_upsampled = false;
}


decoder_context::~decoder_context()
{
  while (!image_units.empty()) {
    delete image_units.back();
    image_units.pop_back();
  }

  // delete the created images in ilRefPic[]
  for (int i=0; i<MAX_REF_LAYERS; i++) {
    if (ilRefPic[i] != NULL) {
      delete ilRefPic[i];
      ilRefPic[i] = NULL;
    }
  }
}


void decoder_context::set_image_allocation_functions(de265_image_allocation* allocfunc,
                                                     void* userdata)
{
  if (allocfunc) {
    param_image_allocation_functions = *allocfunc;
    param_image_allocation_userdata  = userdata;
  }
  else {
    assert(false); // actually, it makes no sense to reset the allocation functions

    param_image_allocation_functions = de265_image::default_image_allocation;
    param_image_allocation_userdata  = NULL;
  }
}


de265_error decoder_context::start_thread_pool(int nThreads)
{
  ::start_thread_pool(&thread_pool_, nThreads);

  num_worker_threads = nThreads;

  return DE265_OK;
}


void decoder_context::stop_thread_pool()
{
  if (get_num_worker_threads()>0) {
    //flush_thread_pool(&ctx->thread_pool);
    ::stop_thread_pool(&thread_pool_);
  }
}


void decoder_context::reset()
{
  if (num_worker_threads>0) {
    //flush_thread_pool(&ctx->thread_pool);
    ::stop_thread_pool(&thread_pool_);
  }

  // --------------------------------------------------

#if 0
  ctx->end_of_stream = false;
  ctx->pending_input_NAL = NULL;
  ctx->current_vps = NULL;
  ctx->current_sps = NULL;
  ctx->current_pps = NULL;
  ctx->num_worker_threads = 0;
  ctx->current_image_poc_lsb = 0;
  ctx->first_decoded_picture = 0;
  ctx->NoRaslOutputFlag = 0;
  ctx->HandleCraAsBlaFlag = 0;
  ctx->FirstAfterEndOfSequenceNAL = 0;
  ctx->PicOrderCntMsb = 0;
  ctx->prevPicOrderCntLsb = 0;
  ctx->prevPicOrderCntMsb = 0;
  ctx->NumPocStCurrBefore=0;
  ctx->NumPocStCurrAfter=0;
  ctx->NumPocStFoll=0;
  ctx->NumPocLtCurr=0;
  ctx->NumPocLtFoll=0;
  ctx->nal_unit_type=0;
  ctx->IdrPicFlag=0;
  ctx->RapPicFlag=0;
#endif

  img = NULL;


  // TODO: remove all pending image_units


  // --- decoded picture buffer ---

  current_image_poc_lsb = -1; // any invalid number
  first_decoded_picture = true;


  // --- remove all pictures from output queue ---

  // there was a bug the peek_next_image did not return NULL on empty output queues.
  // This was (indirectly) fixed by recreating the DPB buffer, but it should actually
  // be sufficient to clear it like this.
  // The error showed while scrubbing the ToS video in VLC.
  dpb.clear();

  nal_parser->remove_pending_input_data();


  while (!image_units.empty()) {
    delete image_units.back();
    image_units.pop_back();
  }

  // --- start threads again ---

  if (num_worker_threads>0) {
    // TODO: need error checking
    start_thread_pool(num_worker_threads);
  }
}

void base_context::set_acceleration_functions(enum de265_acceleration l)
{
  // fill scalar functions first (so that function table is completely filled)

  init_acceleration_functions_fallback(&acceleration);


  // override functions with optimized variants

#ifdef HAVE_SSE4_1
  if (l>=de265_acceleration_SSE) {
    init_acceleration_functions_sse(&acceleration);
  }
#endif
#ifdef HAVE_ARM
  if (l>=de265_acceleration_ARM) {
    init_acceleration_functions_arm(&acceleration);
  }
#endif
}


void decoder_context::init_thread_context(thread_context* tctx)
{
  // zero scrap memory for coefficient blocks
  memset(tctx->_coeffBuf, 0, sizeof(tctx->_coeffBuf));  // TODO: check if we can safely remove this

  tctx->currentQG_x = -1;
  tctx->currentQG_y = -1;



  // --- find QPY that was active at the end of the previous slice ---

  // find the previous CTB in TS order

  const pic_parameter_set* pps = &tctx->img->pps;
  const seq_parameter_set* sps = &tctx->img->sps;


  if (tctx->shdr->slice_segment_address > 0) {
    int prevCtb = pps->CtbAddrTStoRS[ pps->CtbAddrRStoTS[tctx->shdr->slice_segment_address] -1 ];

    int ctbX = prevCtb % sps->PicWidthInCtbsY;
    int ctbY = prevCtb / sps->PicWidthInCtbsY;


    // take the pixel at the bottom right corner (but consider that the image size might be smaller)

    int x = ((ctbX+1) << sps->Log2CtbSizeY)-1;
    int y = ((ctbY+1) << sps->Log2CtbSizeY)-1;

    x = std::min(x,sps->pic_width_in_luma_samples-1);
    y = std::min(y,sps->pic_height_in_luma_samples-1);

    //printf("READ QPY: %d %d -> %d (should %d)\n",x,y,imgunit->img->get_QPY(x,y), tc.currentQPY);

    //if (tctx->shdr->dependent_slice_segment_flag) {  // TODO: do we need this condition ?
    tctx->currentQPY = tctx->img->get_QPY(x,y);
      //}
  }
}


void decoder_context::add_task_decode_CTB_row(thread_context* tctx,
                                              bool firstSliceSubstream,
                                              int ctbRow)
{
  thread_task_ctb_row* task = new thread_task_ctb_row;
  task->firstSliceSubstream = firstSliceSubstream;
  task->tctx = tctx;
  task->debug_startCtbRow = ctbRow;
  tctx->task = task;

  add_task(&thread_pool_, task);

  tctx->imgunit->tasks.push_back(task);
}


void decoder_context::add_task_decode_slice_segment(thread_context* tctx, bool firstSliceSubstream,
                                                    int ctbx,int ctby)
{
  thread_task_slice_segment* task = new thread_task_slice_segment;
  task->firstSliceSubstream = firstSliceSubstream;
  task->tctx = tctx;
  task->debug_startCtbX = ctbx;
  task->debug_startCtbY = ctby;
  tctx->task = task;

  add_task(&thread_pool_, task);

  tctx->imgunit->tasks.push_back(task);
}


de265_error decoder_context::read_vps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"---> read VPS\n");

  video_parameter_set vps;
  de265_error err = vps.read(this,&reader);
  if (err != DE265_OK) {
    return err;
  }

  if (param_vps_headers_fd>=0) {
    vps.dump(param_vps_headers_fd);
  }

  process_vps(&vps);

  return DE265_OK;
}

de265_error decoder_context::read_sps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"----> read SPS\n");

  seq_parameter_set sps;
  de265_error err;

  if ((err=sps.read(this, &reader)) != DE265_OK) {
    return err;
  }

  if (param_sps_headers_fd>=0) {
    sps.dump(param_sps_headers_fd);
  }

  process_sps(&sps);

  return DE265_OK;
}

de265_error decoder_context::read_pps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"----> read PPS\n");

  pic_parameter_set pps;

  bool success = pps.read(&reader,this);

  if (param_pps_headers_fd>=0) {
    pps.dump(param_pps_headers_fd);
  }

  if (success) {
    process_pps(&pps);
  }

  return success ? DE265_OK : DE265_WARNING_PPS_HEADER_INVALID;
}

de265_error decoder_context::read_sei_NAL(bitreader& reader, bool suffix)
{
  logdebug(LogHeaders,"----> read SEI\n");

  sei_message sei;

  //push_current_picture_to_output_queue();

  de265_error err = DE265_OK;

  if ((err=read_sei(&reader,&sei, suffix, current_sps)) == DE265_OK) {
    dump_sei(&sei, current_sps);

    if (image_units.empty()==false && suffix) {
      image_units.back()->suffix_SEIs.push_back(sei);
    }
  }
  else {
    add_warning(err, false);
  }

  return err;
}

de265_error decoder_context::read_eos_NAL(bitreader& reader)
{
  FirstAfterEndOfSequenceNAL = true;
  return DE265_OK;
}

de265_error decoder_context::read_slice_NAL(bitreader& reader, NAL_unit* nal, nal_header& nal_hdr)
{
  logdebug(LogHeaders,"---> read slice segment header\n");


  // --- read slice header ---

  slice_segment_header* shdr = new slice_segment_header;
  bool continueDecoding;
  de265_error err = shdr->read(&reader,this, &continueDecoding, nal_hdr);
  if (!continueDecoding) {
    if (img) { img->integrity = INTEGRITY_NOT_DECODED; }
    nal_parser->free_NAL_unit(nal);
    delete shdr;
    return err;
  }

  if (param_slice_headers_fd>=0) {
    shdr->dump_slice_segment_header(this, param_slice_headers_fd);
  }


  if (process_slice_segment_header(this, shdr, &err, nal->pts, &nal_hdr, nal->user_data) == false)
    {
      if (img!=NULL) img->integrity = INTEGRITY_NOT_DECODED;
      nal_parser->free_NAL_unit(nal);
      delete shdr;
      return err;
    }

  this->img->add_slice_segment_header(shdr);

  skip_bits(&reader,1); // TODO: why?
  prepare_for_CABAC(&reader);


  // modify entry_point_offsets

  int headerLength = reader.data - nal->data();
  for (int i=0;i<shdr->num_entry_point_offsets;i++) {
    shdr->entry_point_offset[i] -= nal->num_skipped_bytes_before(shdr->entry_point_offset[i],
                                                                 headerLength);
  }



  // --- start a new image if this is the first slice ---

  if (shdr->first_slice_segment_in_pic_flag) {
    image_unit* imgunit = new image_unit;
    imgunit->img = this->img;
    image_units.push_back(imgunit);
  }


  // --- add slice to current picture ---

  if ( ! image_units.empty() ) {

    slice_unit* sliceunit = new slice_unit(this);
    sliceunit->nal = nal;
    sliceunit->shdr = shdr;
    sliceunit->reader = reader;

    sliceunit->flush_reorder_buffer = flush_reorder_buffer_at_this_frame;


    image_units.back()->slice_units.push_back(sliceunit);
  }

  bool did_work;
  err = decode_some(&did_work);

  return DE265_OK;
}


template <class T> void pop_front(std::vector<T>& vec)
{
  for (int i=1;i<vec.size();i++)
    vec[i-1] = vec[i];

  vec.pop_back();
}


de265_error decoder_context::decode_some(bool* did_work, bool new_image)
{
  de265_error err = DE265_OK;

  *did_work = false;

  if (image_units.empty()) { return DE265_OK; }  // nothing to do


  // decode something if there is work to do

  if ( ! image_units.empty() ) { // && ! image_units[0]->slice_units.empty() ) {

    image_unit* imgunit = image_units[0];
    slice_unit* sliceunit = imgunit->get_next_unprocessed_slice_segment();

    if (sliceunit != NULL) {

      //pop_front(imgunit->slice_units);

      if (sliceunit->flush_reorder_buffer) {
        dpb.flush_reorder_buffer();
      }

      *did_work = true;

      //err = decode_slice_unit_sequential(imgunit, sliceunit);
      err = decode_slice_unit_parallel(imgunit, sliceunit);
      if (err) {
        return err;
      }

      //delete sliceunit;
    }
  }



  // if we decoded all slices of the current image and there will not
  // be added any more slices to the image, output the image

  if ( ( image_units.size()>=2 && image_units[0]->all_slice_segments_processed()) ||
       ( image_units.size()>=1 && image_units[0]->all_slice_segments_processed() && new_image) ||
       ( image_units.size()>=1 && image_units[0]->all_slice_segments_processed() &&
         nal_parser->number_of_NAL_units_pending()==0 &&
         (nal_parser->is_end_of_stream() || nal_parser->is_end_of_frame()) )) {

    image_unit* imgunit = image_units[0];

    *did_work=true;


    // mark all CTBs as decoded even if they are not, because faulty input
    // streams could miss part of the picture
    // TODO: this will not work when slice decoding is parallel to post-filtering,
    // so we will have to replace this with keeping track of which CTB should have
    // been decoded (but aren't because of the input stream being faulty)

    imgunit->img->mark_all_CTB_progress(CTB_PROGRESS_PREFILTER);



    // run post-processing filters (deblocking & SAO)

    if (img->decctx->num_worker_threads)
      run_postprocessing_filters_parallel(imgunit);
    else
      run_postprocessing_filters_sequential(imgunit->img);

    // process suffix SEIs

    for (int i=0;i<imgunit->suffix_SEIs.size();i++) {
      const sei_message& sei = imgunit->suffix_SEIs[i];

      err = process_sei(&sei, imgunit->img);
      if (err != DE265_OK)
        break;
    }


    push_picture_to_output_queue(imgunit);

    // remove just decoded image unit from queue

    delete imgunit;

    pop_front(image_units);
  }

  return err;
}


de265_error decoder_context::decode_slice_unit_sequential(image_unit* imgunit,
                                                          slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  /*
  printf("decode slice POC=%d addr=%d, img=%p\n",
         sliceunit->shdr->slice_pic_order_cnt_lsb,
         sliceunit->shdr->slice_segment_address,
         imgunit->img);
  */

  remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);

  if (sliceunit->shdr->slice_segment_address >= imgunit->img->pps.CtbAddrRStoTS.size()) {
    return DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA;
  }


  struct thread_context tctx;

  tctx.shdr = sliceunit->shdr;
  tctx.img  = imgunit->img;
  tctx.decctx = this;
  tctx.imgunit = imgunit;
  tctx.sliceunit= sliceunit;
  tctx.CtbAddrInTS = imgunit->img->pps.CtbAddrRStoTS[tctx.shdr->slice_segment_address];
  tctx.task = NULL;

  init_thread_context(&tctx);

  if (sliceunit->reader.bytes_remaining <= 0) {
    return DE265_ERROR_PREMATURE_END_OF_SLICE;
  }

  init_CABAC_decoder(&tctx.cabac_decoder,
                     sliceunit->reader.data,
                     sliceunit->reader.bytes_remaining);

  // alloc CABAC-model array if entropy_coding_sync is enabled

  if (current_pps->entropy_coding_sync_enabled_flag &&
      sliceunit->shdr->first_slice_segment_in_pic_flag) {
    imgunit->ctx_models.resize( (img->sps.PicHeightInCtbsY-1) ); //* CONTEXT_MODEL_TABLE_LENGTH );
  }

  sliceunit->nThreads=1;

  err=read_slice_segment_data(&tctx);

  sliceunit->finished_threads.set_progress(1);

  return err;
}


void decoder_context::mark_whole_slice_as_processed(image_unit* imgunit,
                                                    slice_unit* sliceunit,
                                                    int progress)
{
  //printf("mark whole slice\n");


  // mark all CTBs upto the next slice segment as processed

  slice_unit* nextSegment = imgunit->get_next_slice_segment(sliceunit);
  if (nextSegment) {
    /*
    printf("mark whole slice between %d and %d\n",
           sliceunit->shdr->slice_segment_address,
           nextSegment->shdr->slice_segment_address);
    */

    for (int ctb=sliceunit->shdr->slice_segment_address;
         ctb < nextSegment->shdr->slice_segment_address;
         ctb++)
      {
        if (ctb >= imgunit->img->number_of_ctbs())
          break;

        imgunit->img->ctb_progress[ctb].set_progress(progress);
      }
  }
}


de265_error decoder_context::decode_slice_unit_parallel(image_unit* imgunit,
                                                        slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);

  /*
  printf("-------- decode --------\n");
  printf("IMAGE UNIT %p\n",imgunit);
  sliceunit->shdr->dump_slice_segment_header(sliceunit->ctx, 1);
  imgunit->dump_slices();
  */

  de265_image* img = imgunit->img;
  const pic_parameter_set* pps = &img->pps;

  sliceunit->state = slice_unit::InProgress;

  bool use_WPP = (img->decctx->num_worker_threads > 0 &&
                  pps->entropy_coding_sync_enabled_flag);

  bool use_tiles = (img->decctx->num_worker_threads > 0 &&
                    pps->tiles_enabled_flag);


  // TODO: remove this warning later when we do frame-parallel decoding
  if (img->decctx->num_worker_threads > 0 &&
      pps->entropy_coding_sync_enabled_flag == false &&
      pps->tiles_enabled_flag == false) {

    img->decctx->add_warning(DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING, true);
  }


  // If this is the first slice segment, mark all CTBs before this as processed
  // (the real first slice segment could be missing).

  if (imgunit->is_first_slice_segment(sliceunit)) {
    slice_segment_header* shdr = sliceunit->shdr;
    int firstCTB = shdr->slice_segment_address;

    for (int ctb=0;ctb<firstCTB;ctb++) {
      //printf("mark pre progress %d\n",ctb);
      img->ctb_progress[ctb].set_progress(CTB_PROGRESS_PREFILTER);
    }
  }


  // if there is a previous slice that has been completely decoded,
  // mark all CTBs until the start of this slice as completed

  //printf("this slice: %p\n",sliceunit);
  slice_unit* prevSlice = imgunit->get_prev_slice_segment(sliceunit);
  //if (prevSlice) printf("prev slice state: %d\n",prevSlice->state);
  if (prevSlice && prevSlice->state == slice_unit::Decoded) {
    mark_whole_slice_as_processed(imgunit,prevSlice,CTB_PROGRESS_PREFILTER);
  }


  // TODO: even though we cannot split this into several tasks, we should run it
  // as a background thread
  if (!use_WPP && !use_tiles) {
    //printf("SEQ\n");
    err = decode_slice_unit_sequential(imgunit, sliceunit);
    sliceunit->state = slice_unit::Decoded;
    mark_whole_slice_as_processed(imgunit,sliceunit,CTB_PROGRESS_PREFILTER);
    return err;
  }


  if (use_WPP && use_tiles) {
    // TODO: this is not allowed ... output some warning or error

    return DE265_WARNING_PPS_HEADER_INVALID;
  }


  if (use_WPP) {
    //printf("WPP\n");
    err = decode_slice_unit_WPP(imgunit, sliceunit);
    sliceunit->state = slice_unit::Decoded;
    mark_whole_slice_as_processed(imgunit,sliceunit,CTB_PROGRESS_PREFILTER);
    return err;
  }
  else if (use_tiles) {
    //printf("TILE\n");
    err = decode_slice_unit_tiles(imgunit, sliceunit);
    sliceunit->state = slice_unit::Decoded;
    mark_whole_slice_as_processed(imgunit,sliceunit,CTB_PROGRESS_PREFILTER);
    return err;
  }

  assert(false);
  return err;
}


de265_error decoder_context::decode_slice_unit_WPP(image_unit* imgunit,
                                                   slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  de265_image* img = imgunit->img;
  slice_segment_header* shdr = sliceunit->shdr;
  const pic_parameter_set* pps = &img->pps;

  int nRows = shdr->num_entry_point_offsets +1;
  int ctbsWidth = img->sps.PicWidthInCtbsY;


  assert(img->num_threads_active() == 0);


  // reserve space to store entropy coding context models for each CTB row

  if (shdr->first_slice_segment_in_pic_flag) {
    // reserve space for nRows-1 because we don't need to save the CABAC model in the last CTB row
    imgunit->ctx_models.resize( (img->sps.PicHeightInCtbsY-1) ); //* CONTEXT_MODEL_TABLE_LENGTH );
  }


  sliceunit->allocate_thread_contexts(nRows);


  // first CTB in this slice
  int ctbAddrRS = shdr->slice_segment_address;
  int ctbRow    = ctbAddrRS / ctbsWidth;

  for (int entryPt=0;entryPt<nRows;entryPt++) {
    // entry points other than the first start at CTB rows
    if (entryPt>0) {
      ctbRow++;
      ctbAddrRS = ctbRow * ctbsWidth;
    }
    else if (nRows>1 && (ctbAddrRS % ctbsWidth) != 0) {
      // If slice segment consists of several WPP rows, each of them
      // has to start at a row.

      //printf("does not start at start\n");

      err = DE265_WARNING_SLICEHEADER_INVALID;
      break;
    }


    // prepare thread context

    thread_context* tctx = sliceunit->get_thread_context(entryPt);

    tctx->shdr    = shdr;
    tctx->decctx  = img->decctx;
    tctx->img     = img;
    tctx->imgunit = imgunit;
    tctx->sliceunit= sliceunit;
    tctx->CtbAddrInTS = pps->CtbAddrRStoTS[ctbAddrRS];

    init_thread_context(tctx);


    // init CABAC

    int dataStartIndex;
    if (entryPt==0) { dataStartIndex=0; }
    else            { dataStartIndex=shdr->entry_point_offset[entryPt-1]; }

    int dataEnd;
    if (entryPt==nRows-1) dataEnd = sliceunit->reader.bytes_remaining;
    else                  dataEnd = shdr->entry_point_offset[entryPt];

    if (dataStartIndex<0 || dataEnd>sliceunit->reader.bytes_remaining ||
        dataEnd <= dataStartIndex) {
      //printf("WPP premature end\n");
      err = DE265_ERROR_PREMATURE_END_OF_SLICE;
      break;
    }

    init_CABAC_decoder(&tctx->cabac_decoder,
                       &sliceunit->reader.data[dataStartIndex],
                       dataEnd-dataStartIndex);

    // add task

    //printf("start task for ctb-row: %d\n",ctbRow);
    img->thread_start(1);
    sliceunit->nThreads++;
    add_task_decode_CTB_row(tctx, entryPt==0, ctbRow);
  }

#if 0
  for (;;) {
    printf("q:%d r:%d b:%d f:%d\n",
           img->nThreadsQueued,
           img->nThreadsRunning,
           img->nThreadsBlocked,
           img->nThreadsFinished);

    if (img->debug_is_completed()) break;

    usleep(1000);
  }
#endif

  img->wait_for_completion();

  for (int i=0;i<imgunit->tasks.size();i++)
    delete imgunit->tasks[i];
  imgunit->tasks.clear();

  return DE265_OK;
}

de265_error decoder_context::decode_slice_unit_tiles(image_unit* imgunit,
                                                     slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  de265_image* img = imgunit->img;
  slice_segment_header* shdr = sliceunit->shdr;
  const pic_parameter_set* pps = &img->pps;

  int nTiles = shdr->num_entry_point_offsets +1;
  int ctbsWidth = img->sps.PicWidthInCtbsY;


  assert(img->num_threads_active() == 0);

  sliceunit->allocate_thread_contexts(nTiles);


  // first CTB in this slice
  int ctbAddrRS = shdr->slice_segment_address;
  int tileID = pps->TileIdRS[ctbAddrRS];

  for (int entryPt=0;entryPt<nTiles;entryPt++) {
    // entry points other than the first start at tile beginnings
    if (entryPt>0) {
      tileID++;

      if (tileID >= pps->num_tile_columns * pps->num_tile_rows) {
        err = DE265_WARNING_SLICEHEADER_INVALID;
        break;
      }

      int ctbX = pps->colBd[tileID % pps->num_tile_columns];
      int ctbY = pps->rowBd[tileID / pps->num_tile_columns];
      ctbAddrRS = ctbY * ctbsWidth + ctbX;
    }

    // set thread context

    thread_context* tctx = sliceunit->get_thread_context(entryPt);

    tctx->shdr   = shdr;
    tctx->decctx = img->decctx;
    tctx->img    = img;
    tctx->imgunit = imgunit;
    tctx->sliceunit= sliceunit;
    tctx->CtbAddrInTS = pps->CtbAddrRStoTS[ctbAddrRS];

    init_thread_context(tctx);


    // init CABAC

    int dataStartIndex;
    if (entryPt==0) { dataStartIndex=0; }
    else            { dataStartIndex=shdr->entry_point_offset[entryPt-1]; }

    int dataEnd;
    if (entryPt==nTiles-1) dataEnd = sliceunit->reader.bytes_remaining;
    else                   dataEnd = shdr->entry_point_offset[entryPt];

    if (dataStartIndex<0 || dataEnd>sliceunit->reader.bytes_remaining ||
        dataEnd <= dataStartIndex) {
      err = DE265_ERROR_PREMATURE_END_OF_SLICE;
      break;
    }

    init_CABAC_decoder(&tctx->cabac_decoder,
                       &sliceunit->reader.data[dataStartIndex],
                       dataEnd-dataStartIndex);

    // add task

    //printf("add tiles thread\n");
    img->thread_start(1);
    sliceunit->nThreads++;
    add_task_decode_slice_segment(tctx, entryPt==0,
                                  ctbAddrRS % ctbsWidth,
                                  ctbAddrRS / ctbsWidth);
  }

  img->wait_for_completion();

  for (int i=0;i<imgunit->tasks.size();i++)
    delete imgunit->tasks[i];
  imgunit->tasks.clear();

  return err;
}


de265_error decoder_context::decode_NAL(NAL_unit* nal)
{
  //return decode_NAL_OLD(nal);

  decoder_context* ctx = this;

  de265_error err = DE265_OK;

  bitreader reader;
  bitreader_init(&reader, nal->data(), nal->size());

  nal_header nal_hdr;
  nal_hdr.read(&reader);
  ctx->process_nal_hdr(&nal_hdr);

  if (nal_hdr.nuh_layer_id != layer_ID) {
    // The NAL unit was not meant for this layer decoder
    nal_parser->free_NAL_unit(nal);
    return DE265_OK;
  }

  loginfo(LogHighlevel,"NAL: 0x%x 0x%x -  unit type:%s layer:%d temporal id:%d\n",
          nal->data()[0], nal->data()[1],
          get_NAL_name(nal_hdr.nal_unit_type),
          nal_hdr.nuh_layer_id,
          nal_hdr.nuh_temporal_id);

  /*
    printf("NAL: 0x%x 0x%x -  unit type:%s temporal id:%d\n",
    nal->data()[0], nal->data()[1],
    get_NAL_name(nal_hdr.nal_unit_type),
    nal_hdr.nuh_temporal_id);
  */

  // throw away NALs from higher TIDs than currently selected
  // TODO: better online switching of HighestTID

  //printf("hTid: %d\n", current_HighestTid);

  if (nal_hdr.nuh_temporal_id > current_HighestTid) {
    nal_parser->free_NAL_unit(nal);
    return DE265_OK;
  }


  if (nal_hdr.nal_unit_type<32) {
    err = read_slice_NAL(reader, nal, nal_hdr);
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      err = read_vps_NAL(reader);
      nal_parser->free_NAL_unit(nal);
      break;

    case NAL_UNIT_SPS_NUT:
      err = read_sps_NAL(reader);
      nal_parser->free_NAL_unit(nal);
      break;

    case NAL_UNIT_PPS_NUT:
      err = read_pps_NAL(reader);
      nal_parser->free_NAL_unit(nal);
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      err = read_sei_NAL(reader, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT);
      nal_parser->free_NAL_unit(nal);
      break;

    case NAL_UNIT_EOS_NUT:
      ctx->FirstAfterEndOfSequenceNAL = true;
      nal_parser->free_NAL_unit(nal);
      break;

    default:
      nal_parser->free_NAL_unit(nal);
      break;
    }

  return err;
}


de265_error decoder_context::decode(int* more)
{
  decoder_context* ctx = this;

  // if the stream has ended, and no more NALs are to be decoded, flush all pictures

  if (ctx->nal_parser->get_NAL_queue_length() == 0 &&
      (ctx->nal_parser->is_end_of_stream() || ctx->nal_parser->is_end_of_frame()) &&
      ctx->image_units.empty()) {

    // flush all pending pictures into output queue

    // ctx->push_current_picture_to_output_queue(); // TODO: not with new queue
    ctx->dpb.flush_reorder_buffer();

    if (more) { *more = ctx->dpb.num_pictures_in_output_queue(); }

    return DE265_OK;
  }


  // if NAL-queue is empty, we need more data
  // -> input stalled

  if (ctx->nal_parser->is_end_of_stream() == false &&
      ctx->nal_parser->is_end_of_frame() == false &&
      ctx->nal_parser->get_NAL_queue_length() == 0) {
    if (more) { *more=1; }

    return DE265_ERROR_WAITING_FOR_INPUT_DATA;
  }


  // when there are no free image buffers in the DPB, pause decoding
  // -> output stalled

  if (!ctx->dpb.has_free_dpb_picture(false)) {
    if (more) *more = 1;
    return DE265_ERROR_IMAGE_BUFFER_FULL;
  }


  // decode one NAL from the queue

  de265_error err = DE265_OK;
  bool did_work = false;

  if (ctx->nal_parser->get_NAL_queue_length()) { // number_of_NAL_units_pending()) {
    NAL_unit* nal = ctx->nal_parser->pop_from_NAL_queue();
    assert(nal);
    err = ctx->decode_NAL(nal);
    // ctx->nal_parser.free_NAL_unit(nal); TODO: do not free NAL with new loop
    did_work=true;
  }
  else if (ctx->nal_parser->is_end_of_frame() == true &&
      ctx->image_units.empty()) {
    if (more) { *more=1; }

    return DE265_ERROR_WAITING_FOR_INPUT_DATA;
  }
  else {
    err = decode_some(&did_work);
  }

  if (more) {
    // decoding error is assumed to be unrecoverable
    *more = (err==DE265_OK && did_work);
  }

  return err;
}


void decoder_context::process_nal_hdr(nal_header* nal)
{
  nal_unit_type = nal->nal_unit_type;

  IdrPicFlag = isIdrPic(nal->nal_unit_type);
  RapPicFlag = isRapPic(nal->nal_unit_type);
}


void decoder_context::process_vps(video_parameter_set* vps)
{
  this->vps[ vps->video_parameter_set_id ] = *vps;
  last_vps = &this->vps[ vps->video_parameter_set_id ];
}


void decoder_context::process_sps(seq_parameter_set* sps)
{
  //push_current_picture_to_output_queue();

  this->sps[ sps->seq_parameter_set_id ] = *sps;
}


void decoder_context::process_pps(pic_parameter_set* pps)
{
  //push_current_picture_to_output_queue();

  this->pps[ (int)pps->pic_parameter_set_id ] = *pps;
}


/* 8.3.1
 */
void decoder_context::process_picture_order_count(decoder_context* ctx, slice_segment_header* hdr)
{
  loginfo(LogHeaders,"POC computation. lsb:%d prev.pic.lsb:%d msb:%d\n",
           hdr->slice_pic_order_cnt_lsb,
           ctx->prevPicOrderCntLsb,
           ctx->PicOrderCntMsb);

  if (isIRAP(ctx->nal_unit_type) &&
      ctx->NoRaslOutputFlag)
    {
      ctx->PicOrderCntMsb=0;


      // flush all images from reorder buffer

      flush_reorder_buffer_at_this_frame = true;
      //ctx->dpb.flush_reorder_buffer();
    }
  else
    {
      int MaxPicOrderCntLsb = ctx->current_sps->MaxPicOrderCntLsb;

      if ((hdr->slice_pic_order_cnt_lsb < ctx->prevPicOrderCntLsb) &&
          (ctx->prevPicOrderCntLsb - hdr->slice_pic_order_cnt_lsb) >= MaxPicOrderCntLsb/2) {
        ctx->PicOrderCntMsb = ctx->prevPicOrderCntMsb + MaxPicOrderCntLsb;
      }
      else if ((hdr->slice_pic_order_cnt_lsb > ctx->prevPicOrderCntLsb) &&
               (hdr->slice_pic_order_cnt_lsb - ctx->prevPicOrderCntLsb) > MaxPicOrderCntLsb/2) {
        ctx->PicOrderCntMsb = ctx->prevPicOrderCntMsb - MaxPicOrderCntLsb;
      }
      else {
        ctx->PicOrderCntMsb = ctx->prevPicOrderCntMsb;
      }
    }

  ctx->img->PicOrderCntVal = ctx->PicOrderCntMsb + hdr->slice_pic_order_cnt_lsb;
  ctx->img->picture_order_cnt_lsb = hdr->slice_pic_order_cnt_lsb;

  loginfo(LogHeaders,"POC computation. new msb:%d POC=%d\n",
           ctx->PicOrderCntMsb,
           ctx->img->PicOrderCntVal);

  if (ctx->img->nal_hdr.nuh_temporal_id==0 &&
      !isSublayerNonReference(ctx->nal_unit_type) &&
      !isRASL(ctx->nal_unit_type) &&
      !isRADL(ctx->nal_unit_type))
    {
      loginfo(LogHeaders,"set prevPicOrderCntLsb/Msb\n");

      ctx->prevPicOrderCntLsb = hdr->slice_pic_order_cnt_lsb;
      ctx->prevPicOrderCntMsb = ctx->PicOrderCntMsb;
    }
}


/* 8.3.3.2
   Returns DPB index of the generated picture.
 */
int decoder_context::generate_unavailable_reference_picture(decoder_context* ctx,
                                                            const seq_parameter_set* sps,
                                                            int POC, bool longTerm)
{
  assert(ctx->dpb.has_free_dpb_picture(true));

  int idx = ctx->dpb.new_image(ctx->current_sps, this, 0,0, false);
  assert(idx>=0);
  //printf("-> fill with unavailable POC %d\n",POC);

  // TODO: What if this is an inter layer picture?
  de265_image* img = ctx->dpb.get_image(idx);

  img->fill_image(1<<(sps->BitDepth_Y-1),
                  1<<(sps->BitDepth_C-1),
                  1<<(sps->BitDepth_C-1));

  img->fill_pred_mode(MODE_INTRA);

  img->PicOrderCntVal = POC;
  img->picture_order_cnt_lsb = POC & (sps->MaxPicOrderCntLsb-1);
  img->PicOutputFlag = false;
  img->PicState = (longTerm ? UsedForLongTermReference : UsedForShortTermReference);
  img->integrity = INTEGRITY_UNAVAILABLE_REFERENCE;

  return idx;
}

/* H.8.1.3   invoked once per picture

   This function will mark inter layer pictures in the DPB as 'unused' or 'used for long-term reference'.
   Outputs of this process are updated lists of inter-layer reference pictures RefPicSetInterLayer0 and RefPicSetInterLayer1 and the variables NumActiveRefLayerPics0 and NumActiveRefLayerPics1.
 */
void decoder_context::process_inter_layer_reference_picture_set(decoder_context* ctx, slice_segment_header* hdr)
{
  // The variable currLayerId is set equal to nuh_layer_id of the current picture
  int currLayerId = ctx->img->get_ID();
  int nuh_layer_id = currLayerId;

  // The variables NumRefLayerPicsProcessing, NumRefLayerPicsSampleProcessing, and NumRefLayerPicsMotionProcessing are set equal to 0.
  int NumRefLayerPicsProcessing = 0;
  int NumRefLayerPicsSampleProcessing = 0;
  int NumRefLayerPicsMotionProcessing = 0;

  // The lists RefPicSetInterLayer0 and RefPicSetInterLayer1 are first emptied, ...
  for (int i=0; i<MAX_REF_LAYERS; i++) {
    RefPicSetInterLayer0[i] = 0;
    RefPicSetInterLayer1[i] = 0;
  }
  // ... NumActiveRefLayerPics0 and NumActiveRefLayerPics1 are set equal to 0 ...
  NumActiveRefLayerPics0 = 0;
  NumActiveRefLayerPics1 = 0;

  // Get VPS
  video_parameter_set* vps = ctx->current_vps;
  video_parameter_set_extension* vps_ext = &vps->vps_extension;
  int ViewId[8];
  for (int i=0; i<8; i++) {
    ViewId[i] = vps_ext->view_id_val[i];
  }

  // Derive RefPicLayerId (JCTVC-R1013_v6 F.7.4.7.1 F-53)
  int_1d RefPicLayerId;
  for( int i=0, j=0; i < hdr->NumActiveRefLayerPics; i++ ) {
	  RefPicLayerId[i] = vps_ext->IdDirectRefLayer[nuh_layer_id][hdr->inter_layer_pred_layer_idc[i]];
  }

  // ... and the following applies:
  for( int i=0; i<hdr->NumActiveRefLayerPics; i++ ) {
    bool refPicSet0Flag =
      ((ViewId[currLayerId] <= ViewId[0] && ViewId[currLayerId] <= ViewId[RefPicLayerId[i]]) ||
      (ViewId[currLayerId] >= ViewId[0] && ViewId[currLayerId] >= ViewId[RefPicLayerId[i]]));

    // Look for the lower layer reference picture in the lower layer dpb
    int refLayerID = RefPicLayerId[i];
    decoder_context *ctx_lower = ctx->get_multi_layer_decoder()->get_layer_dec(refLayerID);

    // Get current POC and ID (? wat is this ID?)
    int currentPOC = ctx->img->PicOrderCntVal;
    const int currentID = ctx->img->get_ID();

    int DPBIndex = ctx_lower->dpb.DPB_index_of_picture_with_POC(currentPOC, currentID);
    if (DPBIndex != -1) {
      // an inter-layer reference picture ilRefPic is derived by invoking the process specified in clause H.8.1.4 with picX and RefPicLayerId[ i ] given as inputs
      de265_image* rlPic = ctx_lower->dpb.get_image(DPBIndex);
      derive_inter_layer_reference_picture(ctx, rlPic, refLayerID, i);

      if (refPicSet0Flag) {
        RefPicSetInterLayer0[ NumActiveRefLayerPics0 ] = i;
        NumActiveRefLayerPics0++;
      }
      else {
        RefPicSetInterLayer1[ NumActiveRefLayerPics1 ] = i;
        NumActiveRefLayerPics1++;
      }
      // Mark the inter layer picture as used for long term reference
      ilRefPic[i]->PicState = UsedForLongTermReference;
    }
    else {
      if( refPicSet0Flag ) {
        RefPicSetInterLayer0[ NumActiveRefLayerPics0++ ] = -1;
      }
      else {
        RefPicSetInterLayer1[ NumActiveRefLayerPics1++ ] = -1;
      }
    }
  }
}

/* H.8.1.4 Derivation process for inter-layer reference pictures

   In case of SNR scalalbility: After calling this function ilRefPic[ilRefPicIdx] will contain a pointer to
   the reference layer image.
   In case of spatial scalalbility: After calling this function ilRefPic[ilRefPicIdx] will contain a new image
   containing the upsampeled reference layer image. (Remember to delete these images after decoding)
 */
void decoder_context::derive_inter_layer_reference_picture(decoder_context* ctx, de265_image* rlPic, int rLId, int ilRefPicIdx)
{
  // Get the pps, pps_ext and sps
  pic_parameter_set* pps = ctx->current_pps;
  pps_multilayer_extension* pps_ext = &pps->pps_mult_ext;
  seq_parameter_set* sps = ctx->current_sps;

  // ... are set to ... of the current layer
  int PicWidthInSamplesCurrY  = sps->pic_width_in_luma_samples;
  int PicHeightInSamplesCurrY = sps->pic_height_in_luma_samples;
  int BitDepthCurrY           = sps->BitDepth_Y;
  int BitDepthCurrC           = sps->BitDepth_C;
  int SubWidthCurrC           = sps->SubWidthC;
  int SubHeightCurrC          = sps->SubHeightC;

  // ... are set equal to the width and height of the decoded direct reference layer picture rlPic
  int PicWidthInSamplesRefLayerY  = rlPic->get_width();
  int PicHeightInSamplesRefLayerY = rlPic->get_height();

  // Get the ref layer SPS and pps_extension
  decoder_context *ctx_ref = ctx->get_multi_layer_decoder()->get_layer_dec(rLId);
  seq_parameter_set* sps_ref = ctx_ref->current_sps;

  // ... are set equal to the values of BitDepthY, BitDepthC, SubWidthC and SubHeightC of the direct reference layer picture
  int BitDepthRefLayerY  = sps_ref->BitDepth_Y;
  int BitDepthRefLayerC  = sps_ref->BitDepth_C;
  int SubWidthRefLayerC  = sps_ref->SubWidthC;
  int SubHeightRefLayerC = sps_ref->SubHeightC;

  int RefLayerRegionLeftOffset    = pps_ext->ref_region_left_offset[rLId]   * SubWidthRefLayerC;
  int RefLayerRegionTopOffset     = pps_ext->ref_region_top_offset[rLId]    * SubHeightRefLayerC;
  int RefLayerRegionRightOffset   = pps_ext->ref_region_right_offset[rLId]  * SubWidthRefLayerC;
  int RefLayerRegionBottomOffset  = pps_ext->ref_region_bottom_offset[rLId] * SubHeightRefLayerC;

  int RefLayerRegionWidthInSamplesY  = PicWidthInSamplesRefLayerY - RefLayerRegionLeftOffset - RefLayerRegionRightOffset;
  int RefLayerRegionHeightInSamplesY = PicHeightInSamplesRefLayerY - RefLayerRegionTopOffset - RefLayerRegionBottomOffset;

  int PicWidthInSamplesCurrC      = PicWidthInSamplesCurrY      / SubWidthCurrC;
  int PicHeightInSamplesCurrC     = PicHeightInSamplesCurrY     / SubHeightCurrC;
  int PicWidthInSamplesRefLayerC  = PicWidthInSamplesRefLayerY  / SubWidthRefLayerC;
  int PicHeightInSamplesRefLayerC = PicHeightInSamplesRefLayerY / SubHeightRefLayerC;

  int ScaledRefLayerLeftOffset   = pps_ext->scaled_ref_layer_left_offset[rLId]   * SubWidthCurrC;
  int ScaledRefLayerTopOffset    = pps_ext->scaled_ref_layer_top_offset[rLId]    * SubHeightCurrC;
  int ScaledRefLayerRightOffset  = pps_ext->scaled_ref_layer_right_offset[rLId]  * SubWidthCurrC;
  int ScaledRefLayerBottomOffset = pps_ext->scaled_ref_layer_bottom_offset[rLId] * SubHeightCurrC;

  int ScaledRefRegionWidthInSamplesY = PicWidthInSamplesCurrY - ScaledRefLayerLeftOffset - ScaledRefLayerRightOffset;
  int ScaledRefRegionHeightInSamplesY = PicHeightInSamplesCurrY - ScaledRefLayerTopOffset - ScaledRefLayerBottomOffset;

  int SpatialScaleFactorHorY = ((RefLayerRegionWidthInSamplesY  << 16) + (ScaledRefRegionWidthInSamplesY  >> 1)) / ScaledRefRegionWidthInSamplesY;
  int SpatialScaleFactorVerY = ((RefLayerRegionHeightInSamplesY << 16) + (ScaledRefRegionHeightInSamplesY >> 1)) / ScaledRefRegionHeightInSamplesY;
  int SpatialScaleFactorHorC = (((RefLayerRegionWidthInSamplesY  / SubWidthRefLayerC ) << 16) + ((ScaledRefRegionWidthInSamplesY  / SubWidthCurrC ) >> 1)) / (ScaledRefRegionWidthInSamplesY  / SubWidthCurrC );
  int SpatialScaleFactorVerC = (((RefLayerRegionHeightInSamplesY / SubHeightRefLayerC) << 16) + ((ScaledRefRegionHeightInSamplesY / SubHeightCurrC) >> 1)) / (ScaledRefRegionHeightInSamplesY / SubHeightCurrC);

  int PhaseHorY = pps_ext->phase_hor_luma[rLId];
  int PhaseVerY = pps_ext->phase_ver_luma[rLId];
  int PhaseHorC = pps_ext->phase_hor_chroma[rLId];
  int PhaseVerC = pps_ext->phase_ver_chroma[rLId];

  // This is what the reference software does.
  // TODO: Double check with the actual standard.
  if (!pps_ext->resample_phase_set_present_flag[rLId]) {
    PhaseVerC = (4 * PicHeightInSamplesCurrY + (RefLayerRegionHeightInSamplesY >> 1)) / RefLayerRegionHeightInSamplesY - 4;
  }

  // The following ordered steps are applied to derive the inter-layer reference picture ilRefPic.
  bool sampleProcessingFlag = false;
  bool motionProcessingFlag = false;

  bool equalPictureSizeAndOffsetFlag = (
    (PicWidthInSamplesCurrY == PicWidthInSamplesRefLayerY) &&
    (PicHeightInSamplesCurrY == PicHeightInSamplesRefLayerY) &&
    (ScaledRefLayerLeftOffset == RefLayerRegionLeftOffset) &&
    (ScaledRefLayerTopOffset == RefLayerRegionTopOffset) &&
    (ScaledRefLayerRightOffset == RefLayerRegionRightOffset) &&
    (ScaledRefLayerBottomOffset == RefLayerRegionBottomOffset) &&
    (PhaseHorY == 0 && PhaseVerY == 0 && PhaseHorC == 0 && PhaseVerC == 0) );

  bool currColourMappingEnableFlag = false;
  if (pps_ext->colour_mapping_enabled_flag) {
    for (int i = 0; i <= pps_ext->cm_table.num_cm_ref_layers_minus1; i++) {
      if (pps_ext->cm_table.cm_ref_layer_id[i] == rLId) {
        currColourMappingEnableFlag == true;
      }
    }
  }

  // Create a new inter layer picture buffer
  if (ilRefPic[ilRefPicIdx] == NULL) {
    ilRefPic[ilRefPicIdx] = new de265_image();
    de265_chroma c = rlPic->get_chroma_format();
    ilRefPic[ilRefPicIdx]->alloc_image(PicWidthInSamplesCurrY, PicHeightInSamplesCurrY, c, sps, true, ctx, NULL, 0, 0, false);
  }
  ilRefPic[ilRefPicIdx]->PicOrderCntVal = rlPic->PicOrderCntVal;          // Copy POC

  if (equalPictureSizeAndOffsetFlag &&
     (BitDepthRefLayerY == BitDepthCurrY) &&
     (BitDepthRefLayerC == BitDepthCurrC) &&
     (SubWidthRefLayerC == SubWidthCurrC) &&
     (SubHeightRefLayerC == SubHeightCurrC) &&
     !currColourMappingEnableFlag) {

    // SNR scalability. We do not need to perform any upsampling.
    // Just copy all information from the lower layer reference.
    ilRefPic[ilRefPicIdx]->copy_lines_from(rlPic, 0, rlPic->get_height());  // Copy pixel data
    ilRefPic[ilRefPicIdx]->copy_metadata(rlPic);                            // Copy metadata
  }
  else {
    // Spatial scalability. Perform upsampling.
    video_parameter_set_extension *vps_ext = &ctx->current_vps->vps_extension;
    int currLayerId = ctx->get_layer_id();
    if (vps_ext->VpsInterLayerSamplePredictionEnabled[vps_ext->LayerIdxInVps[currLayerId]][vps_ext->LayerIdxInVps[rLId]]) {

      // Derive the parameters needed by the reference layer sample location derivation function
      // as specified in clause H.8.1.4.1.3. These will bee needed when the reference layer sample location derivation
      // process is invoked.
      int position_params[2][8];
      for (int c=0; c<2; c++) {
        bool chromaFlag = (c != 0);

        // The variables currOffsetLeft, currOffsetTop, refOffsetLeft and refOffsetTop are derived as follows:
        int currOffsetLeft = ScaledRefLayerLeftOffset / (chromaFlag ? SubWidthCurrC  : 1);              // (H 53)
        int currOffsetTop  = ScaledRefLayerTopOffset  / (chromaFlag ? SubHeightCurrC : 1);              // (H 54)
        int refOffsetLeft  = (RefLayerRegionLeftOffset / (chromaFlag ? SubWidthRefLayerC  : 1)) << 4;   // (H 55)
        int refOffsetTop   = (RefLayerRegionTopOffset  / (chromaFlag ? SubHeightRefLayerC : 1)) << 4;   // (H 56)

        // The variables phaseHor, phaseVer, scaleHor and scaleVer are derived as follows:
        int phaseHor = chromaFlag ? PhaseHorC : PhaseHorY;                              // (H 57)
        int phaseVer = chromaFlag ? PhaseVerC : PhaseVerY;                              // (H 58)
        int scaleHor = chromaFlag ? SpatialScaleFactorHorC : SpatialScaleFactorHorY;    // (H 59)
        int scaleVer = chromaFlag ? SpatialScaleFactorVerC : SpatialScaleFactorVerY;    // (H 60)

        // The variables addHor and addVer are derived as follows:
        int addHor = -(( scaleHor * phaseHor + 8) >> 4 );   // (H 61)
        int addVer = -(( scaleVer * phaseVer + 8) >> 4 );   // (H 62)

        // Put parameters into array
        position_params[c][0] = currOffsetLeft;
        position_params[c][1] = currOffsetTop;
        position_params[c][2] = refOffsetLeft;
        position_params[c][3] = refOffsetTop;
        position_params[c][4] = scaleHor;
        position_params[c][5] = scaleVer;
        position_params[c][6] = addHor;
        position_params[c][7] = addVer;
      }

      if (currColourMappingEnableFlag) {
        // The colour mapping process as specified in clause H.8.1.4.3 is invoked
        // TODO
        assert(false);
        if (equalPictureSizeAndOffsetFlag) {
          ilRefPic[ilRefPicIdx]->copy_lines_from(rlPic, 0, rlPic->get_height());  // Copy pixel data
        }
        else {
          // the picture sample resampling process as specified in clause H.8.1.4.1 is invoked
          //resampling_process_of_picture_sample_values(rlPic, ilRefPic[ilRefPicIdx], position_params, BitDepthRefLayer, BitDepthCurr);
          int src_size[2] = {rlPic->get_width(), rlPic->get_height()};
          int dst_size[2] = { ilRefPic[ilRefPicIdx]->get_width(), ilRefPic[ilRefPicIdx]->get_height() };
          ctx->acceleration.resampling_process_of_luma_sample_values(rlPic->get_image_plane(0), rlPic->get_luma_stride(), src_size,
                                                                     ilRefPic[ilRefPicIdx]->get_image_plane(0), ilRefPic[ilRefPicIdx]->get_luma_stride(), dst_size,
                                                                     position_params[0], BitDepthRefLayerY, BitDepthCurrY );

          // Chroma
          src_size[0] = rlPic->get_width(1);
          src_size[1] = rlPic->get_height(1);
          dst_size[0] = ilRefPic[ilRefPicIdx]->get_width(1);
          dst_size[1] = ilRefPic[ilRefPicIdx]->get_height(1);
          ctx->acceleration.resampling_process_of_chroma_sample_values(rlPic->get_image_plane(1), rlPic->get_chroma_stride(), src_size,
                                                                     ilRefPic[ilRefPicIdx]->get_image_plane(1), ilRefPic[ilRefPicIdx]->get_chroma_stride(), dst_size,
                                                                     position_params[1], BitDepthRefLayerC, BitDepthCurrC );
          ctx->acceleration.resampling_process_of_chroma_sample_values(rlPic->get_image_plane(2), rlPic->get_chroma_stride(), src_size,
                                                                     ilRefPic[ilRefPicIdx]->get_image_plane(2), ilRefPic[ilRefPicIdx]->get_chroma_stride(), dst_size,
                                                                     position_params[1], BitDepthRefLayerC, BitDepthCurrC );
        }
        sampleProcessingFlag = true;
      }
      else {
        // the picture sample resampling process as specified in clause H.8.1.4.1 is invoked
        //resampling_process_of_picture_sample_values(rlPic, ilRefPic[ilRefPicIdx], position_params, BitDepthRefLayer, BitDepthCurr);
        int src_size[2] = {rlPic->get_width(), rlPic->get_height()};
        int dst_size[2] = { ilRefPic[ilRefPicIdx]->get_width(), ilRefPic[ilRefPicIdx]->get_height() };
        ctx->acceleration.resampling_process_of_luma_sample_values(rlPic->get_image_plane(0), rlPic->get_luma_stride(), src_size,
                                                                   ilRefPic[ilRefPicIdx]->get_image_plane(0), ilRefPic[ilRefPicIdx]->get_luma_stride(), dst_size,
                                                                   position_params[0], BitDepthRefLayerY, BitDepthCurrY );

        // Chroma
        src_size[0] = rlPic->get_width(1);
        src_size[1] = rlPic->get_height(1);
        dst_size[0] = ilRefPic[ilRefPicIdx]->get_width(1);
        dst_size[1] = ilRefPic[ilRefPicIdx]->get_height(1);
        ctx->acceleration.resampling_process_of_chroma_sample_values(rlPic->get_image_plane(1), rlPic->get_chroma_stride(), src_size,
                                                                   ilRefPic[ilRefPicIdx]->get_image_plane(1), ilRefPic[ilRefPicIdx]->get_chroma_stride(), dst_size,
                                                                   position_params[1], BitDepthRefLayerC, BitDepthCurrC );
        ctx->acceleration.resampling_process_of_chroma_sample_values(rlPic->get_image_plane(2), rlPic->get_chroma_stride(), src_size,
                                                                   ilRefPic[ilRefPicIdx]->get_image_plane(2), ilRefPic[ilRefPicIdx]->get_chroma_stride(), dst_size,
                                                                   position_params[1], BitDepthRefLayerC, BitDepthCurrC );

        //// DEBUG. DUMP TO FILE
        //FILE *fp = fopen("before_upsampling.txt", "wb");
        //int nrBytesY = rlPic->get_width() * rlPic->get_height();
        //int nrBytesUV = nrBytesY / 4;
        //uint8_t *srcY = rlPic->get_image_plane(0);
        //uint8_t *srcU = rlPic->get_image_plane(1);
        //uint8_t *srcV = rlPic->get_image_plane(2);
        //fwrite(srcY, sizeof(uint8_t), nrBytesY, fp);
        //fwrite(srcU, sizeof(uint8_t), nrBytesUV, fp);
        //fwrite(srcV, sizeof(uint8_t), nrBytesUV, fp);
        //fclose(fp);
        //// DEBUG. DUMP TO FILE
        //fp = fopen("after_upsampling.txt", "wb");
        //nrBytesY = ilRefPic[ilRefPicIdx]->get_width() * ilRefPic[ilRefPicIdx]->get_height();
        //nrBytesUV = nrBytesY / 4;
        //uint8_t *dstY = ilRefPic[ilRefPicIdx]->get_image_plane(0);
        //uint8_t *dstU = ilRefPic[ilRefPicIdx]->get_image_plane(1);
        //uint8_t *dstV = ilRefPic[ilRefPicIdx]->get_image_plane(2);
        //fwrite(dstY, sizeof(uint8_t), nrBytesY, fp);
        //fwrite(dstU, sizeof(uint8_t), nrBytesUV, fp);
        //fwrite(dstV, sizeof(uint8_t), nrBytesUV, fp);
        //fclose(fp);
      }
    }

    if (vps_ext->VpsInterLayerMotionPredictionEnabled[vps_ext->LayerIdxInVps[currLayerId]][vps_ext->LayerIdxInVps[rLId]]) {
      if (equalPictureSizeAndOffsetFlag) {
        // Copy metadata
        ilRefPic[ilRefPicIdx]->copy_metadata(rlPic);
      }
      else {
        // The picture motion and mode parameters resampling process as specified in clause H.8.1.4.2 is invoked
        int scaling_parameters[10] = {ScaledRefLayerLeftOffset, ScaledRefLayerTopOffset,
                                      SpatialScaleFactorHorY, SpatialScaleFactorVerY,
                                      RefLayerRegionLeftOffset, RefLayerRegionTopOffset,
                                      ScaledRefRegionWidthInSamplesY, RefLayerRegionWidthInSamplesY,
                                      ScaledRefRegionHeightInSamplesY, RefLayerRegionHeightInSamplesY};
        resampling_process_of_picture_motion_and_mode_parameters(rlPic, ilRefPic[ilRefPicIdx], scaling_parameters);

        motionProcessingFlag  = true;
      }
    }

    ilRefPic_upsampled = true;
  }
}

/* H.8.1.4.2 Resampling process of picture motion and mode parameters
*/
void decoder_context::resampling_process_of_picture_motion_and_mode_parameters(de265_image* src, de265_image* dst, int scaling_parameters[10])
{
  int PicWidthInSamplesCurrY      = dst->get_width();
  int PiPicHeightInSamplesCurrY   = dst->get_height();
  int PicWidthInSamplesRefLayerY  = src->get_width();
  int PicHeightInSamplesRefLayerY = src->get_height();

  int ScaledRefRegionWidthInSamplesY  = scaling_parameters[6];
  int RefLayerRegionWidthInSamplesY   = scaling_parameters[7];
  int ScaledRefRegionHeightInSamplesY = scaling_parameters[8];
  int RefLayerRegionHeightInSamplesY  = scaling_parameters[9];

  int xMax = ((PicWidthInSamplesCurrY    + 15) >> 4) - 1;
  int yMax = ((PiPicHeightInSamplesCurrY + 15) >> 4) - 1;

  int xPb, yPb, xPCtr, yPCtr, xRef, yRef, xRL, yRL, width, height;
  for (int xB = 0; xB <= xMax; xB++) {
    for (int yB = 0; yB <= yMax; yB++) {
      xPb = xB << 4;
      yPb = yB << 4;

      // 1. The center location ( xPCtr, yPCtr ) of the luma prediction block is derived as follows:
      xPCtr = xPb + 8;  // (H 65)
      yPCtr = yPb + 8;  // (H 66)

      // 2. The variables xRef and yRef are derived as follows:
      xRef = (((xPCtr - scaling_parameters[0]) * scaling_parameters[2] + (1 << 15)) >> 16 ) + scaling_parameters[4];  // (H 67)
      yRef = (((yPCtr - scaling_parameters[1]) * scaling_parameters[3] + (1 << 15)) >> 16 ) + scaling_parameters[5];  // (H 68)

      // 3. The rounded reference layer luma sample location ( xRL, yRL ) is derived as follows:
      xRL = ((xRef + 4) >> 4) << 4;  // (H 69)
      yRL = ((yRef + 4) >> 4) << 4;  // (H 70)

      // Calculate the width and height of the block. This is necessary since "set_pred_mode" and
      // "set_mv_info" are not capable of handling blocks that reach outside the picture bundary.
      width = 16;
      height = 16;
      if (xPb+16 > PicWidthInSamplesCurrY || yPb+16 > PiPicHeightInSamplesCurrY) {
        // The width or height of the block is not 16x16
        width = (PicWidthInSamplesCurrY - xPb);
        height = (PiPicHeightInSamplesCurrY - yPb);
      }

      // 4. Upsample the prediction mode. (H 71)
      PredMode rsPredMode;
      if( xRL < 0 || xRL >= PicWidthInSamplesRefLayerY || yRL < 0 || yRL >= PicHeightInSamplesRefLayerY ) {
        rsPredMode = MODE_INTRA;
      }
      else {
        rsPredMode = src->get_pred_mode(xRL, yRL);
      }
      dst->set_pred_mode(xPb, yPb, width, height, rsPredMode);

      // 5. Upsample the motion vectors and prediction flags
      MotionVectorSpec mv_dst;
      if (rsPredMode == MODE_INTER) {
        const MotionVectorSpec *mv_src = src->get_mv_info(xRL, yRL);
        // For X being each of 0 and 1...
        for (int l=0; l<2; l++) {
          // RefIdx, predFlag
          mv_dst.refIdx[l] = mv_src->refIdx[l];     // (H 72)
          mv_dst.predFlag[l] = mv_src->predFlag[l]; // (H 73)

          // Motion vector. X-component.
          if (ScaledRefRegionWidthInSamplesY != RefLayerRegionWidthInSamplesY) {
            int rlMvLX = mv_src->mv[l].x;
            int scaleMVX = Clip3( -4096, 4095, ((ScaledRefRegionWidthInSamplesY << 8) + (RefLayerRegionWidthInSamplesY >> 1)) / RefLayerRegionWidthInSamplesY); // (H 74)
            mv_dst.mv[l].x = Clip3( -32768, 32767, Sign( scaleMVX *	rlMvLX) * ((abs_value(scaleMVX * rlMvLX) + 127) >> 8)); // (H 75)
          }
          else {
            mv_dst.mv[l].x = mv_src->mv[l].x; // (H 76)
          }

          // Motion vector. Y-component.
          if (ScaledRefRegionHeightInSamplesY != RefLayerRegionHeightInSamplesY) {
            int rlMvLX = mv_src->mv[l].y;
            int scaleMVX = Clip3( -4096, 4095, ((ScaledRefRegionHeightInSamplesY << 8) + (RefLayerRegionHeightInSamplesY >> 1)) / RefLayerRegionHeightInSamplesY); // (H 77)
            mv_dst.mv[l].y = Clip3( -32768, 32767, Sign( scaleMVX *	rlMvLX) * ((abs_value(scaleMVX * rlMvLX) + 127) >> 8)); // (H 78)
          }
          else {
            mv_dst.mv[l].y = mv_src->mv[l].y;  // (H 79)
          }
        }
      }
      else {
        // Otherwise (rsPredMode is equal to MODE_INTRA), the following applies:
        mv_dst.mv[0].x = 0;
        mv_dst.mv[0].y = 0;
        mv_dst.mv[1].x = 0;
        mv_dst.mv[1].y = 0;
        mv_dst.refIdx[0] = -1;
        mv_dst.refIdx[1] = -1;
        mv_dst.predFlag[0] = 0;
        mv_dst.predFlag[1] = 0;
      }
      // Set the derived motion info in the output picture
      dst->set_mv_info(xPb, yPb, width, height, mv_dst);
    }
  }
}

/* 8.3.2   invoked once per picture

   This function will mark pictures in the DPB as 'unused' or 'used for long-term reference'
 */
void decoder_context::process_reference_picture_set(decoder_context* ctx, slice_segment_header* hdr)
{
  std::vector<int> removeReferencesList;

  const int currentID = ctx->img->get_ID();


  if (isIRAP(ctx->nal_unit_type) && ctx->NoRaslOutputFlag) {

    int currentPOC = ctx->img->PicOrderCntVal;

    // reset DPB

    /* The standard says: "When the current picture is an IRAP picture with NoRaslOutputFlag
       equal to 1, all reference pictures currently in the DPB (if any) are marked as
       "unused for reference".

       This seems to be wrong as it also throws out the first CRA picture in a stream like
       RAP_A (decoding order: CRA,POC=64, RASL,POC=60). Removing only the pictures with
       lower POCs seems to be compliant to the reference decoder.
    */

    for (int i=0;i<dpb.size();i++) {
      de265_image* img = ctx->dpb.get_image(i);

      if (img->PicState != UnusedForReference &&
          img->PicOrderCntVal < currentPOC &&
          img->removed_at_picture_id > ctx->img->get_ID()) {

        removeReferencesList.push_back(img->get_ID());
        img->removed_at_picture_id = ctx->img->get_ID();

        //printf("will remove ID %d (a)\n",img->get_ID());
      }
    }
  }


  if (isIDR(ctx->nal_unit_type)) {

    // clear all reference pictures

    ctx->NumPocStCurrBefore = 0;
    ctx->NumPocStCurrAfter = 0;
    ctx->NumPocStFoll = 0;
    ctx->NumPocLtCurr = 0;
    ctx->NumPocLtFoll = 0;
  }
  else {
    const ref_pic_set* rps = &hdr->CurrRps;

    // (8-98)

    int i,j,k;

    // scan ref-pic-set for smaller POCs and fill into PocStCurrBefore / PocStFoll

    for (i=0, j=0, k=0;
         i<rps->NumNegativePics;
         i++)
      {
        if (rps->UsedByCurrPicS0[i]) {
          ctx->PocStCurrBefore[j++] = ctx->img->PicOrderCntVal + rps->DeltaPocS0[i];
          //printf("PocStCurrBefore = %d\n",ctx->PocStCurrBefore[j-1]);
        }
        else {
          ctx->PocStFoll[k++] = ctx->img->PicOrderCntVal + rps->DeltaPocS0[i];
        }
      }

    ctx->NumPocStCurrBefore = j;


    // scan ref-pic-set for larger POCs and fill into PocStCurrAfter / PocStFoll

    for (i=0, j=0;
         i<rps->NumPositivePics;
         i++)
      {
        if (rps->UsedByCurrPicS1[i]) {
          ctx->PocStCurrAfter[j++] = ctx->img->PicOrderCntVal + rps->DeltaPocS1[i];
          //printf("PocStCurrAfter = %d\n",ctx->PocStCurrAfter[j-1]);
        }
        else {
          ctx->PocStFoll[k++] = ctx->img->PicOrderCntVal + rps->DeltaPocS1[i];
        }
      }

    ctx->NumPocStCurrAfter = j;
    ctx->NumPocStFoll = k;


    // find used / future long-term references

    for (i=0, j=0, k=0;
         //i<ctx->current_sps->num_long_term_ref_pics_sps + hdr->num_long_term_pics;
         i<hdr->num_long_term_sps + hdr->num_long_term_pics;
         i++)
      {
        int pocLt = ctx->PocLsbLt[i];

        if (hdr->delta_poc_msb_present_flag[i]) {
          int currentPictureMSB = ctx->img->PicOrderCntVal - hdr->slice_pic_order_cnt_lsb;
          pocLt += currentPictureMSB
            - ctx->DeltaPocMsbCycleLt[i] * ctx->current_sps->MaxPicOrderCntLsb;
        }

        if (ctx->UsedByCurrPicLt[i]) {
          ctx->PocLtCurr[j] = pocLt;
          ctx->CurrDeltaPocMsbPresentFlag[j] = hdr->delta_poc_msb_present_flag[i];
          j++;
        }
        else {
          ctx->PocLtFoll[k] = pocLt;
          ctx->FollDeltaPocMsbPresentFlag[k] = hdr->delta_poc_msb_present_flag[i];
          k++;
        }
      }

    ctx->NumPocLtCurr = j;
    ctx->NumPocLtFoll = k;
  }


  // (old 8-99) / (new 8-106)
  // 1.

  std::vector<bool> picInAnyList(dpb.size(), false);


  dpb.log_dpb_content();

  for (int i=0;i<ctx->NumPocLtCurr;i++) {
    int k;
    if (!ctx->CurrDeltaPocMsbPresentFlag[i]) {
      k = ctx->dpb.DPB_index_of_picture_with_LSB(ctx->PocLtCurr[i], currentID, true);
    }
    else {
      k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocLtCurr[i], currentID, true);
    }

    ctx->RefPicSetLtCurr[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      // TODO, CHECK: is it ok that we generate a picture with POC = LSB (PocLtCurr)
      // We do not know the correct MSB
      int concealedPicture = generate_unavailable_reference_picture(ctx, ctx->current_sps,
                                                                    ctx->PocLtCurr[i], true);
      ctx->RefPicSetLtCurr[i] = k = concealedPicture;
      picInAnyList[concealedPicture]=true;
    }

    if (ctx->dpb.get_image(k)->integrity != INTEGRITY_CORRECT) {
      ctx->img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }


  for (int i=0;i<ctx->NumPocLtFoll;i++) {
    int k;
    if (!ctx->FollDeltaPocMsbPresentFlag[i]) {
      k = ctx->dpb.DPB_index_of_picture_with_LSB(ctx->PocLtFoll[i], currentID, true);
    }
    else {
      k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocLtFoll[i], currentID, true);
    }

    ctx->RefPicSetLtFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      int concealedPicture = k = generate_unavailable_reference_picture(ctx, ctx->current_sps,
                                                                        ctx->PocLtFoll[i], true);
      ctx->RefPicSetLtFoll[i] = concealedPicture;
      picInAnyList[concealedPicture]=true;
    }
  }


  // 2. Mark all pictures in RefPicSetLtCurr / RefPicSetLtFoll as UsedForLongTermReference

  for (int i=0;i<ctx->NumPocLtCurr;i++) {
    ctx->dpb.get_image(ctx->RefPicSetLtCurr[i])->PicState = UsedForLongTermReference;
  }

  for (int i=0;i<ctx->NumPocLtFoll;i++) {
    ctx->dpb.get_image(ctx->RefPicSetLtFoll[i])->PicState = UsedForLongTermReference;
  }


  // 3.

  for (int i=0;i<ctx->NumPocStCurrBefore;i++) {
    int k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocStCurrBefore[i], currentID);

    //printf("st curr before, poc=%d -> idx=%d\n",ctx->PocStCurrBefore[i], k);

    ctx->RefPicSetStCurrBefore[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      int concealedPicture = generate_unavailable_reference_picture(ctx, ctx->current_sps,
                                                                    ctx->PocStCurrBefore[i], false);
      ctx->RefPicSetStCurrBefore[i] = k = concealedPicture;
      picInAnyList[concealedPicture]=true;

      //printf("  concealed: %d\n", concealedPicture);
    }

    if (ctx->dpb.get_image(k)->integrity != INTEGRITY_CORRECT) {
      ctx->img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }

  for (int i=0;i<ctx->NumPocStCurrAfter;i++) {
    int k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocStCurrAfter[i], currentID);

    //printf("st curr after, poc=%d -> idx=%d\n",ctx->PocStCurrAfter[i], k);

    ctx->RefPicSetStCurrAfter[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      int concealedPicture = generate_unavailable_reference_picture(ctx, ctx->current_sps,
                                                                    ctx->PocStCurrAfter[i], false);
      ctx->RefPicSetStCurrAfter[i] = k = concealedPicture;
      picInAnyList[concealedPicture]=true;

      //printf("  concealed: %d\n", concealedPicture);
    }

    if (ctx->dpb.get_image(k)->integrity != INTEGRITY_CORRECT) {
      ctx->img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }

  for (int i=0;i<ctx->NumPocStFoll;i++) {
    int k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocStFoll[i], currentID);
    // if (k<0) { assert(false); } // IGNORE

    ctx->RefPicSetStFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  // 4. any picture that is not marked for reference is put into the "UnusedForReference" state

  for (int i=0;i<dpb.size();i++)
    if (!picInAnyList[i])        // no reference
      {
        de265_image* dpbimg = ctx->dpb.get_image(i);
        if (dpbimg != ctx->img &&  // not the current picture
            dpbimg->removed_at_picture_id > ctx->img->get_ID()) // has not been removed before
          {
            if (dpbimg->PicState != UnusedForReference) {
              removeReferencesList.push_back(dpbimg->get_ID());
              //printf("will remove ID %d (b)\n",dpbimg->get_ID());

              dpbimg->removed_at_picture_id = ctx->img->get_ID();
            }
          }
      }

  hdr->RemoveReferencesList = removeReferencesList;

  //remove_images_from_dpb(hdr->RemoveReferencesList);
}


// 8.3.4
// Returns whether we can continue decoding (or whether there is a severe error).
/* Called at beginning of each slice.

   Constructs
   - the RefPicList[2][], containing indices into the DPB, and
   - the RefPicList_POC[2][], containing POCs.
   - LongTermRefPic[2][] is also set to true if it is a long-term reference
   - InterLayerRefPic[2][] is set to true if it is an inter layer reference picture
 */
bool decoder_context::construct_reference_picture_lists(decoder_context* ctx, slice_segment_header* hdr)
{
  int NumPocTotalCurr = hdr->NumPocTotalCurr;
  int NumRpsCurrTempList0 = libde265_max(hdr->num_ref_idx_l0_active, NumPocTotalCurr);

  // TODO: fold code for both lists together

  int RefPicListTemp0[3*MAX_NUM_REF_PICS]; // TODO: what would be the correct maximum ?
  int RefPicListTemp1[3*MAX_NUM_REF_PICS]; // TODO: what would be the correct maximum ?
  char isLongTerm[2][3*MAX_NUM_REF_PICS];
  bool isInterLayer[2][3*MAX_NUM_REF_PICS];

  memset(isLongTerm,0,2*3*MAX_NUM_REF_PICS);
  for (int i = 0; i < 2; i++) {
    for (int j=0; j<3*MAX_NUM_REF_PICS; j++) {
      isInterLayer[i][j] = false;
    }
  }

  /* --- Fill RefPicListTmp0 with reference pictures in this order:
     1) short term, past POC
     2) short term, future POC
     3) long term
  */

  int rIdx=0;
  while (rIdx < NumRpsCurrTempList0) {
    for (int i=0;i<ctx->NumPocStCurrBefore && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetStCurrBefore[i];

    // Multi layer extension (JCTVC-R1013_v6 F.8.3.4 F-64)
    for (int i = 0; i<ctx->NumActiveRefLayerPics0; rIdx++, i++) {
      RefPicListTemp0[rIdx] = ctx->RefPicSetInterLayer0[i];
      isInterLayer[0][rIdx] = true;
      isLongTerm[0][rIdx] = true; // IL-pred picture is marked as long term reference
    }

    for (int i=0;i<ctx->NumPocStCurrAfter && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetStCurrAfter[i];

    for (int i=0;i<ctx->NumPocLtCurr && rIdx<NumRpsCurrTempList0; rIdx++,i++) {
      RefPicListTemp0[rIdx] = ctx->RefPicSetLtCurr[i];
      isLongTerm[0][rIdx] = true;
    }

    // Multi layer extension (JCTVC-R1013_v6 F.8.3.4 F-64)
    for( int i=0; i<ctx->NumActiveRefLayerPics1; rIdx++, i++ ) {
      RefPicListTemp0[rIdx] = ctx->RefPicSetInterLayer1[i];
      isInterLayer[0][rIdx] = true;
      isLongTerm[0][rIdx] = true; // IL-pred picture is marked as long term reference
    }

    // This check is to prevent an endless loop when no images are added above.
    if (rIdx==0) {
      ctx->add_warning(DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST, false);
      return false;
    }
  }

  /*
  if (hdr->num_ref_idx_l0_active > 16) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED, false);
    return false;
  }
  */

  assert(hdr->num_ref_idx_l0_active <= 16);
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    int idx = hdr->ref_pic_list_modification_flag_l0 ? hdr->list_entry_l0[rIdx] : rIdx;

    hdr->RefPicList[0][rIdx] = RefPicListTemp0[idx];
    hdr->LongTermRefPic[0][rIdx] = isLongTerm[0][idx];
    hdr->InterLayerRefPic[0][rIdx] = isInterLayer[0][idx];

    // remember POC of referenced image (needed in motion.c, derive_collocated_motion_vector)
    de265_image* img_0_rIdx = NULL;
    if (hdr->InterLayerRefPic[0][rIdx]) {
      // Get the inter layer reference from ilRefPic
      img_0_rIdx = ilRefPic[hdr->RefPicList[0][rIdx]];
    }
    else {
      img_0_rIdx = ctx->dpb.get_image(hdr->RefPicList[0][rIdx]);
    }
    if (img_0_rIdx==NULL) {
      return false;
    }
    hdr->RefPicList_POC[0][rIdx] = img_0_rIdx->PicOrderCntVal;
    hdr->RefPicList_PicState[0][rIdx] = img_0_rIdx->PicState;
  }

  /* --- Fill RefPicListTmp1 with reference pictures in this order:
     1) short term, future POC
     2) short term, past POC
     3) long term
  */

  if (hdr->slice_type == SLICE_TYPE_B) {
    int NumRpsCurrTempList1 = libde265_max(hdr->num_ref_idx_l1_active, NumPocTotalCurr);

    int rIdx=0;
    while (rIdx < NumRpsCurrTempList1) {
      for (int i=0;i<ctx->NumPocStCurrAfter && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = ctx->RefPicSetStCurrAfter[i];
      }

      // Multi layer extension (JCTVC-R1013_v6 F.8.3.4 F-66)
      for( int i=0;i<ctx->NumActiveRefLayerPics1; rIdx++, i++ ) {
        RefPicListTemp1[rIdx] = ctx->RefPicSetInterLayer1[ i ];
        isInterLayer[1][rIdx] = true;
        isLongTerm[1][rIdx] = true; // IL-pred picture is marked as long term reference
      }

      for (int i=0;i<ctx->NumPocStCurrBefore && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = ctx->RefPicSetStCurrBefore[i];
      }

      for (int i=0;i<ctx->NumPocLtCurr && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = ctx->RefPicSetLtCurr[i];
        isLongTerm[1][rIdx] = true;
      }

      // Multi layer extension (JCTVC-R1013_v6 F.8.3.4 F-66)
      for( int i=0;i<ctx->NumActiveRefLayerPics0; rIdx++, i++ ) {
        RefPicListTemp1[rIdx]=ctx->RefPicSetInterLayer0[i];
        isInterLayer[1][rIdx] = true;
        isLongTerm[1][rIdx] = true; // IL-pred picture is marked as long term reference
      }

      // This check is to prevent an endless loop when no images are added above.
      if (rIdx==0) {
        ctx->add_warning(DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST, false);
        return false;
      }
    }

    if (hdr->num_ref_idx_l0_active > 16) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED, false);
    return false;
  }

    assert(hdr->num_ref_idx_l1_active <= 16);
    for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
      int idx = hdr->ref_pic_list_modification_flag_l1 ? hdr->list_entry_l1[rIdx] : rIdx;

      hdr->RefPicList[1][rIdx] = RefPicListTemp1[idx];
      hdr->LongTermRefPic[1][rIdx] = isLongTerm[1][idx];
      hdr->InterLayerRefPic[1][rIdx] = isInterLayer[1][idx];

      // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
      de265_image* img_1_rIdx = NULL;
      if (hdr->InterLayerRefPic[1][rIdx]) {
        // Inter layer picture. Get the inter layer reference from ilRefPic.
        img_1_rIdx = ilRefPic[hdr->RefPicList[1][rIdx]];
      }
      else {
        img_1_rIdx = ctx->dpb.get_image(hdr->RefPicList[1][rIdx]);
      }
      if (img_1_rIdx == NULL) { return false; }
      hdr->RefPicList_POC[1][rIdx] = img_1_rIdx->PicOrderCntVal;
      hdr->RefPicList_PicState[1][rIdx] = img_1_rIdx->PicState;
    }
  }


  // show reference picture lists

  if (ctx->get_layer_id() > 0) {
    int nrRefDirs = (hdr->slice_type == SLICE_TYPE_B) ? 2 : 1;
    for (int i = 0; i < nrRefDirs; i++) {
      int numRefIdxActive = (i==0) ? hdr->num_ref_idx_l0_active : hdr->num_ref_idx_l1_active;
      loginfo(LogHeaders,"RefPicList[%d] =", i);
      for (rIdx=0; rIdx<numRefIdxActive; rIdx++) {
      loginfo(LogHeaders,"* [%d]=%d (LT=%d) (IL=%d)",
              hdr->RefPicList[i][rIdx],
              hdr->RefPicList_POC[i][rIdx],
              hdr->LongTermRefPic[i][rIdx],
              hdr->InterLayerRefPic[i][rIdx]
              );
      }
      loginfo(LogHeaders,"*\n");
    }
    return true;
  }

  loginfo(LogHeaders,"RefPicList[0] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d (LT=%d)",
            hdr->RefPicList[0][rIdx],
            hdr->RefPicList_POC[0][rIdx],
            hdr->LongTermRefPic[0][rIdx]
            );
  }
  loginfo(LogHeaders,"*\n");

  if (hdr->slice_type == SLICE_TYPE_B) {
    loginfo(LogHeaders,"RefPicList[1] =");
    for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
      loginfo(LogHeaders,"* [%d]=%d (LT=%d)",
              hdr->RefPicList[1][rIdx],
              hdr->RefPicList_POC[1][rIdx],
              hdr->LongTermRefPic[1][rIdx]
              );
    }
    loginfo(LogHeaders,"*\n");
  }

  return true;
}



void decoder_context::run_postprocessing_filters_sequential(de265_image* img)
{
#if SAVE_INTERMEDIATE_IMAGES
    char buf[1000];
    sprintf(buf,"pre-lf-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif

    if (!img->decctx->param_disable_deblocking) {
      apply_deblocking_filter(img);
    }

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"pre-sao-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif

    if (!img->decctx->param_disable_sao) {
      apply_sample_adaptive_offset_sequential(img);
    }

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"sao-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif
}


void decoder_context::run_postprocessing_filters_parallel(image_unit* imgunit)
{
  de265_image* img = imgunit->img;

  int saoWaitsForProgress = CTB_PROGRESS_PREFILTER;
  bool waitForCompletion = false;

  if (!img->decctx->param_disable_deblocking) {
    add_deblocking_tasks(imgunit);
    saoWaitsForProgress = CTB_PROGRESS_DEBLK_H;
  }

  if (!img->decctx->param_disable_sao) {
    waitForCompletion |= add_sao_tasks(imgunit, saoWaitsForProgress);
    //apply_sample_adaptive_offset(img);
  }

  img->wait_for_completion();
}

/*
void decoder_context::push_current_picture_to_output_queue()
{
  push_picture_to_output_queue(img);
}
*/

de265_error decoder_context::push_picture_to_output_queue(image_unit* imgunit)
{
  de265_image* outimg = imgunit->img;

  if (outimg==NULL) { return DE265_OK; }


  // push image into output queue

  if (outimg->PicOutputFlag) {
    loginfo(LogDPB,"new picture has output-flag=true\n");

    if (outimg->integrity != INTEGRITY_CORRECT &&
        param_suppress_faulty_pictures) {
    }
    else {
      dpb.insert_image_into_reorder_buffer(outimg);
    }

    loginfo(LogDPB,"push image %d into reordering queue\n", outimg->PicOrderCntVal);
  }

  // check for full reorder buffers

  int sublayer = outimg->vps.vps_max_sub_layers -1;
  int maxNumPicsInReorderBuffer = outimg->vps.layer[sublayer].vps_max_num_reorder_pics;

  if (dpb.num_pictures_in_reorder_buffer() > maxNumPicsInReorderBuffer) {
    dpb.output_next_picture_in_reorder_buffer();
  }

  dpb.log_dpb_queues();

  return DE265_OK;
}


// returns whether we can continue decoding the stream or whether we should give up
bool decoder_context::process_slice_segment_header(decoder_context* ctx, slice_segment_header* hdr,
                                                   de265_error* err, de265_PTS pts,
                                                   nal_header* nal_hdr,
                                                   void* user_data)
{
  *err = DE265_OK;

  flush_reorder_buffer_at_this_frame = false;


  // get PPS and SPS for this slice

  int pps_id = hdr->slice_pic_parameter_set_id;
  if (ctx->pps[pps_id].pps_read==false) {
    logerror(LogHeaders, "PPS %d has not been read\n", pps_id);
    assert(false); // TODO
  }

  ctx->current_pps = &ctx->pps[pps_id];
  ctx->current_sps = &ctx->sps[ (int)ctx->current_pps->seq_parameter_set_id ];
  ctx->current_vps = get_vps((int)ctx->current_sps->video_parameter_set_id);

  calc_tid_and_framerate_ratio();


  // --- prepare decoding of new picture ---

  if (hdr->first_slice_segment_in_pic_flag) {

    // previous picture has been completely decoded

    //ctx->push_current_picture_to_output_queue();

    ctx->current_image_poc_lsb = hdr->slice_pic_order_cnt_lsb;


    seq_parameter_set* sps = ctx->current_sps;


    // --- find and allocate image buffer for decoding ---

    int image_buffer_idx;
    bool isOutputImage = (!sps->sample_adaptive_offset_enabled_flag || ctx->param_disable_sao);
    image_buffer_idx = ctx->dpb.new_image(sps, this, pts, user_data, isOutputImage);
    if (image_buffer_idx == -1) {
      *err = DE265_ERROR_IMAGE_BUFFER_FULL;
      return false;
    }

    de265_image* img = ctx->dpb.get_image(image_buffer_idx);
    img->nal_hdr = *nal_hdr;
    ctx->img = img;

    img->vps = *ctx->current_vps;
    //img->sps = *ctx->current_sps;  // already set in new_image()
    img->pps = *ctx->current_pps;
    img->decctx = ctx;

    img->clear_metadata();


    if (isIRAP(ctx->nal_unit_type)) {
      if (isIDR(ctx->nal_unit_type) ||
          isBLA(ctx->nal_unit_type) ||
          ctx->first_decoded_picture ||
          ctx->FirstAfterEndOfSequenceNAL)
        {
          ctx->NoRaslOutputFlag = true;
          ctx->FirstAfterEndOfSequenceNAL = false;
        }
      else if (0) // TODO: set HandleCraAsBlaFlag by external means
        {
        }
      else
        {
          ctx->NoRaslOutputFlag   = false;
          ctx->HandleCraAsBlaFlag = false;
        }
    }


    if (isRASL(ctx->nal_unit_type) &&
        ctx->NoRaslOutputFlag)
      {
        ctx->img->PicOutputFlag = false;
      }
    else
      {
        ctx->img->PicOutputFlag = !!hdr->pic_output_flag;
      }

    process_picture_order_count(ctx,hdr);

    if (hdr->first_slice_segment_in_pic_flag) {
      // Multi layer extension
      if (layer_ID > 0) {
        process_inter_layer_reference_picture_set(ctx,hdr);
      }

      // mark picture so that it is not overwritten by unavailable reference frames
      img->PicState = UsedForShortTermReference;

      process_reference_picture_set(ctx,hdr);
    }

    img->PicState = UsedForShortTermReference;

    log_set_current_POC(ctx->img->PicOrderCntVal);


    // next image is not the first anymore

    first_decoded_picture = false;
  }
  else {
    // claims to be not the first slice, but there is no active image available

    if (ctx->img == NULL) {
      return false;
    }
  }

  if (hdr->slice_type == SLICE_TYPE_B ||
      hdr->slice_type == SLICE_TYPE_P)
    {
      bool success = construct_reference_picture_lists(ctx,hdr);
      if (!success) {
        return false;
      }
    }

  //printf("process slice segment header\n");

  loginfo(LogHeaders,"end of process-slice-header\n");
  ctx->dpb.log_dpb_content();


  if (hdr->dependent_slice_segment_flag==0) {
    hdr->SliceAddrRS = hdr->slice_segment_address;
  } else {
    hdr->SliceAddrRS = ctx->previous_slice_header->SliceAddrRS;
  }

  ctx->previous_slice_header = hdr;


  loginfo(LogHeaders,"SliceAddrRS = %d\n",hdr->SliceAddrRS);

  return true;
}


void decoder_context::remove_images_from_dpb(const std::vector<int>& removeImageList)
{
  for (int i=0;i<removeImageList.size();i++) {
    int idx = dpb.DPB_index_of_picture_with_ID( removeImageList[i] );
    if (idx>=0) {
      //printf("remove ID %d\n", removeImageList[i]);
      de265_image* dpbimg = dpb.get_image( idx );
      dpbimg->PicState = UnusedForReference;
    }
  }
}



/*
  .     0     1     2       <- goal_HighestTid
  +-----+-----+-----+
  | -0->| -1->| -2->|
  +-----+-----+-----+
  0     33    66    100     <- framerate_ratio
 */

int  decoder_context::get_highest_TID() const
{
  if (current_sps) { return current_sps->sps_max_sub_layers-1; }
  if (current_vps) { return current_vps->vps_max_sub_layers-1; }

  return 6;
}

void decoder_context::set_limit_TID(int max_tid)
{
  limit_HighestTid = max_tid;
  calc_tid_and_framerate_ratio();
}

int decoder_context::change_framerate(int more)
{
  if (current_sps == NULL) { return framerate_ratio; }

  int highestTid = get_highest_TID();

  assert(more>=-1 && more<=1);

  goal_HighestTid += more;
  goal_HighestTid = std::max(goal_HighestTid, 0);
  goal_HighestTid = std::min(goal_HighestTid, highestTid);

  framerate_ratio = framedrop_tid_index[goal_HighestTid];

  calc_tid_and_framerate_ratio();

  return framerate_ratio;
}

void decoder_context::set_framerate_ratio(int percent)
{
  framerate_ratio = percent;
  calc_tid_and_framerate_ratio();
}

void decoder_context::compute_framedrop_table()
{
  int highestTID = get_highest_TID();

  for (int tid=highestTID ; tid>=0 ; tid--) {
    int lower  = 100 *  tid   /(highestTID+1);
    int higher = 100 * (tid+1)/(highestTID+1);

    for (int l=lower; l<=higher; l++) {
      int ratio = 100 * (l-lower) / (higher-lower);

      // if we would exceed our TID limit, decode the highest TID at full frame-rate
      if (tid > limit_HighestTid) {
        tid   = limit_HighestTid;
        ratio = 100;
      }

      framedrop_tab[l].tid   = tid;
      framedrop_tab[l].ratio = ratio;
    }

    framedrop_tid_index[tid] = higher;
  }

#if 0
  for (int i=0;i<=100;i++) {
    printf("%d%%: %d/%d",i, framedrop_tab[i].tid, framedrop_tab[i].ratio);
    for (int k=0;k<=highestTID;k++) {
      if (framedrop_tid_index[k] == i) printf(" ** TID=%d **",k);
    }
    printf("\n");
  }
#endif
}

void decoder_context::calc_tid_and_framerate_ratio()
{
  int highestTID = get_highest_TID();


  // if number of temporal layers changed, we have to recompute the framedrop table

  if (framedrop_tab[100].tid != highestTID) {
    compute_framedrop_table();
  }

  goal_HighestTid       = framedrop_tab[framerate_ratio].tid;
  layer_framerate_ratio = framedrop_tab[framerate_ratio].ratio;

  // TODO: for now, we switch immediately
  current_HighestTid = goal_HighestTid;
}


void error_queue::add_warning(de265_error warning, bool once)
{
  // check if warning was already shown
  bool add=true;
  if (once) {
    for (int i=0;i<nWarningsShown;i++) {
      if (warnings_shown[i] == warning) {
        add=false;
        break;
      }
    }
  }

  if (!add) {
    return;
  }


  // if this is a one-time warning, remember that it was shown

  if (once) {
    if (nWarningsShown < MAX_WARNINGS) {
      warnings_shown[nWarningsShown++] = warning;
    }
  }


  // add warning to output queue

  if (nWarnings == MAX_WARNINGS) {
    warnings[MAX_WARNINGS-1] = DE265_WARNING_WARNING_BUFFER_FULL;
    return;
  }

  warnings[nWarnings++] = warning;
}

error_queue::error_queue()
{
  nWarnings = 0;
  nWarningsShown = 0;
}

de265_error error_queue::get_warning()
{
  if (nWarnings==0) {
    return DE265_OK;
  }

  de265_error warn = warnings[0];
  nWarnings--;
  memmove(warnings, &warnings[1], nWarnings*sizeof(de265_error));

  return warn;
}

video_parameter_set* decoder_context::get_vps(int id)
{
  assert( id < DE265_MAX_VPS_SETS );
  if (layer_ID != 0) {
    // Multi layer decoding.
    // The VPS is handeled by the base layer decoder. Get it there.
    decoder_context_multilayer* ml_dec = get_multi_layer_decoder();
    return ml_dec->get_layer_dec(0)->get_vps(id);
  }
  return &vps[id];
}
