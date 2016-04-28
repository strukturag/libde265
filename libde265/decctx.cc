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
#include "util.h"
#include "sao.h"
#include "sei.h"
#include "deblock.h"
#include "image-unit.h"

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

#include <iostream> // TODO TMP

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

  IsCuChromaQpOffsetCoded = false;
  CuQpOffsetCb = 0;
  CuQpOffsetCr = 0;

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
  : nal(NULL),
    shdr(NULL),
    imgunit(NULL),
    flush_reorder_buffer(false),
    nThreads(0),
    first_decoded_CTB_RS(-1),
    last_decoded_CTB_RS(-1),
    thread_contexts(NULL),
    ctx(decctx)
{
  state = Unprocessed;
  nThreadContexts = 0;
}

slice_unit::~slice_unit()
{
  ctx->get_NAL_parser().free_NAL_unit(nal);

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
}


base_context::base_context()
{
  set_acceleration_functions(de265_acceleration_AUTO);
}



void decoder_context::set_frame_dropping_ratio(float ratio)
{
  if (ratio==0.0) {
    m_frontend_syntax_decoder.set_image_unit_sink( &m_frame_dropper_nop );
  }
  else {
    m_frame_dropper_ratio.set_dropping_ratio(ratio);
    m_frontend_syntax_decoder.set_image_unit_sink( &m_frame_dropper_ratio );
  }
}


decoder_context::decoder_context()
  : m_frontend_syntax_decoder(this),
    m_main_loop_thread(this)
{
  m_frontend_syntax_decoder.set_image_unit_sink( &m_frame_dropper_nop );

  m_frame_dropper_nop      .set_image_unit_sink( this );
  m_frame_dropper_IRAP_only.set_image_unit_sink( this );
  m_frame_dropper_ratio    .set_image_unit_sink( this );

  m_frame_dropper_ratio.set_decoder_context(*this);


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

  param_image_allocation_functions = image::default_image_allocation;

  /*
  memset(&vps, 0, sizeof(video_parameter_set)*DE265_MAX_VPS_SETS);
  memset(&sps, 0, sizeof(seq_parameter_set)  *DE265_MAX_SPS_SETS);
  memset(&pps, 0, sizeof(pic_parameter_set)  *DE265_MAX_PPS_SETS);
  memset(&slice,0,sizeof(slice_segment_header)*DE265_MAX_SLICES);
  */

  //memset(&thread_pool,0,sizeof(struct thread_pool));
  num_worker_threads = 0;


  // frame-rate

  limit_HighestTid = 6;   // decode all temporal layers (up to layer 6)
  framerate_ratio = 100;  // decode all 100%

  goal_HighestTid = 6;
  current_HighestTid = 6;
  layer_framerate_ratio = 100;

  m_end_of_stream = false;


  compute_framedrop_table();


  //


  // --- internal data ---

  //ctx->FirstAfterEndOfSequenceNAL = true;
  //ctx->last_RAP_picture_NAL_type = NAL_UNIT_UNDEFINED;

  //de265_init_image(&ctx->coeff);

  // --- decoded picture buffer ---


  start_decoding_thread();
}


decoder_context::~decoder_context()
{
}


void decoder_context::set_image_allocation_functions(de265_image_allocation* allocfunc)
{
  if (allocfunc) {
    param_image_allocation_functions = *allocfunc;
  }
  else {
    assert(false); // actually, it makes no sense to reset the allocation functions

    param_image_allocation_functions = image::default_image_allocation;
  }
}


de265_error decoder_context::start_thread_pool(int nThreads)
{
  thread_pool_.start(nThreads);

  num_worker_threads = nThreads;

  return DE265_OK;
}


void decoder_context::stop_thread_pool()
{
  if (get_num_worker_threads()>0) {
    //flush_thread_pool(&ctx->thread_pool);
    thread_pool_.stop();
  }
}


void decoder_context::reset()
{
  if (num_worker_threads>0) {
    //flush_thread_pool(&ctx->thread_pool);
    thread_pool_.stop();
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


  m_frontend_syntax_decoder.reset();


  // --- remove all pictures from output queue ---

  // there was a bug the peek_next_image did not return NULL on empty output queues.
  // This was (indirectly) fixed by recreating the DPB buffer, but it should actually
  // be sufficient to clear it like this.
  // The error showed while scrubbing the ToS video in VLC.
  dpb.clear();


  while (!image_units.empty()) {
    image_units.pop_back();
  }

  m_end_of_stream = false;


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

  const pic_parameter_set& pps = tctx->img->get_pps();
  const seq_parameter_set& sps = tctx->img->get_sps();


  if (tctx->shdr->slice_segment_address > 0) {
    int prevCtb = pps.CtbAddrTStoRS[ pps.CtbAddrRStoTS[tctx->shdr->slice_segment_address] -1 ];

    int ctbX = prevCtb % sps.PicWidthInCtbsY;
    int ctbY = prevCtb / sps.PicWidthInCtbsY;


    // take the pixel at the bottom right corner (but consider that the image size might be smaller)

    int x = ((ctbX+1) << sps.Log2CtbSizeY)-1;
    int y = ((ctbY+1) << sps.Log2CtbSizeY)-1;

    x = std::min(x,sps.pic_width_in_luma_samples-1);
    y = std::min(y,sps.pic_height_in_luma_samples-1);

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
}


void decoder_context::add_task_decode_slice_segment(thread_context* tctx, bool firstSliceSubstream,
                                                    int ctbx,int ctby)
{
  auto task = std::make_shared<thread_task_slice_segment>();
  task->firstSliceSubstream = firstSliceSubstream;
  task->tctx = tctx;
  task->debug_startCtbX = ctbx;
  task->debug_startCtbY = ctby;
  tctx->task = task;

  thread_pool_.add_task(task);

  tctx->imgunit->tasks.push_back(task);
}


template <class T> void pop_front(std::vector<T>& vec)
{
  for (int i=1;i<vec.size();i++)
    vec[i-1] = vec[i];

  vec.pop_back();
}


void decoder_context::start_decoding_thread()
{
  m_main_loop_thread.start();
}

void decoder_context::stop_decoding_thread()
{
  m_main_loop_thread.stop();
}


void decoder_context::run_main_loop()
{
  m_main_loop_mutex.lock();

  // --- wait until we have new image_units and the decoding queue has some space ---

  for (;;) {
    bool queue_full = (m_image_units_in_progress.size() >= m_max_images_processed_in_parallel);
    bool input_empty= (image_units.empty());

    if (queue_full) {
      m_main_loop_full_cond.wait(m_main_loop_mutex);
      continue;
    }

    if (input_empty && !m_end_of_stream) {
      m_input_empty_cond.wait(m_main_loop_mutex);
      continue;
    }

    break;
  }


  // --- move one image_unit to the decoding queue ---

  image_unit_ptr to_be_decoded;

  if (!image_units.empty()) {
    to_be_decoded = image_units.front();
    pop_front(image_units);

    m_image_units_in_progress.push_back(to_be_decoded);
  }

  m_main_loop_mutex.unlock();


  // --- create threads to decode this image ---

  if (to_be_decoded) {
    decode_image_frame_parallel(to_be_decoded);
  }
}


void decoder_context::check_decoding_queue_for_finished_images()
{
  m_main_loop_mutex.lock();

  while (!m_image_units_in_progress.empty() &&
         m_image_units_in_progress.front()->img->debug_is_completed()) {
    image_unit_ptr imgunit = m_image_units_in_progress.front();
    pop_front(m_image_units_in_progress);

    printf("pushing to output queue: %d\n", imgunit->img->PicOrderCntVal);
    push_picture_to_output_queue(imgunit->img);
  }

  m_main_loop_mutex.unlock();
}


void decoder_context::decode_image_frame_parallel(image_unit_ptr imgunit)
{
  std::cout << "decoding of image " << imgunit->img->PicOrderCntVal << "\n";


  for (slice_unit* sliceunit : imgunit->slice_units) {
    de265_error err = decode_slice_unit_frame_parallel(imgunit.get(), sliceunit);
    if (err) {
      // TODO
    }
  }
}


de265_error decoder_context::decode_image_unit(bool* did_work)
{
  de265_error err = DE265_OK;

  *did_work = false;




  *did_work = true; return DE265_OK; // TMP HACK




  if (image_units.empty()) {
    // do not return error, because actually, we do not mind...
    return DE265_OK; // DE265_ERROR_WAITING_FOR_INPUT_DATA; // nothing to do
  }


  // decode something if there is work to do

  if ( ! image_units.empty() ) {

    loginfo(LogHighlevel,"decode_image_unit -> decode a slice\n");

    image_unit* imgunit = image_units[0].get();
    slice_unit* sliceunit = imgunit->get_next_unprocessed_slice_segment();

    //printf("DROP: %s\n", imgunit->state != image_unit::Dropped ? "no":"yes");

    if (sliceunit != NULL && imgunit->state != image_unit::Dropped) {

      if (sliceunit->flush_reorder_buffer) {
        m_output_queue.flush_reorder_buffer();
      }

      *did_work = true;

      err = decode_slice_unit_parallel(imgunit, sliceunit);
      if (err) {
        return err;
      }
    }
  }



  // if we decoded all slices of the current image and there will not
  // be added any more slices to the image, output the image

  /*
  NAL_Parser& nal_parser = m_frontend_syntax_decoder.get_NAL_parser();

  if ( ( image_units.size()>=2 && image_units[0]->all_slice_segments_processed()) ||
       ( image_units.size()>=1 && image_units[0]->all_slice_segments_processed() &&
         nal_parser.number_of_NAL_units_pending()==0 &&
         (nal_parser.is_end_of_stream() || nal_parser.is_end_of_frame()) )) {
  */

  // --- process the dropped frames (process remove-references list and push
  //     dummy frame to output

  if ( image_units.size()>=1 && image_units[0]->state == image_unit::Dropped) {
    // push a dummy 'dropped' image to the output as a placeholder

    image_unit* imgunit = image_units[0].get();

    slice_unit* sliceunit = imgunit->get_next_unprocessed_slice_segment();
    if (sliceunit) {
      remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);
    }

    imgunit->img->integrity = INTEGRITY_NOT_DECODED;

    *did_work=true;

    push_picture_to_output_queue(imgunit->img);

    pop_front(image_units);
  }



  if ( image_units.size()>=1 && image_units[0]->all_slice_segments_processed()) {
    loginfo(LogHighlevel,"postprocess image\n");

    image_unit* imgunit = image_units[0].get();

    *did_work=true;


    // mark all CTBs as decoded even if they are not, because faulty input
    // streams could miss part of the picture
    // TODO: this will not work when slice decoding is parallel to post-filtering,
    // so we will have to replace this with keeping track of which CTB should have
    // been decoded (but aren't because of the input stream being faulty)

    imgunit->img->mark_all_CTB_progress(CTB_PROGRESS_PREFILTER);



    // run post-processing filters (deblocking & SAO)

    if (num_worker_threads)
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


    push_picture_to_output_queue(imgunit->img);

    // remove just decoded image unit from queue

    pop_front(image_units);
  }


  if (m_end_of_stream && image_units.empty()) {
    // flush all pending pictures into output queue

    loginfo(LogHighlevel,"FLUSH\n");

    // ctx->push_current_picture_to_output_queue(); // TODO: not with new queue
    m_output_queue.flush_reorder_buffer();
  }

  dpb.log_dpb_content();

  return err;
}


de265_error decoder_context::decode_slice_unit_sequential(image_unit* imgunit,
                                                          slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  printf("decode slice POC=%d addr=%d, img=%p\n",
         sliceunit->shdr->slice_pic_order_cnt_lsb,
         sliceunit->shdr->slice_segment_address,
         imgunit->img.get());


  // TODO: remove later from DPB
  //remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);

  if (sliceunit->shdr->slice_segment_address >= imgunit->img->get_pps().CtbAddrRStoTS.size()) {
    return DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA;
  }


  sliceunit->allocate_thread_contexts(1);
  thread_context* tctx = sliceunit->get_thread_context(0);

  tctx->shdr = sliceunit->shdr;
  tctx->img  = imgunit->img;
  tctx->decctx = this;
  tctx->imgunit = imgunit;
  tctx->sliceunit= sliceunit;
  tctx->CtbAddrInTS = imgunit->img->get_pps().CtbAddrRStoTS[tctx->shdr->slice_segment_address];
  tctx->task = NULL;

  init_thread_context(tctx);

  if (sliceunit->reader.bytes_remaining <= 0) {
    return DE265_ERROR_PREMATURE_END_OF_SLICE;
  }

  init_CABAC_decoder(&tctx->cabac_decoder,
                     sliceunit->reader.data,
                     sliceunit->reader.bytes_remaining);

  // alloc CABAC-model array if entropy_coding_sync is enabled

  if (imgunit->img->get_pps().entropy_coding_sync_enabled_flag &&
      sliceunit->shdr->first_slice_segment_in_pic_flag) {
    imgunit->ctx_models.resize( (imgunit->img->get_sps().PicHeightInCtbsY-1) ); //* CONTEXT_MODEL_TABLE_LENGTH );
  }

  tctx->img->thread_start(1);
  sliceunit->nThreads++;


  auto task = std::make_shared<thread_task_slice>();
  task->tctx = tctx;
  tctx->task = task;

  thread_pool_.add_task(task);

  tctx->imgunit->tasks.push_back(task);


  //err=read_slice_segment_data(&tctx);
  //sliceunit->finished_threads.set_progress(1);

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

  // TODO: do this when the slice is moved from processing to output queue
  remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);

  /*
  printf("-------- decode --------\n");
  printf("IMAGE UNIT %p\n",imgunit);
  sliceunit->shdr->dump_slice_segment_header(sliceunit->ctx, 1);
  imgunit->dump_slices();
  */

  image_ptr img = imgunit->img;
  const pic_parameter_set& pps = img->get_pps();

  sliceunit->state = slice_unit::InProgress;

  bool use_WPP = (img->decctx->num_worker_threads > 0 &&
                  pps.entropy_coding_sync_enabled_flag);

  bool use_tiles = (img->decctx->num_worker_threads > 0 &&
                    pps.tiles_enabled_flag);


  // TODO: remove this warning later when we do frame-parallel decoding
  if (img->decctx->num_worker_threads > 0 &&
      pps.entropy_coding_sync_enabled_flag == false &&
      pps.tiles_enabled_flag == false) {

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


de265_error decoder_context::decode_slice_unit_frame_parallel(image_unit* imgunit,
                                                              slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  // TODO: do this when the slice is moved from processing to output queue
  // remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);

  /*
  printf("-------- decode --------\n");
  printf("IMAGE UNIT %p\n",imgunit);
  sliceunit->shdr->dump_slice_segment_header(sliceunit->ctx, 1);
  imgunit->dump_slices();
  */

  image_ptr img = imgunit->img;
  const pic_parameter_set& pps = img->get_pps();

  sliceunit->state = slice_unit::InProgress;

  bool use_WPP = (img->decctx->num_worker_threads > 0 &&
                  pps.entropy_coding_sync_enabled_flag);

  bool use_tiles = (img->decctx->num_worker_threads > 0 &&
                    pps.tiles_enabled_flag);


  // TODO: remove this warning later when we do frame-parallel decoding
  if (img->decctx->num_worker_threads > 0 &&
      pps.entropy_coding_sync_enabled_flag == false &&
      pps.tiles_enabled_flag == false) {

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
    printf("SEQ\n");
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

  image_ptr img = imgunit->img;
  slice_segment_header* shdr = sliceunit->shdr;
  const pic_parameter_set& pps = img->get_pps();

  int nRows = shdr->num_entry_point_offsets +1;
  int ctbsWidth = img->get_sps().PicWidthInCtbsY;


  assert(img->num_threads_active() == 0);


  // reserve space to store entropy coding context models for each CTB row

  if (shdr->first_slice_segment_in_pic_flag) {
    // reserve space for nRows-1 because we don't need to save the CABAC model in the last CTB row
    imgunit->ctx_models.resize( (img->get_sps().PicHeightInCtbsY-1) ); //* CONTEXT_MODEL_TABLE_LENGTH );
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
    tctx->CtbAddrInTS = pps.CtbAddrRStoTS[ctbAddrRS];

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

    auto task = std::make_shared<thread_task_ctb_row>();
    task->firstSliceSubstream = (entryPt==0);
    task->tctx = tctx;
    task->debug_startCtbRow = ctbRow;
    tctx->task = task;

    thread_pool_.add_task(task);

    tctx->imgunit->tasks.push_back(task);
  }

#if 1
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

  imgunit->tasks.clear();

  return DE265_OK;
}

de265_error decoder_context::decode_slice_unit_tiles(image_unit* imgunit,
                                                     slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  image_ptr img = imgunit->img;
  slice_segment_header* shdr = sliceunit->shdr;
  const pic_parameter_set& pps = img->get_pps();

  int nTiles = shdr->num_entry_point_offsets +1;
  int ctbsWidth = img->get_sps().PicWidthInCtbsY;


  assert(img->num_threads_active() == 0);

  sliceunit->allocate_thread_contexts(nTiles);


  // first CTB in this slice
  int ctbAddrRS = shdr->slice_segment_address;
  int tileID = pps.TileIdRS[ctbAddrRS];

  for (int entryPt=0;entryPt<nTiles;entryPt++) {
    // entry points other than the first start at tile beginnings
    if (entryPt>0) {
      tileID++;

      if (tileID >= pps.num_tile_columns * pps.num_tile_rows) {
        err = DE265_WARNING_SLICEHEADER_INVALID;
        break;
      }

      int ctbX = pps.colBd[tileID % pps.num_tile_columns];
      int ctbY = pps.rowBd[tileID / pps.num_tile_columns];
      ctbAddrRS = ctbY * ctbsWidth + ctbX;
    }

    // set thread context

    thread_context* tctx = sliceunit->get_thread_context(entryPt);

    tctx->shdr   = shdr;
    tctx->decctx = img->decctx;
    tctx->img    = img;
    tctx->imgunit = imgunit;
    tctx->sliceunit= sliceunit;
    tctx->CtbAddrInTS = pps.CtbAddrRStoTS[ctbAddrRS];

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

  imgunit->tasks.clear();

  return err;
}


void decoder_context::debug_imageunit_state()
{
  loginfo(LogHighlevel,"image_units: ");
  for (int i=0;i<image_units.size();i++) {
    loginfo(LogHighlevel,"%d ", image_units[i]->img->get_ID());
  }
  loginfo(LogHighlevel,"\n");
}



void decoder_context::send_image_unit(image_unit_ptr imgunit)
{
  m_main_loop_mutex.lock();

  image_units.push_back(imgunit);

  m_input_empty_cond.signal();
  m_main_loop_mutex.unlock();

  debug_imageunit_state();
}


void decoder_context::send_end_of_stream()
{
  m_main_loop_mutex.lock();

  m_end_of_stream = true;

  m_input_empty_cond.signal();
  m_main_loop_mutex.unlock();
}


void decoder_context::run_postprocessing_filters_sequential(image_ptr img)
{
#if SAVE_INTERMEDIATE_IMAGES
    char buf[1000];
    sprintf(buf,"pre-lf-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif

    if (!img->decctx->param_disable_deblocking) {
      apply_deblocking_filter(img.get());
    }

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"pre-sao-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif

    if (!img->decctx->param_disable_sao) {
      apply_sample_adaptive_offset_sequential(img.get());
    }

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"sao-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif
}


void decoder_context::run_postprocessing_filters_parallel(image_unit* imgunit)
{
  image* img = imgunit->img.get();

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


de265_error decoder_context::push_picture_to_output_queue(image_ptr outimg)
{
  if (!outimg) { return DE265_OK; }


  // push image into output queue

  if (outimg->has_vps()) {
    int sublayer = outimg->get_vps().vps_max_sub_layers -1;
    const layer_data& layer = outimg->get_vps().layer[sublayer];

    m_output_queue.set_num_reorder_pics( layer.vps_max_num_reorder_pics );
  }


  if (outimg->PicOutputFlag) {
    loginfo(LogDPB,"new picture has output-flag=true\n");

    if (outimg->integrity != INTEGRITY_CORRECT &&
        param_suppress_faulty_pictures) {
    }
    else {
      m_output_queue.insert_image_into_reorder_buffer(outimg);
    }

    loginfo(LogDPB,"push image %d into reordering queue\n", outimg->PicOrderCntVal);
  }

  m_output_queue.log_dpb_queues();

  return DE265_OK;
}


void decoder_context::remove_images_from_dpb(const std::vector<int>& removeImageList)
{
  for (int i=0;i<removeImageList.size();i++) {
    int idx = dpb.DPB_index_of_picture_with_ID( removeImageList[i] );
    if (idx>=0) {
      //printf("remove ID %d\n", removeImageList[i]);
      image_ptr dpbimg = dpb.get_image( idx );
      dpbimg->PicState = UnusedForReference;
    }
  }
}



void decoder_context::set_limit_TID(int max_tid)
{
  limit_HighestTid = max_tid;
  calc_tid_and_framerate_ratio();
}

int decoder_context::change_framerate(int more)
{
  if (!get_frontend_syntax_decoder().get_current_sps()) { return framerate_ratio; }

  int highestTid = get_frontend_syntax_decoder().get_highest_TID();

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

  // TODO: ideally, we should combine this with dropping temporal layers
  // (maybe in another frame_dropper implementation, pipelined)
  // calc_tid_and_framerate_ratio();

  set_frame_dropping_ratio((100-percent)/100.0);
}

void decoder_context::compute_framedrop_table()
{
  int highestTID = get_frontend_syntax_decoder().get_highest_TID();

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
  int highestTID = get_frontend_syntax_decoder().get_highest_TID();


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
