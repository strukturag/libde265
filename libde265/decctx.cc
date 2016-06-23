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
  m_CTB_out_of_image = true;

  IsCuQpDeltaCoded = false;
  CuQpDelta = 0;

  IsCuChromaQpOffsetCoded = false;
  CuQpOffsetCb = 0;
  CuQpOffsetCr = 0;

  decctx = NULL;
  img.reset();
  shdr = NULL;

  imgunit = NULL;
  sliceunit = NULL;


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


void thread_context::init_quantization()
{
  // zero scrap memory for coefficient blocks
  memset(_coeffBuf, 0, sizeof(_coeffBuf));  // TODO: check if we can safely remove this

  currentQG_x = -1;
  currentQG_y = -1;



  // --- find QPY that was active at the end of the previous slice ---

  // find the previous CTB in TS order

  const pic_parameter_set& pps = img->get_pps();
  const seq_parameter_set& sps = img->get_sps();


  if (shdr->slice_segment_address > 0) {
    int prevCtb = pps.CtbAddrTStoRS[ pps.CtbAddrRStoTS[shdr->slice_segment_address] -1 ];

    int ctbX = prevCtb % sps.PicWidthInCtbsY;
    int ctbY = prevCtb / sps.PicWidthInCtbsY;


    // take the pixel at the bottom right corner (but consider that the image size might be smaller)

    int x = ((ctbX+1) << sps.Log2CtbSizeY)-1;
    int y = ((ctbY+1) << sps.Log2CtbSizeY)-1;

    x = std::min(x,sps.pic_width_in_luma_samples-1);
    y = std::min(y,sps.pic_height_in_luma_samples-1);

    //printf("READ QPY: %d %d -> %d (should %d)\n",x,y,imgunit->img->get_QPY(x,y), tc.currentQPY);

    currentQPY = img->get_QPY(x,y);
  }
}


void thread_context::set_CTB_address_RS(int addr)
{
  CtbAddrInTS = img->get_pps().CtbAddrRStoTS[addr];
  setCtbAddrFromTS();
}


bool thread_context::advance_CTB_TS()
{
  CtbAddrInTS++;
  setCtbAddrFromTS();

  return m_CTB_out_of_image;
}




void thread_context::setCtbAddrFromTS()
{
  const seq_parameter_set& sps = img->get_sps();

  if (CtbAddrInTS < sps.PicSizeInCtbsY) {
    CtbAddrInRS = img->get_pps().CtbAddrTStoRS[CtbAddrInTS];

    CtbX = CtbAddrInRS % sps.PicWidthInCtbsY;
    CtbY = CtbAddrInRS / sps.PicWidthInCtbsY;

    m_CTB_out_of_image = false;
  }
  else {
    CtbAddrInRS = sps.PicSizeInCtbsY;

    CtbX = CtbAddrInRS % sps.PicWidthInCtbsY;
    CtbY = CtbAddrInRS / sps.PicWidthInCtbsY;

    m_CTB_out_of_image = true;
  }
}



void thread_context::mark_covered_CTBs_as_processed(int progress)
{
  // mark all CTBs assigned to this slice as processed

  for (int ctb=first_CTB_TS; ctb <= last_CTB_TS; ctb++)
    {
      if (ctb >= img->number_of_ctbs())
        break;

      int ctb_rs = shdr->pps->CtbAddrTStoRS[ctb];

      img->ctb_progress[ctb_rs].set_progress(progress);
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

  // --- processing ---

  param_image_allocation_functions = image::default_image_allocation;

  num_worker_threads = 0;


  // frame-rate

  limit_HighestTid = 6;   // decode all temporal layers (up to layer 6)
  framerate_ratio = 100;  // decode all 100%

  goal_HighestTid = 6;
  current_HighestTid = 6;
  layer_framerate_ratio = 100;

  m_end_of_stream = false;


  compute_framedrop_table();


  // --- decoded picture buffer ---


  // --- start main loop ---

  start_decoding_thread();
}


decoder_context::~decoder_context()
{
  stop_decoding_thread();
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
  m_thread_pool.start(nThreads);
  m_master_thread_pool.start(m_max_images_processed_in_parallel);

  num_worker_threads = nThreads;

  return DE265_OK;
}


void decoder_context::stop_thread_pool()
{
  if (get_num_worker_threads()>0) {
    //flush_thread_pool(&ctx->thread_pool);
    m_thread_pool.stop();
  }

  m_master_thread_pool.stop();
}


void decoder_context::reset()
{
  if (num_worker_threads>0) {
    m_thread_pool.stop();
    m_master_thread_pool.stop();
  }


  m_frontend_syntax_decoder.reset();

  m_frame_dropper_nop.reset();
  m_frame_dropper_IRAP_only.reset();
  m_frame_dropper_ratio.reset();

  m_output_queue.clear();

  // --- remove all pictures from output queue ---

  // there was a bug the peek_next_image did not return NULL on empty output queues.
  // This was (indirectly) fixed by recreating the DPB buffer, but it should actually
  // be sufficient to clear it like this.
  // The error showed while scrubbing the ToS video in VLC.
  dpb.clear();


  m_undecoded_image_units.clear();
  m_decoded_image_units.clear();

  m_end_of_stream = false;


  // --- start threads again ---

  if (num_worker_threads>0) {
    // TODO: need error checking
    start_thread_pool(num_worker_threads);
    m_master_thread_pool.start(m_max_images_processed_in_parallel);
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




void decoder_context::start_decoding_thread()
{
  m_main_loop_thread.start();
}

void decoder_context::stop_decoding_thread()
{
  m_main_loop_thread.stop();
  send_main_loop_stop_signals();
  m_main_loop_thread.join();
}


int  decoder_context::get_action(bool blocking)
{
  int actions = 0;

  m_main_loop_mutex.lock();

  for (;;) {
    // check whether all decoding threads are busy
    bool decoding_slots_full = (m_image_units_in_progress.size() >= m_max_images_processed_in_parallel);

    // at least one image-unit is complete
    int num_pending_input_images = m_undecoded_image_units.size();

    // fill more data until decoding queue is full and there is at least one complete
    // image-unit pending at the input
    bool want_more_data = true;

    if (m_end_of_stream) {
      want_more_data = false;
    }

    // We stop requesting more data when all decoding slots are full and
    // there are already some (two) undecoded images waiting at the input.
    const int norm_num_pending_input_images = 2;
    if (decoding_slots_full && num_pending_input_images >= norm_num_pending_input_images) {
      want_more_data = false;
    }

    if (want_more_data) {
      actions |= de265_action_push_more_input;
    }


    // --- Do we have decoded images ready for the client ? ---

    if (num_pictures_in_output_queue() > 0) {
      actions |= de265_action_get_image;
    }


    // --- check whether decoding has finished ---

    const bool no_input_pending = m_undecoded_image_units.empty();
    const bool decoding_slots_empty = m_image_units_in_progress.empty();
    const bool decoded_images_queue_empty = m_decoded_image_units.empty();
    const bool output_queue_empty = (m_output_queue.num_pictures_in_output_queue() == 0 &&
                                     m_output_queue.num_pictures_in_reorder_buffer() ==0);

    /*
    printf("buffer status: %d %d %d %d %d\n",
           m_end_of_stream,
           no_input_pending,
           decoding_slots_empty,
           decoded_images_queue_empty,
           output_queue_empty);
    */

    if (m_end_of_stream &&
        no_input_pending &&
        decoding_slots_empty &&
        decoded_images_queue_empty &&
        output_queue_empty) {
      actions |= de265_action_end_of_stream;
    }


    if (!blocking) {
      break;
    }
    else if (actions != 0) {
      break;
    }
    else {
      // block until API-action status changes

      m_cond_api_action.wait(m_main_loop_mutex);
    }
  }

  m_main_loop_mutex.unlock();

  return actions;
}


void decoder_context::run_main_loop()
{
  m_main_loop_mutex.lock();

  for (;;) {
    bool did_something = false;


    // === start decoding a new image ===

    // --- check whether we have new image_units and we have some decoding slots available ---

    bool decoding_slots_available = (m_image_units_in_progress.size() <
                                     m_max_images_processed_in_parallel);
    bool input_available= !m_undecoded_image_units.empty();

    if (input_available &&
        decoding_slots_available) {

      // --- move one image_unit to the decoding queue ---

      image_unit_ptr to_be_decoded = m_undecoded_image_units.front();
      m_undecoded_image_units.pop_front();

      m_image_units_in_progress.push_back(to_be_decoded);

      // --- create threads to decode this image ---

      decode_image_frame_parallel(to_be_decoded);

      did_something = true;
    }


    // === move decoded images to output queue ===

    while (!m_decoded_image_units.empty()) {
      image_unit_ptr imgunit = m_decoded_image_units.front();

      // make sure the master-thread has ended
      // we have to temporally unlock the mutex to let the master thread finish

      m_main_loop_mutex.unlock();
      imgunit->master_task->join();
      m_main_loop_mutex.lock();

      m_decoded_image_units.pop_front();


      push_picture_to_output_queue(imgunit);

      did_something = true;

      m_cond_api_action.signal();
    }


    if (m_end_of_stream &&
        m_image_units_in_progress.empty()) {

      m_output_queue.flush_reorder_buffer();
    }



    if (!did_something) {
      m_main_loop_block_cond.wait(m_main_loop_mutex);
    }

    if (m_main_loop_thread.should_stop()) {
      break;
    }
  }

  m_main_loop_mutex.unlock();
}


void decoder_context::on_image_decoding_finished()
{
  m_main_loop_mutex.lock();


  // --- move all pictures that are completely decoded from progress queue to decoded queue ---

  while (!m_image_units_in_progress.empty() &&              // TODO -----> final progress
         m_image_units_in_progress.front()->did_finish_decoding()) {
    //m_image_units_in_progress.front()->img->do_all_CTBs_have_progress(CTB_PROGRESS_PREFILTER)) {

    image_unit_ptr imgunit = m_image_units_in_progress.front();
    m_image_units_in_progress.pop_front();

    // inform main thread that there are now decoding slots available
    m_main_loop_block_cond.signal();

    //imgunit->img->exchange_pixel_data_with(imgunit->sao_output);

    m_decoded_image_units.push_back(imgunit);
  }

  m_cond_api_action.signal();

  m_main_loop_mutex.unlock();
}



class thread_image_master_control : public de265_thread
{
public:
  image_unit* imgunit;

  int ctb_finished_progress;

  // (CTB_PROGRESS_SAO); PREFILTER);

  //virtual std::string name() const { return "image_master_control"; }
  virtual void run() {
    //imgunit->img->wait_until_all_CTBs_have_progress(ctb_finished_progress);
    imgunit->wait_to_finish_decoding();
    imgunit->img->decctx->on_image_decoding_finished();
  }

  /*
  virtual void cleanup() {
    image_unit_ptr i = imgunit;

    imgunit.reset();
    i->decctx->master_tasks.erase(this);
  }
  */
};


void decoder_context::decode_image_frame_parallel(image_unit_ptr imgunit)
{
  // std::cout << "create tasks for decoding of image " << imgunit->img->PicOrderCntVal << "\n";


  // --- build map which CTB is controled by which slice-header ---

  const seq_parameter_set& sps = imgunit->img->get_sps();
  const pic_parameter_set& pps = imgunit->img->get_pps();

  for (slice_unit* sliceunit : imgunit->slice_units) {
    for (int ctb = sliceunit->first_CTB_TS ;
         ctb <= sliceunit->last_CTB_TS ;
         ctb++) {
      int ctb_rs = pps.CtbAddrTStoRS[ctb];

      int xCtb = ctb_rs % sps.PicWidthInCtbsY;
      int yCtb = ctb_rs / sps.PicWidthInCtbsY;

      imgunit->img->set_SliceAddrRS          (xCtb,yCtb, sliceunit->shdr->SliceAddrRS);
      imgunit->img->set_SliceHeaderIndex_ctbs(xCtb,yCtb, sliceunit->shdr->slice_index);
    }
  }


  // --- generate decoding thread tasks for each slice ---

  for (slice_unit* sliceunit : imgunit->slice_units) {

    //printf("decoding slice CTBs: %d - %d\n",sliceunit->first_CTB_TS, sliceunit->last_CTB_TS);

    de265_error err = decode_slice_unit_frame_parallel(imgunit.get(), sliceunit);
    if (err) {
      // TODO
    }
  }



  int final_ctb_progress = CTB_PROGRESS_PREFILTER;

  // --- generate tasks for deblocking ---

  if (!imgunit->img->decctx->param_disable_deblocking) {
    add_deblocking_tasks(imgunit.get());
    final_ctb_progress = CTB_PROGRESS_DEBLK_H;
  }

  // --- generate tasks for SAO ---

  if (!imgunit->img->decctx->param_disable_sao) {
    if (add_sao_tasks(imgunit.get(), final_ctb_progress)) {
    //apply_sample_adaptive_offset(img);
      final_ctb_progress = CTB_PROGRESS_SAO;
    }
  }


  imgunit->img->mFinalCTBProgress = final_ctb_progress;


  // Generate master-control thread (we have to create this after the other threads,
  // because the master thread waits for all other threads to finish).

  auto master_thread = std::make_shared<thread_image_master_control>();
  master_thread->imgunit = imgunit.get();
  master_thread->ctb_finished_progress = final_ctb_progress;

  imgunit->master_task = master_thread;
  master_thread->start();

  //master_tasks.push_back(master_thread);
  //m_master_thread_pool.add_task(master_thread);
}


de265_error decoder_context::decode_image_unit(bool* did_work)
{
  de265_error err = DE265_OK;

  *did_work = false;




  *did_work = true; return DE265_OK; // TMP HACK

#if 0


  if (m_undecoded_image_units.empty()) {
    // do not return error, because actually, we do not mind...
    return DE265_OK; // DE265_ERROR_WAITING_FOR_INPUT_DATA; // nothing to do
  }


  // decode something if there is work to do

  if ( ! m_undecoded_image_units.empty() ) {

    loginfo(LogHighlevel,"decode_image_unit -> decode a slice\n");

    image_unit* imgunit = m_undecoded_image_units[0].get();
    slice_unit* sliceunit = imgunit->get_next_unprocessed_slice_segment();

    //printf("DROP: %s\n", imgunit->state != image_unit::Dropped ? "no":"yes");

    if (sliceunit != NULL && imgunit->state != image_unit::Dropped) {

      if (sliceunit->flush_reorder_buffer) {
        m_output_queue.flush_reorder_buffer();
      }

      *did_work = true;

      err = decode_slice_unit_parallel(sliceunit);
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

  if ( m_undecoded_image_units.size()>=1 &&
       m_undecoded_image_units[0]->state == image_unit::Dropped) {
    // push a dummy 'dropped' image to the output as a placeholder

    image_unit* imgunit = image_units[0].get();

    slice_unit* sliceunit = imgunit->get_next_unprocessed_slice_segment();
    if (sliceunit) {
      remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);
    }

    imgunit->img->integrity = INTEGRITY_NOT_DECODED;

    *did_work=true;

    push_picture_to_output_queue(imgunit);

    m_undecoded_image_units.pop_front();
  }



  if ( m_undecoded_image_units.size()>=1 &&
       m_undecoded_image_units[0]->all_slice_segments_processed()) {
    loginfo(LogHighlevel,"postprocess image\n");

    image_unit* imgunit = m_undecoded_image_units[0].get();

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


    push_picture_to_output_queue(imgunit);

    // remove just decoded image unit from queue

    m_undecoded_image_units.pop_front();
  }


  if (m_end_of_stream && m_undecoded_image_units.empty()) {
    // flush all pending pictures into output queue

    loginfo(LogHighlevel,"FLUSH\n");

    // ctx->push_current_picture_to_output_queue(); // TODO: not with new queue
    m_output_queue.flush_reorder_buffer();
  }

  dpb.log_dpb_content();

  return err;
#endif
}


de265_error decoder_context::decode_slice_unit_sequential(image_unit* imgunit,
                                                          slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  /*
  printf("create tasks to decode slice POC=%d addr=%d, img=%p\n",
         sliceunit->shdr->slice_pic_order_cnt_lsb,
         sliceunit->shdr->slice_segment_address,
         imgunit->img.get());
  */

  // TODO: remove later from DPB
  //remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);

  sliceunit->allocate_thread_contexts(1);
  thread_context* tctx = sliceunit->get_thread_context(0);

  tctx->shdr = sliceunit->shdr;
  tctx->img  = imgunit->img;
  tctx->decctx = this;
  tctx->imgunit = imgunit;
  tctx->sliceunit= sliceunit;
  tctx->task = NULL;
  tctx->first_CTB_TS = sliceunit->first_CTB_TS;
  tctx->last_CTB_TS  = sliceunit->last_CTB_TS;

  if (sliceunit->shdr->slice_segment_address >= imgunit->img->get_pps().CtbAddrRStoTS.size()) {
    tctx->mark_covered_CTBs_as_processed(CTB_PROGRESS_PREFILTER);
    return DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA;
  }

  tctx->set_CTB_address_RS( tctx->shdr->slice_segment_address );

  tctx->init_quantization();

  if (sliceunit->reader.bytes_remaining <= 0) {
    tctx->mark_covered_CTBs_as_processed(CTB_PROGRESS_PREFILTER);
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

  //tctx->img->thread_start(1);
  sliceunit->nThreads++;


  auto task = std::make_shared<thread_task_slice>();
  task->tctx = tctx;
  tctx->task = task;

  m_thread_pool.add_task(task);

  tctx->imgunit->tasks.push_back(task);


  //err=read_slice_segment_data(&tctx);
  //sliceunit->finished_threads.set_progress(1);

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

    //img->decctx->add_warning(DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING, true);
  }


  // TODO: even though we cannot split this into several tasks, we should run it
  // as a background thread
  if (true) { // !use_WPP && !use_tiles) {
    //printf("SEQ\n");
    err = decode_slice_unit_sequential(imgunit, sliceunit);
    return err;
  }
  else if (use_WPP) {
    err = decode_slice_unit_WPP(imgunit, sliceunit);
    return err;
  }
  else if (use_tiles) {
    //printf("TILE\n");
    err = decode_slice_unit_tiles(imgunit, sliceunit);
    //sliceunit->state = slice_unit::Decoded;
    //sliceunit->mark_whole_slice_as_processed(CTB_PROGRESS_PREFILTER);
    return err;
  }
  else if (use_WPP && use_tiles) {
    // TODO: this is not allowed ... output some warning or error

    return DE265_WARNING_PPS_HEADER_INVALID;
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


  //assert(img->num_threads_active() == 0);


  // reserve space to store entropy coding context models for each CTB row

  if (shdr->first_slice_segment_in_pic_flag) {
    // reserve space for nRows-1 because we don't need to save the CABAC model in the last CTB row
    imgunit->ctx_models.resize( (img->get_sps().PicHeightInCtbsY-1) ); //* CONTEXT_MODEL_TABLE_LENGTH );
  }


  sliceunit->allocate_thread_contexts(nRows);


  // first CTB in this slice
  int ctbAddrRS = shdr->slice_segment_address;
  int ctbRow    = ctbAddrRS / ctbsWidth;

  std::vector<thread_task_ptr> tasks(nRows);

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
    tctx->set_CTB_address_RS(ctbAddrRS);
    tctx->first_CTB_TS = tctx->get_CTB_address_TS();

    tctx->init_quantization();


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
    //img->thread_start(1);
    sliceunit->nThreads++;

    auto task = std::make_shared<thread_task_ctb_row>();
    task->firstSliceSubstream = (entryPt==0);
    task->tctx = tctx;
    task->debug_startCtbRow = ctbRow;
    tctx->task = task;

    tasks[entryPt] = task;

    tctx->imgunit->tasks.push_back(task);
  }


  for (int entryPt=0;entryPt<nRows;entryPt++) {
    auto tctx = sliceunit->get_thread_context(entryPt);

    if (entryPt < nRows-1) {
      tctx->last_CTB_TS = sliceunit->get_thread_context(entryPt+1)->first_CTB_TS-1;
    }
    else {
      tctx->last_CTB_TS = sliceunit->last_CTB_TS;
    }
  }


  for (int entryPt=0;entryPt<nRows;entryPt++) {
    m_thread_pool.add_task(tasks[entryPt]);
  }


#if 0
  for (;;) {
    printf("q:%d r:%d b:%d f:%d\n",
           img->nThreadsQueued,
           img->nThreadsRunning,
           img->nThreadsBlocked,
           img->nThreadsFinished);

    if (img->debug_is_completed()) break;

    //usleep(1000);
  }
#endif

  //img->wait_for_completion();

  //imgunit->tasks.clear();

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


  //assert(img->num_threads_active() == 0);

  sliceunit->allocate_thread_contexts(nTiles);


  // first CTB in this slice
  int ctbAddrRS = shdr->slice_segment_address;
  int tileID = pps.TileIdRS[ctbAddrRS];

  std::vector<thread_task_ptr> tasks(nTiles);

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
    tctx->set_CTB_address_RS(ctbAddrRS);
    tctx->first_CTB_TS = tctx->get_CTB_address_TS();

    tctx->init_quantization();


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
    //img->thread_start(1);
    sliceunit->nThreads++;

    auto task = std::make_shared<thread_task_slice_segment>();
    task->firstSliceSubstream = (entryPt==0);
    task->tctx = tctx;
    task->debug_startCtbX = ctbAddrRS % ctbsWidth,
    task->debug_startCtbY = ctbAddrRS / ctbsWidth;
    tctx->task = task;

    tasks[entryPt] = task;

    m_thread_pool.add_task(task);

    tctx->imgunit->tasks.push_back(task);
  }


  for (int entryPt=0;entryPt<nTiles;entryPt++) {
    auto tctx = sliceunit->get_thread_context(entryPt);

    if (entryPt < nTiles-1) {
      tctx->last_CTB_TS = sliceunit->get_thread_context(entryPt+1)->first_CTB_TS-1;
    }
    else {
      tctx->last_CTB_TS = sliceunit->last_CTB_TS;
    }
  }


  for (int entryPt=0;entryPt<nTiles;entryPt++) {
    m_thread_pool.add_task(tasks[entryPt]);
  }


  //img->wait_for_completion();

  imgunit->tasks.clear();

  return err;
}


void decoder_context::debug_imageunit_state()
{
  m_main_loop_mutex.lock();

  loginfo(LogHighlevel,"image_units: ");
  for (int i=0;i<m_undecoded_image_units.size();i++) {
    loginfo(LogHighlevel,"%d ", m_undecoded_image_units[i]->img->get_ID());
  }
  loginfo(LogHighlevel,"\n");

  m_main_loop_mutex.unlock();
}



void decoder_context::send_image_unit(image_unit_ptr imgunit)
{
  m_main_loop_mutex.lock();

  m_undecoded_image_units.push_back(imgunit);

  m_main_loop_block_cond.signal();
  m_cond_api_action.signal();

  m_main_loop_mutex.unlock();

  debug_imageunit_state();
}


void decoder_context::send_end_of_stream()
{
  m_main_loop_mutex.lock();

  m_end_of_stream = true;

  m_main_loop_block_cond.signal();
  m_cond_api_action.signal();

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

  //img->wait_for_completion();
}


de265_error decoder_context::push_picture_to_output_queue(image_unit_ptr outimgunit)
{
  image_ptr outimg = outimgunit->img;

  if (!outimg) { return DE265_OK; }


  if (!outimgunit->slice_units.empty()) {
    remove_images_from_dpb(outimgunit->slice_units[0]->shdr->RemoveReferencesList);
  }


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
