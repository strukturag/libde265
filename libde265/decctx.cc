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

#define SAVE_INTERMEDIATE_IMAGES 0

extern void thread_decode_CTB_row(void* d);
extern void thread_decode_slice_segment(void* d);


slice_unit::~slice_unit()
{
  ctx->nal_parser.free_NAL_unit(nal);
}


decoder_context::decoder_context()
{
  //memset(ctx, 0, sizeof(decoder_context));

  // --- parameters ---

  param_sei_check_hash = false;
  param_HighestTid = 999; // unlimited
  param_conceal_stream_errors = true;
  param_suppress_faulty_pictures = false;

  // --- processing ---

  param_sps_headers_fd = -1;
  param_vps_headers_fd = -1;
  param_pps_headers_fd = -1;
  param_slice_headers_fd = -1;

  set_acceleration_functions(de265_acceleration_AUTO);

  param_image_allocation_functions = de265_image::default_image_allocation;

  /*
  memset(&vps, 0, sizeof(video_parameter_set)*DE265_MAX_VPS_SETS);
  memset(&sps, 0, sizeof(seq_parameter_set)  *DE265_MAX_SPS_SETS);
  memset(&pps, 0, sizeof(pic_parameter_set)  *DE265_MAX_PPS_SETS);
  memset(&slice,0,sizeof(slice_segment_header)*DE265_MAX_SLICES);
  */

  current_vps = NULL;
  current_sps = NULL;
  current_pps = NULL;

  memset(&thread_pool,0,sizeof(struct thread_pool));
  num_worker_threads = 0;

  HighestTid = 0;

  last_decoded_image = NULL;

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

  memset(thread_contexts,0,sizeof(struct thread_context)*MAX_THREAD_CONTEXTS);


  // --- internal data ---

  first_decoded_picture = true;
  //ctx->FirstAfterEndOfSequenceNAL = true;
  //ctx->last_RAP_picture_NAL_type = NAL_UNIT_UNDEFINED;

  //de265_init_image(&ctx->coeff);

  // --- decoded picture buffer ---

  current_image_poc_lsb = -1; // any invalid number

  for (int i=0;i<MAX_THREAD_CONTEXTS;i++) {
    thread_contexts[i].coeffBuf = (int16_t *) &thread_contexts[i]._coeffBuf;
    // some compilers/linkers don't align struct members correctly,
    // adjust if necessary
    int offset = (uintptr_t)thread_contexts[i].coeffBuf & 0x0f;
    if (offset != 0) {
      thread_contexts[i].coeffBuf = (int16_t *) (((uint8_t *)thread_contexts[i].coeffBuf) +
                                                 (16-offset));
    }
  }
}


decoder_context::~decoder_context()
{
}


de265_error decoder_context::start_thread_pool(int nThreads)
{
  ::start_thread_pool(&thread_pool, nThreads);

  num_worker_threads = nThreads;

  return DE265_OK;
}


void decoder_context::stop_thread_pool()
{
  if (get_num_worker_threads()>0) {
    //flush_thread_pool(&ctx->thread_pool);
    ::stop_thread_pool(&thread_pool);
  }
}


void decoder_context::reset()
{
  if (num_worker_threads>0) {
    //flush_thread_pool(&ctx->thread_pool);
    ::stop_thread_pool(&thread_pool);
  }

  // --------------------------------------------------

#if 0
  ctx->end_of_stream = false;
  ctx->pending_input_NAL = NULL;
  ctx->current_vps = NULL;
  ctx->current_sps = NULL;
  ctx->current_pps = NULL;
  ctx->num_worker_threads = 0;
  ctx->HighestTid = 0;
  ctx->last_decoded_image = NULL;
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

  nal_parser.remove_pending_input_data();


  // --- start threads again ---

  if (num_worker_threads>0) {
    // TODO: need error checking
    start_thread_pool(num_worker_threads);
  }
}

void decoder_context::set_acceleration_functions(enum de265_acceleration l)
{
  // fill scalar functions first (so that function table is completely filled)

  init_acceleration_functions_fallback(&acceleration);


  // override functions with optimized variants

#ifdef HAVE_SSE4_1
  if (l>=de265_acceleration_SSE) {
    init_acceleration_functions_sse(&acceleration);
  }
#endif
}


void decoder_context::init_thread_context(thread_context* tctx)
{
  // zero scrap memory for coefficient blocks
  memset(tctx->_coeffBuf, 0, sizeof(tctx->_coeffBuf));

  tctx->currentQG_x = -1;
  tctx->currentQG_y = -1;

  tctx->inUse = true;
}


void decoder_context::add_task_decode_CTB_row(int thread_id, bool initCABAC)
{
  thread_task task;
  task.task_id = 0; // no ID
  task.task_cmd = THREAD_TASK_DECODE_CTB_ROW;
  task.work_routine = thread_decode_CTB_row;
  task.data.task_ctb_row.img = img;
  task.data.task_ctb_row.initCABAC = initCABAC;
  task.data.task_ctb_row.thread_context_id = thread_id;
  add_task(&thread_pool, &task);
}


void decoder_context::add_task_decode_slice_segment(int thread_id)
{
  thread_task task;
  task.task_id = 0; // no ID
  task.task_cmd = THREAD_TASK_DECODE_SLICE_SEGMENT;
  task.work_routine = thread_decode_slice_segment;
  task.data.task_ctb_row.img = img;
  task.data.task_ctb_row.thread_context_id = thread_id;
  add_task(&thread_pool, &task);
}


de265_error decoder_context::read_vps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"---> read VPS\n");

  video_parameter_set vps;
  de265_error err = ::read_vps(this,&reader,&vps);
  if (err != DE265_OK) {
    return err;
  }

  if (param_vps_headers_fd>=0) {
    dump_vps(&vps, param_vps_headers_fd);
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
    sps.dump_sps(param_sps_headers_fd);
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
    pps.dump_pps(param_pps_headers_fd);
  }

  if (success) {
    process_pps(&pps);
  }

  return success ? DE265_OK : DE265_WARNING_PPS_HEADER_INVALID;
}

de265_error decoder_context::read_sei_NAL(bitreader& reader, bool suffix)
{
  logdebug(LogHeaders,"----> read SEI\n");

  return DE265_OK;
  assert(false); // TODO: currently broken

  sei_message sei;

  push_current_picture_to_output_queue();

  if (read_sei(&reader,&sei, suffix, current_sps)) {
    dump_sei(&sei, current_sps);

    de265_error err = process_sei(&sei, last_decoded_image);
    return err;
  }

  return DE265_OK;
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
  de265_error err = shdr->read(&reader,this, &continueDecoding);
  if (!continueDecoding) {
    if (img) { img->integrity = INTEGRITY_NOT_DECODED; }
    delete shdr;
    return err;
  }

  if (param_slice_headers_fd>=0) {
    shdr->dump_slice_segment_header(this, param_slice_headers_fd);
  }


  if (process_slice_segment_header(this, shdr, &err, nal->pts, nal->user_data) == false)
    {
      img->integrity = INTEGRITY_NOT_DECODED;
      delete shdr;
      return err;
    }

  this->img->add_slice_segment_header(shdr);
  this->img->nal_hdr = nal_hdr;

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
    //decode_slice_unit_sequential(image_units.back(), sliceunit);

    decode_some();
  }

  return DE265_OK;
}


template <class T> void pop_front(std::vector<T>& vec)
{
  for (int i=1;i<vec.size();i++)
    vec[i-1] = vec[i];

  vec.pop_back();
}


de265_error decoder_context::decode_some()
{
  de265_error err = DE265_OK;

  if (image_units.empty()) { return DE265_OK; }  // nothing to do

  if ( ! image_units.empty() && ! image_units[0]->slice_units.empty() ) {

    image_unit* imgunit = image_units[0];
    slice_unit* sliceunit = imgunit->slice_units[0];

    pop_front(imgunit->slice_units);

    if (sliceunit->flush_reorder_buffer) {
      dpb.flush_reorder_buffer();
    }

    err = decode_slice_unit_sequential(imgunit, sliceunit);
    if (err) {
      return err;
    }
  }


  if ( ( image_units.size()>=2 && image_units[0]->slice_units.empty() ) ||
       ( image_units.size()>=1 && image_units[0]->slice_units.empty() &&
         nal_parser.number_of_NAL_units_pending()==0 && nal_parser.is_end_of_stream() )) {

    image_unit* imgunit = image_units[0];

    push_picture_to_output_queue(imgunit->img);

    // remove just decoded image unit from queue

    delete imgunit;

    pop_front(image_units);
  }

  return err;
}



de265_error decoder_context::decode_image_unit_sequential(image_unit* imgunit)
{
  slice_unit* sliceunit;
  de265_error err = DE265_OK;

  printf("decode image unit sequential ----------------------------- POC = %d\n",
         imgunit->img->PicOrderCntVal);

  dpb.log_dpb_content();

  //remove_images_from_dpb(imgunit->slice_units[0]->shdr->RemoveImageList);

  printf("removed images:\n");
  dpb.log_dpb_content();

  for (int i=0;i<imgunit->slice_units.size();i++) {
    sliceunit = imgunit->slice_units[i];

    err = decode_slice_unit_sequential(imgunit, sliceunit);
    if (err) {
      break;
    }
  }


  if (err==DE265_OK) {
    push_picture_to_output_queue(imgunit->img);

    printf("output decoded image:\n");
    dpb.log_dpb_content();
  }

  return err;
}


de265_error decoder_context::decode_slice_unit_sequential(image_unit* imgunit,
                                                          slice_unit* sliceunit)
{
  de265_error err = DE265_OK;

  printf("decode slice POC=%d addr=%d, img=%p\n",
         sliceunit->shdr->slice_pic_order_cnt_lsb,
         sliceunit->shdr->slice_segment_address,
         imgunit->img);


  remove_images_from_dpb(sliceunit->shdr->RemoveReferencesList);


  int thread_context_idx=0;
  struct thread_context* tctx = &thread_contexts[thread_context_idx];

  init_thread_context(tctx);

  init_CABAC_decoder(&tctx->cabac_decoder,
                     sliceunit->reader.data,
                     sliceunit->reader.bytes_remaining);

  tctx->shdr = sliceunit->shdr;
  tctx->img  = imgunit->img;
  tctx->decctx = this;
  tctx->CtbAddrInTS = imgunit->img->pps.CtbAddrRStoTS[tctx->shdr->slice_segment_address];


  // fixed context 0
  if ((err=read_slice_segment_data(tctx)) != DE265_OK)
    { return err; }

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
  nal_read_header(&reader, &nal_hdr);
  ctx->process_nal_hdr(&nal_hdr);

  loginfo(LogHighlevel,"NAL: 0x%x 0x%x -  unit type:%s temporal id:%d\n",
          nal->data()[0], nal->data()[1],
          get_NAL_name(nal_hdr.nal_unit_type),
          nal_hdr.nuh_temporal_id);

  if (nal_hdr.nal_unit_type<32) {
    err = read_slice_NAL(reader, nal, nal_hdr);
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      err = read_vps_NAL(reader);
      break;

    case NAL_UNIT_SPS_NUT:
      err = read_sps_NAL(reader);
      break;

    case NAL_UNIT_PPS_NUT:
      err = read_pps_NAL(reader);
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      err = read_sei_NAL(reader, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT);
      break;

    case NAL_UNIT_EOS_NUT:
      ctx->FirstAfterEndOfSequenceNAL = true;
      break;
    }

  return err;
}


de265_error decoder_context::decode_NAL_OLD(NAL_unit* nal)
{
  decoder_context* ctx = this;

  de265_error err = DE265_OK;

  bitreader reader;
  bitreader_init(&reader, nal->data(), nal->size());

  nal_header nal_hdr;
  nal_read_header(&reader, &nal_hdr);
  ctx->process_nal_hdr(&nal_hdr);

  loginfo(LogHighlevel,"NAL: 0x%x 0x%x -  unit type:%s temporal id:%d\n",
          nal->data()[0], nal->data()[1],
          get_NAL_name(nal_hdr.nal_unit_type),
          nal_hdr.nuh_temporal_id);

  if (nal_hdr.nal_unit_type<32) {
    logdebug(LogHeaders,"---> read slice segment header\n");

    //printf("-------- slice header --------\n");

    slice_segment_header* hdr = new slice_segment_header;
    bool continueDecoding;
    err = hdr->read(&reader,ctx, &continueDecoding);
    if (!continueDecoding) {
      if (ctx->img) { ctx->img->integrity = INTEGRITY_NOT_DECODED; }
      delete hdr;
      return err;
    }
    else {
      if (ctx->param_slice_headers_fd>=0) {
        hdr->dump_slice_segment_header(ctx, ctx->param_slice_headers_fd);
      }

      if (ctx->process_slice_segment_header(ctx, hdr, &err, nal->pts, nal->user_data) == false)
        {
          ctx->img->integrity = INTEGRITY_NOT_DECODED;
          delete hdr;
          return err;
        }

      ctx->img->add_slice_segment_header(hdr);
      ctx->img->nal_hdr = nal_hdr;

      skip_bits(&reader,1); // TODO: why?
      prepare_for_CABAC(&reader);


      // modify entry_point_offsets

      int headerLength = reader.data - nal->data();
      for (int i=0;i<hdr->num_entry_point_offsets;i++) {
        hdr->entry_point_offset[i] -= nal->num_skipped_bytes_before(hdr->entry_point_offset[i],
                                                                    headerLength);
      }

      const pic_parameter_set* pps = ctx->current_pps;
      int ctbsWidth = ctx->current_sps->PicWidthInCtbsY;

      int nRows = hdr->num_entry_point_offsets +1;

      bool use_WPP = (ctx->num_worker_threads > 0 &&
                      ctx->current_pps->entropy_coding_sync_enabled_flag);

      bool use_tiles = (ctx->num_worker_threads > 0 &&
                        ctx->current_pps->tiles_enabled_flag);

      if (use_WPP && use_tiles) {
        //add_warning(ctx, DE265_WARNING_STREAMS_APPLIES_TILES_AND_WPP, true);
      }

      if (ctx->num_worker_threads > 0 &&
          ctx->current_pps->entropy_coding_sync_enabled_flag == false &&
          ctx->current_pps->tiles_enabled_flag == false) {

        // TODO: new error should be: no WPP and no Tiles ...
        ctx->add_warning(DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING, true);
      }

      if (!use_WPP && !use_tiles) {
        // --- single threaded decoding ---

#if 0
        int thread_context_idx = get_next_thread_context_index(ctx);
        if (thread_context_idx<0) {
          assert(false); // TODO
        }
#else
        int thread_context_idx=0;
#endif

        thread_context* tctx = &ctx->thread_contexts[thread_context_idx];

        init_thread_context(tctx);

        init_CABAC_decoder(&tctx->cabac_decoder,
                           reader.data,
                           reader.bytes_remaining);

        tctx->shdr = hdr;
        tctx->img  = ctx->img;
        tctx->decctx = ctx;
        tctx->CtbAddrInTS = pps->CtbAddrRStoTS[hdr->slice_segment_address];

        // fixed context 0
        if ((err=read_slice_segment_data(tctx)) != DE265_OK)
          { return err; }
      }
      else if (use_tiles && !use_WPP) {
        int nTiles = nRows;  // TODO: rename 'nRows'

        if (nTiles > MAX_THREAD_CONTEXTS) {
          return DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED;
        }

        assert(nTiles == pps->num_tile_columns * pps->num_tile_rows); // TODO: handle other cases

        assert(ctx->img->num_tasks_pending() == 0);
        ctx->img->increase_pending_tasks(nTiles);

        for (int ty=0;ty<pps->num_tile_rows;ty++)
          for (int tx=0;tx<pps->num_tile_columns;tx++) {
            int tile = tx + ty*pps->num_tile_columns;

            // set thread context

            ctx->thread_contexts[tile].shdr = hdr;
            ctx->thread_contexts[tile].decctx = ctx;
            ctx->thread_contexts[tile].img    = ctx->img;

            ctx->thread_contexts[tile].CtbAddrInTS = pps->CtbAddrRStoTS[pps->colBd[tx] + pps->rowBd[ty]*ctbsWidth];


            // init CABAC

            int dataStartIndex;
            if (tile==0) { dataStartIndex=0; }
            else         { dataStartIndex=hdr->entry_point_offset[tile-1]; }

            int dataEnd;
            if (tile==nRows-1) dataEnd = reader.bytes_remaining;
            else               dataEnd = hdr->entry_point_offset[tile];

            init_thread_context(&ctx->thread_contexts[tile]);

            init_CABAC_decoder(&ctx->thread_contexts[tile].cabac_decoder,
                               &reader.data[dataStartIndex],
                               dataEnd-dataStartIndex);
          }

        // add tasks

        for (int i=0;i<nTiles;i++) {
          add_task_decode_slice_segment(i);
        }

        ctx->img->wait_for_completion();
      }
      else {
        if (nRows > MAX_THREAD_CONTEXTS) {
          return DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED;
        }

        assert(ctx->img->num_tasks_pending() == 0);
        ctx->img->increase_pending_tasks(nRows);

        //printf("-------- decode --------\n");


        for (int y=0;y<nRows;y++) {

          // set thread context

          for (int x=0;x<ctbsWidth;x++) {
            ctx->img->set_ThreadContextID(x,y, y); // TODO: shouldn't be hardcoded
          }

          ctx->thread_contexts[y].shdr = hdr;
          ctx->thread_contexts[y].decctx = ctx;
          ctx->thread_contexts[y].img    = ctx->img;
          ctx->thread_contexts[y].CtbAddrInTS = pps->CtbAddrRStoTS[0 + y*ctbsWidth];


          // init CABAC

          int dataStartIndex;
          if (y==0) { dataStartIndex=0; }
          else      { dataStartIndex=hdr->entry_point_offset[y-1]; }

          int dataEnd;
          if (y==nRows-1) dataEnd = reader.bytes_remaining;
          else            dataEnd = hdr->entry_point_offset[y];

          init_thread_context(&ctx->thread_contexts[y]);

          init_CABAC_decoder(&ctx->thread_contexts[y].cabac_decoder,
                             &reader.data[dataStartIndex],
                             dataEnd-dataStartIndex);
        }

        // add tasks

        for (int y=0;y<nRows;y++) {
          add_task_decode_CTB_row(y, y==0);
        }

        ctx->img->wait_for_completion();
      }
    }
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      err = read_vps_NAL(reader);
      break;

    case NAL_UNIT_SPS_NUT:
      err = read_sps_NAL(reader);
      break;

    case NAL_UNIT_PPS_NUT:
      err = read_pps_NAL(reader);
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      err = read_sei_NAL(reader, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT);
      break;

    case NAL_UNIT_EOS_NUT:
      ctx->FirstAfterEndOfSequenceNAL = true;
      break;
    }

  return err;
}


de265_error decoder_context::decode(int* more)
{
  decoder_context* ctx = this;

  // if the stream has ended, and no more NALs are to be decoded, flush all pictures

  if (ctx->nal_parser.get_NAL_queue_length() == 0 &&
      ctx->nal_parser.is_end_of_stream() &&
      ctx->image_units.empty()) {

    // flush all pending pictures into output queue

    // ctx->push_current_picture_to_output_queue(); // TODO: not with new queue
    ctx->dpb.flush_reorder_buffer();

    if (more) { *more = ctx->dpb.num_pictures_in_output_queue(); }

    return DE265_OK;
  }


  // if NAL-queue is empty, we need more data
  // -> input stalled

  if (ctx->nal_parser.is_end_of_stream() == false &&
      ctx->nal_parser.get_NAL_queue_length() == 0) {
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

  if (ctx->nal_parser.number_of_NAL_units_pending()) {
    NAL_unit* nal = ctx->nal_parser.pop_from_NAL_queue();
    assert(nal);
    err = ctx->decode_NAL(nal);
    // ctx->nal_parser.free_NAL_unit(nal); TODO: do not free NAL with new loop
  }
  else {
    err = decode_some();
  }

  if (more) {
    // decoding error is assumed to be unrecoverable
    *more = (err==DE265_OK);
  }

  return err;
}


void decoder_context::process_nal_hdr(nal_header* nal)
{
  nal_unit_type = nal->nal_unit_type;

  IdrPicFlag = (nal->nal_unit_type == NAL_UNIT_IDR_W_RADL ||
                nal->nal_unit_type == NAL_UNIT_IDR_N_LP);

  RapPicFlag = (nal->nal_unit_type >= 16 &&
                nal->nal_unit_type <= 23);
}


void decoder_context::process_vps(video_parameter_set* vps)
{
  this->vps[ vps->video_parameter_set_id ] = *vps;
}


void decoder_context::process_sps(seq_parameter_set* sps)
{
  //push_current_picture_to_output_queue();

  this->sps[ sps->seq_parameter_set_id ] = *sps;

  HighestTid = libde265_min(sps->sps_max_sub_layers-1, param_HighestTid);
}


void decoder_context::process_pps(pic_parameter_set* pps)
{
  //push_current_picture_to_output_queue();

  this->pps[ (int)pps->pic_parameter_set_id ] = *pps;
}


/* The returned index rotates through [0;MAX_THREAD_CONTEXTS) and is not reset at each new picture.
   Returns -1 if no more context data structure available.
 */
int decoder_context::get_next_thread_context_index(decoder_context* ctx)
{
  for (int i=0;i<MAX_THREAD_CONTEXTS;i++) {
    if (ctx->thread_contexts[i].inUse == false) {
      return i;
    }
  }

  // TODO: make this dynamic, increase storage when completely full

  return -1;
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

  if (1 /* TemporalID==0 */ && // TODO
      (isReferenceNALU(ctx->nal_unit_type) &&
       (!isRASL(ctx->nal_unit_type) && !isRADL(ctx->nal_unit_type))) &&
      1 /* sub-layer non-reference picture */) // TODO
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

  int idx = ctx->dpb.new_image(ctx->current_sps, &param_image_allocation_functions);
  assert(idx>=0);
  //printf("-> fill with unavailable POC %d\n",POC);

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
  /*
  int w = sps->pic_width_in_luma_samples;
  int h = sps->pic_height_in_luma_samples;
  de265_alloc_image(ctx->img, w,h, chroma, sps);
  QQQ
  */

  return idx;
}


/* 8.3.2   invoked once per picture

   This function will mark pictures in the DPB as 'unused' or 'used for long-term reference'
 */
void decoder_context::process_reference_picture_set(decoder_context* ctx, slice_segment_header* hdr)
{
  std::vector<int> removeReferencesList;

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

    for (int i=0;i<DE265_DPB_SIZE;i++) {
      de265_image* img = ctx->dpb.get_image(i);

      if (img->PicState != UnusedForReference &&
          img->PicOrderCntVal < currentPOC) {

        removeReferencesList.push_back(img->get_ID());
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
         i<ctx->current_sps->num_long_term_ref_pics_sps + hdr->num_long_term_pics;
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

  bool picInAnyList[DE265_DPB_SIZE];
  memset(picInAnyList,0, DE265_DPB_SIZE*sizeof(bool));


  for (int i=0;i<ctx->NumPocLtCurr;i++) {
    int k;
    if (!ctx->CurrDeltaPocMsbPresentFlag[i]) {
      k = ctx->dpb.DPB_index_of_picture_with_LSB(ctx->PocLtCurr[i]);
    }
    else {
      k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocLtCurr[i]);
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
      k = ctx->dpb.DPB_index_of_picture_with_LSB(ctx->PocLtFoll[i]);
    }
    else {
      k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocLtFoll[i]);
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
    int k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocStCurrBefore[i]);

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
    int k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocStCurrAfter[i]);

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
    int k = ctx->dpb.DPB_index_of_picture_with_POC(ctx->PocStFoll[i]);
    // if (k<0) { assert(false); } // IGNORE

    ctx->RefPicSetStFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  // 4. any picture that is not marked for reference is put into the "UnusedForReference" state

  for (int i=0;i<DE265_DPB_SIZE;i++)
    if (!picInAnyList[i])        // no reference
      {
        de265_image* dpbimg = ctx->dpb.get_image(i);
        if (dpbimg != ctx->img)  // not the current picture
          {
            if (dpbimg->PicState != UnusedForReference) {

              removeReferencesList.push_back(dpbimg->get_ID());
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
 */
bool decoder_context::construct_reference_picture_lists(decoder_context* ctx, slice_segment_header* hdr)
{
  int NumPocTotalCurr = hdr->CurrRps.NumPocTotalCurr;
  int NumRpsCurrTempList0 = libde265_max(hdr->num_ref_idx_l0_active, NumPocTotalCurr);

  // TODO: fold code for both lists together

  int RefPicListTemp0[DE265_DPB_SIZE]; // TODO: what would be the correct maximum ?
  int RefPicListTemp1[DE265_DPB_SIZE]; // TODO: what would be the correct maximum ?
  char isLongTerm[2][DE265_DPB_SIZE];

  memset(isLongTerm,0,2*DE265_DPB_SIZE);

  /* --- Fill RefPicListTmp0 with reference pictures in this order:
     1) short term, past POC
     2) short term, future POC
     3) long term
  */

  int rIdx=0;
  while (rIdx < NumRpsCurrTempList0) {
    for (int i=0;i<ctx->NumPocStCurrBefore && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetStCurrBefore[i];

    for (int i=0;i<ctx->NumPocStCurrAfter && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetStCurrAfter[i];

    for (int i=0;i<ctx->NumPocLtCurr && rIdx<NumRpsCurrTempList0; rIdx++,i++) {
      RefPicListTemp0[rIdx] = ctx->RefPicSetLtCurr[i];
      isLongTerm[0][rIdx] = true;
    }

    // This check is to prevent an endless loop when no images are added above.
    if (rIdx==0) {
      ctx->add_warning(DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST, false);
      return false;
    }
  }

  if (hdr->num_ref_idx_l0_active > 15) {
    ctx->add_warning(DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED, false);
    return false;
  }

  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    int idx = hdr->ref_pic_list_modification_flag_l0 ? hdr->list_entry_l0[rIdx] : rIdx;

    hdr->RefPicList[0][rIdx] = RefPicListTemp0[idx];
    hdr->LongTermRefPic[0][rIdx] = isLongTerm[0][idx];

    // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
    hdr->RefPicList_POC[0][rIdx] = ctx->dpb.get_image(hdr->RefPicList[0][rIdx])->PicOrderCntVal;
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
      for (int i=0;i<ctx->NumPocStCurrAfter && rIdx<NumRpsCurrTempList1; rIdx++,i++)
        RefPicListTemp1[rIdx] = ctx->RefPicSetStCurrAfter[i];

      for (int i=0;i<ctx->NumPocStCurrBefore && rIdx<NumRpsCurrTempList1; rIdx++,i++)
        RefPicListTemp1[rIdx] = ctx->RefPicSetStCurrBefore[i];

      for (int i=0;i<ctx->NumPocLtCurr && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = ctx->RefPicSetLtCurr[i];
        isLongTerm[1][rIdx] = true;
      }
    }

    assert(hdr->num_ref_idx_l1_active <= 15);
    for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
      int idx = hdr->ref_pic_list_modification_flag_l1 ? hdr->list_entry_l1[rIdx] : rIdx;

      hdr->RefPicList[1][rIdx] = RefPicListTemp1[idx];
      hdr->LongTermRefPic[1][rIdx] = isLongTerm[1][idx];

      // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
      hdr->RefPicList_POC[1][rIdx] = ctx->dpb.get_image(hdr->RefPicList[1][rIdx])->PicOrderCntVal;
    }
  }


  // show reference picture lists

#if 0
  loginfo(LogHeaders,"RefPicList[0] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d",
            hdr->RefPicList[0][rIdx],
            ctx->dpb.get_image(hdr->RefPicList[0][rIdx])->PicOrderCntVal
            );
  }
  loginfo(LogHeaders,"*\n");

  loginfo(LogHeaders,"RefPicList[1] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d",
            hdr->RefPicList[1][rIdx],
            ctx->dpb.get_image(hdr->RefPicList[1][rIdx])->PicOrderCntVal
            );
  }
  loginfo(LogHeaders,"*\n");
#endif

  return true;
}



void run_postprocessing_filters(de265_image* img)
{
#if SAVE_INTERMEDIATE_IMAGES
    char buf[1000];
    sprintf(buf,"pre-lf-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif

    apply_deblocking_filter(img);

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"pre-sao-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif

    apply_sample_adaptive_offset(img);

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"sao-%05d.yuv", img->PicOrderCntVal);
    write_picture_to_file(img, buf);
#endif
}

void decoder_context::push_current_picture_to_output_queue()
{
  push_picture_to_output_queue(img);
}

void decoder_context::push_picture_to_output_queue(de265_image* outimg)
{
  if (outimg==NULL) { return; }


  // run post-processing filters (deblocking & SAO)

  run_postprocessing_filters(outimg);


  // push image into output queue

  if (outimg->PicOutputFlag) {
    loginfo(LogDPB,"new picture has output-flag=true\n");

    if (outimg->integrity != INTEGRITY_CORRECT &&
        param_suppress_faulty_pictures) {
    }
    else {
      assert(dpb.num_pictures_in_output_queue() < DE265_DPB_SIZE);
      dpb.insert_image_into_reorder_buffer(outimg);
    }

    loginfo(LogDPB,"push image %d into reordering queue\n", outimg->PicOrderCntVal);
  }

  // last_decoded_image = outimg;    TODO: not with new loop
  // this->img = NULL;               TODO: not with new loop

  // check for full reorder buffers

  int sublayer = outimg->vps.vps_max_sub_layers -1;
  int maxNumPicsInReorderBuffer = outimg->vps.layer[sublayer].vps_max_num_reorder_pics;

  if (dpb.num_pictures_in_reorder_buffer() > maxNumPicsInReorderBuffer) {
    dpb.output_next_picture_in_reorder_buffer();
  }

  dpb.log_dpb_queues();
}


// returns whether we can continue decoding the stream or whether we should give up
bool decoder_context::process_slice_segment_header(decoder_context* ctx, slice_segment_header* hdr,
                                                   de265_error* err, de265_PTS pts, void* user_data)
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
  ctx->current_vps = &ctx->vps[ (int)ctx->current_sps->video_parameter_set_id ];

  
  // --- prepare decoding of new picture ---

  if (hdr->first_slice_segment_in_pic_flag) {

    // previous picture has been completely decoded

    //ctx->push_current_picture_to_output_queue();

    ctx->current_image_poc_lsb = hdr->slice_pic_order_cnt_lsb;


    seq_parameter_set* sps = ctx->current_sps;


    // --- find and allocate image buffer for decoding ---

    int image_buffer_idx;
    image_buffer_idx = ctx->dpb.new_image(sps, &param_image_allocation_functions);
    if (image_buffer_idx == -1) {
      *err = DE265_ERROR_IMAGE_BUFFER_FULL;
      return false;
    }

    de265_image* img = ctx->dpb.get_image(image_buffer_idx);
    img->pts = pts;
    img->user_data = user_data;
    ctx->img = img;

    img->vps = *ctx->current_vps;
    img->sps = *ctx->current_sps;
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
      // mark picture so that it is not overwritten by unavailable reference frames
      img->PicState = UsedForShortTermReference;

      process_reference_picture_set(ctx,hdr);
    }

    img->PicState = UsedForShortTermReference;

    log_set_current_POC(ctx->img->PicOrderCntVal);


    // next image is not the first anymore

    first_decoded_picture = false;
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
    const pic_parameter_set* pps = &img->pps;
    int prevCtb = pps->CtbAddrTStoRS[ pps->CtbAddrRStoTS[hdr->slice_segment_address] -1 ];

    //hdr->SliceAddrRS = ctx->img->get_SliceAddrRS_atCtbRS(prevCtb);
    hdr->SliceAddrRS = ctx->prevSliceAddrRS;
  }

  ctx->prevSliceAddrRS = hdr->SliceAddrRS;


  loginfo(LogHeaders,"SliceAddrRS = %d\n",hdr->SliceAddrRS);

  return true;
}


void decoder_context::remove_images_from_dpb(const std::vector<int>& removeImageList)
{
  for (int i=0;i<removeImageList.size();i++) {
    printf("remove image with ID : %d\n",removeImageList[i]);

    int idx = dpb.DPB_index_of_picture_with_ID( removeImageList[i] );
    if (idx>=0) {
      printf("idx = %d\n",idx);
      de265_image* dpbimg = dpb.get_image( idx );
      dpbimg->PicState = UnusedForReference;
    }
  }
}




void error_queue::add_warning(de265_error warning, bool once)
{
  printf("---------------------------------------------------------------- WARN: %d\n",warning);
  exit(0);

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
