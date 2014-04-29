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

#define DEBUG_INSERT_STREAM_ERRORS 0


#include "de265.h"
#include "decctx.h"
#include "slice_func.h"
#include "pps_func.h"
#include "sps_func.h"
#include "util.h"
#include "scan.h"
#include "image.h"
#include "sei.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>


extern void thread_decode_CTB_row(void* d);
extern void thread_decode_slice_segment(void* d);

extern "C" {
de265_error de265_decode_NAL(de265_decoder_context* de265ctx, NAL_unit* nal);
}

// TODO: should be in some vps.c related header
de265_error read_vps(decoder_context* ctx, bitreader* reader, video_parameter_set* vps);

extern "C" {
LIBDE265_API const char *de265_get_version(void)
{
    return (LIBDE265_VERSION);
}

LIBDE265_API uint32_t de265_get_version_number(void)
{
    return (LIBDE265_NUMERIC_VERSION);
}

LIBDE265_API const char* de265_get_error_text(de265_error err)
{
  switch (err) {
  case DE265_OK: return "no error";
  case DE265_ERROR_NO_SUCH_FILE: return "no such file";
    //case DE265_ERROR_NO_STARTCODE: return "no startcode found";
  case DE265_ERROR_EOF: return "end of file";
  case DE265_ERROR_COEFFICIENT_OUT_OF_IMAGE_BOUNDS: return "coefficient out of image bounds";
  case DE265_ERROR_CHECKSUM_MISMATCH: return "image checksum mismatch";
  case DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA: return "CTB outside of image area";
  case DE265_ERROR_OUT_OF_MEMORY: return "out of memory";
  case DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE: return "coded parameter out of range";
  case DE265_ERROR_IMAGE_BUFFER_FULL: return "DPB/output queue full";
  case DE265_ERROR_CANNOT_START_THREADPOOL: return "cannot start decoding threads";
  case DE265_ERROR_LIBRARY_INITIALIZATION_FAILED: return "global library initialization failed";
  case DE265_ERROR_LIBRARY_NOT_INITIALIZED: return "cannot free library data (not initialized";

  case DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED:
    return "internal error: maximum number of thread contexts exceeded";
  case DE265_ERROR_MAX_NUMBER_OF_SLICES_EXCEEDED:
    return "internal error: maximum number of slices exceeded";
    //case DE265_ERROR_SCALING_LIST_NOT_IMPLEMENTED:
    //return "scaling list not implemented";
  case DE265_ERROR_WAITING_FOR_INPUT_DATA:
    return "no more input data, decoder stalled";
  case DE265_ERROR_CANNOT_PROCESS_SEI:
    return "SEI data cannot be processed";

  case DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING:
    return "Cannot run decoder multi-threaded because stream does not support WPP";
  case DE265_WARNING_WARNING_BUFFER_FULL:
    return "Too many warnings queued";
  case DE265_WARNING_PREMATURE_END_OF_SLICE_SEGMENT:
    return "Premature end of slice segment";
  case DE265_WARNING_INCORRECT_ENTRY_POINT_OFFSET:
    return "Incorrect entry-point offset";
  case DE265_WARNING_CTB_OUTSIDE_IMAGE_AREA:
    return "CTB outside of image area (concealing stream error...)";
  case DE265_WARNING_SPS_HEADER_INVALID:
    return "sps header invalid";
  case DE265_WARNING_PPS_HEADER_INVALID:
    return "pps header invalid";
  case DE265_WARNING_SLICEHEADER_INVALID:
    return "slice header invalid";
  case DE265_WARNING_INCORRECT_MOTION_VECTOR_SCALING:
    return "impossible motion vector scaling";
  case DE265_WARNING_NONEXISTING_PPS_REFERENCED:
    return "non-existing PPS referenced";
  case DE265_WARNING_NONEXISTING_SPS_REFERENCED:
    return "non-existing SPS referenced";
  case DE265_WARNING_BOTH_PREDFLAGS_ZERO:
    return "both predFlags[] are zero in MC";
  case DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED:
    return "non-existing reference picture accessed";
  case DE265_WARNING_NUMMVP_NOT_EQUAL_TO_NUMMVQ:
    return "numMV_P != numMV_Q in deblocking";
  case DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE:
    return "number of short-term ref-pic-sets out of range";
  case DE265_WARNING_SHORT_TERM_REF_PIC_SET_OUT_OF_RANGE:
    return "short-term ref-pic-set index out of range";
  case DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST:
    return "faulty reference picture list";
  case DE265_WARNING_EOSS_BIT_NOT_SET:
    return "end_of_sub_stream_one_bit not set to 1 when it should be";
  case DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED:
    return "maximum number of reference pictures exceeded";
  case DE265_WARNING_INVALID_CHROMA_FORMAT:
    return "invalid chroma format in SPS header";
  case DE265_WARNING_SLICE_SEGMENT_ADDRESS_INVALID:
    return "slice segment address invalid";
  case DE265_WARNING_DEPENDENT_SLICE_WITH_ADDRESS_ZERO:
    return "dependent slice with address 0";
  case DE265_WARNING_NUMBER_OF_THREADS_LIMITED_TO_MAXIMUM:
    return "number of threads limited to maximum amount";
  case DE265_NON_EXISTING_LT_REFERENCE_CANDIDATE_IN_SLICE_HEADER:
    return "non-existing long-term reference candidate specified in slice header";

  default: return "unknown error";
  }
}

LIBDE265_API int de265_isOK(de265_error err)
{
  return err == DE265_OK || err >= 1000;
}



ALIGNED_8(static de265_sync_int de265_init_count) = 0;

LIBDE265_API de265_error de265_init()
{
  int cnt = de265_sync_add_and_fetch(&de265_init_count,1);
  if (cnt>1) {
    // we are not the first -> already initialized

    return DE265_OK;
  }


  // do initializations

  init_scan_orders();

  if (!alloc_and_init_significant_coeff_ctxIdx_lookupTable()) {
    de265_sync_sub_and_fetch(&de265_init_count,1);
    return DE265_ERROR_LIBRARY_INITIALIZATION_FAILED;
  }

  return DE265_OK;
}

LIBDE265_API de265_error de265_free()
{
  int cnt = de265_sync_sub_and_fetch(&de265_init_count,1);
  if (cnt<0) {
    de265_sync_add_and_fetch(&de265_init_count,1);
    return DE265_ERROR_LIBRARY_NOT_INITIALIZED;
  }

  if (cnt==0) {
    free_significant_coeff_ctxIdx_lookupTable();
  }

  return DE265_OK;
}


LIBDE265_API de265_decoder_context* de265_new_decoder()
{
  de265_error init_err = de265_init();
  if (init_err != DE265_OK) {
    return NULL;
  }

  decoder_context* ctx = (decoder_context*)calloc(sizeof(decoder_context),1);
  if (!ctx) {
    de265_free();
    return NULL;
  }

  init_decoder_context(ctx);

  return (de265_decoder_context*)ctx;
}


LIBDE265_API de265_error de265_free_decoder(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  if (ctx->num_worker_threads>0) {
    //flush_thread_pool(&ctx->thread_pool);
    stop_thread_pool(&ctx->thread_pool);
  }

  free_decoder_context(ctx);
  free(de265ctx);

  return de265_free();
}


LIBDE265_API de265_error de265_start_worker_threads(de265_decoder_context* de265ctx, int number_of_threads)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  if (number_of_threads > MAX_THREADS) {
    number_of_threads = MAX_THREADS;
  }

  ctx->num_worker_threads = number_of_threads;

  if (number_of_threads>0) {
    de265_error err = start_thread_pool(&ctx->thread_pool, number_of_threads);
    if (de265_isOK(err)) {
      err = DE265_OK;
    }
    return err;
  }
  else {
    return DE265_OK;
  }
}


void nal_insert_skipped_byte(NAL_unit* nal, int pos)
{
  if (nal->max_skipped_bytes == nal->num_skipped_bytes) {
    if (nal->max_skipped_bytes == 0) {
      nal->max_skipped_bytes = DE265_SKIPPED_BYTES_INITIAL_SIZE;
    } else {
      nal->max_skipped_bytes <<= 2;
    }

    // TODO: handle case where realloc fails
    nal->skipped_bytes = (int *)realloc(nal->skipped_bytes,
                                        nal->max_skipped_bytes * sizeof(int));
  }

  nal->skipped_bytes[nal->num_skipped_bytes] = pos;
  nal->num_skipped_bytes++;
}


#ifndef LIBDE265_DISABLE_DEPRECATED
LIBDE265_API de265_error de265_decode_data(de265_decoder_context* de265ctx,
                                           const void* data8, int len)
{
  //decoder_context* ctx = (decoder_context*)de265ctx;
  de265_error err;
  if (len > 0) {
    err = de265_push_data(de265ctx, data8, len, 0, NULL);
  } else {
    err = de265_flush_data(de265ctx);
  }
  if (err != DE265_OK) {
    return err;
  }

  int more = 0;
  do {
    err = de265_decode(de265ctx, &more);
    if (err != DE265_OK) {
        more = 0;
    }

    switch (err) {
    case DE265_ERROR_WAITING_FOR_INPUT_DATA:
      // ignore error (didn't exist in 0.4 and before)
      err = DE265_OK;
      break;
    default:
      break;
    }
  } while (more);
  return err;
}
#endif

LIBDE265_API de265_error de265_push_data(de265_decoder_context* de265ctx,
                                         const void* data8, int len,
                                         de265_PTS pts, void* user_data)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  uint8_t* data = (uint8_t*)data8;

  if (ctx->pending_input_NAL == NULL) {
    ctx->pending_input_NAL = alloc_NAL_unit(ctx, len+3, DE265_SKIPPED_BYTES_INITIAL_SIZE);
    ctx->pending_input_NAL->pts = pts;
    ctx->pending_input_NAL->user_data = user_data;
  }

  NAL_unit* nal = ctx->pending_input_NAL; // shortcut

  // Resize output buffer so that complete input would fit.
  // We add 3, because in the worst case 3 extra bytes are created for an input byte.
  rbsp_buffer_resize(&nal->nal_data, nal->nal_data.size + len + 3);

  unsigned char* out = nal->nal_data.data + nal->nal_data.size;

  for (int i=0;i<len;i++) {
    /*
    printf("state=%d input=%02x (%p) (output size: %d)\n",ctx->input_push_state, *data, data,
           out - ctx->nal_data.data);
    */

    switch (ctx->input_push_state) {
    case 0:
    case 1:
      if (*data == 0) { ctx->input_push_state++; }
      else { ctx->input_push_state=0; }
      break;
    case 2:
      if      (*data == 1) { ctx->input_push_state=3; nal->num_skipped_bytes=0; }
      else if (*data == 0) { } // *out++ = 0; }
      else { ctx->input_push_state=0; }
      break;
    case 3:
      *out++ = *data;
      ctx->input_push_state = 4;
      break;
    case 4:
      *out++ = *data;
      ctx->input_push_state = 5;
      break;

    case 5:
      if (*data==0) { ctx->input_push_state=6; }
      else { *out++ = *data; }
      break;

    case 6:
      if (*data==0) { ctx->input_push_state=7; }
      else {
        *out++ = 0;
        *out++ = *data;
        ctx->input_push_state=5;
      }
      break;

    case 7:
      if      (*data==0) { *out++ = 0; }
      else if (*data==3) {
        *out++ = 0; *out++ = 0; ctx->input_push_state=5;

        // remember which byte we removed
        nal_insert_skipped_byte(nal, (out - nal->nal_data.data) + nal->num_skipped_bytes);
      }
      else if (*data==1) {

#if DEBUG_INSERT_STREAM_ERRORS
        if ((rand()%100)<90 && ctx->nal_data.size>0) {
          int pos = rand()%ctx->nal_data.size;
          int bit = rand()%8;
          nal->nal_data.data[pos] ^= 1<<bit;

          //printf("inserted error...\n");
        }
#endif

        nal->nal_data.size = out - nal->nal_data.data;

        // push this NAL decoder queue
        push_to_NAL_queue(ctx, nal);


        // initialize new, empty NAL unit

        ctx->pending_input_NAL = alloc_NAL_unit(ctx, len+3, DE265_SKIPPED_BYTES_INITIAL_SIZE);
        ctx->pending_input_NAL->pts = pts;
        nal = ctx->pending_input_NAL;
        out = nal->nal_data.data;

        ctx->input_push_state=3;
        nal->num_skipped_bytes=0;
      }
      else {
        *out++ = 0;
        *out++ = 0;
        *out++ = *data;

        ctx->input_push_state=5;
      }
      break;
    }

    data++;
  }

  nal->nal_data.size = out - nal->nal_data.data;
  return DE265_OK;
}


void remove_stuffing_bytes(NAL_unit* nal)
{
  uint8_t* p = nal->nal_data.data;

  for (int i=0;i<nal->nal_data.size-2;i++)
    {
#if 0
        for (int k=i;k<i+64;k++) 
          if (i*0+k<nal->nal_data.size) {
            printf("%c%02x", (k==i) ? '[':' ', nal->nal_data.data[k]);
          }
        printf("\n");
#endif

      if (p[2]!=3 && p[2]!=0) {
        // fast forward 3 bytes (2+1)
        p+=2;
        i+=2;
      }
      else {
        if (p[0]==0 && p[1]==0 && p[2]==3) {
          //printf("SKIP NAL @ %d\n",i+2+nal->num_skipped_bytes);
          nal_insert_skipped_byte(nal, i+2 + nal->num_skipped_bytes);

          memmove(p+2, p+3, nal->nal_data.size-i-3);
          nal->nal_data.size--;

          p++;
          i++;
        }
      }

      p++;
    }
}


LIBDE265_API de265_error de265_push_NAL(de265_decoder_context* de265ctx,
                                        const void* data8, int len,
                                        de265_PTS pts, void* user_data)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  uint8_t* data = (uint8_t*)data8;

  // Cannot use byte-stream input and NAL input at the same time.
  assert(ctx->pending_input_NAL == NULL);

  NAL_unit* nal = alloc_NAL_unit(ctx, len, DE265_SKIPPED_BYTES_INITIAL_SIZE);
  rbsp_buffer_resize(&nal->nal_data, len);
  nal->nal_data.size = len;
  nal->pts = pts;
  nal->user_data = user_data;
  memcpy(nal->nal_data.data, data, len);

  remove_stuffing_bytes(nal);

  push_to_NAL_queue(ctx, nal);

  return DE265_OK;
}


LIBDE265_API de265_error de265_decode(de265_decoder_context* de265ctx, int* more)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  // if the stream has ended, and no more NALs are to be decoded, flush all pictures

  if (ctx->NAL_queue_len == 0 && ctx->end_of_stream) {
    if (more) { *more=0; } // 0 if no more pictures in queue

    push_current_picture_to_output_queue(ctx);

    bool images_in_reorder_buffer;
    images_in_reorder_buffer = ctx->dpb.flush_reorder_buffer();
    if (more && images_in_reorder_buffer) { *more=1; }

    return DE265_OK;
  }


  // if NAL-queue is empty, we need more data
  // -> input stalled

  if (ctx->NAL_queue_len == 0) {
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

  NAL_unit* nal = pop_from_NAL_queue(ctx);
  assert(nal);
  de265_error err = de265_decode_NAL(de265ctx, nal);
  free_NAL_unit(ctx,nal);

  if (more) {
    // decoding error is assumed to be unrecoverable
    *more = (err==DE265_OK);
  }

  return err;
}


LIBDE265_API de265_error de265_flush_data(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  if (ctx->pending_input_NAL) {
    NAL_unit* nal = ctx->pending_input_NAL;
    uint8_t null[2] = { 0,0 };

    // append bytes that are implied by the push state

    if (ctx->input_push_state==6) { rbsp_buffer_append(&nal->nal_data,null,1); }
    if (ctx->input_push_state==7) { rbsp_buffer_append(&nal->nal_data,null,2); }


    // only push the NAL if it contains at least the NAL header

    if (ctx->input_push_state>=5) {
      push_to_NAL_queue(ctx, nal);
      ctx->pending_input_NAL = NULL;
    }

    ctx->input_push_state = 0;
  }

  ctx->end_of_stream = true;

  return DE265_OK;
}


void init_thread_context(thread_context* tctx)
{
  // zero scrap memory for coefficient blocks
  memset(tctx->_coeffBuf, 0, sizeof(tctx->_coeffBuf));

  tctx->currentQG_x = -1;
  tctx->currentQG_y = -1;

  tctx->inUse = true;
}



void add_task_decode_CTB_row(decoder_context* ctx, int thread_id, bool initCABAC)
{
  thread_task task;
  task.task_id = 0; // no ID
  task.task_cmd = THREAD_TASK_DECODE_CTB_ROW;
  task.work_routine = thread_decode_CTB_row;
  task.data.task_ctb_row.ctx = ctx;
  task.data.task_ctb_row.initCABAC = initCABAC;
  task.data.task_ctb_row.thread_context_id = thread_id;
  add_task(&ctx->thread_pool, &task);
}


void add_task_decode_slice_segment(decoder_context* ctx, int thread_id)
{
  thread_task task;
  task.task_id = 0; // no ID
  task.task_cmd = THREAD_TASK_DECODE_SLICE_SEGMENT;
  task.work_routine = thread_decode_slice_segment;
  task.data.task_ctb_row.ctx = ctx;
  task.data.task_ctb_row.thread_context_id = thread_id;
  add_task(&ctx->thread_pool, &task);
}


de265_error de265_decode_NAL(de265_decoder_context* de265ctx, NAL_unit* nal)
{
  decoder_context* ctx = (decoder_context*)de265ctx;
  rbsp_buffer* data = &nal->nal_data;

  de265_error err = DE265_OK;

  bitreader reader;
  bitreader_init(&reader, data);

  nal_header nal_hdr;
  nal_read_header(&reader, &nal_hdr);
  process_nal_hdr(ctx, &nal_hdr);

  loginfo(LogHighlevel,"NAL: 0x%x 0x%x -  unit type:%s temporal id:%d\n",
          data->data[0], data->data[1],
          get_NAL_name(nal_hdr.nal_unit_type),
          nal_hdr.nuh_temporal_id);

  if (nal_hdr.nal_unit_type<32) {
    logdebug(LogHeaders,"---> read slice segment header\n");

    //printf("-------- slice header --------\n");

    int sliceIndex = get_next_slice_index(ctx);
    if (sliceIndex<0) {
      add_warning(ctx,DE265_ERROR_MAX_NUMBER_OF_SLICES_EXCEEDED, true);
      return DE265_ERROR_MAX_NUMBER_OF_SLICES_EXCEEDED;
    }

    slice_segment_header* hdr = &ctx->slice[sliceIndex];
    bool continueDecoding;
    err = read_slice_segment_header(&reader,hdr,ctx, &continueDecoding);
    if (!continueDecoding) {
      ctx->img->integrity = INTEGRITY_NOT_DECODED;
      return err;
    }
    else {
      hdr->slice_index = sliceIndex;

      if (ctx->param_slice_headers_fd>=0) {
        dump_slice_segment_header(hdr, ctx, ctx->param_slice_headers_fd);
      }

      if (process_slice_segment_header(ctx, hdr, &err, nal->pts, nal->user_data) == false)
        {
          ctx->img->integrity = INTEGRITY_NOT_DECODED;
          return err;
        }

      ctx->img->nal_hdr = nal_hdr;

      skip_bits(&reader,1); // TODO: why?
      prepare_for_CABAC(&reader);


      // modify entry_point_offsets

      int headerLength = reader.data - data->data;
      for (int i=0;i<nal->num_skipped_bytes;i++)
        {
          nal->skipped_bytes[i] -= headerLength;
        }

      for (int i=0;i<hdr->num_entry_point_offsets;i++) {
        for (int k=nal->num_skipped_bytes-1;k>=0;k--)
          if (nal->skipped_bytes[k] <= hdr->entry_point_offset[i]) {
            hdr->entry_point_offset[i] -= k+1;
            break;
          }
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
        add_warning(ctx, DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING, true);
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

        thread_context* tctx = &ctx->thread_context[thread_context_idx];

        init_thread_context(tctx);

        init_CABAC_decoder(&tctx->cabac_decoder,
                           reader.data,
                           reader.bytes_remaining);

        tctx->shdr = hdr;
        tctx->decctx = ctx;
        tctx->CtbAddrInTS = pps->CtbAddrRStoTS[hdr->slice_segment_address];

        // fixed context 0
        if ((err=read_slice_segment_data(ctx, tctx)) != DE265_OK)
          { return err; }
      }
      else if (use_tiles && !use_WPP) {
        int nTiles = nRows;  // TODO: rename 'nRows'

        if (nTiles > MAX_THREAD_CONTEXTS) {
          return DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED;
        }

        assert(nTiles == pps->num_tile_columns * pps->num_tile_rows); // TODO: handle other cases

        assert(ctx->img->tasks_pending == 0);
        increase_pending_tasks(ctx->img, nTiles);

        for (int ty=0;ty<pps->num_tile_rows;ty++)
          for (int tx=0;tx<pps->num_tile_columns;tx++) {
            int tile = tx + ty*pps->num_tile_columns;

            // set thread context

            ctx->thread_context[tile].shdr = hdr;
            ctx->thread_context[tile].decctx = ctx;

            ctx->thread_context[tile].CtbAddrInTS = pps->CtbAddrRStoTS[pps->colBd[tx] + pps->rowBd[ty]*ctbsWidth];


            // init CABAC

            int dataStartIndex;
            if (tile==0) { dataStartIndex=0; }
            else         { dataStartIndex=hdr->entry_point_offset[tile-1]; }

            int dataEnd;
            if (tile==nRows-1) dataEnd = reader.bytes_remaining;
            else               dataEnd = hdr->entry_point_offset[tile];

            init_thread_context(&ctx->thread_context[tile]);

            init_CABAC_decoder(&ctx->thread_context[tile].cabac_decoder,
                               &reader.data[dataStartIndex],
                               dataEnd-dataStartIndex);
          }

        // add tasks

        for (int i=0;i<nTiles;i++) {
          add_task_decode_slice_segment(ctx, i);
        }

        wait_for_completion(ctx->img);
      }
      else {
        if (nRows > MAX_THREAD_CONTEXTS) {
          return DE265_ERROR_MAX_THREAD_CONTEXTS_EXCEEDED;
        }

        assert(ctx->img->tasks_pending == 0);
        increase_pending_tasks(ctx->img, nRows);

        //printf("-------- decode --------\n");


        for (int y=0;y<nRows;y++) {

          // set thread context

          for (int x=0;x<ctbsWidth;x++) {
            ctx->img->ctb_info[x+y*ctbsWidth].thread_context_id = y; // TODO: shouldn't be hardcoded
          }

          ctx->thread_context[y].shdr = hdr;
          ctx->thread_context[y].decctx = ctx;
          ctx->thread_context[y].CtbAddrInTS = pps->CtbAddrRStoTS[0 + y*ctbsWidth];


          // init CABAC

          int dataStartIndex;
          if (y==0) { dataStartIndex=0; }
          else      { dataStartIndex=hdr->entry_point_offset[y-1]; }

          int dataEnd;
          if (y==nRows-1) dataEnd = reader.bytes_remaining;
          else            dataEnd = hdr->entry_point_offset[y];

          init_thread_context(&ctx->thread_context[y]);

          init_CABAC_decoder(&ctx->thread_context[y].cabac_decoder,
                             &reader.data[dataStartIndex],
                             dataEnd-dataStartIndex);
        }

        // add tasks

        for (int y=0;y<nRows;y++) {
          add_task_decode_CTB_row(ctx, y, y==0);
        }

        wait_for_completion(ctx->img);
      }
    }
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      {
        logdebug(LogHeaders,"---> read VPS\n");

        video_parameter_set vps;
        err=read_vps(ctx,&reader,&vps);
        if (err != DE265_OK) {
          break;
        }

        if (ctx->param_vps_headers_fd>=0) {
          dump_vps(&vps, ctx->param_vps_headers_fd);
        }

        process_vps(ctx, &vps);
      }
      break;

    case NAL_UNIT_SPS_NUT:
      {
        logdebug(LogHeaders,"----> read SPS\n");

        seq_parameter_set sps;
        init_sps(&sps);

        if ((err=read_sps(ctx, &reader,&sps)) != DE265_OK) {
          break;
        }

        if (ctx->param_sps_headers_fd>=0) {
          dump_sps(&sps, ctx->param_sps_headers_fd);
        }

        process_sps(ctx, &sps);
      }
      break;

    case NAL_UNIT_PPS_NUT:
      {
        logdebug(LogHeaders,"----> read PPS\n");

        pic_parameter_set pps;

        init_pps(&pps);
        bool success = read_pps(&reader,&pps,ctx);

        if (ctx->param_pps_headers_fd>=0) {
          dump_pps(&pps, ctx->param_pps_headers_fd);
        }

        if (success) {
          process_pps(ctx,&pps);
        }
      }
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      logdebug(LogHeaders,"----> read SEI\n");

      sei_message sei;

      push_current_picture_to_output_queue(ctx);

      if (read_sei(&reader,&sei, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT, ctx)) {
        dump_sei(&sei, ctx);

        err = process_sei(&sei, ctx);
      }
      break;

    case NAL_UNIT_EOS_NUT:
      ctx->FirstAfterEndOfSequenceNAL = true;
      break;
    }

  return err;
}


LIBDE265_API void de265_reset(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  int num_worker_threads = ctx->num_worker_threads;
  if (num_worker_threads>0) {
    //flush_thread_pool(&ctx->thread_pool);
    stop_thread_pool(&ctx->thread_pool);
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
  ctx->img = NULL;



  // --- decoded picture buffer ---

  ctx->current_image_poc_lsb = -1; // any invalid number
  ctx->first_decoded_picture = true;


  // --- remove all pictures from output queue ---

  // there was a bug the peek_next_image did not return NULL on empty output queues.
  // This was (indirectly) fixed by recreating the DPB buffer, but it should actually
  // be sufficient to clear it like this.
  // The error showed while scrubbing the ToS video in VLC.
  ctx->dpb.clear_images(ctx);


  // --- remove pending input data ---

  if (ctx->pending_input_NAL) {
    free_NAL_unit(ctx,ctx->pending_input_NAL);
    ctx->pending_input_NAL = NULL;
  }

  for (;;) {
    NAL_unit* nal = pop_from_NAL_queue(ctx);
    if (nal) { free_NAL_unit(ctx,nal); }
    else break;
  }

  ctx->input_push_state = 0;
  ctx->nBytes_in_NAL_queue = 0;


  // --- start threads again ---

  if (num_worker_threads>0) {
    // TODO: need error checking
    de265_start_worker_threads(de265ctx, num_worker_threads);
  }
}


LIBDE265_API const struct de265_image* de265_get_next_picture(de265_decoder_context* de265ctx)
{
  const struct de265_image* img = de265_peek_next_picture(de265ctx);
  if (img) {
    de265_release_next_picture(de265ctx);
  }

  return img;
}


LIBDE265_API const struct de265_image* de265_peek_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  if (ctx->dpb.num_pictures_in_output_queue()>0) {
    de265_image* img = ctx->dpb.get_next_picture_in_output_queue();
    return img;
  }
  else {
    return NULL;
  }
}


LIBDE265_API void de265_release_next_picture(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  // no active output picture -> ignore release request

  if (ctx->dpb.num_pictures_in_output_queue()==0) { return; }

  de265_image* next_image = ctx->dpb.get_next_picture_in_output_queue();

  loginfo(LogDPB, "release DPB with POC=%d\n",next_image->PicOrderCntVal);

  next_image->PicOutputFlag = false;
  cleanup_image(ctx, next_image);

  // pop output queue

  ctx->dpb.pop_next_picture_in_output_queue();
}


LIBDE265_API de265_error de265_get_warning(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  return get_warning(ctx);
}

LIBDE265_API void de265_set_parameter_bool(de265_decoder_context* de265ctx, enum de265_param param, int value)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  switch (param)
    {
    case DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH:
      ctx->param_sei_check_hash = !!value;
      break;

    case DE265_DECODER_PARAM_SUPPRESS_FAULTY_PICTURES:
      ctx->param_suppress_faulty_pictures = !!value;
      break;

    default:
      assert(false);
      break;
    }
}


LIBDE265_API void de265_set_parameter_int(de265_decoder_context* de265ctx, enum de265_param param, int value)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  switch (param)
    {
    case DE265_DECODER_PARAM_DUMP_SPS_HEADERS:
      ctx->param_sps_headers_fd = value;
      break;

    case DE265_DECODER_PARAM_DUMP_VPS_HEADERS:
      ctx->param_vps_headers_fd = value;
      break;

    case DE265_DECODER_PARAM_DUMP_PPS_HEADERS:
      ctx->param_pps_headers_fd = value;
      break;

    case DE265_DECODER_PARAM_DUMP_SLICE_HEADERS:
      ctx->param_slice_headers_fd = value;
      break;

    case DE265_DECODER_PARAM_ACCELERATION_CODE:
      set_acceleration_functions(ctx, (enum de265_acceleration)value);
      break;

    default:
      assert(false);
      break;
    }
}




LIBDE265_API int de265_get_parameter_bool(de265_decoder_context* de265ctx, enum de265_param param)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  switch (param)
    {
    case DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH:
      return ctx->param_sei_check_hash;

    case DE265_DECODER_PARAM_SUPPRESS_FAULTY_PICTURES:
      return ctx->param_suppress_faulty_pictures;

    default:
      assert(false);
      return false;
    }
}


LIBDE265_API int de265_get_number_of_input_bytes_pending(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  int size = ctx->nBytes_in_NAL_queue;
  if (ctx->pending_input_NAL) { size += ctx->pending_input_NAL->nal_data.size; }
  return size;
}


LIBDE265_API int de265_get_number_of_NAL_units_pending(de265_decoder_context* de265ctx)
{
  decoder_context* ctx = (decoder_context*)de265ctx;

  int size = ctx->NAL_queue_len;
  if (ctx->pending_input_NAL) { size++; }

  return size;
}


LIBDE265_API int de265_get_image_width(const struct de265_image* img,int channel)
{
  switch (channel) {
  case 0:
    return img->width_confwin;
  case 1:
  case 2:
    return img->chroma_width_confwin;
  default:
    return 0;
  }
}

LIBDE265_API int de265_get_image_height(const struct de265_image* img,int channel)
{
  switch (channel) {
  case 0:
    return img->height_confwin;
  case 1:
  case 2:
    return img->chroma_height_confwin;
  default:
    return 0;
  }
}

LIBDE265_API enum de265_chroma de265_get_chroma_format(const struct de265_image* img)
{
  return img->chroma_format;
}

LIBDE265_API const uint8_t* de265_get_image_plane(const de265_image* img, int channel, int* stride)
{
  uint8_t* data;

  switch (channel) {
  case 0: data = img->y_confwin;  if (stride) *stride = img->stride; break;
  case 1: data = img->cb_confwin; if (stride) *stride = img->chroma_stride; break;
  case 2: data = img->cr_confwin; if (stride) *stride = img->chroma_stride; break;
  default: data = NULL; if (stride) *stride = 0; break;
  }

  return data;
}

LIBDE265_API de265_PTS de265_get_image_PTS(const struct de265_image* img)
{
  return img->pts;
}

LIBDE265_API void* de265_get_image_user_data(const struct de265_image* img)
{
  return img->user_data;
}

LIBDE265_API void de265_get_image_NAL_header(const struct de265_image* img,
                                             int* nal_unit_type,
                                             const char** nal_unit_name,
                                             int* nuh_layer_id,
                                             int* nuh_temporal_id)
{
  if (nal_unit_type)   *nal_unit_type   = img->nal_hdr.nal_unit_type;
  if (nal_unit_name)   *nal_unit_name   = get_NAL_name(img->nal_hdr.nal_unit_type);
  if (nuh_layer_id)    *nuh_layer_id    = img->nal_hdr.nuh_layer_id;
  if (nuh_temporal_id) *nuh_temporal_id = img->nal_hdr.nuh_temporal_id;
}
}

