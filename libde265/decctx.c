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
#include "pps_func.h"
#include "sps_func.h"
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


void init_decoder_context(decoder_context* ctx)
{
  memset(ctx, 0, sizeof(decoder_context));

  // --- parameters ---

  ctx->param_sei_check_hash = false;
  ctx->param_HighestTid = 999; // unlimited
  ctx->param_conceal_stream_errors = true;
  ctx->param_suppress_faulty_pictures = false;

  // --- processing ---

  set_acceleration_functions(ctx,de265_acceleration_AUTO);

  ctx->param_sps_headers_fd = -1;
  ctx->param_vps_headers_fd = -1;
  ctx->param_pps_headers_fd = -1;
  ctx->param_slice_headers_fd = -1;

  // --- internal data ---

  for (int i=0;i<DE265_MAX_SPS_SETS;i++) {
    init_sps(&ctx->sps[i]);
  }

  init_dpb(&ctx->dpb);

  ctx->first_decoded_picture = true;
  //ctx->FirstAfterEndOfSequenceNAL = true;
  //ctx->last_RAP_picture_NAL_type = NAL_UNIT_UNDEFINED;

  //de265_init_image(&ctx->coeff);

  // --- decoded picture buffer ---

  ctx->current_image_poc_lsb = -1; // any invalid number

  for (int i=0;i<MAX_THREAD_CONTEXTS;i++) {
    ctx->thread_context[i].coeffBuf = (int16_t *) &ctx->thread_context[i]._coeffBuf;
    // some compilers/linkers don't align struct members correctly,
    // adjust if necessary
    int offset = (uintptr_t) ctx->thread_context[i].coeffBuf & 0x0f;
    if (offset != 0) {
        ctx->thread_context[i].coeffBuf = (int16_t *) (((uint8_t *)ctx->thread_context[i].coeffBuf) + (16-offset));
    }
  }
}


void set_acceleration_functions(decoder_context* ctx, enum de265_acceleration l)
{
  // fill scalar functions first (so that function table is completely filled)

  init_acceleration_functions_fallback(&ctx->acceleration);


  // override functions with optimized variants

#ifdef HAVE_SSE4_1
  if (l>=de265_acceleration_SSE) {
    init_acceleration_functions_sse(&ctx->acceleration);
  }
#endif
}


NAL_unit* alloc_NAL_unit(decoder_context* ctx, int size, int skipped_size)
{
  NAL_unit* nal;

  // --- get NAL-unit object ---

  if (ctx->NAL_free_list == NULL ||
      ctx->NAL_free_list_len==0) {
    nal = (NAL_unit*)calloc( sizeof(NAL_unit),1 );
    rbsp_buffer_init(&nal->nal_data);
  }
  else {
    ctx->NAL_free_list_len--;
    nal = ctx->NAL_free_list[ctx->NAL_free_list_len];
  }


  // --- allocate skipped-bytes set ---

  if (skipped_size>0 && skipped_size>nal->max_skipped_bytes) {
    nal->skipped_bytes = (int*)realloc( nal->skipped_bytes, skipped_size*sizeof(int) );
    nal->max_skipped_bytes = skipped_size;
  }

  nal->num_skipped_bytes = 0;
  nal->nal_data.size = 0;
  rbsp_buffer_resize(&nal->nal_data, size);

  return nal;
}

void      free_NAL_unit(decoder_context* ctx, NAL_unit* nal)
{
  // --- allocate free list if not already there ---

  if (ctx->NAL_free_list == NULL) {
    ctx->NAL_free_list_size = DE265_NAL_FREE_LIST_SIZE;
    ctx->NAL_free_list = (NAL_unit**)malloc( ctx->NAL_free_list_size * sizeof(NAL_unit*) );
  }


  // --- put into free-list if not full ---

  if (ctx->NAL_free_list_len < ctx->NAL_free_list_size) {
    ctx->NAL_free_list[ ctx->NAL_free_list_len ] = nal;
    ctx->NAL_free_list_len++;
  }
  else {
    rbsp_buffer_free(&nal->nal_data);
    free(nal->skipped_bytes);
    free(nal);
  }
}

NAL_unit* pop_from_NAL_queue(decoder_context* ctx)
{
  if (ctx->NAL_queue_len==0) {
    return NULL;
  }
  else {
    assert(ctx->NAL_queue != NULL);
    ctx->NAL_queue_len--;

    NAL_unit* nal = ctx->NAL_queue[0];
    memmove(ctx->NAL_queue, ctx->NAL_queue+1, sizeof(NAL_unit*)* ctx->NAL_queue_len);

    ctx->nBytes_in_NAL_queue -= nal->nal_data.size;

    return nal;
  }
}

void push_to_NAL_queue(decoder_context* ctx,NAL_unit* nal)
{
  if (ctx->NAL_queue == NULL ||
      ctx->NAL_queue_len == ctx->NAL_queue_size) {
    ctx->NAL_queue_size += 10;
    ctx->NAL_queue = (NAL_unit**)realloc(ctx->NAL_queue,
                                         sizeof(NAL_unit*) * ctx->NAL_queue_size);
  }

  ctx->NAL_queue[ ctx->NAL_queue_len ] = nal;
  ctx->NAL_queue_len++;

  ctx->nBytes_in_NAL_queue += nal->nal_data.size;
}


void free_decoder_context(decoder_context* ctx)
{
  // --- free NAL queues ---

  // empty NAL queue

  NAL_unit* nal;
  while ( (nal = pop_from_NAL_queue(ctx)) ) {
    free_NAL_unit(ctx,nal);
  }

  // free the pending input NAL

  if (ctx->pending_input_NAL != NULL) {
    free_NAL_unit(ctx, ctx->pending_input_NAL);
  }

  // free all NALs in free-list

  for (int i=0;i<ctx->NAL_free_list_len;i++)
    {
      rbsp_buffer_free(&ctx->NAL_free_list[i]->nal_data);
      free(ctx->NAL_free_list[i]->skipped_bytes);
      free(ctx->NAL_free_list[i]);
    }

  // remove lists themselves

  free(ctx->NAL_queue);
  free(ctx->NAL_free_list);


  for (int i=0;i<DE265_MAX_SPS_SETS;i++) {
    free_sps(&ctx->sps[i]);
  }

  free_dpb(&ctx->dpb);

  for (int i=0;i<DE265_MAX_PPS_SETS;i++) {
    free_pps(&ctx->pps[i]);
  }
}


void process_nal_hdr(decoder_context* ctx, nal_header* nal)
{
  ctx->nal_unit_type = nal->nal_unit_type;

  ctx->IdrPicFlag = (nal->nal_unit_type == NAL_UNIT_IDR_W_RADL ||
                     nal->nal_unit_type == NAL_UNIT_IDR_N_LP);

  ctx->RapPicFlag = (nal->nal_unit_type >= 16 &&
                     nal->nal_unit_type <= 23);
}


void process_vps(decoder_context* ctx, video_parameter_set* vps)
{
  memcpy(&ctx->vps[ vps->video_parameter_set_id ], vps, sizeof(video_parameter_set));
}


void process_sps(decoder_context* ctx, seq_parameter_set* sps)
{
  push_current_picture_to_output_queue(ctx);

  move_sps(&ctx->sps[ sps->seq_parameter_set_id ], sps);
  ctx->HighestTid = libde265_min(sps->sps_max_sub_layers-1, ctx->param_HighestTid);
}


void process_pps(decoder_context* ctx, pic_parameter_set* pps)
{
  push_current_picture_to_output_queue(ctx);

  free_pps(&ctx->pps[ (int)pps->pic_parameter_set_id ]);
  memcpy(&ctx->pps[ (int)pps->pic_parameter_set_id ], pps, sizeof(pic_parameter_set));
}


seq_parameter_set* get_sps(decoder_context* ctx, int id)
{
  if (ctx->sps[id].sps_read==false) {
    logerror(LogHeaders, "SPS %d has not been read\n", id);
    return NULL;
  }

  return &ctx->sps[id];
}


/* The returned index rotates through [0;DE265_MAX_SLICES) and is not reset at each new picture.
   Returns -1 if no more slice data structure available.
 */
int get_next_slice_index(decoder_context* ctx)
{
  for (int i=0;i<DE265_MAX_SLICES;i++) {
    if (ctx->slice[i].inUse == false) {
      return i;
    }
  }

  // TODO: make this dynamic, increase storage when completely full

  return -1;
}


/* The returned index rotates through [0;MAX_THREAD_CONTEXTS) and is not reset at each new picture.
   Returns -1 if no more context data structure available.
 */
int get_next_thread_context_index(decoder_context* ctx)
{
  for (int i=0;i<MAX_THREAD_CONTEXTS;i++) {
    if (ctx->thread_context[i].inUse == false) {
      return i;
    }
  }

  // TODO: make this dynamic, increase storage when completely full

  return -1;
}


/* 8.3.1
 */
void process_picture_order_count(decoder_context* ctx, slice_segment_header* hdr)
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

      flush_reorder_buffer(&ctx->dpb);
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
int generate_unavailable_reference_picture(decoder_context* ctx, const seq_parameter_set* sps,
                                           int POC, bool longTerm)
{
  assert(has_free_dpb_picture(&ctx->dpb, true));

  //printf("generate_unavailable_reference_picture(%d,%d)\n",POC,longTerm);

  int idx = initialize_new_DPB_image(&ctx->dpb, ctx->current_sps);
  assert(idx>=0);
  //printf("-> fill with unavailable POC %d\n",POC);

  de265_image* img = dpb_get_image(&ctx->dpb, idx);
  assert(img->border==0);

  memset( img->y - img->border, 1<<(sps->BitDepth_Y-1), img->stride * img->height );
  memset( img->cb- img->border, 1<<(sps->BitDepth_C-1), img->chroma_stride * img->chroma_height );
  memset( img->cr- img->border, 1<<(sps->BitDepth_C-1), img->chroma_stride * img->chroma_height );

  for (int i=0;i<img->cb_info_size;i++)
    { img->cb_info[i].PredMode = MODE_INTRA; }


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
void process_reference_picture_set(decoder_context* ctx, slice_segment_header* hdr)
{
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
      de265_image* img = dpb_get_image(&ctx->dpb, i);

      if (img->PicState != UnusedForReference &&
          img->PicOrderCntVal < currentPOC) {
        img->PicState = UnusedForReference;

        cleanup_image(ctx, img);
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
    const ref_pic_set* rps = hdr->CurrRps;

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
      k = DPB_index_of_picture_with_LSB(&ctx->dpb, ctx->PocLtCurr[i]);
    }
    else {
      k = DPB_index_of_picture_with_POC(&ctx->dpb, ctx->PocLtCurr[i]);
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

    if (dpb_get_image(&ctx->dpb,k)->integrity != INTEGRITY_CORRECT) {
      ctx->img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }


  for (int i=0;i<ctx->NumPocLtFoll;i++) {
    int k;
    if (!ctx->FollDeltaPocMsbPresentFlag[i]) {
      k = DPB_index_of_picture_with_LSB(&ctx->dpb, ctx->PocLtFoll[i]);
    }
    else {
      k = DPB_index_of_picture_with_POC(&ctx->dpb, ctx->PocLtFoll[i]);
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
    dpb_get_image(&ctx->dpb, ctx->RefPicSetLtCurr[i])->PicState = UsedForLongTermReference;
  }

  for (int i=0;i<ctx->NumPocLtFoll;i++) {
    dpb_get_image(&ctx->dpb, ctx->RefPicSetLtFoll[i])->PicState = UsedForLongTermReference;
  }


  // 3.

  for (int i=0;i<ctx->NumPocStCurrBefore;i++) {
    int k = DPB_index_of_picture_with_POC(&ctx->dpb, ctx->PocStCurrBefore[i]);

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

    if (dpb_get_image(&ctx->dpb,k)->integrity != INTEGRITY_CORRECT) {
      ctx->img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }

  for (int i=0;i<ctx->NumPocStCurrAfter;i++) {
    int k = DPB_index_of_picture_with_POC(&ctx->dpb, ctx->PocStCurrAfter[i]);

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

    if (dpb_get_image(&ctx->dpb,k)->integrity != INTEGRITY_CORRECT) {
      ctx->img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }

  for (int i=0;i<ctx->NumPocStFoll;i++) {
    int k = DPB_index_of_picture_with_POC(&ctx->dpb, ctx->PocStFoll[i]);
    // if (k<0) { assert(false); } // IGNORE

    ctx->RefPicSetStFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  // 4. any picture that is not marked for reference is put into the "UnusedForReference" state

  for (int i=0;i<DE265_DPB_SIZE;i++)
    if (!picInAnyList[i])        // no reference
      {
        de265_image* dpbimg = dpb_get_image(&ctx->dpb, i);
        if (dpbimg != ctx->img)  // not the current picture
          {
            if (dpbimg->PicState != UnusedForReference) {
              dpbimg->PicState = UnusedForReference;
              
              cleanup_image(ctx, dpbimg);
            }
          }
      }
}


// 8.3.3
/*
void generate_unavailable_reference_pictures(decoder_context* ctx, slice_segment_header* hdr)
{
  for (int i=0;i<ctx->NumPocStCurrBefore;i++) {
    if (ctx->RefPicSetStCurrBefore[i] < 0) {
      //int idx = generate_unavailable_picture(ctx,ctx->current_sps,
    }
  }

  for (int i=0;i<ctx->NumPocStCurrAfter;i++) {
    if (ctx->RefPicSetStCurrAfter[i] < 0) {
      //int idx = initialize_new_DPB_image(ctx, ctx->current_sps);
    }
  }
}
*/

// 8.3.4
// Returns whether we can continue decoding (or whether there is a severe error).
/* Called at beginning of each slice.

   Constructs
   - the RefPicList[2][], containing indices into the DPB, and
   - the RefPicList_POC[2][], containing POCs.
   - LongTermRefPic[2][] is also set to true if it is a long-term reference
 */
bool construct_reference_picture_lists(decoder_context* ctx, slice_segment_header* hdr)
{
  int NumPocTotalCurr = hdr->CurrRps->NumPocTotalCurr;
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
      add_warning(ctx, DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST, false);
      return false;
    }
  }

  if (hdr->num_ref_idx_l0_active > 15) {
    add_warning(ctx, DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED, false);
    return false;
  }

  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    int idx = hdr->ref_pic_list_modification_flag_l0 ? hdr->list_entry_l0[rIdx] : rIdx;

    hdr->RefPicList[0][rIdx] = RefPicListTemp0[idx];
    hdr->LongTermRefPic[0][rIdx] = isLongTerm[0][idx];

    // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
    hdr->RefPicList_POC[0][rIdx] = dpb_get_image(&ctx->dpb, hdr->RefPicList[0][rIdx])->PicOrderCntVal;
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
      hdr->RefPicList_POC[1][rIdx] = dpb_get_image(&ctx->dpb, hdr->RefPicList[1][rIdx])->PicOrderCntVal;
    }
  }


  // show reference picture lists

  loginfo(LogHeaders,"RefPicList[0] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d",
            hdr->RefPicList[0][rIdx],
            ctx->dpb[hdr->RefPicList[0][rIdx]].PicOrderCntVal
            );
  }
  loginfo(LogHeaders,"*\n");

  loginfo(LogHeaders,"RefPicList[1] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d",
            hdr->RefPicList[1][rIdx],
            ctx->dpb[hdr->RefPicList[1][rIdx]].PicOrderCntVal
            );
  }
  loginfo(LogHeaders,"*\n");

  return true;
}



void cleanup_image(decoder_context* ctx, de265_image* img)
{
  if (img->PicState != UnusedForReference) { return; } // still required for reference
  if (img->PicOutputFlag) { return; } // required for output

  if (img->sps==NULL) { return; } // might be an unavailable-reference replacement image


  //printf("cleanup_image POC=%d\n",img->PicOrderCntVal);

  // mark all slice-headers locked by this image as unused

  /* Note: cannot use SPS here, because this may already be outdated when a
     new SPS was sent before cleaning up this image.
  */

  for (int i=0;i<img->ctb_info_size;i++)
    {
      int sliceHeaderIdx = img->ctb_info[i].SliceHeaderIndex;

      slice_segment_header* shdr;
      shdr = &ctx->slice[ sliceHeaderIdx ];
      
      //printf("cleanup SHDR %d\n",sliceHeaderIdx);
      
      shdr->inUse = false;
    }

  img->sps = NULL; // this may not be valid anymore in the future
  img->pps = NULL; // this may not be valid anymore in the future
}


void writeFrame_Y(decoder_context* ctx,const char* filename)
{
  int w = ctx->img->width;
  int h = ctx->img->height;
  //int c_idx=0;
  int ctb_size = 64; // HACK

  int stride = ctx->img->stride;

  for (int ctbY=0;ctbY<ctx->current_sps->PicHeightInCtbsY;ctbY++)
    for (int ctbX=0;ctbX<ctx->current_sps->PicWidthInCtbsY;ctbX++)
      {
        int x0 = ctbX*ctb_size;
        int y0 = ctbY*ctb_size;

        
        uint8_t *src = &ctx->img->y[y0 * stride + x0];

        printf("%s %d %d\n",filename,x0,y0);
        int dx,dy;
        for (dy=0;dy<ctb_size;dy++)
          if (y0+dy < h)
            {
              printf("%s %d %d ",filename,y0+dy,x0);

              for (dx=0;dx<ctb_size;dx++)
                if (x0+dx < w)
                  {
                    printf("%02x ",*(src+dx+dy*stride));
                  }

              printf("\n");
            }
      }
}


void push_current_picture_to_output_queue(decoder_context* ctx)
{
  if (ctx->img) {
    //ctx->img->PicState = UsedForShortTermReference;

    // post-process image

#if SAVE_INTERMEDIATE_IMAGES
    char buf[1000];
    sprintf(buf,"pre-lf-%05d.yuv", ctx->img->PicOrderCntVal);
    write_picture_to_file(ctx->img, buf);
#endif

    //writeFrame_Y(ctx,"raw");
    apply_deblocking_filter(ctx);
    //writeFrame_Y(ctx,"deblk");

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"pre-sao-%05d.yuv", ctx->img->PicOrderCntVal);
    write_picture_to_file(ctx->img, buf);
#endif

    apply_sample_adaptive_offset(ctx);
    //writeFrame_Y(ctx,"sao");

#if SAVE_INTERMEDIATE_IMAGES
    sprintf(buf,"sao-%05d.yuv", ctx->img->PicOrderCntVal);
    write_picture_to_file(ctx->img, buf);
#endif

    // push image into output queue

    if (ctx->img->PicOutputFlag) {
      set_conformance_window(ctx->img,
                             ctx->current_sps->conf_win_left_offset,
                             ctx->current_sps->conf_win_right_offset,
                             ctx->current_sps->conf_win_top_offset,
                             ctx->current_sps->conf_win_bottom_offset);

      loginfo(LogDPB,"new picture has output-flag=true\n");

      if (ctx->img->integrity != INTEGRITY_CORRECT &&
          ctx->param_suppress_faulty_pictures) {
        cleanup_image(ctx, ctx->img);
      }
      else {
        assert(dpb_num_pictures_in_output_queue(&ctx->dpb) < DE265_DPB_SIZE);
        dpb_insert_image_into_reorder_buffer(&ctx->dpb, ctx->img);
      }

      loginfo(LogDPB,"push image %d into reordering queue\n", ctx->img->PicOrderCntVal);
    }

    ctx->last_decoded_image = ctx->img;
    ctx->img = NULL;

    /*
      if (isRAP(ctx->nal_unit_type)) {
      ctx->last_RAP_picture_NAL_type = ctx->nal_unit_type;

      ctx->last_RAP_was_CRA_and_first_image_of_sequence =
      isCRA(ctx->nal_unit_type) && ctx->first_decoded_picture;
      }
    */

    // next image is not the first anymore

    ctx->first_decoded_picture = false;


    // check for full reorder buffers

    int sublayer = ctx->current_vps->vps_max_sub_layers -1;

    int maxNumPicsInReorderBuffer = ctx->current_vps->layer[sublayer].vps_max_num_reorder_pics;

    /*
    printf("reorder buffer: ");
    for (int i=0;i<ctx->reorder_output_queue_length;i++) {
      printf("%d ",ctx->reorder_output_queue[i]->PicOrderCntVal);
    }
    printf("\n");
    */

    if (dpb_num_pictures_in_reorder_buffer(&ctx->dpb) > maxNumPicsInReorderBuffer) {
      flush_next_picture_from_reorder_buffer(&ctx->dpb);
    }


    log_dpb_queues(&ctx->dpb);
  }
}


// returns whether we can continue decoding the stream or whether we should give up
bool process_slice_segment_header(decoder_context* ctx, slice_segment_header* hdr,
                                  de265_error* err, de265_PTS pts, void* user_data)
{
  *err = DE265_OK;

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

  //if (hdr->slice_pic_order_cnt_lsb != ctx->current_image_poc_lsb) {
  if (hdr->first_slice_segment_in_pic_flag) {

    // previous picture has been completely decoded

    push_current_picture_to_output_queue(ctx);

    ctx->current_image_poc_lsb = hdr->slice_pic_order_cnt_lsb;


    seq_parameter_set* sps = ctx->current_sps;


    // --- find and allocate image buffer for decoding ---

    int image_buffer_idx;
    image_buffer_idx = initialize_new_DPB_image(&ctx->dpb,sps);
    if (image_buffer_idx == -1) {
      *err = DE265_ERROR_IMAGE_BUFFER_FULL;
      return false;
    }

    de265_image* img = dpb_get_image(&ctx->dpb, image_buffer_idx);
    img->pts = pts;
    img->user_data = user_data;
    ctx->img = img;

    img->sps = ctx->current_sps;
    img->pps = ctx->current_pps;

    img_clear_decoding_data(ctx->img);


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

    //generate_unavailable_reference_pictures(ctx,hdr);

    log_set_current_POC(ctx->img->PicOrderCntVal);
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
  log_dpb_content(&ctx->dpb);


  if (hdr->dependent_slice_segment_flag==0) {
    hdr->SliceAddrRS = hdr->slice_segment_address;
  } else {
    const pic_parameter_set* pps = ctx->current_pps;
    int prevCtb = pps->CtbAddrTStoRS[ pps->CtbAddrRStoTS[hdr->slice_segment_address] -1 ];

    hdr->SliceAddrRS = ctx->img->ctb_info[prevCtb].SliceAddrRS;
  }

  loginfo(LogHeaders,"SliceAddrRS = %d\n",hdr->SliceAddrRS);

  return true;
}


slice_segment_header* get_SliceHeader(decoder_context* ctx, int x, int y)
{
  return &ctx->slice[ get_SliceHeaderIndex(ctx->img, ctx->current_sps,x,y) ];
}

slice_segment_header* get_SliceHeaderCtb(decoder_context* ctx, int ctbX, int ctbY)
{
  return &ctx->slice[ ctx->img->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex ];
}


const PredVectorInfo* get_mv_info(const decoder_context* ctx,int x,int y)
{
  int log2PuSize = 2; // (ctx->current_sps->Log2MinCbSizeY-f);
  int idx = (x>>log2PuSize) + (y>>log2PuSize)*ctx->img->pb_info_stride;

  //int rootIdx = ctx->img->pb_rootIdx[idx];
  //return &ctx->img->pb_info[rootIdx].mvi;

  return &ctx->img->pb_info[idx].mvi;
}


const PredVectorInfo* get_img_mv_info(const decoder_context* ctx,
                                      const de265_image* img, int x,int y)
{
  int log2PuSize = 2; // (ctx->current_sps->Log2MinCbSizeY-f);
  int idx = (x>>log2PuSize) + (y>>log2PuSize)*ctx->img->pb_info_stride;

  //int rootIdx = img->pb_rootIdx[idx];
  //return &img->pb_info[rootIdx].mvi;

  return &img->pb_info[idx].mvi;
}


void set_mv_info(decoder_context* ctx,int x,int y, int nPbW,int nPbH, const PredVectorInfo* mv)
{
  int log2PuSize = 2; // (ctx->current_sps->Log2MinCbSizeY-f);

  int xPu = x >> log2PuSize;
  int yPu = y >> log2PuSize;
  int wPu = nPbW >> log2PuSize;
  int hPu = nPbH >> log2PuSize;

  int stride = ctx->img->pb_info_stride; // ctx->current_sps->PicWidthInMinCbsY << f;

  //int rootIdx = ctx->img->pb_info_nextRootIdx++;
  //ctx->img->pb_info[rootIdx].mvi = *mv;

  for (int pby=0;pby<hPu;pby++)
    for (int pbx=0;pbx<wPu;pbx++)
      {               
        //ctx->img->pb_rootIdx[ xPu+pbx + (yPu+pby)*stride ] = rootIdx;
        ctx->img->pb_info[ xPu+pbx + (yPu+pby)*stride ].mvi = *mv;
      }

  //printf("%dx%d -> %dx%d  size %d\n",nPbW,nPbH, wPu,hPu,sizeof(*mv));

  /*
  fprintf(stderr,"set_mv_info %d;%d [%d;%d] to %d;%d (POC=%d)\n",x,y,nPbW,nPbH,
          mv->mv[0].x,mv->mv[0].y,
          ctx->img->PicOrderCntVal);
  */
}



bool available_zscan(const de265_image* img,
                     int xCurr,int yCurr, int xN,int yN)
{
  seq_parameter_set* sps = img->sps;
  pic_parameter_set* pps = img->pps;

  if (xN<0 || yN<0) return false;
  if (xN>=sps->pic_width_in_luma_samples ||
      yN>=sps->pic_height_in_luma_samples) return false;

  int minBlockAddrN = pps->MinTbAddrZS[ (xN>>sps->Log2MinTrafoSize) +
                                        (yN>>sps->Log2MinTrafoSize) * sps->PicWidthInTbsY ];
  int minBlockAddrCurr = pps->MinTbAddrZS[ (xCurr>>sps->Log2MinTrafoSize) +
                                           (yCurr>>sps->Log2MinTrafoSize) * sps->PicWidthInTbsY ];

  if (minBlockAddrN > minBlockAddrCurr) return false;

  int xCurrCtb = xCurr >> sps->Log2CtbSizeY;
  int yCurrCtb = yCurr >> sps->Log2CtbSizeY;
  int xNCtb = xN >> sps->Log2CtbSizeY;
  int yNCtb = yN >> sps->Log2CtbSizeY;

  if (get_SliceAddrRS(img,sps, xCurrCtb,yCurrCtb) !=
      get_SliceAddrRS(img,sps, xNCtb,   yNCtb)) {
    return false;
  }

  if (pps->TileIdRS[xCurrCtb + yCurrCtb*sps->PicWidthInCtbsY] !=
      pps->TileIdRS[xNCtb    + yNCtb   *sps->PicWidthInCtbsY]) {
    return false;
  }

  return true;
}


bool available_pred_blk(const decoder_context* ctx,
                        int xC,int yC, int nCbS, int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN)
{
  logtrace(LogMotion,"C:%d;%d P:%d;%d N:%d;%d size=%d;%d\n",xC,yC,xP,yP,xN,yN,nPbW,nPbH);

  int sameCb = (xC <= xN && xN < xC+nCbS &&
                yC <= yN && yN < yC+nCbS);

  bool availableN;

  if (!sameCb) {
    availableN = available_zscan(ctx->img,xP,yP,xN,yN);
  }
  else {
    availableN = !(nPbW<<1 == nCbS && nPbH<<1 == nCbS &&
                   partIdx==1 &&
                   yN >= yC+nPbH && xN < xC+nPbW);
  }

  if (availableN && get_pred_mode(ctx->img,ctx->current_sps,xN,yN) == MODE_INTRA) {
    availableN = false;
  }

  return availableN;
}


static const char *output_filename;

LIBDE265_API void set_output_filename(const char* filename)
{
  output_filename = filename;
}

LIBDE265_API void write_picture(const de265_image* img)
{
  static FILE* fh = NULL;
  if (fh==NULL) { fh = fopen(output_filename, "wb"); }

  for (int y=0;y<de265_get_image_height(img,0);y++)
    fwrite(img->y + y*img->stride, de265_get_image_width(img,0), 1, fh);

  for (int y=0;y<de265_get_image_height(img,1);y++)
    fwrite(img->cb + y*img->chroma_stride, de265_get_image_width(img,1), 1, fh);

  for (int y=0;y<de265_get_image_height(img,2);y++)
    fwrite(img->cr + y*img->chroma_stride, de265_get_image_width(img,2), 1, fh);

  fflush(fh);
  //fclose(fh);
}


void write_picture_to_file(const de265_image* img, const char* filename)
{
  FILE* fh = fopen(filename, "wb");

  for (int y=0;y<de265_get_image_height(img,0);y++)
    fwrite(img->y + y*img->stride, de265_get_image_width(img,0), 1, fh);

  for (int y=0;y<de265_get_image_height(img,1);y++)
    fwrite(img->cb + y*img->chroma_stride, de265_get_image_width(img,1), 1, fh);

  for (int y=0;y<de265_get_image_height(img,2);y++)
    fwrite(img->cr + y*img->chroma_stride, de265_get_image_width(img,2), 1, fh);

  fflush(fh);
  fclose(fh);
}


void draw_block_boundary(const decoder_context* ctx,
                         uint8_t* img,int stride,
                         int x,int y,int hBlkSize, int vBlkSize, uint8_t value)
{
  for (int i=0;i<vBlkSize;i++)
    {
      int yi = y + i;
      
      if (yi < ctx->current_sps->pic_height_in_luma_samples) {
        img[yi*stride + x] = value;
      }
    }

  for (int i=0;i<hBlkSize;i++)
    {
      int xi = x + i;
      
      if (xi < ctx->current_sps->pic_width_in_luma_samples) {
        img[y*stride + xi] = value;
      }
    }
}


#include "intrapred.h"

void draw_intra_pred_mode(const decoder_context* ctx,
                          uint8_t* img,int stride,
                          int x0,int y0,int log2BlkSize,
                          enum IntraPredMode mode, uint8_t value)
{
  int w = 1<<log2BlkSize;

  if (mode==0) {
    // Planar -> draw square

    for (int i=-w*1/4;i<=w*1/4;i++)
      {
        img[(y0+w/2+i)*stride + x0+w*1/4] = value;
        img[(y0+w/2+i)*stride + x0+w*3/4] = value;
        img[(y0+w*1/4)*stride + x0+w/2+i] = value;
        img[(y0+w*3/4)*stride + x0+w/2+i] = value;
      }
  }
  else if (mode==1) {
    // DC -> draw circle

    for (int i=-w/4;i<w/4;i++)
      {
        int k = (sqrt((double)(w*w - i*i*16))+2)/4;

        img[(y0+w/2+k)*stride + x0+w/2+i] = value;
        img[(y0+w/2-k)*stride + x0+w/2+i] = value;
        img[(y0+w/2+i)*stride + x0+w/2+k] = value;
        img[(y0+w/2+i)*stride + x0+w/2-k] = value;
      }
  }
  else {
    // angular -> draw line in prediction direction

    int slope = intraPredAngle_table[mode];
    bool horiz = (mode<18);

    if (horiz) {
      for (int i=-w/2;i<w/2;i++)
        {
          int dy = (slope*i+Sign(slope*i)*16)/32;
          int y = y0+w/2-dy;
          if (y>=0 && y<ctx->current_sps->pic_height_in_luma_samples) {
            img[y*stride + x0+i+w/2] = value;
          }
        }
    }
    else {
      for (int i=-w/2;i<w/2;i++)
        {
          int dx = (slope*i+Sign(slope*i)*16)/32;
          int x = x0+w/2-dx;
          if (x>=0 && x<ctx->current_sps->pic_width_in_luma_samples) {
            img[(y0+i+w/2)*stride + x] = value;
          }
        }
    }
  }
}


void drawTBgrid(const decoder_context* ctx, uint8_t* img, int stride,
                int x0,int y0, uint8_t value, int log2CbSize, int trafoDepth)
{
  int split_transform_flag = get_split_transform_flag(ctx->img, ctx->current_sps,x0,y0,trafoDepth);
  if (split_transform_flag) {
    int x1 = x0 + ((1<<(log2CbSize-trafoDepth))>>1);
    int y1 = y0 + ((1<<(log2CbSize-trafoDepth))>>1);
    drawTBgrid(ctx,img,stride,x0,y0,value,log2CbSize,trafoDepth+1);
    drawTBgrid(ctx,img,stride,x1,y0,value,log2CbSize,trafoDepth+1);
    drawTBgrid(ctx,img,stride,x0,y1,value,log2CbSize,trafoDepth+1);
    drawTBgrid(ctx,img,stride,x1,y1,value,log2CbSize,trafoDepth+1);
  }
  else {
    draw_block_boundary(ctx,img,stride,x0,y0,1<<(log2CbSize-trafoDepth),1<<(log2CbSize-trafoDepth), value);
  }
}


enum DrawMode {
  Partitioning_CB,
  Partitioning_TB,
  Partitioning_PB,
  IntraPredMode,
  PBPredMode,
  PBMotionVectors
};


void tint_rect(uint8_t* img, int stride, int x0,int y0,int w,int h, uint8_t color)
{
  for (int y=0;y<h;y++)
    for (int x=0;x<w;x++)
      {
        int xp = x0+x;
        int yp = y0+y;

        img[xp+yp*stride] = (img[xp+yp*stride] + color)/2;
      }
}


void draw_PB_block(const decoder_context* ctx,uint8_t* img,int stride,
                   int x0,int y0, int w,int h, enum DrawMode what, uint8_t value)
{
  if (what == Partitioning_PB) {
    draw_block_boundary(ctx,img,stride,x0,y0,w,h, value);
  }
  else if (what == PBPredMode) {
    enum PredMode predMode = get_pred_mode(ctx->img,ctx->current_sps,x0,y0);

    uint8_t cols[3][3] = { { 255,0,0 }, { 0,0,255 }, { 0,255,0 } };

    tint_rect(img,stride, x0,y0,w,h, cols[predMode][value]);
  }
  else if (what == PBMotionVectors) {
    assert(false); // TODO
  }
}


void draw_tree_grid(const decoder_context* ctx, uint8_t* img, int stride,
                    uint8_t value, enum DrawMode what)
{
  const seq_parameter_set* sps = ctx->current_sps;
  int minCbSize = sps->MinCbSizeY;

  for (int y0=0;y0<sps->PicHeightInMinCbsY;y0++)
    for (int x0=0;x0<sps->PicWidthInMinCbsY;x0++)
      {
        int log2CbSize = get_log2CbSize_cbUnits(ctx->img,sps,x0,y0);
        if (log2CbSize==0) {
          continue;
        }

        int xb = x0*minCbSize;
        int yb = y0*minCbSize;


        if (what == Partitioning_TB) {
          drawTBgrid(ctx,img,stride,x0*minCbSize,y0*minCbSize, value, log2CbSize, 0);
        }
        else if (what == Partitioning_CB) {
          draw_block_boundary(ctx,img,stride,xb,yb, 1<<log2CbSize,1<<log2CbSize, value);
        }
        else if (what == Partitioning_PB ||
                 what == PBPredMode) {
          enum PartMode partMode = get_PartMode(ctx->img,sps,xb,yb);

          int CbSize = 1<<log2CbSize;
          int HalfCbSize = (1<<(log2CbSize-1));

          switch (partMode) {
          case PART_2Nx2N:
            draw_PB_block(ctx,img,stride,xb,yb,CbSize,CbSize, what,value);
            break;
          case PART_NxN:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize/2,CbSize/2, what,value);
            draw_PB_block(ctx,img,stride,xb+HalfCbSize,yb,           CbSize/2,CbSize/2, what,value);
            draw_PB_block(ctx,img,stride,xb           ,yb+HalfCbSize,CbSize/2,CbSize/2, what,value);
            draw_PB_block(ctx,img,stride,xb+HalfCbSize,yb+HalfCbSize,CbSize/2,CbSize/2, what,value);
            break;
          case PART_2NxN:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize  ,CbSize/2, what,value);
            draw_PB_block(ctx,img,stride,xb,           yb+HalfCbSize,CbSize  ,CbSize/2, what,value);
            break;
          case PART_Nx2N:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize/2,CbSize, what,value);
            draw_PB_block(ctx,img,stride,xb+HalfCbSize,yb,           CbSize/2,CbSize, what,value);
            break;
          case PART_2NxnU:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize  ,CbSize/4,   what,value);
            draw_PB_block(ctx,img,stride,xb,           yb+CbSize/4  ,CbSize  ,CbSize*3/4, what,value);
            break;
          case PART_2NxnD:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize  ,CbSize*3/4, what,value);
            draw_PB_block(ctx,img,stride,xb,           yb+CbSize*3/4,CbSize  ,CbSize/4,   what,value);
            break;
          case PART_nLx2N:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize/4  ,CbSize, what,value);
            draw_PB_block(ctx,img,stride,xb+CbSize/4  ,yb,           CbSize*3/4,CbSize, what,value);
            break;
          case PART_nRx2N:
            draw_PB_block(ctx,img,stride,xb,           yb,           CbSize*3/4,CbSize, what,value);
            draw_PB_block(ctx,img,stride,xb+CbSize*3/4,yb,           CbSize/4  ,CbSize, what,value);
            break;
          default:
            assert(false);
            break;
          }
        }
        else if (what==IntraPredMode) {
          enum PredMode predMode = get_pred_mode(ctx->img,sps,xb,yb);
          if (predMode == MODE_INTRA) {
            enum PartMode partMode = get_PartMode(ctx->img,sps,xb,yb);

            int HalfCbSize = (1<<(log2CbSize-1));

            switch (partMode) {
            case PART_2Nx2N:
              draw_intra_pred_mode(ctx,img,stride,xb,yb,log2CbSize,
                                   get_IntraPredMode(ctx->img,sps,xb,yb), value);
              break;
            case PART_NxN:
              draw_intra_pred_mode(ctx,img,stride,xb,           yb,           log2CbSize-1,
                                   get_IntraPredMode(ctx->img,sps,xb,yb), value);
              draw_intra_pred_mode(ctx,img,stride,xb+HalfCbSize,yb,           log2CbSize-1,
                                   get_IntraPredMode(ctx->img,sps,xb+HalfCbSize,yb), value);
              draw_intra_pred_mode(ctx,img,stride,xb           ,yb+HalfCbSize,log2CbSize-1,
                                   get_IntraPredMode(ctx->img,sps,xb,yb+HalfCbSize), value);
              draw_intra_pred_mode(ctx,img,stride,xb+HalfCbSize,yb+HalfCbSize,log2CbSize-1,
                                   get_IntraPredMode(ctx->img,sps,xb+HalfCbSize,yb+HalfCbSize), value);
              break;
            default:
              assert(false);
              break;
            }
          }
        }
      }
}


void draw_CB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, Partitioning_CB);
}

void draw_TB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, Partitioning_TB);
}

void draw_PB_grid(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, Partitioning_PB);
}

void draw_intra_pred_modes(const decoder_context* ctx, uint8_t* img, int stride, uint8_t value)
{
  draw_tree_grid(ctx,img,stride,value, IntraPredMode);
}

void draw_PB_pred_modes(const decoder_context* ctx, uint8_t* r, uint8_t* g, uint8_t* b, int stride)
{
  draw_tree_grid(ctx,r,stride,0, PBPredMode);
  draw_tree_grid(ctx,g,stride,1, PBPredMode);
  draw_tree_grid(ctx,b,stride,2, PBPredMode);
}


void add_warning(decoder_context* ctx, de265_error warning, bool once)
{
  // check if warning was already shown
  bool add=true;
  if (once) {
    for (int i=0;i<ctx->nWarningsShown;i++) {
      if (ctx->warnings_shown[i] == warning) {
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
    if (ctx->nWarningsShown < MAX_WARNINGS) {
      ctx->warnings_shown[ctx->nWarningsShown++] = warning;
    }
  }


  // add warning to output queue

  if (ctx->nWarnings == MAX_WARNINGS) {
    ctx->warnings[MAX_WARNINGS-1] = DE265_WARNING_WARNING_BUFFER_FULL;
    return;
  }

  ctx->warnings[ctx->nWarnings++] = warning;

}

de265_error get_warning(decoder_context* ctx)
{
  if (ctx->nWarnings==0) {
    return DE265_OK;
  }

  de265_error warn = ctx->warnings[0];
  ctx->nWarnings--;
  memmove(ctx->warnings, &ctx->warnings[1], ctx->nWarnings*sizeof(de265_error));

  return warn;
}
