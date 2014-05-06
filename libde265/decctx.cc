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

  memset(thread_context,0,sizeof(struct thread_context)*MAX_THREAD_CONTEXTS);


  // --- internal data ---

  first_decoded_picture = true;
  //ctx->FirstAfterEndOfSequenceNAL = true;
  //ctx->last_RAP_picture_NAL_type = NAL_UNIT_UNDEFINED;

  //de265_init_image(&ctx->coeff);

  // --- decoded picture buffer ---

  current_image_poc_lsb = -1; // any invalid number

  for (int i=0;i<MAX_THREAD_CONTEXTS;i++) {
    thread_context[i].coeffBuf = (int16_t *) &thread_context[i]._coeffBuf;
    // some compilers/linkers don't align struct members correctly,
    // adjust if necessary
    int offset = (uintptr_t)thread_context[i].coeffBuf & 0x0f;
    if (offset != 0) {
      thread_context[i].coeffBuf = (int16_t *) (((uint8_t *)thread_context[i].coeffBuf) +
                                                (16-offset));
    }
  }
}


decoder_context::~decoder_context()
{
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

  ctx->sps[ sps->seq_parameter_set_id ] = *sps;

  ctx->HighestTid = libde265_min(sps->sps_max_sub_layers-1, ctx->param_HighestTid);
}


void process_pps(decoder_context* ctx, pic_parameter_set* pps)
{
  push_current_picture_to_output_queue(ctx);

  ctx->pps[ (int)pps->pic_parameter_set_id ] = *pps;
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
#if 0
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
#endif


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

      ctx->dpb.flush_reorder_buffer();
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
  assert(ctx->dpb.has_free_dpb_picture(true));

  int idx = ctx->dpb.new_image(ctx->current_sps);
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
      de265_image* img = ctx->dpb.get_image(i);

      if (img->PicState != UnusedForReference &&
          img->PicOrderCntVal < currentPOC) {
        img->PicState = UnusedForReference;

        if (img->PicOutputFlag==false) {
          cleanup_image(ctx, img);
        }
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
              dpbimg->PicState = UnusedForReference;

              if (dpbimg->PicOutputFlag==false) {
                cleanup_image(ctx, dpbimg);
              }
            }
          }
      }
}


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

  loginfo(LogHeaders,"RefPicList[0] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d",
            hdr->RefPicList[0][rIdx],
            ctx->dpb.dpb[hdr->RefPicList[0][rIdx]].PicOrderCntVal
            );
  }
  loginfo(LogHeaders,"*\n");

  loginfo(LogHeaders,"RefPicList[1] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
    loginfo(LogHeaders,"* [%d]=%d",
            hdr->RefPicList[1][rIdx],
            ctx->dpb.dpb[hdr->RefPicList[1][rIdx]].PicOrderCntVal
            );
  }
  loginfo(LogHeaders,"*\n");

  return true;
}



void cleanup_image(decoder_context* ctx, de265_image* img)
{
  if (img->PicState != UnusedForReference) { return; } // still required for reference
  if (img->PicOutputFlag) { return; } // required for output


  //printf("cleanup_image POC=%d  (%p) from %s\n",img->PicOrderCntVal,img,why);

  // mark all slice-headers locked by this image as unused

  /* Note: cannot use SPS here, because this may already be outdated when a
     new SPS was sent before cleaning up this image.
  */
  img->mark_slice_headers_as_unused(ctx);
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

void push_current_picture_to_output_queue(decoder_context* ctx)
{
  if (ctx->img) {

    // run post-processing filters (deblocking & SAO)

    run_postprocessing_filters(ctx->img);


    // push image into output queue

    if (ctx->img->PicOutputFlag) {
      ctx->img->set_conformance_window();

      loginfo(LogDPB,"new picture has output-flag=true\n");

      if (ctx->img->integrity != INTEGRITY_CORRECT &&
          ctx->param_suppress_faulty_pictures) {
        cleanup_image(ctx, ctx->img);
      }
      else {
        assert(ctx->dpb.num_pictures_in_output_queue() < DE265_DPB_SIZE);
        ctx->dpb.insert_image_into_reorder_buffer(ctx->img);
      }

      loginfo(LogDPB,"push image %d into reordering queue\n", ctx->img->PicOrderCntVal);
    }

    ctx->last_decoded_image = ctx->img;
    ctx->img = NULL;

    // next image is not the first anymore

    ctx->first_decoded_picture = false;


    // check for full reorder buffers

    int sublayer = ctx->current_vps->vps_max_sub_layers -1;
    int maxNumPicsInReorderBuffer = ctx->current_vps->layer[sublayer].vps_max_num_reorder_pics;

    if (ctx->dpb.num_pictures_in_reorder_buffer() > maxNumPicsInReorderBuffer) {
      ctx->dpb.output_next_picture_in_reorder_buffer();
    }


    ctx->dpb.log_dpb_queues();
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

  if (hdr->first_slice_segment_in_pic_flag) {

    // previous picture has been completely decoded

    push_current_picture_to_output_queue(ctx);

    ctx->current_image_poc_lsb = hdr->slice_pic_order_cnt_lsb;


    seq_parameter_set* sps = ctx->current_sps;


    // --- find and allocate image buffer for decoding ---

    int image_buffer_idx;
    image_buffer_idx = ctx->dpb.new_image(sps);
    if (image_buffer_idx == -1) {
      *err = DE265_ERROR_IMAGE_BUFFER_FULL;
      return false;
    }

    de265_image* img = ctx->dpb.get_image(image_buffer_idx);
    img->pts = pts;
    img->user_data = user_data;
    ctx->img = img;

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
    const pic_parameter_set* pps = ctx->current_pps;
    int prevCtb = pps->CtbAddrTStoRS[ pps->CtbAddrRStoTS[hdr->slice_segment_address] -1 ];

    hdr->SliceAddrRS = ctx->img->get_SliceAddrRS_atCtbRS(prevCtb);
  }

  loginfo(LogHeaders,"SliceAddrRS = %d\n",hdr->SliceAddrRS);

  return true;
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
