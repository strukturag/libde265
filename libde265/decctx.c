/*
 * H.265 video codec.
 * Copyright (c) 2013 StrukturAG, Dirk Farin, <farin@struktur.de>
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
#include "sao.h"
#include "sei.h"
#include "deblock.h"

#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


void init_decoder_context(decoder_context* ctx)
{
  memset(ctx, 0, sizeof(decoder_context));

  // --- parameters ---

  ctx->param_sei_check_hash = true;
  ctx->param_HighestTid = 999; // unlimited

  // --- processing ---

  ctx->num_worker_threads = 0; // default: no background threads

  // --- internal data ---

  rbsp_buffer_init(&ctx->pending_input_data);
  ctx->end_of_stream=false;

  ctx->skipped_bytes = NULL;
  ctx->num_skipped_bytes = 0;

  rbsp_buffer_init(&ctx->nal_data);
  ctx->input_push_state = 0;

  ctx->ref_pic_sets = NULL;

  for (int i=0;i<DE265_DPB_SIZE;i++) {
    de265_init_image(&ctx->dpb[i]);
  }

  ctx->img = NULL;
  ctx->last_decoded_image = NULL;
  ctx->image_output_queue_length = 0;
  ctx->reorder_output_queue_length = 0;
  ctx->first_decoded_picture = true;
  ctx->PicOrderCntMsb = 0;
  //ctx->last_RAP_picture_NAL_type = NAL_UNIT_UNDEFINED;

  de265_init_image(&ctx->coeff);

  for (int i=0;i<DE265_DPB_SIZE;i++) {
    ctx->image_output_queue[i] = NULL;
  }

  // --- decoded picture buffer ---

  ctx->current_image_poc_lsb = -1; // any invalid number
}


void reset_decoder_context_for_new_picture(decoder_context* ctx)
{
  memset(ctx->ctb_info, 0,sizeof(CTB_info) * ctx->ctb_info_size);
  memset(ctx->cb_info,  0,sizeof(CB_info)  * ctx->cb_info_size);
  memset(ctx->tu_info,  0,sizeof(TU_info)  * ctx->tu_info_size);
  memset(ctx->pb_info,  0,sizeof(PB_info)  * ctx->pb_info_size);
  memset(ctx->deblk_info,  0,sizeof(deblock_info)  * ctx->deblk_info_size);

  // HACK de265_fill_image(&ctx->coeff, 0,0,0);

  ctx->next_free_slice_index = 0;
}

void prepare_new_picture(decoder_context* ctx)
{
  // initialize threading tasks (TODO: move this to picture initialization)

  int w = ctx->current_sps->PicWidthInCtbsY;
  int h = ctx->current_sps->PicHeightInCtbsY;

  for (int y=0;y<h;y++)
    for (int x=0;x<w;x++)
      {
        int cnt=2;
        if (y==0 || x==0) cnt--;
        set_CTB_deblocking_cnt(ctx,x,y, cnt);
      }

  ctx->thread_pool.tasks_pending = w*h;
}


void free_info_arrays(decoder_context* ctx)
{
  if (ctx->ctb_info) { free(ctx->ctb_info); ctx->ctb_info=NULL; }
  if (ctx->cb_info)  { free(ctx->cb_info);  ctx->cb_info =NULL; }
  if (ctx->tu_info)  { free(ctx->tu_info);  ctx->tu_info =NULL; }
  if (ctx->pb_info)  { free(ctx->pb_info);  ctx->pb_info =NULL; }
  if (ctx->deblk_info)  { free(ctx->deblk_info);  ctx->deblk_info =NULL; }
}


de265_error allocate_info_arrays(decoder_context* ctx)
{
  seq_parameter_set* sps = ctx->current_sps;

  int deblk_w = (ctx->current_sps->pic_width_in_luma_samples+3)/4;
  int deblk_h = (ctx->current_sps->pic_height_in_luma_samples+3)/4;

  if (ctx->ctb_info_size != sps->PicSizeInCtbsY ||
      ctx->cb_info_size  != sps->PicSizeInMinCbsY ||
      ctx->tu_info_size  != sps->PicSizeInTbsY ||
      ctx->deblk_info_size != deblk_w*deblk_h)
    {
      free_info_arrays(ctx);

      ctx->ctb_info_size  = sps->PicSizeInCtbsY;
      ctx->cb_info_size   = sps->PicSizeInMinCbsY;
      ctx->pb_info_size   = sps->PicSizeInMinCbsY*4*4;
      ctx->tu_info_size   = sps->PicSizeInTbsY;
      ctx->deblk_info_size= deblk_w*deblk_h;
      ctx->deblk_width    = deblk_w;
      ctx->deblk_height   = deblk_h;

      // TODO: CHECK: *1 was *2 previously, but I guess this was only for debugging...
      ctx->ctb_info   = (CTB_info *)malloc( sizeof(CTB_info)   * ctx->ctb_info_size *1);
      ctx->cb_info    = (CB_info  *)malloc( sizeof(CB_info)    * ctx->cb_info_size  *1);
      ctx->pb_info    = (PB_info  *)malloc( sizeof(PB_info)    * ctx->pb_info_size  *1);
      ctx->tu_info    = (TU_info  *)malloc( sizeof(TU_info)    * ctx->tu_info_size  *1);
      ctx->deblk_info = (deblock_info*)malloc( sizeof(deblock_info) * ctx->deblk_info_size);

      if (ctx->ctb_info==NULL || ctx->cb_info==NULL || ctx->tu_info==NULL || ctx->deblk_info==NULL) {
	free_info_arrays(ctx);
	return DE265_ERROR_OUT_OF_MEMORY;
      }
    }

  return DE265_OK;
}


void free_decoder_context(decoder_context* ctx)
{
  rbsp_buffer_free(&ctx->nal_data);
  if (ctx->skipped_bytes) free(ctx->skipped_bytes);

  free_ref_pic_sets(&ctx->ref_pic_sets);

  for (int i=0;i<DE265_DPB_SIZE;i++) {
    de265_free_image(&ctx->dpb[i]);
  }
  //de265_free_image(&ctx->img);
  // HACK de265_free_image(&ctx->coeff);

  free_info_arrays(ctx);
  //free_image(&ctx->intra_pred_available);

  free_info_arrays(ctx);

  //video_parameter_set vps[ MAX_VPS_SETS ];
  //seq_parameter_set   sps[ MAX_SPS_SETS ];

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
  memcpy(&ctx->sps[ sps->seq_parameter_set_id ], sps, sizeof(seq_parameter_set));


  ctx->HighestTid = min(sps->sps_max_sub_layers-1, ctx->param_HighestTid);
}


void process_pps(decoder_context* ctx, pic_parameter_set* pps)
{
  memcpy(&ctx->pps[ (int)pps->pic_parameter_set_id ], pps, sizeof(pic_parameter_set));
}


seq_parameter_set* get_sps(decoder_context* ctx, int id)
{
  if (ctx->sps[id].sps_read==false) {
    logerror(LogHeaders, "SPS %d has not been read\n", id);
    assert(false); // TODO
    return NULL;
  }

  return &ctx->sps[id];
}


/* The returned index rotates through [0;DE265_MAX_SLICES) and is not reset at each new picture.
 */
int get_next_slice_index(decoder_context* ctx)
{
  assert(ctx->next_free_slice_index < DE265_MAX_SLICES);

  int sliceID = ctx->next_free_slice_index;

  ctx->next_free_slice_index = ((ctx->next_free_slice_index+1) % DE265_MAX_SLICES);

  return sliceID;
}


static void log_dpb_content(const decoder_context* ctx)
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    loginfo(LogHighlevel, " DPB %d: POC=%d %s %s\n", i, ctx->dpb[i].PicOrderCntVal,
            ctx->dpb[i].PicState == UnusedForReference ? "unused" :
            ctx->dpb[i].PicState == UsedForShortTermReference ? "short-term" : "long-term",
            ctx->dpb[i].PicOutputFlag ? "output" : "---");
  }
}


/* 8.3.1
 */
void process_picture_order_count(decoder_context* ctx, slice_segment_header* hdr)
{
  logdebug(LogHeaders,"POC computation. lsb:%d prev.pic.lsb:%d msb:%d\n",
           hdr->slice_pic_order_cnt_lsb,
           ctx->prevPicOrderCntLsb,
           ctx->PicOrderCntMsb);

  if (isIRAP(ctx->nal_unit_type) &&
      ctx->NoRaslOutputFlag)
    {
      ctx->PicOrderCntMsb=0;


      // flush all images from reorder buffer

      while (ctx->reorder_output_queue_length>0) {
        flush_next_picture_from_reorder_buffer(ctx);
      }
    }
  else
    {
      int MaxPicOrderCntLsb = ctx->current_sps->MaxPicOrderCntLsb;

      if ((hdr->slice_pic_order_cnt_lsb < ctx->prevPicOrderCntLsb) &&
          (ctx->prevPicOrderCntLsb - hdr->slice_pic_order_cnt_lsb) > MaxPicOrderCntLsb/2) {
        ctx->PicOrderCntMsb = ctx->prevPicOrderCntMsb + MaxPicOrderCntLsb;
      }
      else if ((hdr->slice_pic_order_cnt_lsb > ctx->prevPicOrderCntLsb) &&
               (hdr->slice_pic_order_cnt_lsb - ctx->prevPicOrderCntLsb) > MaxPicOrderCntLsb/2) {
        ctx->PicOrderCntMsb = ctx->prevPicOrderCntMsb - MaxPicOrderCntLsb;
      }
      else {
        // leave PicOrderCntMsb unchanged
      }
    }

  ctx->img->PicOrderCntVal = ctx->PicOrderCntMsb + hdr->slice_pic_order_cnt_lsb;

  logdebug(LogHeaders,"POC computation. new msb:%d POC=%d\n",
           ctx->PicOrderCntMsb,
           ctx->img->PicOrderCntVal);

  //  if (1 /* TemporalID==0 */ && // TODO
  //    !isRASL(ctx->nal_unit_type) &&
  //    !isRADL(ctx->nal_unit_type) &&
  //    1 /* sub-layer non-reference picture */) // TODO
    {
      ctx->prevPicOrderCntLsb = hdr->slice_pic_order_cnt_lsb;
      ctx->prevPicOrderCntMsb = ctx->PicOrderCntMsb;
    }
}


bool has_free_dpb_picture(const decoder_context* ctx)
{
  for (int i=0;i<DE265_DPB_SIZE;i++) {
    if (ctx->dpb[i].PicOutputFlag==false && ctx->dpb[i].PicState == UnusedForReference) {
      return true;
    }
  }

  return false;
}


static int DPB_index_of_st_ref_picture(decoder_context* ctx, int poc)
{
  logdebug(LogHeaders,"get access to POC %d from DPB\n",poc);

  //log_dpb_content(ctx);
  //loginfo(LogDPB,"searching for short-term reference POC=%d\n",poc);

  for (int k=0;k<DE265_DPB_SIZE;k++) {
    if (ctx->dpb[k].PicOrderCntVal == poc &&
        ctx->dpb[k].PicState == UsedForShortTermReference) {
      return k;
    }
  }

  assert(false);

  return -1;
}


/* 8.3.2
 */
void process_reference_picture_set(decoder_context* ctx, slice_segment_header* hdr)
{
  if (isRASL(ctx->nal_unit_type) && ctx->NoRaslOutputFlag) {
    // reset DPB

    for (int i=0;i<DE265_DPB_SIZE;i++) {
      ctx->dpb[i].PicState = UnusedForReference;
    }
  }


  if (isIDR(ctx->nal_unit_type)) {
    ctx->NumPocStCurrBefore = 0;
    ctx->NumPocStCurrAfter = 0;
    ctx->NumPocStFoll = 0;
    ctx->NumPocLtCurr = 0;
    ctx->NumPocLtFoll = 0;
  }
  else {
    const ref_pic_set* rps = &ctx->ref_pic_sets[hdr->CurrRpsIdx];

    // (8-98)

    int i,j,k;

    for (i=0, j=0, k=0;
         i<rps->NumNegativePics;
         i++)
      {
        if (rps->UsedByCurrPicS0[i]) {
          ctx->PocStCurrBefore[j++] = ctx->img->PicOrderCntVal + rps->DeltaPocS0[i];
        }
        else {
          ctx->PocStFoll[k++] = ctx->img->PicOrderCntVal + rps->DeltaPocS0[i];
        }
      }

    ctx->NumPocStCurrBefore = j;

    for (i=0, j=0;
         i<rps->NumPositivePics;
         i++)
      {
        if (rps->UsedByCurrPicS1[i]) {
          ctx->PocStCurrAfter[j++] = ctx->img->PicOrderCntVal + rps->DeltaPocS1[i];
        }
        else {
          ctx->PocStFoll[k++] = ctx->img->PicOrderCntVal + rps->DeltaPocS1[i];
        }
      }

    ctx->NumPocStCurrAfter = j;
    ctx->NumPocStFoll = k;

    for (i=0, j=0, k=0;
         i<ctx->current_sps->num_long_term_ref_pics_sps + hdr->num_long_term_pics;
         i++)
      {
        //int pocLt = 
        assert(false);
      }

    ctx->NumPocLtCurr = j;
    ctx->NumPocLtFoll = k;
  }


  // (8-99)
  // 1.

  for (int i=0;i<ctx->NumPocLtCurr;i++) {
    assert(false); // TODO
  }

  for (int i=0;i<ctx->NumPocLtFoll;i++) {
    assert(false); // TODO
  }


  bool picInAnyList[DE265_DPB_SIZE];
  memset(picInAnyList,0, DE265_DPB_SIZE*sizeof(bool));

  // TODO: 2.

  // 3.

  for (int i=0;i<ctx->NumPocStCurrBefore;i++) {
    int k = DPB_index_of_st_ref_picture(ctx, ctx->PocStCurrBefore[i]);

    ctx->RefPicSetStCurrBefore[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  for (int i=0;i<ctx->NumPocStCurrAfter;i++) {
    int k = DPB_index_of_st_ref_picture(ctx, ctx->PocStCurrAfter[i]);

    ctx->RefPicSetStCurrAfter[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  for (int i=0;i<ctx->NumPocStFoll;i++) {
    int k = DPB_index_of_st_ref_picture(ctx, ctx->PocStFoll[i]);

    ctx->RefPicSetStFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  // 4.

  for (int i=0;i<DE265_DPB_SIZE;i++)
    if (!picInAnyList[i]) {
      ctx->dpb[i].PicState = UnusedForReference;
    }
}


// 8.3.3
void generate_unavailable_reference_pictures(decoder_context* ctx, slice_segment_header* hdr)
{
  // TODO
}


// 8.3.4
void construct_reference_picture_lists(decoder_context* ctx, slice_segment_header* hdr)
{
  int NumPocTotalCurr = ctx->ref_pic_sets[hdr->CurrRpsIdx].NumPocTotalCurr;
  int NumRpsCurrTempList0 = max(hdr->num_ref_idx_l0_active, NumPocTotalCurr);

  // TODO: fold code for both lists together

  int RefPicListTemp0[DE265_DPB_SIZE]; // TODO: what would be the correct maximum ?
  int RefPicListTemp1[DE265_DPB_SIZE]; // TODO: what would be the correct maximum ?

  int rIdx=0;
  while (rIdx < NumRpsCurrTempList0) {
    for (int i=0;i<ctx->NumPocStCurrBefore && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetStCurrBefore[i];

    for (int i=0;i<ctx->NumPocStCurrAfter && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetStCurrAfter[i];

    for (int i=0;i<ctx->NumPocLtCurr && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = ctx->RefPicSetLtCurr[i];
  }

  assert(hdr->num_ref_idx_l0_active <= 15);
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    hdr->RefPicList[0][rIdx] = hdr->ref_pic_list_modification_flag_l0 ?
      RefPicListTemp0[hdr->list_entry_l0[rIdx]] : RefPicListTemp0[rIdx];

    // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
    ctx->img->RefPicList_POC[0][rIdx] = ctx->dpb[ hdr->RefPicList[0][rIdx] ].PicOrderCntVal;
  }


  if (hdr->slice_type == SLICE_TYPE_B) {
    int NumRpsCurrTempList1 = max(hdr->num_ref_idx_l1_active, NumPocTotalCurr);

    int rIdx=0;
    while (rIdx < NumRpsCurrTempList1) {
      for (int i=0;i<ctx->NumPocStCurrAfter && rIdx<NumRpsCurrTempList1; rIdx++,i++)
        RefPicListTemp1[rIdx] = ctx->RefPicSetStCurrAfter[i];

      for (int i=0;i<ctx->NumPocStCurrBefore && rIdx<NumRpsCurrTempList1; rIdx++,i++)
        RefPicListTemp1[rIdx] = ctx->RefPicSetStCurrBefore[i];

      for (int i=0;i<ctx->NumPocLtCurr && rIdx<NumRpsCurrTempList1; rIdx++,i++)
        RefPicListTemp1[rIdx] = ctx->RefPicSetLtCurr[i];
    }

    assert(hdr->num_ref_idx_l1_active <= 15);
    for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
      hdr->RefPicList[1][rIdx] = hdr->ref_pic_list_modification_flag_l1 ?
        RefPicListTemp1[hdr->list_entry_l1[rIdx]] : RefPicListTemp1[rIdx];

      // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
      ctx->img->RefPicList_POC[1][rIdx] = ctx->dpb[ hdr->RefPicList[1][rIdx] ].PicOrderCntVal;
    }
  }


  // show reference picture lists

  logtrace(LogHeaders,"RefPicList[0] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    logtrace(LogHeaders," %d", hdr->RefPicList[0][rIdx]);
  }
  logtrace(LogHeaders,"\n");

  logtrace(LogHeaders,"RefPicList[1] =");
  for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
    logtrace(LogHeaders," %d", hdr->RefPicList[1][rIdx]);
  }
  logtrace(LogHeaders,"\n");
}



void flush_next_picture_from_reorder_buffer(decoder_context* ctx)
{
  assert(ctx->reorder_output_queue_length>0);

  // search for picture in reorder buffer with minimum POC

  int minPOC = ctx->reorder_output_queue[0]->PicOrderCntVal;
  int minIdx = 0;
  for (int i=1;i<ctx->reorder_output_queue_length;i++)
    {
      if (ctx->reorder_output_queue[i]->PicOrderCntVal < minPOC) {
        minPOC = ctx->reorder_output_queue[i]->PicOrderCntVal;
        minIdx = i;
      }
    }


  // put image into output queue

  assert(ctx->image_output_queue_length < DE265_DPB_SIZE);
  ctx->image_output_queue[ ctx->image_output_queue_length ] = ctx->reorder_output_queue[minIdx];
  ctx->image_output_queue_length++;


  // remove image from reorder buffer

  for (int i=minIdx+1; i<ctx->reorder_output_queue_length; i++) {
    ctx->reorder_output_queue[i-1] = ctx->reorder_output_queue[i];
  }
  ctx->reorder_output_queue_length--;
}


void push_current_picture_to_output_queue(decoder_context* ctx)
{
  if (ctx->img) {
    ctx->img->PicState = UsedForShortTermReference;

    // post-process image

    apply_deblocking_filter(ctx);
    apply_sample_adaptive_offset(ctx);

    // push image into output queue

    if (ctx->img->PicOutputFlag) {
      loginfo(LogDPB,"new picture has output-flag=true\n");

      assert(ctx->reorder_output_queue_length < DE265_DPB_SIZE);
      ctx->reorder_output_queue[ ctx->reorder_output_queue_length++ ] = ctx->img;

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

    int maxNumPicsInReorderBuffer = ctx->current_vps->layer[0].vps_max_num_reorder_pics;

    if (ctx->reorder_output_queue_length > maxNumPicsInReorderBuffer) {
      flush_next_picture_from_reorder_buffer(ctx);
    }


    loginfo(LogDPB, "DPB reorder queue (after push): ");
    for (int i=0;i<ctx->reorder_output_queue_length;i++) {
      loginfo(LogDPB, "*%d ", ctx->reorder_output_queue[i]->PicOrderCntVal);
    }
    loginfo(LogDPB,"*\n");

    loginfo(LogDPB, "DPB output queue (after push): ");
    for (int i=0;i<ctx->image_output_queue_length;i++) {
      loginfo(LogDPB, "*%d ", ctx->image_output_queue[i]->PicOrderCntVal);
    }
    loginfo(LogDPB,"*\n");
  }
}


de265_error process_slice_segment_header(decoder_context* ctx, slice_segment_header* hdr)
{
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

  if (hdr->slice_pic_order_cnt_lsb != ctx->current_image_poc_lsb) {

    // previous picture has been completely decoded

    push_current_picture_to_output_queue(ctx);

    ctx->current_image_poc_lsb = hdr->slice_pic_order_cnt_lsb;


    // allocate info arrays

    de265_error err = allocate_info_arrays(ctx);
    if (err != DE265_OK) { return err; }

    seq_parameter_set* sps = ctx->current_sps;


    // --- find and allocate image buffer for decoding ---

    int free_image_buffer_idx = -1;
    for (int i=0;i<DE265_DPB_SIZE;i++) {
      if (ctx->dpb[i].PicOutputFlag==false && ctx->dpb[i].PicState == UnusedForReference) {
        free_image_buffer_idx = i;
        break;
      }
    }

    if (free_image_buffer_idx == -1) {
      return DE265_ERROR_IMAGE_BUFFER_FULL;
    }

    ctx->img = &ctx->dpb[free_image_buffer_idx];


    int w = sps->pic_width_in_luma_samples;
    int h = sps->pic_height_in_luma_samples;

    enum de265_chroma chroma;
    switch (sps->chroma_format_idc) {
    case 0: chroma = de265_chroma_mono; break;
    case 1: chroma = de265_chroma_420;  break;
    case 2: chroma = de265_chroma_422;  break;
    case 3: chroma = de265_chroma_444;  break;
    default: chroma = de265_chroma_420; assert(0); break; // should never happen
    }

    de265_alloc_image(ctx->img,
                      w,h,
                      chroma,
                      0 /* border */); // border large enough for intra prediction

    if (ctx->img->cb_info_size != ctx->current_sps->PicSizeInMinCbsY ||
        ctx->img->cb_info == NULL) {
      ctx->img->cb_info_size = ctx->current_sps->PicSizeInMinCbsY;
      ctx->img->cb_info = (CB_ref_info*)malloc(sizeof(CB_ref_info) * ctx->img->cb_info_size);
    }

    if (ctx->img->pb_info_size != ctx->current_sps->PicSizeInMinCbsY *4 *4 ||
        ctx->img->pb_info == NULL) {
      ctx->img->pb_info_size = ctx->current_sps->PicSizeInMinCbsY *4 *4;
      ctx->img->pb_info = (PB_ref_info*)malloc(sizeof(PB_ref_info) * ctx->img->pb_info_size);
    }


    /* HACK
    de265_alloc_image(&ctx->coeff,
                      w*2,h,  // 2 bytes per pixel
                      chroma,
                      0); // border
    */

    reset_decoder_context_for_new_picture(ctx);
    prepare_new_picture(ctx);


    if (isIRAP(ctx->nal_unit_type)) {
      if (isIDR(ctx->nal_unit_type) ||
          isBLA(ctx->nal_unit_type) ||
          ctx->first_decoded_picture ||
          0 /* first after EndOfSequence NAL */)
        {
          ctx->NoRaslOutputFlag = true;
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
        ctx->img->PicOutputFlag = hdr->pic_output_flag;
      }

    process_picture_order_count(ctx,hdr);
    process_reference_picture_set(ctx,hdr);
    generate_unavailable_reference_pictures(ctx,hdr);

    log_set_current_POC(ctx->img->PicOrderCntVal);

    if (hdr->slice_type == SLICE_TYPE_B ||
        hdr->slice_type == SLICE_TYPE_P)
      {
        construct_reference_picture_lists(ctx,hdr);
      }
  }

  log_dpb_content(ctx);

  return DE265_OK;
}


void debug_dump_cb_info(const decoder_context* ctx)
{
  for (int y=0;y<ctx->current_sps->PicHeightInMinCbsY;y++)
    {
      for (int x=0;x<ctx->current_sps->PicWidthInMinCbsY;x++)
        {
          logtrace(LogPixels,"*%d ", ctx->cb_info[ y*ctx->current_sps->PicWidthInMinCbsY + x].depth);
        }
      logtrace(LogPixels,"*\n");
    }
}


#define PIXEL2CB(x) (x >> ctx->current_sps->Log2MinCbSizeY)
#define CB_IDX(x0,y0) (PIXEL2CB(x0) + PIXEL2CB(y0)*ctx->current_sps->PicWidthInMinCbsY)
#define GET_CB_BLK(x,y) ctx->cb_info[CB_IDX(x,y)]

//#define GET_CB_BLK(x,y) (assert(CB_IDX(x,y) < ctx->cb_info_size) , ctx->cb_info[CB_IDX(x,y)])
#define SET_CB_BLK(x,y,log2BlkWidth,  Field,value)                      \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinCbSizeY);   \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      {                                                                 \
        { assert( cbx + cby*ctx->current_sps->PicWidthInMinCbsY < ctx->cb_info_size ); } \
        ctx->cb_info[ cbx + cby*ctx->current_sps->PicWidthInMinCbsY ].Field = value; \
      }

#define SET_IMG_CB_BLK(x,y,log2BlkWidth,  Field,value)                      \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinCbSizeY);   \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      {                                                                 \
        { assert( cbx + cby*ctx->current_sps->PicWidthInMinCbsY < ctx->cb_info_size ); } \
        ctx->img->cb_info[ cbx + cby*ctx->current_sps->PicWidthInMinCbsY ].Field = value; \
      }

#define SET_CB_BLK_SAVE(x,y,log2BlkWidth,  Field,value)                 \
  int cbX = PIXEL2CB(x);                                                \
  int cbY = PIXEL2CB(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinCbSizeY);   \
  for (int cby=cbY;cby<cbY+width;cby++)                                 \
    for (int cbx=cbX;cbx<cbX+width;cbx++)                               \
      if (cbx < ctx->current_sps->PicWidthInMinCbsY &&                  \
          cby < ctx->current_sps->PicHeightInMinCbsY)                   \
      {                                                                 \
        { assert( cbx + cby*ctx->current_sps->PicWidthInMinCbsY < ctx->cb_info_size ); } \
        ctx->cb_info[ cbx + cby*ctx->current_sps->PicWidthInMinCbsY ].Field = value; \
      }

void set_ctDepth(decoder_context* ctx, int x,int y, int log2BlkWidth, int depth)
{
  SET_CB_BLK(x,y,log2BlkWidth, depth, depth);
}

int get_ctDepth(const decoder_context* ctx, int x,int y)
{
  return ctx->cb_info[ CB_IDX(x,y) ].depth;
}


void    set_cu_skip_flag(decoder_context* ctx, int x,int y, int log2BlkWidth, uint8_t flag)
{
  SET_CB_BLK(x,y,log2BlkWidth, cu_skip_flag, flag);
}

uint8_t get_cu_skip_flag(const decoder_context* ctx, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].cu_skip_flag;
}

void          set_PartMode(decoder_context* ctx, int x,int y, enum PartMode mode)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].PartMode = mode;
}

enum PartMode get_PartMode(const decoder_context* ctx, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return (enum PartMode)ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].PartMode;
}


void    set_intra_split_flag(decoder_context* ctx, int x,int y, uint8_t flag)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].intra_split_flag = flag;
}

uint8_t get_intra_split_flag(decoder_context* ctx, int x,int y)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].intra_split_flag;
}


void    set_cu_split_flag(decoder_context* ctx, int x,int y,int log2CbSize)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].split_cu_flag |= 1<<log2CbSize;
}

uint8_t get_cu_split_flag(decoder_context* ctx, int x,int y, int log2CbSize)
{
  int cbX = PIXEL2CB(x);
  int cbY = PIXEL2CB(y);

  return ctx->cb_info[ cbX + cbY*ctx->current_sps->PicWidthInMinCbsY ].split_cu_flag & 1<<log2CbSize;
}


void set_pred_mode(decoder_context* ctx, int x,int y, int log2BlkWidth, enum PredMode mode)
{
  { SET_CB_BLK(x,y,log2BlkWidth, PredMode, mode); }
  { SET_IMG_CB_BLK(x,y,log2BlkWidth, PredMode, mode); }
}

enum PredMode get_pred_mode(const decoder_context* ctx, int x,int y)
{
  return (enum PredMode)ctx->cb_info[ CB_IDX(x,y) ].PredMode;
}

enum PredMode get_img_pred_mode(const decoder_context* ctx,
                                const de265_image* img, int x,int y)
{
  return (enum PredMode)img->cb_info[ CB_IDX(x,y) ].PredMode;
}


void set_rqt_root_cbf(decoder_context* ctx,int x,int y, int log2BlkWidth, int rqt_root_cbf_value)
{
  SET_CB_BLK(x,y,log2BlkWidth, rqt_root_cbf, rqt_root_cbf_value);
}

int  get_rqt_root_cbf(const decoder_context* ctx,int x,int y)
{
  return GET_CB_BLK(x,y).rqt_root_cbf;
}


#define PIXEL2TU(x) (x >> ctx->current_sps->Log2MinTrafoSize)
#define TU_IDX(x0,y0) (PIXEL2TU(x0) + PIXEL2TU(y0)*ctx->current_sps->PicWidthInTbsY)
#define GET_TU_BLK(x,y) (assert(TU_IDX(x,y) < ctx->tu_info_size) , ctx->tu_info[TU_IDX(x,y)])
#define SET_TU_BLK(x,y,log2BlkWidth,  Field,value)                      \
  int tuX = PIXEL2TU(x);                                                \
  int tuY = PIXEL2TU(y);                                                \
  int width = 1 << (log2BlkWidth - ctx->current_sps->Log2MinTrafoSize); \
  for (int tuy=tuY;tuy<tuY+width;tuy++)                                 \
    for (int tux=tuX;tux<tuX+width;tux++)                               \
      {                                                                 \
        ctx->tu_info[ tux + tuy*ctx->current_sps->PicWidthInTbsY ].Field = value; \
      }

void set_cbf_cb(decoder_context* ctx, int x0,int y0, int depth)
{
  logtrace(LogSlice,"set_cbf_cb at %d;%d depth %d\n",x0,y0,depth);
  ctx->tu_info[TU_IDX(x0,y0)].cbf_cb |= (1<<depth);
}

void set_cbf_cr(decoder_context* ctx, int x0,int y0, int depth)
{
  logtrace(LogSlice,"set_cbf_cr at %d;%d depth %d\n",x0,y0,depth);
  ctx->tu_info[TU_IDX(x0,y0)].cbf_cr |= (1<<depth);
}

int  get_cbf_cb(const decoder_context* ctx, int x0,int y0, int depth)
{
  return (ctx->tu_info[TU_IDX(x0,y0)].cbf_cb & (1<<depth)) ? 1:0;
}

int  get_cbf_cr(const decoder_context* ctx, int x0,int y0, int depth)
{
  return (ctx->tu_info[TU_IDX(x0,y0)].cbf_cr & (1<<depth)) ? 1:0;
}

void set_IntraPredMode(decoder_context* ctx, int x,int y, int log2BlkWidth, enum IntraPredMode mode)
{
  SET_TU_BLK(x,y,log2BlkWidth, IntraPredMode,mode);
}

enum IntraPredMode get_IntraPredMode(const decoder_context* ctx, int x,int y)
{
  return (enum IntraPredMode)GET_TU_BLK(x,y).IntraPredMode;
}

void set_IntraPredModeC(decoder_context* ctx, int x,int y, int log2BlkWidth, enum IntraPredMode mode)
{
  SET_TU_BLK(x,y,log2BlkWidth, IntraPredModeC,mode);
}

enum IntraPredMode get_IntraPredModeC(const decoder_context* ctx, int x,int y)
{
  return (enum IntraPredMode)GET_TU_BLK(x,y).IntraPredModeC;
}

void set_SliceAddrRS(decoder_context* ctx, int ctbX, int ctbY, int SliceAddrRS)
{
  assert(ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY < ctx->ctb_info_size);
  ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceAddrRS = SliceAddrRS;
}

int  get_SliceAddrRS(const decoder_context* ctx, int ctbX, int ctbY)
{
  return ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceAddrRS;
}


void set_SliceHeaderIndex(decoder_context* ctx, int x, int y, int SliceHeaderIndex)
{
  int ctbX = x >> ctx->current_sps->Log2CtbSizeY;
  int ctbY = y >> ctx->current_sps->Log2CtbSizeY;
  ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex = SliceHeaderIndex;
}

int  get_SliceHeaderIndex(const decoder_context* ctx, int x, int y)
{
  int ctbX = x >> ctx->current_sps->Log2CtbSizeY;
  int ctbY = y >> ctx->current_sps->Log2CtbSizeY;
  return ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex;
}

slice_segment_header* get_SliceHeader(decoder_context* ctx, int x, int y)
{
  return &ctx->slice[ get_SliceHeaderIndex(ctx,x,y) ];
}

slice_segment_header* get_SliceHeaderCtb(decoder_context* ctx, int ctbX, int ctbY)
{
  return &ctx->slice[ ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].SliceHeaderIndex ];
}

void set_split_transform_flag(decoder_context* ctx,int x0,int y0,int trafoDepth)
{
  ctx->tu_info[TU_IDX(x0,y0)].split_transform_flag |= (1<<trafoDepth);
}

int  get_split_transform_flag(const decoder_context* ctx,int x0,int y0,int trafoDepth)
{
  int idx = TU_IDX(x0,y0);
  return (ctx->tu_info[idx].split_transform_flag & (1<<trafoDepth)) ? 1:0;
}


void set_transform_skip_flag(decoder_context* ctx,int x0,int y0,int cIdx)
{
  ctx->tu_info[TU_IDX(x0,y0)].transform_skip_flag |= (1<<cIdx);
}

int  get_transform_skip_flag(const decoder_context* ctx,int x0,int y0,int cIdx)
{
  int idx = TU_IDX(x0,y0);
  return (ctx->tu_info[idx].transform_skip_flag & (1<<cIdx)) ? 1:0;
}


void set_nonzero_coefficient(decoder_context* ctx,int x,int y, int log2TrafoSize)
{
  SET_TU_BLK(x,y,log2TrafoSize, nonzero_coefficient, 1);
}


int  get_nonzero_coefficient(const decoder_context* ctx,int x,int y)
{
  return GET_TU_BLK(x,y).nonzero_coefficient;
}


void set_QPY(decoder_context* ctx,int x,int y, int QP_Y)
{
  assert(x>=0 && x<ctx->current_sps->pic_width_in_luma_samples);
  assert(y>=0 && y<ctx->current_sps->pic_height_in_luma_samples);

  SET_CB_BLK_SAVE(x,y,ctx->current_pps->Log2MinCuQpDeltaSize, QP_Y, QP_Y);
}

int  get_QPY(const decoder_context* ctx,int x,int y)
{
  return GET_CB_BLK(x,y).QP_Y;
}



void get_image_plane(const decoder_context* ctx, int cIdx, uint8_t** image, int* stride)
{
  switch (cIdx) {
  case 0: *image = ctx->img->y;  *stride = ctx->img->stride; break;
  case 1: *image = ctx->img->cb; *stride = ctx->img->chroma_stride; break;
  case 2: *image = ctx->img->cr; *stride = ctx->img->chroma_stride; break;
  }
}


void get_coeff_plane(const decoder_context* ctx, int cIdx, int16_t** image, int* stride)
{
  switch (cIdx) {
  case 0: *image = (int16_t*)ctx->coeff.y;  *stride = ctx->coeff.stride/2; break;
  case 1: *image = (int16_t*)ctx->coeff.cb; *stride = ctx->coeff.chroma_stride/2; break;
  case 2: *image = (int16_t*)ctx->coeff.cr; *stride = ctx->coeff.chroma_stride/2; break;
  }
}


void set_log2CbSize(decoder_context* ctx, int x0, int y0, int log2CbSize)
{
  ctx->cb_info[ CB_IDX(x0,y0) ].CB_size = log2CbSize;

  // assume that remaining cb_info blocks are initialized to zero
}

int get_log2CbSize(const decoder_context* ctx, int x0, int y0)
{
  return ctx->cb_info[ CB_IDX(x0,y0) ].CB_size;
}

int get_log2CbSize_cbUnits(const decoder_context* ctx, int x0, int y0)
{
  return ctx->cb_info[ x0 + y0 * ctx->current_sps->PicWidthInMinCbsY ].CB_size;
}

void    set_deblk_flags(decoder_context* ctx, int x0,int y0, uint8_t flags)
{
  ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags |= flags;
}

uint8_t get_deblk_flags(const decoder_context* ctx, int x0,int y0)
{
  return ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags;
}

void    set_deblk_bS(decoder_context* ctx, int x0,int y0, uint8_t bS)
{
  uint8_t* data = &ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags;
  *data &= ~DEBLOCK_BS_MASK;
  *data |= bS;
}

uint8_t get_deblk_bS(const decoder_context* ctx, int x0,int y0)
{
  return ctx->deblk_info[x0/4 + y0/4*ctx->deblk_width].deblock_flags & DEBLOCK_BS_MASK;
}

void            set_sao_info(decoder_context* ctx, int ctbX,int ctbY,const sao_info* saoinfo)
{
  assert(ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY < ctx->ctb_info_size);
  memcpy(&ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].saoInfo,
         saoinfo,
         sizeof(sao_info));
}

const sao_info* get_sao_info(const decoder_context* ctx, int ctbX,int ctbY)
{
  assert(ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY < ctx->ctb_info_size);
  return &ctx->ctb_info[ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY].saoInfo;
}



#define PIXEL2PB(x) (x >> (ctx->current_sps->Log2MinCbSizeY-2))
#define PB_IDX(x0,y0) (PIXEL2PB(x0) + PIXEL2PB(y0)*ctx->current_sps->PicWidthInMinCbsY*4)
#define GET_PB_BLK(x,y) ctx->pb_info[PB_IDX(x,y)]
#define SET_PB_BLK(x,y,nPbW,nPbH, Field, value)                         \
  int blksize = 1<<(ctx->current_sps->Log2MinCbSizeY-2);                \
  for (int pby=y;pby<y+nPbH;pby+=blksize)                               \
    for (int pbx=x;pbx<x+nPbW;pbx+=blksize)                             \
      {                                                                 \
        ctx->pb_info[PB_IDX(pbx,pby)].Field = value;                    \
      }

#define SET_IMG_PB_BLK(x,y,nPbW,nPbH,  Field,value)                     \
  int blksize = 1<<(ctx->current_sps->Log2MinCbSizeY-2);                \
  for (int pby=y;pby<y+nPbH;pby+=blksize)                               \
    for (int pbx=x;pbx<x+nPbW;pbx+=blksize)                             \
      {                                                                 \
        ctx->img->pb_info[PB_IDX(pbx,pby)].Field = value;               \
      }

//        fprintf(stderr,"PB %d;%d = %d;%d\n",pbx,pby,(value).mv[0].x,(value).mv[0].y); \


const PredVectorInfo* get_mv_info(const decoder_context* ctx,int x,int y)
{
  int idx = PB_IDX(x,y);

  return &ctx->img->pb_info[ PB_IDX(x,y) ].mvi;
}


const PredVectorInfo* get_img_mv_info(const decoder_context* ctx,
                                      const de265_image* img, int x,int y)
{
  return &img->pb_info[ PB_IDX(x,y) ].mvi;
}


void set_mv_info(decoder_context* ctx,int x,int y, int nPbW,int nPbH, const PredVectorInfo* mv)
{
  /*
  fprintf(stderr,"set_mv_info %d;%d [%d;%d] to %d;%d (POC=%d)\n",x,y,nPbW,nPbH,
          mv->mv[0].x,mv->mv[0].y,
          ctx->img->PicOrderCntVal);
  */

  //{ SET_PB_BLK(x,y,nPbW,nPbH, pred_vector, *mv); }
  { SET_IMG_PB_BLK(x,y,nPbW,nPbH, mvi, *mv); }
}


int get_merge_idx(const decoder_context* ctx,int xP,int yP)
{
  int idx = PB_IDX(xP,yP);
  return ctx->pb_info[idx].merge_idx;
}

void set_merge_idx(decoder_context* ctx,int x0,int y0,int nPbW,int nPbH, int new_merge_idx)
{
  SET_PB_BLK(x0,y0,nPbW,nPbH,merge_idx, new_merge_idx);
}


uint8_t get_merge_flag(const decoder_context* ctx,int xP,int yP)
{
  int idx = PB_IDX(xP,yP);
  return ctx->pb_info[idx].merge_flag;
}

void set_merge_flag(decoder_context* ctx,int x0,int y0,int nPbW,int nPbH, uint8_t new_merge_flag)
{
  SET_PB_BLK(x0,y0,nPbW,nPbH,merge_flag, new_merge_flag);
}


uint8_t get_mvp_flag(const decoder_context* ctx,int xP,int yP, int l)
{
  int idx = PB_IDX(xP,yP);
  return ctx->pb_info[idx].mvp_lX_flag[l];
}

void set_mvp_flag(decoder_context* ctx,int x0,int y0,int nPbW,int nPbH,
                  int l, uint8_t new_flag)
{
  SET_PB_BLK(x0,y0,nPbW,nPbH,mvp_lX_flag[l], new_flag);
}


void    set_mvd(decoder_context* ctx,int x0,int y0,int reflist, int16_t dx,int16_t dy)
{
  int idx = PB_IDX(x0,y0);
  ctx->pb_info[idx].mvd[reflist][0] = dx;
  ctx->pb_info[idx].mvd[reflist][1] = dy;
}

int16_t get_mvd_x(const decoder_context* ctx,int x0,int y0,int reflist)
{
  int idx = PB_IDX(x0,y0);
  return ctx->pb_info[idx].mvd[reflist][0];
}

int16_t get_mvd_y(const decoder_context* ctx,int x0,int y0,int reflist)
{
  int idx = PB_IDX(x0,y0);
  return ctx->pb_info[idx].mvd[reflist][1];
}

void    set_ref_idx(decoder_context* ctx,int x0,int y0,int nPbW,int nPbH,int l, int ref_idx)
{
  // TODO: is it possible to mode this into the image's PB data or is this at a different resolution?
  // It appears that the resolution is different, because it does not work out of the box.

  SET_PB_BLK(x0,y0,nPbW,nPbH, refIdx[l], ref_idx);
  //de265_image* img = ctx->img;
  //{ SET_IMG_PB_BLK(x0,y0,nPbW,nPbH, mvi.refIdx[l], ref_idx); }
}

uint8_t get_ref_idx(const decoder_context* ctx,int x0,int y0,int l)
{
  int idx = PB_IDX(x0,y0);
  return ctx->pb_info[idx].refIdx[l];
  //return &ctx->img->pb_info[ PB_IDX(x0,y0) ].mvi.refIdx[l];
}


void set_inter_pred_idc(decoder_context* ctx,int x0,int y0,enum InterPredIdc idc)
{
  int idx = PB_IDX(x0,y0);
  ctx->pb_info[idx].inter_pred_idc = (uint8_t)idc;
}

enum InterPredIdc get_inter_pred_idc(const decoder_context* ctx,int x0,int y0)
{
  int idx = PB_IDX(x0,y0);
  return (enum InterPredIdc)(ctx->pb_info[idx].inter_pred_idc);
}


void set_CTB_deblocking_cnt(decoder_context* ctx,int ctbX,int ctbY, int cnt)
{
  int idx = ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY;
  ctx->ctb_info[idx].task_blocking_cnt = cnt;
}

uint8_t decrease_CTB_deblocking_cnt(decoder_context* ctx,int ctbX,int ctbY)
{
  int idx = ctbX + ctbY*ctx->current_sps->PicWidthInCtbsY;

  uint8_t blkcnt = __sync_sub_and_fetch(&ctx->ctb_info[idx].task_blocking_cnt, 1);
  return blkcnt;
}


bool available_zscan(const decoder_context* ctx,
                     int xCurr,int yCurr, int xN,int yN)
{
  seq_parameter_set* sps = ctx->current_sps;
  pic_parameter_set* pps = ctx->current_pps;


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

  if (get_SliceAddrRS(ctx, xCurrCtb,yCurrCtb) !=
      get_SliceAddrRS(ctx, xNCtb,   yNCtb)) {
    return false;
  }

  if (pps->TileId[xCurrCtb + yCurrCtb*sps->PicWidthInCtbsY] !=
      pps->TileId[xNCtb    + yNCtb   *sps->PicWidthInCtbsY]) {
    return false;
  }

  return true;
}


bool available_pred_blk(const decoder_context* ctx,
                        int xC,int yC, int nCbS, int xP, int yP, int nPbW, int nPbH, int partIdx,
                        int xN,int yN)
{
  int sameCb = (xC <= xN && xN < xC+nCbS &&
                yC <= yN && yN < yC+nCbS);

  bool availableN;

  if (!sameCb) {
    availableN = available_zscan(ctx,xP,yP,xN,yN);
  }
  else {
    availableN = !(nPbW<<1 == nCbS && nPbH<<1 == nCbS &&
                   partIdx==1 &&
                   yN >= yC+nPbH && xN < xC+nPbW);
  }

  if (availableN && get_pred_mode(ctx,xN,yN) == MODE_INTRA) {
    availableN = false;
  }

  return availableN;
}


void write_picture(const de265_image* img)
{
  static FILE* fh = NULL;
  if (fh==NULL) { fh = fopen("out.yuv","wb"); }

  for (int y=0;y<img->height;y++)
    fwrite(img->y + y*img->stride, img->width, 1, fh);

  for (int y=0;y<img->chroma_height;y++)
    fwrite(img->cb + y*img->chroma_stride, img->chroma_width, 1, fh);

  for (int y=0;y<img->chroma_height;y++)
    fwrite(img->cr + y*img->chroma_stride, img->chroma_width, 1, fh);

  fflush(fh);
  //fclose(fh);
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
  if (get_split_transform_flag(ctx,x0,y0,trafoDepth)) {
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
    enum PredMode predMode = get_pred_mode(ctx,x0,y0);

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
  int minCbSize = ctx->current_sps->MinCbSizeY;

  for (int y0=0;y0<ctx->current_sps->PicHeightInMinCbsY;y0++)
    for (int x0=0;x0<ctx->current_sps->PicWidthInMinCbsY;x0++)
      {
        int log2CbSize = get_log2CbSize_cbUnits(ctx,x0,y0);
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
          enum PartMode partMode = get_PartMode(ctx,xb,yb);

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
          enum PredMode predMode = get_pred_mode(ctx,xb,yb);
          if (predMode == MODE_INTRA) {
            enum PartMode partMode = get_PartMode(ctx,xb,yb);

            int HalfCbSize = (1<<(log2CbSize-1));

            switch (partMode) {
            case PART_2Nx2N:
              draw_intra_pred_mode(ctx,img,stride,xb,yb,log2CbSize,
                                   get_IntraPredMode(ctx,xb,yb), value);
              break;
            case PART_NxN:
              draw_intra_pred_mode(ctx,img,stride,xb,           yb,           log2CbSize-1,
                                   get_IntraPredMode(ctx,xb,yb), value);
              draw_intra_pred_mode(ctx,img,stride,xb+HalfCbSize,yb,           log2CbSize-1,
                                   get_IntraPredMode(ctx,xb+HalfCbSize,yb), value);
              draw_intra_pred_mode(ctx,img,stride,xb           ,yb+HalfCbSize,log2CbSize-1,
                                   get_IntraPredMode(ctx,xb,yb+HalfCbSize), value);
              draw_intra_pred_mode(ctx,img,stride,xb+HalfCbSize,yb+HalfCbSize,log2CbSize-1,
                                   get_IntraPredMode(ctx,xb+HalfCbSize,yb+HalfCbSize), value);
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
