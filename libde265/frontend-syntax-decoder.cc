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


frontend_syntax_decoder::frontend_syntax_decoder(decoder_context* ctx)
{
  m_decctx=ctx;
  m_image_unit_sink = nullptr;

  reset();


  param_header_callback = nullptr;

  nal_parser.set_on_NAL_inserted_listener(this);
}


void frontend_syntax_decoder::reset()
{
  // --- decoded picture buffer ---

  first_decoded_picture = true;
  NoRaslOutputFlag = false;
  HandleCraAsBlaFlag = false;
  FirstAfterEndOfSequenceNAL = false;

  current_image_poc_lsb = -1; // any invalid number
  PicOrderCntMsb = 0;
  prevPicOrderCntLsb = 0;
  prevPicOrderCntMsb = 0;

  nal_parser.remove_pending_input_data();

  m_curr_image_unit.reset();
  m_curr_img.reset();

  previous_slice_header=NULL;

  current_vps.reset();
  current_sps.reset();
  current_pps.reset();
}


de265_error frontend_syntax_decoder::read_vps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"---> read VPS\n");

  std::shared_ptr<video_parameter_set> new_vps = std::make_shared<video_parameter_set>();
  de265_error err = new_vps->read(m_decctx,&reader);
  if (err != DE265_OK) {
    return err;
  }

  if (param_header_callback != nullptr) {
    std::string hdr = new_vps->dump();
    param_header_callback(NAL_UNIT_VPS_NUT, hdr.c_str());
  }

  vps[ new_vps->video_parameter_set_id ] = new_vps;

  return DE265_OK;
}


de265_error frontend_syntax_decoder::read_sps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"----> read SPS\n");

  std::shared_ptr<seq_parameter_set> new_sps = std::make_shared<seq_parameter_set>();
  de265_error err;

  if ((err=new_sps->read(m_decctx, &reader)) != DE265_OK) {
    return err;
  }

  if (param_header_callback != nullptr) {
    std::string str = new_sps->dump();
    param_header_callback(NAL_UNIT_SPS_NUT, str.c_str());
  }

  sps[ new_sps->seq_parameter_set_id ] = new_sps;

  return DE265_OK;
}


de265_error frontend_syntax_decoder::read_pps_NAL(bitreader& reader)
{
  logdebug(LogHeaders,"----> read PPS\n");

  std::shared_ptr<pic_parameter_set> new_pps = std::make_shared<pic_parameter_set>();

  bool success = new_pps->read(&reader,m_decctx);

  if (param_header_callback != nullptr) {
    std::string dump = new_pps->dump();
    param_header_callback(NAL_UNIT_PPS_NUT, dump.c_str());
  }

  //printf("read PPS (success=%d)\n",success);
  if (success) {
    pps[ (int)new_pps->pic_parameter_set_id ] = new_pps;
  }

  return success ? DE265_OK : DE265_WARNING_PPS_HEADER_INVALID;
}


de265_error frontend_syntax_decoder::read_sei_NAL(bitreader& reader, bool suffix)
{
  logdebug(LogHeaders,"----> read SEI\n");

  sei_message sei;

  de265_error err = DE265_OK;

  if ((err=read_sei(&reader,&sei, suffix, current_sps.get())) == DE265_OK) {
    dump_sei(&sei, current_sps.get());

    if (m_curr_image_unit && suffix) {
      m_curr_image_unit->suffix_SEIs.push_back(sei);
    }
  }
  else {
    m_decctx->add_warning(err, false);
  }

  return err;
}


de265_error frontend_syntax_decoder::read_eos_NAL(bitreader& reader)
{
  FirstAfterEndOfSequenceNAL = true;
  return DE265_OK;
}


de265_error frontend_syntax_decoder::read_slice_NAL(bitreader& reader, NAL_unit_ptr nal, nal_header& nal_hdr)
{
  logdebug(LogHeaders,"---> read slice segment header\n");


  // --- read slice header ---

  slice_segment_header* shdr = new slice_segment_header;

  bool continueDecoding;
  de265_error err = shdr->read(&reader,m_decctx, nal_unit_type, &continueDecoding);
  if (!continueDecoding) {
    if (m_curr_img) { m_curr_img->integrity = INTEGRITY_NOT_DECODED; }
    delete shdr;
    return err;
  }

  if (param_header_callback != nullptr) {
    std::string dump = shdr->dump_slice_segment_header(m_decctx, nal_unit_type);
    param_header_callback(nal_unit_type, dump.c_str());
  }


  // --- get PPS and SPS for this slice ---

  int pps_id = shdr->slice_pic_parameter_set_id;
  if (pps[pps_id]->pps_read==false) {
    logerror(LogHeaders, "PPS %d has not been read\n", pps_id);
    assert(false); // TODO
  }

  // We only set the current PPS/SPS/VPS for the first slice segment in the picture
  // because faulty streams may send new a PPS between slices, for example with a different
  // tile structure which then completely confuses the decoder.
  // E.g. fuzzing/id:000225,sig:11,src:002437+005717,op:splice,rep:128.bin
  if (shdr->first_slice_segment_in_pic_flag
      // || (m_curr_image_unit && !m_curr_image_unit->slice_units.empty())
      ) {
    current_pps = pps[pps_id];
    current_sps = sps[ (int)current_pps->seq_parameter_set_id ];
    current_vps = vps[ (int)current_sps->video_parameter_set_id ];
  }

  shdr->set_pps( current_pps );


  m_decctx->calc_tid_and_framerate_ratio();


  // --- start a new image if this is the first slice ---

  // TODO: what happens when the first slice of an image is missing? Are the remaining
  // slices added to the previous image and decoded twice without noticing the error?
  if (shdr->first_slice_segment_in_pic_flag) {

    // output previous image_unit if available

    if (m_curr_image_unit) {
      m_image_unit_sink->send_image_unit(m_curr_image_unit);
    }


    // --- find and allocate image buffer for decoding ---

    seq_parameter_set* sps = current_sps.get();
    decoded_picture_buffer& dpb = m_decctx->dpb;

    int image_buffer_idx;
    image_buffer_idx = dpb.new_image(current_sps, m_decctx, nal->pts, nal->user_data,
                                     &m_decctx->param_image_allocation_functions);
    if (image_buffer_idx == -1) {
      return DE265_ERROR_IMAGE_BUFFER_FULL;
    }

    m_curr_img = dpb.get_image(image_buffer_idx);
    m_curr_img->nal_hdr = nal_hdr;

    // Note: sps is already set in new_image() -> ??? still the case with shared_ptr ?

    m_curr_img->set_headers(current_vps, current_sps, current_pps);
    m_curr_img->decctx = m_decctx;
    m_curr_img->clear_metadata();


    // --- create new image_unit ---

    m_curr_image_unit = std::make_shared<image_unit>();
    m_curr_image_unit->img = m_curr_img;

    m_curr_image_unit->nal_unit_type = nal_unit_type;
  }


  if (process_slice_segment_header(shdr, &err, nal->pts, &nal_hdr, nal->user_data) == false)
    {
      if (m_curr_img!=NULL) m_curr_img->integrity = INTEGRITY_NOT_DECODED;
      delete shdr;
      return err;
    }


  m_curr_img->add_slice_segment_header(shdr);

  skip_bits(&reader,1); // TODO: why?
  prepare_for_CABAC(&reader);


  // modify entry_point_offsets

  int headerLength = reader.data - nal->data();
  for (int i=0;i<shdr->num_entry_point_offsets;i++) {
    shdr->entry_point_offset[i] -= nal->num_skipped_bytes_before(shdr->entry_point_offset[i],
                                                                 headerLength);
  }



  // --- add slice to current picture ---

  if (m_curr_image_unit) {

    slice_unit* sliceunit = new slice_unit(m_decctx);
    sliceunit->nal = nal;
    sliceunit->shdr = shdr;
    sliceunit->reader = reader;
    sliceunit->imgunit = m_curr_image_unit.get();

    sliceunit->flush_reorder_buffer = flush_reorder_buffer_at_this_frame;


    // --- assign CTB-range that is covered by this slice-unit ---

    sliceunit->first_CTB_TS = shdr->get_pps()->CtbAddrTStoRS[shdr->slice_segment_address];
    sliceunit->last_CTB_TS  = shdr->get_pps()->sps->PicSizeInCtbsY -1;

    bool first_observed_slice_unit = (m_curr_image_unit->slice_units.empty());

    if (!first_observed_slice_unit) {
      m_curr_image_unit->slice_units.back()->last_CTB_TS = sliceunit->first_CTB_TS - 1;
    }

    if (first_observed_slice_unit) {
      sliceunit->first_CTB_TS = 0;
    }


    // --- add slice-unit to image-unit ---

    m_curr_image_unit->slice_units.push_back(sliceunit);
  }

  //TODO TMP DISABLE bool did_work;
  //TODO TMP DISABLE err = m_decctx->decode_image_unit(&did_work);

  return DE265_OK;
}


void frontend_syntax_decoder::debug_imageunit_state()
{
  loginfo(LogHighlevel,"m_curr_image_unit: ");
  if (m_curr_image_unit)
    loginfo(LogHighlevel,"%d\n", m_curr_image_unit->img->get_ID());
  else
    loginfo(LogHighlevel,"NULL\n");

  loginfo(LogHighlevel,"NALs: %d EOS: %d EOF: %d\n",
          nal_parser.get_NAL_queue_length(),
          nal_parser.is_end_of_stream(),
          nal_parser.is_end_of_frame() );
}


void frontend_syntax_decoder::debug_dump_state()
{
}


de265_error frontend_syntax_decoder::decode_NAL(NAL_unit_ptr nal)
{
  decoder_context* ctx = m_decctx;

  de265_error err = DE265_OK;

  bitreader reader;
  bitreader_init(&reader, nal->data(), nal->size());

  nal_header nal_hdr;
  nal_hdr.read(&reader);


  nal_unit_type = nal_hdr.nal_unit_type;

  //IdrPicFlag = isIdrPic(nal_hdr.nal_unit_type);
  RapPicFlag = isRapPic(nal_hdr.nal_unit_type);


  if (nal_hdr.nuh_layer_id > 0) {
    // Discard all NAL units with nuh_layer_id > 0
    // These will have to be handeled by an SHVC decoder.
    return DE265_OK;
  }

  loginfo(LogHighlevel,"NAL: 0x%x 0x%x -  unit type:%s temporal id:%d\n",
          nal->data()[0], nal->data()[1],
          get_NAL_name(nal_hdr.nal_unit_type),
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

  if (nal_hdr.nuh_temporal_id > m_decctx->get_current_TID()) {
    return DE265_OK;
  }


  if (nal_hdr.nal_unit_type<32) {
    loginfo(LogHighlevel, "read slice NAL\n");
    err = read_slice_NAL(reader, nal, nal_hdr);
  }
  else switch (nal_hdr.nal_unit_type) {
    case NAL_UNIT_VPS_NUT:
      loginfo(LogHighlevel, "read VPS NAL\n");
      err = read_vps_NAL(reader);
      break;

    case NAL_UNIT_SPS_NUT:
      loginfo(LogHighlevel, "read SPS NAL\n");
      err = read_sps_NAL(reader);
      break;

    case NAL_UNIT_PPS_NUT:
      loginfo(LogHighlevel, "read PPS NAL\n");
      err = read_pps_NAL(reader);
      break;

    case NAL_UNIT_PREFIX_SEI_NUT:
    case NAL_UNIT_SUFFIX_SEI_NUT:
      loginfo(LogHighlevel, "read SEI NAL\n");
      err = read_sei_NAL(reader, nal_hdr.nal_unit_type==NAL_UNIT_SUFFIX_SEI_NUT);
      break;

    case NAL_UNIT_EOS_NUT:
      loginfo(LogHighlevel, "read EOS NAL\n");
      FirstAfterEndOfSequenceNAL = true;
      break;

    default:
      break;
    }

  return err;
}


de265_error frontend_syntax_decoder::on_NAL_inserted()
{
  loginfo(LogHighlevel,"================== ON-NAL-inserted\n");

  de265_error err = DE265_OK;

  while (nal_parser.get_NAL_queue_length() > 0) {
    NAL_unit_ptr nal = nal_parser.pop_from_NAL_queue();
    assert(nal);
    err = decode_NAL(nal);
  }

  debug_imageunit_state();

  return err;
}


void frontend_syntax_decoder::on_end_of_stream()
{
  on_end_of_frame();
}


void frontend_syntax_decoder::on_end_of_frame()
{
  loginfo(LogHighlevel,"================== ON-EOF\n");

  if (m_curr_image_unit) {
    m_image_unit_sink->send_image_unit(m_curr_image_unit);
  }

  m_curr_image_unit.reset();

  m_image_unit_sink->send_end_of_stream();

  debug_imageunit_state();
}


bool frontend_syntax_decoder::is_input_buffer_full() const
{
  return !m_decctx->dpb.has_free_dpb_picture(false);
  // return DE265_ERROR_IMAGE_BUFFER_FULL;
}



/* 8.3.1
 */
void frontend_syntax_decoder::process_picture_order_count(slice_segment_header* hdr)
{
  loginfo(LogHeaders,"POC computation. lsb:%d prev.pic.lsb:%d msb:%d\n",
          hdr->slice_pic_order_cnt_lsb,
          prevPicOrderCntLsb,
          PicOrderCntMsb);

  if (isIRAP(nal_unit_type) &&
      NoRaslOutputFlag)
    {
      PicOrderCntMsb=0;


      // flush all images from reorder buffer

      flush_reorder_buffer_at_this_frame = true;
      //m_decctx->dpb.flush_reorder_buffer();
    }
  else
    {
      int MaxPicOrderCntLsb = current_sps->MaxPicOrderCntLsb;

      if ((hdr->slice_pic_order_cnt_lsb < prevPicOrderCntLsb) &&
          (prevPicOrderCntLsb - hdr->slice_pic_order_cnt_lsb) >= MaxPicOrderCntLsb/2) {
        PicOrderCntMsb = prevPicOrderCntMsb + MaxPicOrderCntLsb;
      }
      else if ((hdr->slice_pic_order_cnt_lsb > prevPicOrderCntLsb) &&
               (hdr->slice_pic_order_cnt_lsb - prevPicOrderCntLsb) > MaxPicOrderCntLsb/2) {
        PicOrderCntMsb = prevPicOrderCntMsb - MaxPicOrderCntLsb;
      }
      else {
        PicOrderCntMsb = prevPicOrderCntMsb;
      }
    }

  m_curr_img->PicOrderCntVal = PicOrderCntMsb + hdr->slice_pic_order_cnt_lsb;
  m_curr_img->picture_order_cnt_lsb = hdr->slice_pic_order_cnt_lsb;
  m_curr_img->PicLatencyCount = 0;

  loginfo(LogHeaders,"POC computation. new msb:%d POC=%d\n",
          PicOrderCntMsb,
          m_curr_img->PicOrderCntVal);

  if (m_curr_img->nal_hdr.nuh_temporal_id==0 &&
      !isSublayerNonReference(nal_unit_type) &&
      !isRASL(nal_unit_type) &&
      !isRADL(nal_unit_type))
    {
      loginfo(LogHeaders,"set prevPicOrderCntLsb/Msb\n");

      prevPicOrderCntLsb = hdr->slice_pic_order_cnt_lsb;
      prevPicOrderCntMsb = PicOrderCntMsb;
    }
}


/* 8.3.3.2
   Returns DPB index of the generated picture.
 */
int frontend_syntax_decoder::generate_unavailable_reference_picture(const seq_parameter_set* sps,
                                                                    int POC, bool longTerm)
{
  decoded_picture_buffer& dpb = m_decctx->dpb;

  assert(dpb.has_free_dpb_picture(true));

  std::shared_ptr<const seq_parameter_set> current_sps = this->sps[ (int)current_pps->seq_parameter_set_id ];

  de265_image_allocation* alloc_functions = nullptr; // use internal allocation
  int idx = dpb.new_image(current_sps, m_decctx, 0,0, alloc_functions);
  assert(idx>=0);
  //printf("-> fill with unavailable POC %d\n",POC);

  image_ptr img = dpb.get_image(idx);

  img->fill_image(1<<(sps->BitDepth_Y-1),
                  1<<(sps->BitDepth_C-1),
                  1<<(sps->BitDepth_C-1));

  img->fill_pred_mode(MODE_INTRA);

  img->PicOrderCntVal = POC;
  img->picture_order_cnt_lsb = POC & (sps->MaxPicOrderCntLsb-1);
  img->PicOutputFlag = false;
  img->PicState = (longTerm ? UsedForLongTermReference : UsedForShortTermReference);
  img->integrity = INTEGRITY_UNAVAILABLE_REFERENCE;
  img->mFinalCTBProgress = CTB_PROGRESS_SAO;

  img->mark_all_CTB_progress(CTB_PROGRESS_SAO);

  return idx;
}


/* 8.3.2   invoked once per picture

   This function will mark pictures in the DPB as 'unused' or 'used for long-term reference'.
   Note: this function will not mark pictures as unused immediately, but put all unused
   picture IDs into a list. They can be removed from the DPB later with
   remove_images_from_dpb(vector).
*/
void frontend_syntax_decoder::process_reference_picture_set(slice_segment_header* hdr)
{
  decoded_picture_buffer& dpb = m_decctx->dpb;

  std::vector<int> removeReferencesList;

  const int currentID = m_curr_img->get_ID();


  if (isIRAP(nal_unit_type) && NoRaslOutputFlag) {

    int currentPOC = m_curr_img->PicOrderCntVal;

    // reset DPB

    /* The standard says: "When the current picture is an IRAP picture with NoRaslOutputFlag
       equal to 1, all reference pictures currently in the DPB (if any) are marked as
       "unused for reference".

       This seems to be wrong as it also throws out the first CRA picture in a stream like
       RAP_A (decoding order: CRA,POC=64, RASL,POC=60). Removing only the pictures with
       lower POCs seems to be compliant to the reference decoder.
    */

    for (int i=0;i<dpb.size();i++) {
      image_ptr img = dpb.get_image(i);

      if (img->PicState != UnusedForReference &&
          img->PicOrderCntVal < currentPOC &&
          img->removed_at_picture_id > img->get_ID()) {

        removeReferencesList.push_back(img->get_ID());
        img->removed_at_picture_id = img->get_ID();

        //printf("will remove ID %d (a)\n",img->get_ID());
      }
    }
  }


  if (isIDR(nal_unit_type)) {

    // clear all reference pictures

    NumPocStCurrBefore = 0;
    NumPocStCurrAfter = 0;
    NumPocStFoll = 0;
    NumPocLtCurr = 0;
    NumPocLtFoll = 0;
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
          PocStCurrBefore[j++] = m_curr_img->PicOrderCntVal + rps->DeltaPocS0[i];
          //printf("PocStCurrBefore = %d\n",PocStCurrBefore[j-1]);
        }
        else {
          PocStFoll[k++] = m_curr_img->PicOrderCntVal + rps->DeltaPocS0[i];
        }
      }

    NumPocStCurrBefore = j;


    // scan ref-pic-set for larger POCs and fill into PocStCurrAfter / PocStFoll

    for (i=0, j=0;
         i<rps->NumPositivePics;
         i++)
      {
        if (rps->UsedByCurrPicS1[i]) {
          PocStCurrAfter[j++] = m_curr_img->PicOrderCntVal + rps->DeltaPocS1[i];
          //printf("PocStCurrAfter = %d\n",PocStCurrAfter[j-1]);
        }
        else {
          PocStFoll[k++] = m_curr_img->PicOrderCntVal + rps->DeltaPocS1[i];
        }
      }

    NumPocStCurrAfter = j;
    NumPocStFoll = k;


    // find used / future long-term references

    for (i=0, j=0, k=0;
         //i<current_sps->num_long_term_ref_pics_sps + hdr->num_long_term_pics;
         i<hdr->num_long_term_sps + hdr->num_long_term_pics;
         i++)
      {
        int pocLt = PocLsbLt[i];

        if (hdr->delta_poc_msb_present_flag[i]) {
          int currentPictureMSB = m_curr_img->PicOrderCntVal - hdr->slice_pic_order_cnt_lsb;
          pocLt += currentPictureMSB
            - DeltaPocMsbCycleLt[i] * current_sps->MaxPicOrderCntLsb;
        }

        if (UsedByCurrPicLt[i]) {
          PocLtCurr[j] = pocLt;
          CurrDeltaPocMsbPresentFlag[j] = hdr->delta_poc_msb_present_flag[i];
          j++;
        }
        else {
          PocLtFoll[k] = pocLt;
          FollDeltaPocMsbPresentFlag[k] = hdr->delta_poc_msb_present_flag[i];
          k++;
        }
      }

    NumPocLtCurr = j;
    NumPocLtFoll = k;
  }


  // (old 8-99) / (new 8-106)
  // 1.

  std::vector<bool> picInAnyList(dpb.size(), false);


  dpb.log_dpb_content();

  for (int i=0;i<NumPocLtCurr;i++) {
    int k;
    if (!CurrDeltaPocMsbPresentFlag[i]) {
      k = dpb.DPB_index_of_picture_with_LSB(PocLtCurr[i], currentID, true);
    }
    else {
      k = dpb.DPB_index_of_picture_with_POC(PocLtCurr[i], currentID, true);
    }

    RefPicSetLtCurr[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      // TODO, CHECK: is it ok that we generate a picture with POC = LSB (PocLtCurr)
      // We do not know the correct MSB
      int concealedPicture = generate_unavailable_reference_picture(current_sps.get(),
                                                                    PocLtCurr[i], true);
      RefPicSetLtCurr[i] = k = concealedPicture;
      picInAnyList[concealedPicture]=true;
    }

    if (dpb.get_image(k)->integrity != INTEGRITY_CORRECT) {
      m_curr_img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }


  for (int i=0;i<NumPocLtFoll;i++) {
    int k;
    if (!FollDeltaPocMsbPresentFlag[i]) {
      k = dpb.DPB_index_of_picture_with_LSB(PocLtFoll[i], currentID, true);
    }
    else {
      k = dpb.DPB_index_of_picture_with_POC(PocLtFoll[i], currentID, true);
    }

    RefPicSetLtFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      int concealedPicture = k = generate_unavailable_reference_picture(current_sps.get(),
                                                                        PocLtFoll[i], true);
      RefPicSetLtFoll[i] = concealedPicture;
      picInAnyList[concealedPicture]=true;
    }
  }


  // 2. Mark all pictures in RefPicSetLtCurr / RefPicSetLtFoll as UsedForLongTermReference

  for (int i=0;i<NumPocLtCurr;i++) {
    dpb.get_image(RefPicSetLtCurr[i])->PicState = UsedForLongTermReference;
  }

  for (int i=0;i<NumPocLtFoll;i++) {
    dpb.get_image(RefPicSetLtFoll[i])->PicState = UsedForLongTermReference;
  }


  // 3.

  for (int i=0;i<NumPocStCurrBefore;i++) {
    int k = dpb.DPB_index_of_picture_with_POC(PocStCurrBefore[i], currentID);

    //printf("st curr before, poc=%d -> idx=%d\n",PocStCurrBefore[i], k);

    RefPicSetStCurrBefore[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      int concealedPicture = generate_unavailable_reference_picture(current_sps.get(),
                                                                    PocStCurrBefore[i], false);
      RefPicSetStCurrBefore[i] = k = concealedPicture;
      picInAnyList[concealedPicture]=true;

      //printf("  concealed: %d\n", concealedPicture);
    }

    if (dpb.get_image(k)->integrity != INTEGRITY_CORRECT) {
      m_curr_img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }

  for (int i=0;i<NumPocStCurrAfter;i++) {
    int k = dpb.DPB_index_of_picture_with_POC(PocStCurrAfter[i], currentID);

    //printf("st curr after, poc=%d -> idx=%d\n",PocStCurrAfter[i], k);

    RefPicSetStCurrAfter[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
    else {
      int concealedPicture = generate_unavailable_reference_picture(current_sps.get(),
                                                                    PocStCurrAfter[i], false);
      RefPicSetStCurrAfter[i] = k = concealedPicture;
      picInAnyList[concealedPicture]=true;

      //printf("  concealed: %d\n", concealedPicture);
    }

    if (dpb.get_image(k)->integrity != INTEGRITY_CORRECT) {
      m_curr_img->integrity = INTEGRITY_DERIVED_FROM_FAULTY_REFERENCE;
    }
  }

  for (int i=0;i<NumPocStFoll;i++) {
    int k = dpb.DPB_index_of_picture_with_POC(PocStFoll[i], currentID);
    // if (k<0) { assert(false); } // IGNORE

    RefPicSetStFoll[i] = k; // -1 == "no reference picture"
    if (k>=0) picInAnyList[k]=true;
  }

  // 4. any picture that is not marked for reference is put into the "UnusedForReference" state

  for (int i=0;i<dpb.size();i++)
    if (!picInAnyList[i])        // no reference
      {
        image_ptr dpbimg = dpb.get_image(i);
        if (dpbimg != m_curr_img &&  // not the current picture
            dpbimg->removed_at_picture_id > m_curr_img->get_ID()) // has not been removed before
          {
            if (dpbimg->PicState != UnusedForReference) {
              removeReferencesList.push_back(dpbimg->get_ID());
              //printf("will remove ID %d (b)\n",dpbimg->get_ID());

              dpbimg->removed_at_picture_id = m_curr_img->get_ID();
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
bool frontend_syntax_decoder::construct_reference_picture_lists(slice_segment_header* hdr)
{
  decoded_picture_buffer& dpb = m_decctx->dpb;

  int NumPocTotalCurr = hdr->NumPocTotalCurr;
  int NumRpsCurrTempList0 = libde265_max(hdr->num_ref_idx_l0_active, NumPocTotalCurr);

  // TODO: fold code for both lists together

  int RefPicListTemp0[3*MAX_NUM_REF_PICS]; // TODO: what would be the correct maximum ?
  int RefPicListTemp1[3*MAX_NUM_REF_PICS]; // TODO: what would be the correct maximum ?
  char isLongTerm[2][3*MAX_NUM_REF_PICS];

  memset(isLongTerm,0,2*3*MAX_NUM_REF_PICS);

  /* --- Fill RefPicListTmp0 with reference pictures in this order:
     1) short term, past POC
     2) short term, future POC
     3) long term
  */

  int rIdx=0;
  while (rIdx < NumRpsCurrTempList0) {
    for (int i=0;i<NumPocStCurrBefore && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = RefPicSetStCurrBefore[i];

    for (int i=0;i<NumPocStCurrAfter && rIdx<NumRpsCurrTempList0; rIdx++,i++)
      RefPicListTemp0[rIdx] = RefPicSetStCurrAfter[i];

    for (int i=0;i<NumPocLtCurr && rIdx<NumRpsCurrTempList0; rIdx++,i++) {
      RefPicListTemp0[rIdx] = RefPicSetLtCurr[i];
      isLongTerm[0][rIdx] = true;
    }

    // This check is to prevent an endless loop when no images are added above.
    if (rIdx==0) {
      m_decctx->add_warning(DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST, false);
      return false;
    }
  }

  /*
  if (hdr->num_ref_idx_l0_active > 16) {
    add_warning(DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED, false);
    return false;
  }
  */

  assert(hdr->num_ref_idx_l0_active <= 16);
  for (rIdx=0; rIdx<hdr->num_ref_idx_l0_active; rIdx++) {
    int idx = hdr->ref_pic_list_modification_flag_l0 ? hdr->list_entry_l0[rIdx] : rIdx;

    hdr->RefPicList[0][rIdx] = RefPicListTemp0[idx];
    hdr->LongTermRefPic[0][rIdx] = isLongTerm[0][idx];

    // remember POC of referenced image (needed in motion.c, derive_collocated_motion_vector)
    image_ptr img_0_rIdx = dpb.get_image(hdr->RefPicList[0][rIdx]);
    if (!img_0_rIdx) {
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
      for (int i=0;i<NumPocStCurrAfter && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = RefPicSetStCurrAfter[i];
      }

      for (int i=0;i<NumPocStCurrBefore && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = RefPicSetStCurrBefore[i];
      }

      for (int i=0;i<NumPocLtCurr && rIdx<NumRpsCurrTempList1; rIdx++,i++) {
        RefPicListTemp1[rIdx] = RefPicSetLtCurr[i];
        isLongTerm[1][rIdx] = true;
      }

      // This check is to prevent an endless loop when no images are added above.
      if (rIdx==0) {
        m_decctx->add_warning(DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST, false);
        return false;
      }
    }

    if (hdr->num_ref_idx_l0_active > 16) {
      m_decctx->add_warning(DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED, false);
      return false;
    }

    assert(hdr->num_ref_idx_l1_active <= 16);
    for (rIdx=0; rIdx<hdr->num_ref_idx_l1_active; rIdx++) {
      int idx = hdr->ref_pic_list_modification_flag_l1 ? hdr->list_entry_l1[rIdx] : rIdx;

      hdr->RefPicList[1][rIdx] = RefPicListTemp1[idx];
      hdr->LongTermRefPic[1][rIdx] = isLongTerm[1][idx];

      // remember POC of referenced imaged (needed in motion.c, derive_collocated_motion_vector)
      image_ptr img_1_rIdx = dpb.get_image(hdr->RefPicList[1][rIdx]);
      if (!img_1_rIdx) { return false; }
      hdr->RefPicList_POC[1][rIdx] = img_1_rIdx->PicOrderCntVal;
      hdr->RefPicList_PicState[1][rIdx] = img_1_rIdx->PicState;
    }
  }


  // show reference picture lists

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


// returns whether we can continue decoding the stream or whether we should give up
bool frontend_syntax_decoder::process_slice_segment_header(slice_segment_header* hdr,
                                                           de265_error* err, de265_PTS pts,
                                                           nal_header* nal_hdr,
                                                           void* user_data)
{
  *err = DE265_OK;

  flush_reorder_buffer_at_this_frame = false;


  // --- prepare decoding of new picture ---

  if (hdr->first_slice_segment_in_pic_flag) {

    // previous picture has been completely decoded

    //ctx->push_current_picture_to_output_queue();

    current_image_poc_lsb = hdr->slice_pic_order_cnt_lsb;



    if (isIRAP(nal_unit_type)) {
      if (isIDR(nal_unit_type) ||
          isBLA(nal_unit_type) ||
          first_decoded_picture ||
          FirstAfterEndOfSequenceNAL)
        {
          NoRaslOutputFlag = true;
          FirstAfterEndOfSequenceNAL = false;
        }
      else if (0) // TODO: set HandleCraAsBlaFlag by external means
        {
        }
      else
        {
          NoRaslOutputFlag   = false;
          HandleCraAsBlaFlag = false;
        }
    }


    if (isRASL(nal_unit_type) &&
        NoRaslOutputFlag)
      {
        m_curr_img->PicOutputFlag = false;
      }
    else
      {
        m_curr_img->PicOutputFlag = !!hdr->pic_output_flag;
      }

    process_picture_order_count(hdr);

    // mark picture so that it is not overwritten by unavailable reference frames
    m_curr_img->PicState = UsedForShortTermReference;

    process_reference_picture_set(hdr);

    m_curr_img->PicState = UsedForShortTermReference;

    log_set_current_POC(m_curr_img->PicOrderCntVal);


    // check whether RefPicLists erroneously reference current picture

    /*
    for (int i=0;i<hdr->num_ref_idx_l0_active;i++) {
      if (m_decctx->get_image(hdr->RefPicList[0][i]) == m_curr_img) {
        return false;
      }
    }

    for (int i=0;i<hdr->num_ref_idx_l1_active;i++) {
      if (m_decctx->get_image(hdr->RefPicList[1][i]) == m_curr_img) {
        return false;
      }
    }
    */

    // next image is not the first anymore

    first_decoded_picture = false;
  }
  else {
    // claims to be not the first slice, but there is no active image available

    if (m_curr_img == NULL) {
      return false;
    }
  }

  if (hdr->slice_type == SLICE_TYPE_B ||
      hdr->slice_type == SLICE_TYPE_P)
    {
      bool success = construct_reference_picture_lists(hdr);
      if (!success) {
        return false;
      }
    }

  //printf("process slice segment header\n");

  loginfo(LogHeaders,"end of process-slice-header\n");
  m_decctx->dpb.log_dpb_content();


  if (hdr->dependent_slice_segment_flag==0) {
    hdr->SliceAddrRS = hdr->slice_segment_address;
  } else {
    hdr->SliceAddrRS = previous_slice_header->SliceAddrRS;
  }

  previous_slice_header = hdr;


  loginfo(LogHeaders,"SliceAddrRS = %d\n",hdr->SliceAddrRS);

  return true;
}



/*
  .     0     1     2       <- goal_HighestTid
  +-----+-----+-----+
  | -0->| -1->| -2->|
  +-----+-----+-----+
  0     33    66    100     <- framerate_ratio
 */

int  frontend_syntax_decoder::get_highest_TID() const
{
  if (current_sps) { return current_sps->sps_max_sub_layers-1; }
  if (current_vps) { return current_vps->vps_max_sub_layers-1; }

  return 6;
}
