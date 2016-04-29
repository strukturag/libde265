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

#ifndef DE265_FRONTEND_SYNTAX_DECODER_H
#define DE265_FRONTEND_SYNTAX_DECODER_H

#include "libde265/vps.h"
#include "libde265/sps.h"
#include "libde265/pps.h"
#include "libde265/nal.h"
#include "libde265/slice.h"
#include "libde265/image.h"
#include "libde265/motion.h"
#include "libde265/de265.h"
#include "libde265/dpb.h"
#include "libde265/sei.h"
#include "libde265/threads.h"
#include "libde265/acceleration.h"
#include "libde265/nal-parser.h"
#include "libde265/image-unit.h"
#include "libde265/frame-dropper.h"

#include <memory>

#define DE265_MAX_VPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_SPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_PPS_SETS 64   // this is the maximum as defined in the standard

#define MAX_WARNINGS 20


class slice_segment_header;
class image_unit;
class slice_unit;
class decoder_context;


class image_unit_sink;


class frontend_syntax_decoder : private on_NAL_inserted_listener
{
 public:
  frontend_syntax_decoder(decoder_context* ctx);

  void reset();


  // --- frontend for pushing data into the decoder ---

  bool is_input_buffer_full() const;

  // Pushing data into the NAL-parser will automatically forward the complete NALs
  // the the frontend_syntax_decoder, where they are subsequently combines into image_units.
  NAL_Parser& get_NAL_parser() { return nal_parser; }


  // Complete image_units are forwarded to this sink.
  void set_image_unit_sink(image_unit_sink* sink) { m_image_unit_sink = sink; }



  // --- query the current state ---

  bool has_sps(int id) const { return (bool)sps[id]; }
  bool has_pps(int id) const { return (bool)pps[id]; }

  /* */ seq_parameter_set* get_sps(int id)       { return sps[id].get(); }
  const seq_parameter_set* get_sps(int id) const { return sps[id].get(); }
  /* */ pic_parameter_set* get_pps(int id)       { return pps[id].get(); }
  const pic_parameter_set* get_pps(int id) const { return pps[id].get(); }
  std::shared_ptr<const seq_parameter_set> get_sps_ptr(int id) const { return sps[id]; }
  std::shared_ptr<const pic_parameter_set> get_pps_ptr(int id) const { return pps[id]; }

  std::shared_ptr<seq_parameter_set>  get_current_sps() { return current_sps; }

  // get highest temporal sub-layer ID
  int  get_highest_TID() const;


  // The previously decoded slice header (so that we can copy it in dependent slices)
  bool has_previous_slice_header() const { return previous_slice_header != NULL; }
  const slice_segment_header& get_previous_slice_header() { return *previous_slice_header; }


  // --- modify state by slice header ---

  void set_PocLsbLt(int picIdx, int lsb) { PocLsbLt[picIdx] = lsb; }
  void set_UsedByCurrPicLt(int picIdx, int used) { UsedByCurrPicLt[picIdx] = used; }
  void set_DeltaPocMsbCycleLt(int picIdx, int delta) { DeltaPocMsbCycleLt[picIdx] = delta; }

  bool is_UsedByCurrPicLt(int picIdx) { return UsedByCurrPicLt[picIdx]; }
  int  get_DeltaPocMsbCycleLt(int picIdx) { return DeltaPocMsbCycleLt[picIdx]; }


  // --- debugging ---

  int  param_sps_headers_fd;
  int  param_vps_headers_fd;
  int  param_pps_headers_fd;
  int  param_slice_headers_fd;



  // --- (TODO) make this private and reorganize ---

  void debug_imageunit_state();

 private:
  de265_error decode_NAL(NAL_unit_ptr nal);

  de265_error read_vps_NAL(bitreader&);
  de265_error read_sps_NAL(bitreader&);
  de265_error read_pps_NAL(bitreader&);
  de265_error read_sei_NAL(bitreader& reader, bool suffix);
  de265_error read_eos_NAL(bitreader& reader);
  de265_error read_slice_NAL(bitreader&, NAL_unit_ptr nal, nal_header& nal_hdr);

  bool process_slice_segment_header(slice_segment_header*,
                                    de265_error*, de265_PTS pts,
                                    nal_header* nal_hdr, void* user_data);


  // ---

  decoder_context* m_decctx;
  image_unit_sink* m_image_unit_sink;


  // --- input stream data ---

  int  current_image_poc_lsb;
  bool first_decoded_picture;
  bool NoRaslOutputFlag;
  bool HandleCraAsBlaFlag;
  bool FirstAfterEndOfSequenceNAL;

  bool flush_reorder_buffer_at_this_frame;

  int PicOrderCntMsb;
  int prevPicOrderCntLsb;  // at precTid0Pic
  int prevPicOrderCntMsb;  // at precTid0Pic

  const slice_segment_header* previous_slice_header; /* Remember the last slice for a successive
                                                        dependent slice. */


  // --- motion compensation ---

  int PocLsbLt[MAX_NUM_REF_PICS];
  int UsedByCurrPicLt[MAX_NUM_REF_PICS];
  int DeltaPocMsbCycleLt[MAX_NUM_REF_PICS];

  int CurrDeltaPocMsbPresentFlag[MAX_NUM_REF_PICS];
  int FollDeltaPocMsbPresentFlag[MAX_NUM_REF_PICS];

  // The number of entries in the lists below.
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
  int RefPicSetStCurrBefore[MAX_NUM_REF_PICS];
  int RefPicSetStCurrAfter[MAX_NUM_REF_PICS];
  int RefPicSetStFoll[MAX_NUM_REF_PICS];
  int RefPicSetLtCurr[MAX_NUM_REF_PICS];
  int RefPicSetLtFoll[MAX_NUM_REF_PICS];

  void process_picture_order_count(slice_segment_header* hdr);
  int  generate_unavailable_reference_picture(const seq_parameter_set* sps,
                                              int POC, bool longTerm);
  void process_reference_picture_set(slice_segment_header* hdr);
  bool construct_reference_picture_lists(slice_segment_header* hdr);



  // --- parameters derived from parameter sets ---


  // --- current NAL ---

  NAL_Parser nal_parser;

  uint8_t nal_unit_type;

  //char IdrPicFlag; // unused ?
  char RapPicFlag;


  // --- building the next image_unit ---

  image_unit_ptr m_curr_image_unit;
  image_ptr m_curr_img;


  // --- internal data ---

  std::shared_ptr<video_parameter_set>  vps[ DE265_MAX_VPS_SETS ];
  std::shared_ptr<seq_parameter_set>    sps[ DE265_MAX_SPS_SETS ];
  std::shared_ptr<pic_parameter_set>    pps[ DE265_MAX_PPS_SETS ];

  std::shared_ptr<video_parameter_set>  current_vps;
  std::shared_ptr<seq_parameter_set>    current_sps;
  std::shared_ptr<pic_parameter_set>    current_pps;

  // on_NAL_inserted_listener

  virtual de265_error on_NAL_inserted();
  virtual void on_end_of_stream();
  virtual void on_end_of_frame();
};


#endif
