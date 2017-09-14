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

#ifndef DE265_ENCPICBUF_H
#define DE265_ENCPICBUF_H

#include "libde265/image.h"
#include "libde265/sps.h"
#include "libde265/encoder/encoder-types.h"

#include <deque>
#include <vector>


class encoder_picture_buffer;

/* TODO: we need a way to quickly access pictures with a stable ID, like in the DPB.
 */


// This data record collects all encoding related data for a picture:
// - the input image
// - the reconstructed (decoded) image, used for motion compensation
// - ...
struct picture_encoding_data
{
  picture_encoding_data(class encoder_picture_buffer* encpicbuf); // DEPRECATED

  picture_encoding_data(image_ptr img,
                        int frame_number,
                        std::shared_ptr<video_parameter_set>& vps,
                        std::shared_ptr<seq_parameter_set>& sps,
                        std::shared_ptr<pic_parameter_set>& pps,
                        int ctb_size_log2);

  ~picture_encoding_data();


  void setEncPicBuf(encoder_picture_buffer* encpicbuf) { mEncPicBuf = encpicbuf; }

  // --- picture encoding state ---

  // change state by notifying it that SOP metadata has been set
  void mark_sop_metadata_set();

  void mark_encoding_started();
  void mark_encoding_finished();

  void set_reconstruction_image(image_ptr);

  //void set_prediction_image(int frame_number, image_ptr); // store it just for debugging fun





  // --- the current processing state of this picture ---

  /* unprocessed              only input image has been inserted, no metadata
     sop_metadata_available   sop-creator has filled in references and skipping metadata
     a) encoding              encoding started for this frame, reconstruction image was created
     .  keep_for_reference    encoding finished, picture is kept in the buffer for reference
     b) skipped               image was skipped, no encoding was done, no reconstruction image
  */
  enum state {
    state_unprocessed,
    state_sop_metadata_available,
    state_encoding,
    state_keep_for_reference,
    state_skipped
  } state;


  bool in_use;



  // --- picture data ---

  int frame_number;

  std::shared_ptr<const image> input;    // unmodified input image
  std::shared_ptr<image> reconstruction; // decoded image, used for motion compensation

  //std::shared_ptr<image> prediction;  // this is only used for debugging

  CTBTreeMatrix ctbs;


  // --- headers ---

  //std::shared_ptr<pic_parameter_set> pps;

  nal_header nal; // TODO: image split into several NALs (always same NAL header?)

  // The vps/sps/pps headers will be set by the SOPCreator. When encoding a picture,
  // the encoder_context will check whether this header has to be coded in the bitstream or
  // whether it is already active. It will also ask the EncoderCore to set additional fields.
  // Finally, it will choose an ID for the header, code it in the bitstream and store it in
  // its encoder_context header arrays.
  std::shared_ptr<video_parameter_set> vps;
  std::shared_ptr<seq_parameter_set>   sps;
  std::shared_ptr<pic_parameter_set>   pps;


  // --- SOP metadata ---

  /* TODO */
  std::vector<int> ref0;
  std::vector<int> ref1;
  std::vector<int> longterm;
  std::vector<int> keep;
  int sps_index;
  int skip_priority;  // unused


  // --- SOP structure ---

  /* TODO */
  //void set_intra();
  void set_NAL_type(uint8_t nalType);
  void set_NAL_temporal_id(int temporal_id);
  void set_references(int sps_index, // -1 -> custom
                      const std::vector<int>& l0, const std::vector<int>& l1,
                      const std::vector<int>& lt,
                      const std::vector<int>& keepMoreReferences);
  void set_skip_priority(int skip_priority);


  // Input images can be released after encoding and when the output packet is released.
  // This is important to do as soon as possible, as the image might actually wrap
  // scarce resources like camera picture buffers.
  // This function does release (only) the raw input data.
  void release_input_image() { input.reset(); }


  std::deque<en265_packet*> output_packets;

private:
  class encoder_picture_buffer* mEncPicBuf;
};



// The encoder_picture_buffer is a queue of pictures to be encoded.
// Pictures are inserted into this queue in encoding order and removed after they are not
// used anymore for reference by other pictures.
// At the end of the input picture stream, an end-of-input flag can be pushed to indicate
// to the encoder that no more pictures will follow.

class encoder_picture_buffer
{
 public:
  encoder_picture_buffer();
  ~encoder_picture_buffer();


  // clear picture queue and also clear end-of-stream flag
  void clear();


  // --- input pushed by the input process ---

  void insert_next_image_in_encoding_order(std::shared_ptr<picture_encoding_data> picdata);

  void insert_end_of_input();


  // --- data access ---

  bool have_more_frames_to_encode() const;

  // get next picture to be encoded or return NULL if no more picture is available
  std::shared_ptr<picture_encoding_data> get_next_picture_to_encode();

  std::shared_ptr<const picture_encoding_data> get_picture(int frame_number) const;

  bool has_picture(int frame_number) const;



  // --- internal use only ---

  void purge_unused_images_from_queue();

 private:
  bool mEndOfInput;
  std::deque< std::shared_ptr<picture_encoding_data> > mImages;

  std::shared_ptr<picture_encoding_data> get_picture(int frame_number);
};


#endif
