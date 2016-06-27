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

#ifndef DE265_DECCTX_H
#define DE265_DECCTX_H

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
#include "libde265/frontend-syntax-decoder.h"

#include <memory>

#define DE265_MAX_VPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_SPS_SETS 16   // this is the maximum as defined in the standard
#define DE265_MAX_PPS_SETS 64   // this is the maximum as defined in the standard

#define MAX_WARNINGS 20


class slice_segment_header;
class image_unit;
class slice_unit;
class decoder_context;

//typedef std::shared_ptr<image_unit> image_unit_ptr;


class thread_context
{
public:
  thread_context();

  void init_quantization();


  // --- current CTB address --

  int  get_CTB_address_TS() const { return CtbAddrInTS; }
  int  get_CTB_address_RS() const { return CtbAddrInRS; }

  int  get_CTB_x() const { return CtbX; }
  int  get_CTB_y() const { return CtbY; }

  void set_CTB_address_RS(int addr);

  // returns 'true' when we left the image area
  bool advance_CTB_TS();

  bool current_CTB_outside_image() const { return m_CTB_out_of_image; }


  // range of CTBs covered by this thread_context
  int first_CTB_TS;
  int last_CTB_TS;

  void mark_covered_CTBs_as_processed(int progress);


  // --- motion vectors ---

  PBMotionCoding motion;


  // prediction

  // enum IntraPredMode IntraPredModeC[4]; // chroma intra-prediction mode for current CB
  int ResScaleVal;


  // residual data

  uint8_t cu_transquant_bypass_flag;
  uint8_t transform_skip_flag[3];
  uint8_t explicit_rdpcm_flag;
  uint8_t explicit_rdpcm_dir;

  ALIGNED_16(int16_t) _coeffBuf[(32*32)+8]; // alignment required for SSE code !
  int16_t *coeffBuf;

  int16_t coeffList[3][32*32];
  int16_t coeffPos[3][32*32];
  int16_t nCoeff[3];

  int32_t residual_luma[32*32]; // only used when cross-comp-prediction is enabled


  // quantization

  int IsCuQpDeltaCoded;
  int CuQpDelta;
  int IsCuChromaQpOffsetCoded;
  int CuQpOffsetCb, CuQpOffsetCr;

  int currentQPY;
  int currentQG_x, currentQG_y;
  int lastQPYinPreviousQG;

  int qPYPrime, qPCbPrime, qPCrPrime;

  CABAC_decoder cabac_decoder;

  context_model_table ctx_model;
  uint8_t StatCoeff[4];


  // --- decoder pointers ---

  decoder_context* decctx;
  image_ptr img;
  slice_segment_header* shdr;

  image_unit* imgunit;
  slice_unit* sliceunit;
  thread_task_ptr task; // executing thread_task or NULL if not multi-threaded

private:
  bool m_CTB_out_of_image;
  int CtbAddrInTS;  // primary CTB address, this is incremented during decoding
  int CtbAddrInRS;  // derived CTB address in raster scan

  int CtbX, CtbY;

  // Take CtbAddrInTS and compute: CtbAddrInRS, CtbX, CtbY, out-of-image flag.
  void setCtbAddrFromTS();




  thread_context(const thread_context&); // not allowed
  const thread_context& operator=(const thread_context&); // not allowed
};



class error_queue
{
 public:
  error_queue();

  void add_warning(de265_error warning, bool once);
  de265_error get_warning();

 private:
  de265_error warnings[MAX_WARNINGS];
  int nWarnings;
  de265_error warnings_shown[MAX_WARNINGS]; // warnings that have already occurred
  int nWarningsShown;
};



class image_history
{
 public:
  virtual ~image_history() { }

  //virtual /* */ de265_image* get_image(int dpb_index)       { return dpb.get_image(dpb_index); }
  virtual std::shared_ptr<const image> get_image(int frame_id) const = 0;
  virtual bool has_image(int frame_id) const = 0;

  virtual void debug_dump() const { }
};



class base_context : public error_queue,
                     public image_history
{
 public:
  base_context();
  virtual ~base_context() { }

  // --- accelerated DSP functions ---

  void set_acceleration_functions(enum de265_acceleration);

  struct acceleration_functions acceleration; // CPU optimized functions
};



class image_unit_sink;




class decoder_context : public base_context,
                        public image_unit_sink
{
 public:
  decoder_context();
  ~decoder_context();

  de265_error start_thread_pool(int nThreads);
  void        stop_thread_pool();

  thread_pool& get_thread_pool() { return m_thread_pool; }

  int get_num_worker_threads() const { return num_worker_threads; }


  void reset();

  void set_image_allocation_functions(de265_image_allocation* allocfunc);



  // -------------------------------------------------- frontend_syntax_decoder

  NAL_Parser& get_NAL_parser() { return m_frontend_syntax_decoder.get_NAL_parser(); }

  /* */ frontend_syntax_decoder& get_frontend_syntax_decoder()       { return m_frontend_syntax_decoder; }
  const frontend_syntax_decoder& get_frontend_syntax_decoder() const { return m_frontend_syntax_decoder; }



  // -------------------------------------------------- decoding loop ---

  de265_error decode_image_unit(bool* did_work);

  // --- frame-parallel decoding ---

  void start_decoding_thread();
  void stop_decoding_thread();

  int  get_action(bool blocking);

  void on_image_decoding_finished(); // internal use only (by decoding tasks)



  // -------------------------------------------------- output

  std::shared_ptr</* */ image> get_image(int dpb_index)       { return dpb.get_image(dpb_index); }
  std::shared_ptr<const image> get_image(int dpb_index) const { return dpb.get_image(dpb_index); }

  void debug_dump() const { dpb.log_dpb_content(); }


  bool has_image(int dpb_index) const { return dpb_index>=0 && dpb_index<dpb.size(); }

  image_ptr get_next_picture_in_output_queue() { return m_output_queue.get_next_picture_in_output_queue(); }
  int    num_pictures_in_output_queue() const { return m_output_queue.num_pictures_in_output_queue(); }
  void   pop_next_picture_in_output_queue() {
    m_main_loop_mutex.lock();
    m_output_queue.pop_next_picture_in_output_queue();

    m_cond_api_action.signal();
    m_main_loop_mutex.unlock();
  }


  void debug_imageunit_state();


 private:

  frontend_syntax_decoder m_frontend_syntax_decoder;

  // -------------------------------------------------- image_unit classifier

  frame_dropper_nop        m_frame_dropper_nop;
  frame_dropper_IRAP_only  m_frame_dropper_IRAP_only;
  frame_dropper_ratio      m_frame_dropper_ratio;


  // -------------------------------------------------- decoding main loop

  // --- image_unit input queue ---

  virtual void send_image_unit(image_unit_ptr imgunit);
  virtual void send_end_of_stream();


  thread_pool m_thread_pool;
  thread_pool m_master_thread_pool;  // pool for all the master threads

  //std::set<thread_task_ptr> master_tasks;

  de265_error decode_slice_unit_sequential(image_unit* imgunit, slice_unit* sliceunit);
  de265_error decode_slice_unit_frame_parallel(image_unit* imgunit, slice_unit* sliceunit);
  de265_error decode_slice_unit_WPP(image_unit* imgunit, slice_unit* sliceunit);
  de265_error decode_slice_unit_tiles(image_unit* imgunit, slice_unit* sliceunit);


  de265_error push_picture_to_output_queue(image_unit_ptr);


 public:
  // --- parameters ---

  bool param_sei_check_hash;
  bool param_conceal_stream_errors;
  bool param_suppress_faulty_pictures;

  bool param_disable_deblocking;
  bool param_disable_sao;
  //bool param_disable_mc_residual_idct;  // not implemented yet
  //bool param_disable_intra_residual_idct;  // not implemented yet

  void set_frame_dropping_ratio(float ratio);


  de265_image_allocation param_image_allocation_functions;



 private:
  int num_worker_threads;  // input parameter: the number of desired worker threads


  // --- main loop ---

  class thread_main_loop : public de265_thread {
  public:
    thread_main_loop(decoder_context* dctx) : m_decctx(dctx) { }
    void run() { m_decctx->run_main_loop(); }

  private:
    decoder_context* m_decctx;
  };

  thread_main_loop m_main_loop_thread;
  de265_mutex m_main_loop_mutex;
  //de265_cond  m_decoding_loop_has_space_cond;
  //de265_cond  m_input_available_cond;
  de265_cond m_main_loop_block_cond;

  void run_main_loop();

  void send_main_loop_stop_signals() {
    m_main_loop_mutex.lock();
    //m_decoding_loop_has_space_cond.signal();
    //m_input_available_cond.signal();
    m_main_loop_block_cond.signal();
    m_main_loop_mutex.unlock();
  }


  void decode_image_frame_parallel(image_unit_ptr imgunit);


 public:
  // --- decoded picture buffer ---
  decoded_picture_buffer dpb;

 private:
  // --- image unit queue ---

  //public:
  std::deque<image_unit_ptr> m_undecoded_image_units;
  bool m_end_of_stream;

  std::deque<image_unit_ptr> m_image_units_in_progress;
  static const int m_max_images_processed_in_parallel = 10; //////////////////// PARAMETER

  picture_output_queue   m_output_queue;


  // condition variable that signals when api-user action might change
  de265_cond  m_cond_api_action;



 private:
  void mark_whole_slice_as_processed(slice_unit* sliceunit, int progress);

  void remove_images_from_dpb(const std::vector<int>& removeImageList);
  void run_postprocessing_filters_sequential(image_ptr img);
  void run_postprocessing_filters_parallel(image_unit* img);

 public:
  // --- frame dropping ---

  void set_limit_TID(int tid);
  int  get_current_TID() const { return current_HighestTid; }
  int  change_framerate(int more_vs_less); // 1: more, -1: less
  void set_framerate_ratio(int percent);

 private:
  // input parameters
  int limit_HighestTid;    // never switch to a layer above this one
  int framerate_ratio;

  // current control parameters
  int goal_HighestTid;     // this is the layer we want to decode at
  int layer_framerate_ratio; // ratio of frames to keep in the current layer

  int current_HighestTid;  // the layer which we are currently decoding

  struct {
    int8_t tid;
    int8_t ratio;
  } framedrop_tab[100+1];
  int framedrop_tid_index[6+1];

  void compute_framedrop_table();

 public:
  void calc_tid_and_framerate_ratio();
};


#endif
