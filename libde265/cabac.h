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

#ifndef DE265_CABAC_H
#define DE265_CABAC_H

#include <stdint.h>
#include "contextmodel.h"


class CABAC_decoder {
public:
  void init(uint8_t* bitstream, int length);
  void init_CABAC();
  int  decode_bit(context_model* model);
  int  decode_TU(int cMax, context_model* model);
  int  decode_term_bit();

  int  decode_bypass();
  int  decode_TU_bypass(int cMax);
  uint32_t  decode_FL_bypass(int nBits);
  int  decode_TR_bypass(int cRiceParam, int cTRMax);
  uint32_t  decode_EGk_bypass(int k);

  uint8_t* bitstream_start = nullptr;
  uint8_t* bitstream_curr = nullptr;
  uint8_t* bitstream_end = nullptr;

private:
  uint32_t range = 0;
  uint32_t value = 0;
  int16_t  bits_needed = 0;

  int  decode_FL_bypass_parallel(int nBits);
};


// ---------------------------------------------------------------------------

class CABAC_encoder
{
public:
 CABAC_encoder() : mCtxModels(nullptr) { }
  virtual ~CABAC_encoder() { }

  virtual int size() const = 0;
  virtual void reset() = 0;

  // --- VLC ---

  virtual void write_bits(uint32_t bits,int n) = 0;
  virtual void write_bit(int bit) { write_bits(bit,1); }
  virtual void write_uvlc(int value);
  virtual void write_svlc(int value);
  virtual bool write_startcode() = 0;
  virtual void skip_bits(int nBits) = 0;

  virtual void add_trailing_bits();
  virtual int  number_free_bits_in_byte() const = 0;

  // output all remaining bits and fill with zeros to next byte boundary
  virtual void flush_VLC() { }


  // --- CABAC ---

  void set_context_models(context_model_table* models) { mCtxModels=models; }

  virtual void init_CABAC() { }
  virtual void write_CABAC_bit(int modelIdx, int bit) = 0;
  virtual void write_CABAC_bypass(int bit) = 0;
  virtual void write_CABAC_TU_bypass(int value, int cMax);
  virtual void write_CABAC_FL_bypass(int value, int nBits);
  virtual void write_CABAC_term_bit(int bit) = 0;
  virtual void flush_CABAC()  { }

  void write_CABAC_EGk(int absolute_symbol, int k); // absolute_symbol >= 0

  virtual bool modifies_context() const = 0;

  float RDBits_for_CABAC_bin(int modelIdx, int bit);

 protected:
  context_model_table* mCtxModels;
};


class CABAC_encoder_bitstream : public CABAC_encoder
{
public:
  CABAC_encoder_bitstream();
  ~CABAC_encoder_bitstream();

  void reset() override;

  int size() const override { return data_size; }
  uint8_t* data() const { return data_mem; }

  // --- VLC ---

  void write_bits(uint32_t bits,int n) override;
  bool write_startcode() override;
  void skip_bits(int nBits) override;

  int  number_free_bits_in_byte() const override;

  // output all remaining bits and fill with zeros to next byte boundary
  void flush_VLC() override;


  // --- CABAC ---

  void init_CABAC() override;
  void write_CABAC_bit(int modelIdx, int bit) override;
  void write_CABAC_bypass(int bit) override;
  void write_CABAC_term_bit(int bit) override;
  void flush_CABAC() override;

  bool modifies_context() const override { return true; }

private:
  // data buffer

  uint8_t* data_mem = nullptr;
  uint32_t data_capacity = 0;
  uint32_t data_size = 0;
  char     state = 0; // for inserting emulation-prevention bytes

  // VLC

  uint32_t vlc_buffer;
  uint32_t vlc_buffer_len = 0;


  // CABAC

  uint32_t range;
  uint32_t low;
  int8_t   bits_left;
  uint8_t  buffered_byte;
  uint16_t num_buffered_bytes;


  bool check_size_and_resize(int nBytes);
  void testAndWriteOut();
  void write_out();
  bool append_byte(int byte);
};


class CABAC_encoder_estim : public CABAC_encoder
{
public:
  CABAC_encoder_estim() : mFracBits(0) { }

  void reset() override { mFracBits=0; }

  int size() const override { return mFracBits>>(15+3); }

  uint64_t getFracBits() const { return mFracBits; }
  float    getRDBits() const { return mFracBits / float(1<<15); }

  // --- VLC ---

  void write_bits(uint32_t bits,int n) override { mFracBits += n<<15; }
  void write_bit(int bit) override { mFracBits+=1<<15; }
  bool write_startcode() override { mFracBits += (1<<15)*8*3; return true; }
  void skip_bits(int nBits) override { mFracBits += nBits<<15; }
  int  number_free_bits_in_byte() const override { return 0; } // TODO, good enough for now

  // --- CABAC ---

  void write_CABAC_bit(int modelIdx, int bit) override;
  void write_CABAC_bypass(int bit) override {
    mFracBits += 0x8000;
  }
  void write_CABAC_FL_bypass(int value, int nBits) override {
    mFracBits += nBits<<15;
  }
  void write_CABAC_term_bit(int bit) override { /* not implemented (not needed) */ }

  bool modifies_context() const override { return true; }

 protected:
  uint64_t mFracBits;
};


class CABAC_encoder_estim_constant : public CABAC_encoder_estim
{
 public:
  void write_CABAC_bit(int modelIdx, int bit) override;

  bool modifies_context() const override { return false; }
};

#endif
