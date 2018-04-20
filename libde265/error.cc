/*
 * H.265 video codec.
 * Copyright (c) 2013-2017 struktur AG, Dirk Farin <farin@struktur.de>
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

#include "error.h"


ErrorBuffer errors;


de265_error ErrorBuffer::add(de265_error_code code, std::string message)
{
  return add(nullptr, code, message);
}


de265_error ErrorBuffer::add(const void* context, de265_error_code code, std::string message)
{
  m_mutex.lock();

  // --- if error queue is getting large, remove oldest error

  int purge_idx=0;
  while (m_errors.size() > c_MAX_MESSAGES-1) {

    // reached end of message queue -> quit
    if (purge_idx == m_errors.size()) {
      break;
    }

    // if we can remove the message at purge_idx, do so.
    // if not, try the next message
    const void* purge_context = m_errors[purge_idx].context;
    if (purge_context == nullptr ||
        m_context_usage[purge_context] > c_MIN_CONTEXT_MESSAGES) {
      remove_message_at_index(purge_idx);
    }
    else {
      purge_idx++;
    }
  }


  // --- add the new error

  struct error_item item;
  item.id.id = m_next_error_id;
  item.context = context;
  item.code = code;
  item.message = message;
  m_errors.push_back(item);

  m_context_usage[context]++;

  m_next_error_id+=2;

  m_mutex.unlock();

  return item.id;
}


void ErrorBuffer::remove_message_at_index(int purge_idx)
{
  const void* purge_context = m_errors[purge_idx].context;

  if (purge_context != nullptr) {
    m_context_usage[purge_context]--;
  }


  if (purge_idx==0) {
    m_errors.pop_front();
  }
  else {
    for (int k=purge_idx+1; k<m_errors.size(); k++) {
      m_errors[k-1] = m_errors[k];
    }

    m_errors.pop_back();
  }
}


void ErrorBuffer::set_context(de265_error err, void* context)
{
  m_mutex.lock();

  for (int i=0; i<m_errors.size(); i++) {
    if (m_errors[i].id.id == err.id) {
      if (m_errors[i].context) {
        m_context_usage[ m_errors[i].context ]--;
      }

      m_errors[i].context = context;

      if (context) {
        m_context_usage[ context ]++;
      }

      break;
    }
  }

  m_mutex.unlock();
}


void ErrorBuffer::append_message(de265_error err, std::string message)
{
  m_mutex.lock();

  for (int i=0; i<m_errors.size(); i++) {
    if (m_errors[i].id.id == err.id) {
      m_errors[i].message += ' ';
      m_errors[i].message += message;
      break;
    }
  }

  m_mutex.unlock();
}


void ErrorBuffer::end_context(void* context)
{
  m_mutex.lock();

  int purge_idx = 0;
  while (purge_idx<m_errors.size()) {
    if (m_errors[purge_idx].context == context) {
      remove_message_at_index(purge_idx);
    }
    else {
      purge_idx++;
    }
  }

  m_mutex.unlock();
}


std::string ErrorBuffer::get_message(de265_error err) const
{
  m_mutex.lock();

  for (int i=0; i<m_errors.size(); i++) {
    if (m_errors[i].id.id == err.id) {
      std::string message = m_errors[i].message;

      if (message.empty()) {
        message = get_standard_error_message(m_errors[i].code);
      }

      m_mutex.unlock();
      return message;
    }
  }

  m_mutex.unlock();

  return c_UNKNOWN_ERROR_MESSAGE;
}


std::string ErrorBuffer::get_recursive_message(de265_error err) const
{
  m_mutex.lock();

  for (int i=0; i<m_errors.size(); i++) {
    if (m_errors[i].id.id == err.id) {
      std::string message = m_errors[i].message;
      de265_error_code code = m_errors[i].code;
      m_mutex.unlock();

      while (code != DE265_ERROR_UNKNOWN) {
        if (message.empty()) {
          message = get_standard_error_message(code);
        }
        else {
          std::string std_message = get_standard_error_message(code);
          message = std_message + ": " + message;
        }

        code = get_parent_error_code(code);
      }

      return message;
    }
  }

  m_mutex.unlock();

  return c_UNKNOWN_ERROR_MESSAGE;
}


de265_error_code ErrorBuffer::get_error_code(de265_error err) const
{
  m_mutex.lock();

  for (int i=0; i<m_errors.size(); i++) {
    if (m_errors[i].id.id == err.id) {
      de265_error_code code = m_errors[i].code;
      m_mutex.unlock();
      return code;
    }
  }

  m_mutex.unlock();

  return DE265_ERROR_UNKNOWN;
}


bool error_code_matches(de265_error_code error, de265_error_code base_code)
{
  if (error==base_code) {
    return true;
  }

  if (error != DE265_ERROR_UNKNOWN) {
    // check base error class

    de265_error_code parent = get_parent_error_code(error);
    return error_code_matches(parent, base_code);
  }
  else {
    return false;
  }
}


// TODO: move to get_parent error_code, and then use this to produce multi-part error messages:
// "corrupt stream: invalid sps header: invalid chroma format"
de265_error_code get_parent_error_code(de265_error_code error)
{
  switch (error) {
  case DE265_ERROR_INVALID_VUI_HEADER:
  case DE265_ERROR_INVALID_SPS_HEADER:
  case DE265_ERROR_INVALID_PPS_HEADER:
  case DE265_ERROR_INVALID_VPS_HEADER:
  case DE265_ERROR_INVALID_SLICE_HEADER:
  case DE265_ERROR_NONEXISTING_REFERENCE_PICTURE_ACCESSED:
  case DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA:
  case DE265_ERROR_INVALID_REF_PIC_SET:
  case DE265_ERROR_SEI_CHECKSUM_MISMATCH:
    return DE265_ERROR_CORRUPT_STREAM;

  case DE265_ERROR_INVALID_CHROMA_FORMAT:
    return DE265_ERROR_INVALID_SPS_HEADER;

  case DE265_ERROR_NONEXISTING_PPS_REFERENCED:
    return DE265_ERROR_INVALID_SLICE_HEADER;

  case DE265_ERROR_NONEXISTING_SPS_REFERENCED:
    return DE265_ERROR_INVALID_PPS_HEADER;
  }

  return DE265_ERROR_UNKNOWN;
}


const char* get_standard_error_message(de265_error_code error)
{
  switch (error) {
  case DE265_OK: return "no error";

  case DE265_ERROR_UNKNOWN:
    return "unknown error";
  case DE265_ERROR_CORRUPT_STREAM:
    return "corrupt input";
  case DE265_ERROR_PARAMETER_PARSING:
    return "command-line parameter error";

  case DE265_ERROR_OUT_OF_MEMORY:
    return "out of memory";

  case DE265_ERROR_INVALID_VUI_HEADER:
    return "VUI header parameter invalid";
  case DE265_ERROR_INVALID_SPS_HEADER:
    return "SPS header parameter invalid";
  case DE265_ERROR_INVALID_PPS_HEADER:
    return "PPS header parameter invalid";
  case DE265_ERROR_INVALID_VPS_HEADER:
    return "VPS header parameter invalid";
  case DE265_ERROR_INVALID_SLICE_HEADER:
    return "slice header parameter invalid";
  case DE265_ERROR_NONEXISTING_REFERENCE_PICTURE_ACCESSED:
    return "non-existing reference picture accessed";
  case DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA:
    return "CTB outside of image area";
  case DE265_ERROR_PREMATURE_END_OF_SLICE:
    return "premature end of slice data";
  case DE265_ERROR_INVALID_CHROMA_FORMAT:
    return "invalid chroma format";
  case DE265_ERROR_NONEXISTING_SPS_REFERENCED:
    return "non-existing SPS referenced";
  case DE265_ERROR_NONEXISTING_PPS_REFERENCED:
    return "non-existing PPS referenced";
  case DE265_ERROR_CANNOT_START_THREADS:
    return "cannot start threads";
  case DE265_ERROR_SEI_CHECKSUM_MISMATCH:
    return "SEI image checksum mismatch";


    /*
  case DE265_ERROR_LIBRARY_NOT_INITIALIZED: return "library is not initialized";

  case DE265_ERROR_MANDATORY_FUNCTIONALITY_NOT_IMPLEMENTED_YET:
    return "unimplemented mandatory decoder feature";
  case DE265_WARNING_OPTIONAL_FUNCTIONALITY_NOT_IMPLEMENTED_YET:
    return "unimplemented optional decoder feature";

  case DE265_WARNING_DEPENDENT_SLICE_WITHOUT_INITIAL_SLICE_HEADER:
    return "initial slice header missing, cannot decode dependent slice";

  case DE265_WARNING_WARNING_BUFFER_FULL:
    return "Too many warnings queued";
  case DE265_WARNING_INCORRECT_ENTRY_POINT_OFFSET:
    return "Incorrect entry-point offsets";
  case DE265_WARNING_SLICEHEADER_INVALID:
    return "slice header invalid";
  case DE265_WARNING_INCORRECT_MOTION_VECTOR_SCALING:
    return "impossible motion vector scaling";
  case DE265_WARNING_BOTH_PREDFLAGS_ZERO:
    return "both predFlags[] are zero in MC";
  case DE265_WARNING_NUMMVP_NOT_EQUAL_TO_NUMMVQ:
    return "numMV_P != numMV_Q in deblocking";
  case DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE:
    return "number of short-term ref-pic-sets out of range";
  case DE265_WARNING_SHORT_TERM_REF_PIC_SET_OUT_OF_RANGE:
    return "short-term ref-pic-set index out of range";
  case DE265_WARNING_SHORT_TERM_REF_PIC_SET_PARAMETER_OUT_OF_RANGE:
    return "parameter in short-term ref-pic-set is out of range";
  case DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST:
    return "faulty reference picture list";
  case DE265_WARNING_END_OF_SUBSTREAM_BIT_NOT_SET:
    return "end_of_sub_stream_one_bit not set to 1 when it should be";
  case DE265_WARNING_SLICE_SEGMENT_ADDRESS_INVALID:
    return "slice segment address invalid";
  case DE265_WARNING_DEPENDENT_SLICE_WITH_ADDRESS_ZERO:
    return "dependent slice with address 0";
  case DE265_WARNING_INVALID_LT_REFERENCE_CANDIDATE:
    return "invalid long-term reference candidate specified in slice header";
  case DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY:
    return "cannot apply SAO because we ran out of memory";
  case DE265_WARNING_CANNOT_DECODE_SEI_BECAUSE_SPS_IS_MISSING:
    return "cannot decode SEI because SPS header is missing";
  case DE265_WARNING_COLLOCATED_MOTION_VECTOR_OUTSIDE_IMAGE_AREA:
    return "collocated motion-vector is outside image area";
  case DE265_WARNING_DECODING_ERROR:
    return "noncritical error occurred during decoding";
    */

  default: return "unknown error message";
  }
}
