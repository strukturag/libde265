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

#ifndef DE265_ERROR_H
#define DE265_ERROR_H

#include "de265.h"

#include <mutex>
#include <deque>
#include <map>


class ErrorBuffer
{
public:
  ErrorBuffer() { }
  ~ErrorBuffer() { }

  de265_error add(const void* context, de265_error_code code, std::string message="");
  de265_error add(de265_error_code code, std::string message="");

  void set_context(de265_error err, void* context);

  void append_message(de265_error err, std::string message);

  // the context has been closed, we do not need its error messages anymore
  void end_context(void* context);

  std::string get_message(de265_error err) const;
  std::string get_recursive_message(de265_error err) const;
  de265_error_code get_error_code(de265_error err) const;

  static constexpr de265_error ok { };

private:
  mutable std::mutex m_mutex;

  struct error_item {
    de265_error id;
    const void* context; // the context this error belongs to

    de265_error_code code;
    std::string message;
  };

  uint32_t m_next_error_id = 2; // TODO: change type again to de265_error

  // TODO: this bit is set on error_ids that are warnings
  constexpr static uint32_t c_IS_WARNING_BIT = 0x0001;

  std::deque<error_item> m_errors;

  std::map<const void*, int> m_context_usage;

  constexpr static int c_MIN_CONTEXT_MESSAGES = 10;
  constexpr static int c_MAX_MESSAGES = 50;

  constexpr static const char* c_UNKNOWN_ERROR_MESSAGE = "unknown error";

  void remove_message_at_index(int purge_idx);
};


extern ErrorBuffer errors;


bool error_code_matches(de265_error_code error, de265_error_code base_code);

de265_error_code get_parent_error_code(de265_error_code error);

const char* get_standard_error_message(de265_error_code error);

#endif
