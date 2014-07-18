/*
 * H.265 video codec.
 * Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>
 *
 * Authors: struktur AG, Dirk Farin <farin@struktur.de>
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

#ifndef CONFIG_PARAM_H
#define CONFIG_PARAM_H

#include <climits>
#include <vector>
#include <stddef.h>


class config_parameters
{
 public:
  static const int NO_LIMIT = INT_MAX;

  void register_config_int(const char* name, char short_option, size_t offset,
                           int default_value,
                           int low_limit = NO_LIMIT, int high_limit = NO_LIMIT);

  void register_config_string(const char* name, char short_option, size_t offset,
                              const char* default_value);

  void show_params() const;

  void set_defaults(void* dst);
  bool parse_command_line_params(int* argc, char** argv,  void* dst,
                                 bool ignore_unknown_options=false);

 private:
  struct config_param
  {
    const char* name;
    char  short_option; // 0 if no short option

    union {
      int         int_default;
      const char* string_default;
      bool        bool_default;
    };

    enum { Config_Int, Config_Bool, Config_String } type;

    int int_low_limit;
    int int_high_limit;

    size_t offset;

    bool set_value(const char* value, void* dst, const char* name) const;
  };


  std::vector<config_param> params;
};

void register_encoder_params(config_parameters* config);

#endif
