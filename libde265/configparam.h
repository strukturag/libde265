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
#include <string>
#include <stddef.h>


class choice_option
{
 public:
 choice_option() : validValue(false) { }

  // --- initialization ---

  void addChoice(const std::string& s, int id, bool default_value=false) {
    choices.push_back( std::make_pair(s,id) );
    if (default_value) { setValue(s); }
  }


  // --- usage ---

  bool setValue(const std::string& val); // returns false if it is not a valid option
  bool isValidValue() const { return validValue; }

  const std::string& getValue() const { return selectedValue; }
  const int getID() const { return selectedID; }
  const std::vector< std::pair<std::string,int> >& getChoices() const { return choices; }

 private:
  std::vector< std::pair<std::string,int> > choices;
  std::string selectedValue;
  int selectedID;
  bool validValue;
};



class config_parameters
{
 public:
  static const int NO_LIMIT = INT_MAX;

  void register_config_int(const char* name, char short_option, size_t offset,
                           int default_value,
                           int low_limit = NO_LIMIT, int high_limit = NO_LIMIT);

  void register_config_string(const char* name, char short_option, size_t offset,
                              const char* default_value);

  void register_config_choice(const char* name, char short_option, size_t offset);

  void show_params(void* params) const;

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

    enum { Config_Int, Config_Bool, Config_String, Config_Choice } type;

    int int_low_limit;
    int int_high_limit;

    size_t offset;

    bool set_value(const char* value, void* dst, const char* name) const;
  };


  std::vector<config_param> params;
};

void register_encoder_params(config_parameters* config);

#endif
