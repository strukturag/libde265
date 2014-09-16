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

#include "configparam.h"

#include "encoder-params.h"
#include <ctype.h>
#include <sstream>
#include <iomanip>
#include <iostream>


bool choice_option::setValue(const std::string& val)
{
  for (int i=0;i<choices.size();i++) {
    if (val == choices[i].first) {
      validValue = true;
      selectedID = choices[i].second;
      selectedValue=val;
      return true;
    }
  }

  validValue=false;
  selectedID=-1;
  selectedValue="";
  return false;
}



void config_parameters::register_config_int(const char* name, char short_option, size_t offset,
                                            int default_value,
                                            int low_limit, int high_limit)
{
  struct config_param param;
  param.type = config_param::Config_Int;
  param.name = name;
  param.short_option = short_option;
  param.int_low_limit  = low_limit;
  param.int_high_limit = high_limit;
  param.int_default = default_value;
  param.offset = offset;

  params.push_back(param);
}


void config_parameters::register_config_string(const char* name, char short_option, size_t offset,
                                               const char* default_value)
{
  struct config_param param;
  param.type = config_param::Config_String;
  param.name = name;
  param.short_option = short_option;
  param.string_default = default_value;
  param.offset = offset;

  params.push_back(param);
}


void config_parameters::register_config_choice(const char* name, char short_option, size_t offset)
{
  struct config_param param;
  param.type = config_param::Config_Choice;
  param.name = name;
  param.short_option = short_option;
  param.offset = offset;

  params.push_back(param);
}


static void remove_option(int* argc,char** argv,int idx, int n=1)
{
  for (int i=idx+n;i<*argc;i++) {
    argv[i-n] = argv[i];
  }

  *argc-=n;
}


void config_parameters::set_defaults(void* dst)
{
  for (int o=0;o<params.size();o++) {
    const config_param& param = params[o];

    switch (param.type) {
    case config_param::Config_Bool:
      *(bool*)((char*)dst+param.offset) = param.bool_default;
      break;

    case config_param::Config_Int:
      *(int*)((char*)dst+param.offset) = param.int_default;
      break;

    case config_param::Config_String:
      *(const char**)((char*)dst+param.offset) = param.string_default;
      break;

    case config_param::Config_Choice:
      break;
    }
  }
}


bool config_parameters::config_param::set_value(const char* value, void* dst, const char* name) const
{
  // --- int ---

  if (type == config_param::Config_Int) {
    if (!isdigit(value[0]) && value[0]!='-' && value[0]!='+') {
      fprintf(stderr,"Parameter value '%s' is no integer number.\n", value);
      return false;
    }

    for (int i=1;value[i];i++) {
      if (!isdigit(value[i])) {
        fprintf(stderr,"Parameter value '%s' is no integer number.\n", value);
        return false;
      }
    }

    int value_int = atoi(value);
    
    if (int_low_limit != NO_LIMIT && value_int < int_low_limit) {
      fprintf(stderr,"Parameter value %d for %s is below minimum value %d.\n",
              value_int, name, int_low_limit);
      return false;
    }

    if (int_high_limit != NO_LIMIT && value_int > int_high_limit) {
      fprintf(stderr,"Parameter value %d for %s is above maximum value %d.\n",
              value_int, name, int_high_limit);
      return false;
    }
        
    *(int*)((char*)dst+offset) = value_int;
  }

  // --- string ---

  else if (type == config_param::Config_String) {
    *(const char**)((char*)dst+offset) = value;
  }

  // --- choice ---

  else if (type == config_param::Config_Choice) {
    bool valid = ((choice_option*)((char*)dst+offset))->setValue(value);
    if (!valid) {
      fprintf(stderr,"Parameter value %s for %s is not a valid value.\n", value, name);
      return false;
    }
  }

  return true;
}


bool config_parameters::parse_command_line_params(int* argc, char** argv, void* dst,
                                                  bool ignore_unknown_options)
{
  set_defaults(dst);


  for (int i=1;i < *argc;i++) {
    if (argv[i][0]=='-') {
      // option

      if (argv[i][1]=='-') {
        // long option

        bool option_found=false;

        for (int o=0;o<params.size();o++) {
          if (strcmp(params[o].name, argv[i]+2)==0) {
            option_found=true;

            const config_param& param = params[o];

            if (param.type == config_param::Config_Bool) {
              *(bool*)((char*)dst+param.offset) = !param.bool_default;
              remove_option(argc,argv,i);
              break;
            }

            if (i == *argc-1) {
              fprintf(stderr,"Parameter for option %s missing.\n",argv[i]);
              return false;
            }

            const char* value = argv[i+1];
            remove_option(argc,argv,i, 2);

            bool success = param.set_value(value, dst,argv[i]);
            if (!success) { return false; }

            i--;
            break;
          }
        }

        if (option_found == false && !ignore_unknown_options) {
          fprintf(stderr,"unknown option %s.\n", argv[i]);
          return false;
        }
      }
      else {
        // short option

        bool is_single_option = (argv[i][1] != 0 && argv[i][2]==0);
        bool do_remove_option = true;

        for (int n=1; argv[i][n]; n++) {
          char option = argv[i][n];

          bool option_found=false;

          for (int o=0;o<params.size();o++) {
            if (params[o].short_option == option) {
              option_found=true;

              const config_param& param = params[o];
              
              if (param.type == config_param::Config_Bool) {
                *(bool*)((char*)dst+param.offset) = !param.bool_default;
                break;
              }
              
              if (!is_single_option) {
                fprintf(stderr,"Short option with parameters cannot be combined into a single option.\n");
                return false;
              }
              
              if (i == *argc-1) {
                fprintf(stderr,"Parameter for option -%c missing.\n",option);
                return false;
              }
              
              const char* value = argv[i+1];
              remove_option(argc,argv,i+1);
              
              bool success = param.set_value(value, dst,argv[i]);
              if (!success) { return false; }
              
              break;
            }
          }

          if (!option_found) {
            do_remove_option=false;
          }

          if (!option_found && !ignore_unknown_options) {
            fprintf(stderr, "unknown option -%c\n",option);
            return false;
          }

        } // all short options

        if (do_remove_option) {
          remove_option(argc,argv,i);
          i--;
        }
      } // is short option
    } // is option
  } // all command line arguments

  return true;
}


void config_parameters::show_params(void* paramstruct) const
{
  for (int i=0;i<params.size();i++) {
    const config_param& p = params[i];

    std::stringstream sstr;
    sstr << "  ";
    if (p.short_option != 0) {
      sstr << '-' << p.short_option;
    } else {
      sstr << "  ";
    }

    if (p.short_option != 0 && p.name != NULL) {
      sstr << ", ";
    } else {
      sstr << "  ";
    }

    if (p.name != NULL) {
      sstr << "--" << std::setw(12) << std::left << p.name;
    } else {
      sstr << "              ";
    }

    sstr << " ";
    switch (p.type) {
    case config_param::Config_Int:    sstr << "(int)"; break;
    case config_param::Config_Bool:   sstr << "(bool)"; break;
    case config_param::Config_String: sstr << "(string)"; break;
    case config_param::Config_Choice: { sstr << "{";
      const std::vector< std::pair<std::string,int> > choices =
        ((choice_option*)((char*)paramstruct+p.offset))->getChoices();

      for (int i=0;i<choices.size();i++) {
        if (i>0) { sstr << ","; }
        sstr << choices[i].first;
      }

      sstr << "}";
    }
      break;
    }

    if (p.type==config_param::Config_Int) {
      if (p.int_low_limit != NO_LIMIT) {
        sstr << " >=" << p.int_low_limit;
      }
      if (p.int_high_limit != NO_LIMIT) {
        sstr << " <=" << p.int_high_limit;
      }
    }

    sstr << " default=";
    switch (p.type) {
    case config_param::Config_Int:    sstr << p.int_default; break;
    case config_param::Config_Bool:   sstr << (p.bool_default ? "on":"off"); break;
    case config_param::Config_String: sstr << p.string_default; break;
    case config_param::Config_Choice: {
      choice_option* opt = (choice_option*)((char*)paramstruct+p.offset);

      const std::vector< std::pair<std::string,int> > choices = opt->getChoices();

      for (int i=0;i<choices.size();i++) {
        if (choices[i].second == opt->getID())
          { sstr << choices[i].first; break; }
      }
    }
      break;
    }

    sstr << "\n";

    std::cerr << sstr.str();
  }
}


void register_encoder_params(config_parameters* config)
{
  const int NO_LIMIT = config_parameters::NO_LIMIT;

#define eoffset(name) offsetof(encoder_params, name)

  config->register_config_string("input", 'i', eoffset(input_yuv), "paris_cif.yuv");
  config->register_config_int("width",  'w', eoffset(input_width),
                              352,      1,NO_LIMIT);
  config->register_config_int("height", 'h', eoffset(input_height),
                              288,      1,NO_LIMIT);

  config->register_config_string("output", 'o' , eoffset(output_filename), "out.bin");
  config->register_config_string("reconstruction", 'O' , eoffset(reconstruction_yuv), "recon.yuv");

  config->register_config_int("first-frame",  0 , eoffset(first_frame),
                              0,       0,NO_LIMIT);
  config->register_config_int("frames",      'f', eoffset(max_number_of_frames),
                              INT_MAX, 1,NO_LIMIT);

  config->register_config_choice("TB-IntraPredMode", 0, eoffset(mAlgo_TB_IntraPredMode));
  config->register_config_choice("TB-IntraPredMode-subset",0, eoffset(mAlgo_TB_IntraPredMode_Subset));
  config->register_config_choice("CB-IntraPartMode", 0, eoffset(mAlgo_CB_IntraPartMode));
  config->register_config_choice("CB-IntraPartMode-Fixed-partMode", 0, eoffset(CB_IntraPartMode_Fixed.partMode));
  config->register_config_choice("IntraPredMode-estimator", 0, eoffset(TB_IntraPredMode_FastBrute.bitrateEstimMethod));

  config->register_config_int("constant-qp", 'q', eoffset(CTB_QScale_Constant.mQP),
                              27,      1,51);

  config->register_config_int("min-cb-size",  0 , eoffset(min_cb_size),
                              8 ,      8,64);
  config->register_config_int("max-cb-size",  0 , eoffset(max_cb_size),
                              32,      8,64);

  config->register_config_int("min-tb-size",  0 , eoffset(min_tb_size),
                              4 ,      4,32);
  config->register_config_int("max-tb-size",  0 , eoffset(max_tb_size),
                              32,      8,32);

  config->register_config_int("max-transform-hierarchy-depth-intra",  0 ,
                              eoffset(max_transform_hierarchy_depth_intra),
                              3,       0, 4);

  config->register_config_int("lambda",  0 ,
                              eoffset(lambda),
                              50,      0, 1000);
}
