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


static void remove_option(int* argc,char** argv,int idx, int n=1)
{
  for (int i=idx+n;i<*argc;i++) {
    argv[i-n] = argv[i];
  }

  *argc-=n;
}


bool option_string::processCmdLineArguments(char** argv, int* argc, int idx)
{
  if (argv==NULL)   { return false; }
  if (idx >= *argc) { return false; }

  value = argv[idx];
  value_set = true;

  remove_option(argc,argv,idx,1);

  return true;
}


void option_int::set_range(int mini,int maxi)
{
  have_low_limit =true;
  have_high_limit=true;
  low_limit =mini;
  high_limit=maxi;
}

std::string option_int::getTypeDescr() const
{
  std::stringstream sstr;
  sstr << "(int)";

  if (have_low_limit || have_high_limit) { sstr << " "; }
  if (have_low_limit) { sstr << low_limit << " <= "; }
  if (have_low_limit || have_high_limit) { sstr << "x"; }
  if (have_high_limit) { sstr << " <= " << high_limit; }

  if (!valid_values_set.empty()) {
    sstr << " {";
    bool first=true;
    for (int v : valid_values_set) {
      if (!first) sstr << ","; else first=false;
      sstr << v;
    }
    sstr << "}";
  }

  return sstr.str();
}

bool option_int::processCmdLineArguments(char** argv, int* argc, int idx)
{
  if (argv==NULL)   { return false; }
  if (idx >= *argc) { return false; }

  int v = atoi(argv[idx]);
  if (have_low_limit  && v<low_limit)  { return false; }
  if (have_high_limit && v>high_limit) { return false; }

  value = v;
  value_set = true;

  remove_option(argc,argv,idx,1);

  return true;
}

std::string option_int::get_default_string() const
{
  std::stringstream sstr;
  sstr << default_value;
  return sstr.str();
}


std::string choice_option_base::getTypeDescr() const
{
  std::vector<std::string> choices = get_choice_names();

  std::stringstream sstr;
  sstr << "{";

  bool first=true;
  for (auto c : choices) {
    if (first) { first=false; }
    else { sstr << ","; }

    sstr << c;
  }

  sstr << "}";
  return sstr.str();
}


bool choice_option_base::processCmdLineArguments(char** argv, int* argc, int idx)
{
  if (argv==NULL)   { return false; }
  if (idx >= *argc) { return false; }

  std::string value = argv[idx];

  std::cout << "set " << value << "\n";
  bool success = set_value(value);
  std::cout << "success " << success << "\n";

  remove_option(argc,argv,idx,1);

  return success;
}


static void print_cmdline(int argc,char** argv)
{
  for (int i=0;i<argc;i++)
    {
      printf("%d: %s\n",i,argv[i]);
    }
}


bool config_parameters::parse_command_line_params(int* argc, char** argv, int* first_idx_ptr,
                                                  bool ignore_unknown_options)
{
  int first_idx=1;
  if (first_idx_ptr) { first_idx = *first_idx_ptr; }

  for (int i=first_idx;i < *argc;i++) {

    printf("process argument %s\n",argv[i]);

    if (argv[i][0]=='-') {
      // option

      if (argv[i][1]=='-') {
        // long option

        bool option_found=false;

        for (int o=0;o<mOptions.size();o++) {
          if (mOptions[o]->hasLongOption() && strcmp(mOptions[o]->getLongOption().c_str(),
                                                     argv[i]+2)==0) {
            option_found=true;

            printf("found long option\n");

            bool success = mOptions[o]->processCmdLineArguments(argv,argc, i+1);
            if (!success) {
              if (first_idx_ptr) { *first_idx_ptr = i; }
              return false;
            }

            remove_option(argc,argv,i);
            i--;
            print_cmdline(*argc,argv);

            break;
          }
        }

        if (option_found == false && !ignore_unknown_options) {
          return false;
        }
      }
      else {
        // short option

        bool is_single_option = (argv[i][1] != 0 && argv[i][2]==0);
        bool do_remove_option = true;

        for (int n=1; argv[i][n]; n++) {
          char option = argv[i][n];

          printf("process short option: %c\n",option);

          bool option_found=false;

          for (int o=0;o<mOptions.size();o++) {
            if (mOptions[o]->getShortOption() == option) {
              option_found=true;

              printf("found short option\n");

              bool success;
              if (is_single_option) {
                success = mOptions[o]->processCmdLineArguments(argv,argc, i+1);
              }
              else {
                success = mOptions[o]->processCmdLineArguments(NULL,NULL, 0);
              }

              if (!success) {
                if (first_idx_ptr) { *first_idx_ptr = i; }
                return false;
              }

              break;
            }
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


void config_parameters::print_params() const
{
  for (int i=0;i<mOptions.size();i++) {
    const option_base* o = mOptions[i];

    std::stringstream sstr;
    sstr << "  ";
    if (o->hasShortOption()) {
      sstr << '-' << o->getShortOption();
    } else {
      sstr << "  ";
    }

    if (o->hasShortOption() && o->hasLongOption()) {
      sstr << ", ";
    } else {
      sstr << "  ";
    }

    if (o->hasLongOption()) {
      sstr << "--" << std::setw(12) << std::left << o->getLongOption();
    } else {
      sstr << "              ";
    }

    sstr << " ";
    sstr << o->getTypeDescr();

    sstr << ", default=" << o->get_default_string();
    sstr << "\n";

    std::cerr << sstr.str();
  }
}


void config_parameters::add_option(option_base* o)
{
  mOptions.push_back(o);
}
