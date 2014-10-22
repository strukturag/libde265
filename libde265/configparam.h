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
#include <assert.h>


/* Notes: probably best to keep cmd-line-options here. So it will be:
   - automatically consistent even when having different combinations of algorithms
   - no other place to edit
   - if needed, one can still override it at another place
 */


class option_base
{
 public:
 option_base() : mShortOption(0), mLongOption(NULL) { }
 option_base(const char* name) : mShortOption(0), mLongOption(NULL), mIDName(name) { }
  virtual ~option_base() { }


  // --- option identifier ---

  void set_ID(const char* name) { mIDName=name; }
  void add_namespace_prefix(std::string prefix) { mPrefix = prefix + ":" + mPrefix; }

  std::string getName() const { return mPrefix + mIDName; }


  // --- description ---

  void set_description(std::string descr) { mDescription = descr; }
  std::string get_description() const { return mDescription; }


  // --- command line options ----

  void set_cmd_line_options(const char* long_option, char short_option = 0)
  {
    mShortOption = short_option;
    mLongOption  = long_option;
  }

  void set_short_option(char short_option) { mShortOption=short_option; }

  void unsetCmdLineOption()
  {
    mShortOption = 0;
    mLongOption  = NULL;
  }

  bool hasShortOption() const { return mShortOption!=0; }
  char getShortOption() const { return mShortOption; }
  bool hasLongOption() const { return true; } //mLongOption!=NULL; }
  std::string getLongOption() const { return mLongOption ? std::string(mLongOption) : getName(); }

  virtual bool processCmdLineArguments(char** argv, int* argc, int idx) { return false; }



  virtual std::string getTypeDescr() const = 0;

  virtual std::string get_default_string() const { return "N/A"; }

 private:
  std::string mPrefix;
  std::string mIDName;

  std::string mDescription;

  char mShortOption;
  const char* mLongOption;
};



class option_bool : public option_base
{
public:
  option_bool() : value_set(false), default_value(false) { }

  operator bool() const { return value_set ? value : default_value; }

  void set_default(bool v) { default_value=v; }
  virtual std::string get_default_string() const { return default_value ? "true":"false"; }

  virtual std::string getTypeDescr() const { return "boolean"; }
  virtual bool processCmdLineArguments(char** argv, int* argc, int idx) { value=true; return true; }

 private:
  bool value_set;
  bool value;
  bool default_value;
};


class option_string : public option_base
{
public:
  option_string() : value_set(false), default_value("") { }

  const option_string& operator=(std::string v) { value=v; value_set=true; return *this; }

  operator std::string() const { return get(); }
  std::string get() const { return value_set ? value : default_value; }

  void set_default(std::string v) { default_value=v; }
  virtual std::string get_default_string() const { return default_value; }

  virtual std::string getTypeDescr() const { return "(string)"; }
  virtual bool processCmdLineArguments(char** argv, int* argc, int idx);

 private:
  bool value_set;
  std::string value;
  std::string default_value;
};


class option_int : public option_base
{
public:
  option_int() : value_set(false), default_value(0),
    have_low_limit(false), have_high_limit(false) { }

  void set_range(int mini,int maxi);

  const option_int& operator=(int v) { value=v; value_set=true; return *this; }

  int operator() () const { return value_set ? value : default_value; }
  operator int() const { return operator()(); }

  void set_default(int v) { default_value=v; }
  virtual std::string get_default_string() const;

  virtual std::string getTypeDescr() const;
  virtual bool processCmdLineArguments(char** argv, int* argc, int idx);

 private:
  bool value_set;
  int value;
  int default_value;

  bool have_low_limit, have_high_limit;
  int  low_limit, high_limit;
};



class choice_option_base : public option_base
{
public:
  virtual bool set_value(const std::string& val) = 0;
  virtual std::vector<std::string> get_choice_names() const = 0;

  virtual std::string getTypeDescr() const;
  virtual bool processCmdLineArguments(char** argv, int* argc, int idx);
};


template <class T> class choice_option : public choice_option_base
{
 public:
  // --- initialization ---

  void add_choice(const std::string& s, T id, bool default_value=false) {
    choices.push_back( std::make_pair(s,id) );
    if (default_value) {
      defaultID = id;
      defaultValue = s;
    }
  }

  void set_default(T val) {
    for (auto c : choices)
      if (c.second == val) {
        defaultID = val;
        defaultValue = c.first;
        return;
      }

    assert(false); // value does not exist
  }


  // --- usage ---

  bool set_value(const std::string& val) // returns false if it is not a valid option
  {
    value_set = true;
    selectedValue=val;

    validValue = false;

    for (auto c : choices) {
      if (val == c.first) {
        selectedID = c.second;
        validValue = true;
      }
    }

    return validValue;
  }

  bool isValidValue() const { return validValue; }

  const std::string& getValue() const { return value_set ? selectedValue : defaultValue; }
  void setID(T id) { selectedID=id; validValue=true; }
  const T getID() const { return value_set ? selectedID : defaultID; }

  std::vector<std::string> get_choice_names() const
  {
    std::vector<std::string> names;
    for (auto p : choices) {
      names.push_back(p.first);
    }
    return names;
  }

  std::string get_default_string() const { return defaultValue; }

  T operator() () const { return (T)getID(); }

 private:
  std::vector< std::pair<std::string,T> > choices;

  std::string defaultValue;
  T defaultID;

  bool value_set;
  std::string selectedValue;
  T selectedID;

  bool validValue;
};




class config_parameters
{
 public:
  void add_option(option_base* o);

  void print_params() const;
  bool parse_command_line_params(int* argc, char** argv, int* first_idx=NULL,
                                 bool ignore_unknown_options=false);

 private:
  std::vector<option_base*> mOptions;
};

#endif
