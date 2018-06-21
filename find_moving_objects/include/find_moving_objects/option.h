/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Andreas Gustavsson.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Andreas Gustavsson
*********************************************************************/

#ifndef OPTIONS_HPP
#define OPTIONS_HPP

#include <limits>
#include <boost/variant.hpp>

namespace find_moving_objects
{

/**
 * A command line option.
 */
class Option
{
  typedef enum {
  O_BOOL = 0,
  O_LONG,
  O_FLOAT,
  O_STRING,
  O_NR_TYPES
  } option_value_t;

private:
  
  friend std::ostream& operator<<(std::ostream& os, const Option& o);
  std::string getString();
  
  static int nr_options_;
  std::string name_;
  std::string description_;
  bool mandatory_;
  bool given_;
  
  bool getGiven() const {return given_;}
  void setGiven() {given_ = true;}
  void unsetGiven() {given_ = false;}
  
  option_value_t type_;
  boost::variant<bool, 
                 long,
                 double,
                 std::string> value_;
  boost::variant<bool, 
                 long,
                 double,
                 std::string> value_min_;
  boost::variant<bool, 
                 long,
                 double,
                 std::string> value_max_;
                 
public:
  /**
   * Creates a <code>string</code> option. 
   * @param mandatory Whether the option must be specified by the user.
   * @param name The command line name of the option.
   * @param description A text describing what the option affects.
   * @param value The default value of the option.
   */
  Option(const bool mandatory,
         const std::string name,
         const std::string description,
         const std::string value);
  
  /**
   * Creates a <code>string</code> option. 
   * @param mandatory Whether the option must be specified by the user.
   * @param name The command line name of the option.
   * @param description A text describing what the option affects.
   * @param value The default value of the option.
   */
  Option(const bool mandatory,
         const std::string name,
         const std::string description,
         const char * value);
  
  /**
   * Creates a <code>bool</code> option. 
   * @param mandatory Whether the option must be specified by the user.
   * @param name The command line name of the option.
   * @param description A text describing what the option affects.
   * @param value The default value of the option.
   */
  Option(const bool mandatory,
         const std::string name,
         const std::string description,
         const bool value);
  
  /**
   * Creates a <code>long</code> option. 
   * @param mandatory Whether the option must be specified by the user.
   * @param name The command line name of the option.
   * @param description A text describing what the option affects.
   * @param value The default value of the option.
   * @param value_min The lower bound of the value.
   * @param value_max The upper bound of the value.
   */
  Option(const bool mandatory,
         const std::string name,
         const std::string description,
         const long value,
         const long value_min = std::numeric_limits<long>::min(),
         const long value_max = std::numeric_limits<long>::max());
  
  /**
   * Creates a <code>long</code> option. 
   * @param mandatory Whether the option must be specified by the user.
   * @param name The command line name of the option.
   * @param description A text describing what the option affects.
   * @param value The default value of the option.
   * @param value_min The lower bound of the value.
   * @param value_max The upper bound of the value.
   */
  Option(const bool mandatory,
         const std::string name,
         const std::string description,
         const int value,
         const int value_min = std::numeric_limits<int>::min(),
         const int value_max = std::numeric_limits<int>::max());

  /**
   * Creates a <code>double</code> option. 
   * @param mandatory Whether the option must be specified by the user.
   * @param name The command line name of the option.
   * @param description A text describing what the option affects.
   * @param value The default value of the option.
   * @param value_min The lower bound of the value.
   * @param value_max The upper bound of the value.
   */
  Option(const bool mandatory,
         const std::string name,
         const std::string description,
         const double value,
         const double value_min = std::numeric_limits<double>::min(),
         const double value_max = std::numeric_limits<double>::max());

  /**
   * Removes this option. 
   */
  ~Option();
  
  /** @return The number of specified objects. */
  static int getCount();
  
  /** @return The command line name of the option. */
  std::string getName() const {return name_;}
  /** @return The helpful description text of the option. */
  std::string getDescription() const {return description_;}
  
  /** @return The value of the option. */
  boost::variant<bool, long, double, std::string> getValue() {return value_;}
  /** @return The <code>bool</code> value of the option. */
  bool        getBoolValue()   {return boost::get<bool>(value_);}
  /** @return The <code>long</code> value of the option. */
  long        getLongValue()   {return boost::get<long>(value_);}
  /** @return The <code>double</code> value of the option. */
  double      getDoubleValue() {return boost::get<double>(value_);}
  /** @return The <code>string</code> value of the option. */
  std::string getStringValue() {return boost::get<std::string>(value_);}
  
  /** @return The lower bound of the value of the option. */
  boost::variant<bool, long, double, std::string> getValueMin() {return value_min_;}
  /** @return The lower bound of the <code>bool</code> value of the option. */
  bool        getBoolValueMin()   {return boost::get<bool>(value_min_);}
  /** @return The lower bound of the <code>long</code> value of the option. */
  long        getLongValueMin()   {return boost::get<long>(value_min_);}
  /** @return The lower bound of the <code>double</code> value of the option. */
  double      getDoubleValueMin() {return boost::get<double>(value_min_);}
  /** @return The lower bound of the <code>string</code> value of the option. */
  std::string getStringValueMin() {return boost::get<std::string>(value_min_);}
  
  /** @return The upper bound of the value of the option. */
  boost::variant<bool, long, double, std::string> getValueMax() {return value_max_;}
  /** @return The upper bound of the <code>bool</code> value of the option. */
  bool        getBoolValueMax()   {return boost::get<bool>(value_max_);}
  /** @return The upper bound of the <code>long</code> value of the option. */
  long        getLongValueMax()   {return boost::get<long>(value_max_);}
  /** @return The upper bound of the <code>double</code> value of the option. */
  double      getDoubleValueMax() {return boost::get<double>(value_max_);}
  /** @return The upper bound of the <code>string</code> value of the option. */
  std::string getStringValueMax() {return boost::get<std::string>(value_max_);}
  
  /** 
   * Sets the value of the option the one specified.
   * No bounds of type check is performed so be careful!
   * @param val The new value for the option.
   */
  void setValue(boost::variant<bool, long, double, std::string> val) {value_ = val;}
  /** @return The type of the option. */
  option_value_t getType() const {return type_;}
  
  /** @return Whether this option is mandatory or not. */
  bool getMandatory() {return mandatory_;}
  
  /**
   * Prints usage information for the program.
   * @param prg The name of the program.
   * @param options An array of options.
   */
  static void printUsage(char* prg, Option * options);

  /**
   * Scans the command line arguments and updates the values of the options.
   * @param argc The <code>main()</code> function argument.
   * @param argv The <code>main()</code> function argument.
   * @param options An array of options.
   */
  static void scanArgs(int argc, char** argv, Option * options);
};

} // namespace find_moving_objects

#endif // OPTIONS_HPP
