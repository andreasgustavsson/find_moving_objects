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

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <nodelet/nodelet.h>
#include <find_moving_objects/option.h>
#include <iostream> // streams
#include <sstream> // ostringstream
#include <iomanip> // setw

namespace find_moving_objects
{

/* Static member variables and functions */
int Option::nr_options_ = 0; // none by default

int Option::getCount()
{
  return nr_options_;
}


/* Constructors */
Option::Option(const bool mandatory,
               const std::string name,
               const std::string description,
               const std::string value)

{
  this->mandatory_ = mandatory;
  this->name_ = name;
  this->description_ = description;
  this->type_ = O_STRING;
  this->value_ = value;
  this->value_min_ = "";
  this->value_max_ = "";
  given_ = false;
  nr_options_++;
};

Option::Option(const bool mandatory,
               const std::string name,
               const std::string description,
               const char * value)

{
  this->mandatory_ = mandatory;
  this->name_ = name;
  this->description_ = description;
  this->type_ = O_STRING;
  this->value_ = value;
  this->value_min_ = "";
  this->value_max_ = "";
  given_ = false;
  nr_options_++;
};

Option::Option(const bool mandatory,
               const std::string name,
               const std::string description,
               const bool value)
{
  this->mandatory_ = mandatory;
  this->name_ = name;
  this->description_ = description;
  this->type_ = O_BOOL;
  this->value_ = value;
  this->value_min_ = false;
  this->value_max_ = true;
  given_ = false;
  nr_options_++;
};

Option::Option(const bool mandatory,
               const std::string name,
               const std::string description,
               const long value,
               const long value_min,
               const long value_max)
{
  this->mandatory_ = mandatory;
  this->name_ = name;
  this->description_ = description;
  this->type_ = O_LONG;
  this->value_ = value;
  this->value_min_ = value_min;
  this->value_max_ = value_max;
  given_ = false;
  nr_options_++;
};

Option::Option(const bool mandatory,
               const std::string name,
               const std::string description,
               const int value,
               const int value_min,
               const int value_max)
{
  this->mandatory_ = mandatory;
  this->name_ = name;
  this->description_ = description;
  this->type_ = O_LONG;
  this->value_ = (long) value;
  this->value_min_ = (long) value_min;
  this->value_max_ = (long) value_max;
  given_ = false;
  nr_options_++;
};

Option::Option(const bool mandatory,
               const std::string name,
               const std::string description,
               const double value,
               const double value_min,
               const double value_max)
{
  this->mandatory_ = mandatory;
  this->name_ = name;
  this->description_ = description;
  this->type_ = O_FLOAT;
  this->value_ = value;
  this->value_min_ = value_min;
  this->value_max_ = value_max;
  given_ = false;
  nr_options_++;
};



/* Destructors */
Option::~Option()
{
  nr_options_--;
}



/* Friend operator for printing */
std::ostream& operator<<(std::ostream& os, const Option& o)
{
  os << o.name_;
  switch(o.type_)
  {
    case Option::O_BOOL: 
      os << " (" << (boost::get<bool>(o.value_) ? "true" : "false") << ")";
      break;
    case Option::O_LONG:
      os << " <integer value in [" << boost::get<long>(o.value_min_) << "," 
      << boost::get<long>(o.value_max_) << "] (" 
      << boost::get<long>(o.value_) << ")>";
      break;
    case Option::O_FLOAT:
      os << " <floating point value in [" << boost::get<double>(o.value_min_) << "," 
      << boost::get<double>(o.value_max_) << "] (" 
      << boost::get<double>(o.value_) << ")>";
      break;
    case Option::O_STRING:
      os << " <string (" << boost::get<std::string>(o.value_) << ")>";
      break;
    default:
      std::ostringstream msg_stream;
      msg_stream << "Uknown option type: " << o.getType();
      std::string msg = msg_stream.str();
      ROS_ASSERT_MSG(o.getType() < Option::O_NR_TYPES, "%s", msg.c_str());
  }
  
  os << std::endl << "        " << o.description_;
  if (o.mandatory_)
  {
    os << " " << "MANDATORY";
  }
  
  return os;
}


/* Option class member functions */
void Option::printUsage(char* prg, Option * options)
{
//   std::cout 
  std::ostringstream usage;
  usage << "Usage: " << prg << " [OPTIONS]" << std::endl
        << "  Where OPTIONS are the following" << std::endl
        << "    -h or --help " << std::endl
        << "        Prints this text, showing default argument values, and exits" << std::endl
        << "    --print_all_options_values (false)" << std::endl
        << "        Prints the values of all options" << std::endl
        << "    --print_given_options (false)" << std::endl
        << "        Prints the values of the user-specified options" << std::endl;
  
  for (unsigned int i=0; i<nr_options_; ++i)
  {
    usage << "    " << options[i] << std::endl;
  }

  std::string string = usage.str();
  ROS_INFO("%s", string.c_str());
}


inline std::string Option::getString()
{
  std::ostringstream stream;
  stream << *this;
  std::string string = stream.str();
  return string;
}


void Option::scanArgs(int argc, char** argv, Option * options)
{
  // Set info
  bool options_given = false;
  bool print_given_options = false;
  bool print_all_options_values = false;
  
  // Node handle
  ros::NodeHandle n("~");
  
  // Print help and exit?
  for (unsigned int i=1; i<argc; ++i)
  {
    if (strcmp(argv[i],"-h") == 0 ||
        strcmp(argv[i],"--help") == 0)
    {
      printUsage(argv[0], options);
      exit(0);
    }
  }
  
  // Loop through command line arguments and adapt options[]
  for (unsigned int i=1; i<argc; ++i)
  {
    if (strcmp(argv[i],"--print_all_options") == 0)
    {
      print_all_options_values = true;
      continue;
    }
    
    if (strcmp(argv[i],"--print_given_options") == 0)
    {
      print_given_options = true;
      continue;
    }
    
    bool found_match = false;
    for (unsigned int j=0; j<nr_options_; ++j)
    {
      Option * o = &options[j];
        if (strcmp(o->getName().c_str(), argv[i]) == 0)
        {
          found_match = true;
          
          if (o->getType() == O_BOOL)
          { 
            // No arg to to this option
            o->setValue(true); // Just toggle value
          }
          else
          {
            // This option should have arg
            if (argc <= i+1)
            {
              ROS_WARN("Skipping option because it has no argument: %s", o->getString().c_str());
              break; // Will execute rest of j loop and do the matching test (unsuccessfully), but we can live with that
            }
            
            // Read argument
            char * end;
            boost::variant<long, double, std::string> arg;
            bool should_skip = false;
            switch (o->getType())
            {
              case O_LONG:
                arg = strtol(argv[i+1], &end, 10);
                if (argv[i+1] == end || // Unsuccessful conversion to long
                    boost::get<long>(arg) < boost::get<long>(o->getValueMin()) ||
                    boost::get<long>(o->getValueMax()) < boost::get<long>(arg))
                {
                  ROS_WARN("Skipping option because its argument is not valid: %s", o->getString().c_str());
                  should_skip = true;
                  break;
                }
                o->setValue(arg);
                break;
              case O_FLOAT:
                arg = strtod(argv[i+1], &end);
                if (argv[i+1] == end || 
                  boost::get<double>(arg) < boost::get<double>(o->getValueMin()) ||
                  boost::get<double>(o->getValueMax()) < boost::get<double>(arg))
                {
                  ROS_WARN("Skipping option because its argument is not valid: %s", o->getString().c_str());
                  should_skip = true;
                  break;
                }
                o->setValue(arg);
                break;
              case O_STRING:
                arg = argv[i+1];
                o->setValue(arg);
                break;
              default:
                ROS_FATAL("This should not occur. There is something wrong with Option::scanArgs!");
            }
            
            if (should_skip)
            {
              // We could not read the next argument on the cmd line
              break;
            }
            else
            {
              // We have read the next argument so step passed it
              i++;
            }
          }
          
          // Option has been given, don't accept it again
          o->setGiven();
          options_given = true;
        }
    }
    
    if (!found_match)
    {
      std::ostringstream stream;
      stream << "Skipping unknown option: " << argv[i] << std::endl;
      std::string string = stream.str();
      ROS_WARN("%s", string.c_str());
    }
  }
  
  // Find length of longest given name
  int chars_in_longest_name = 0;
  int chars_in_longest_name_all = 0;
  for (unsigned int i=0; i<nr_options_; ++i)
  {
    const int len = strlen(options[i].getName().c_str());
    if (options[i].getGiven())
    {
      if (chars_in_longest_name < len)
      {
        chars_in_longest_name = len;
      }
    }
    if (chars_in_longest_name_all < len)
    {
      chars_in_longest_name_all = len;
    }
  }
  
  // If option was not specified on the command line, then check if it is given on the parameter server
  // If option was specified on the command line, then update/set its value on the parameter server
  for (unsigned int i=0; i<nr_options_; ++i)
  {
    Option * o = &options[i];
    char parameterNameWhole[o->getName().length()+1];
    strcpy(parameterNameWhole, o->getName().c_str());
    char * parameterName = &parameterNameWhole[0];
    
    // Remove initial dashes from option/paramter name
    while (!(('a' <= parameterName[0] && parameterName[0] <= 'z') ||
             ('z' <= parameterName[0] && parameterName[0] <= 'Z') || 
             parameterName[0] == '~'))
    {
      parameterName = &parameterName[1];
    }
    
    if (o->getGiven() || !n.hasParam(parameterName))
    {
      // Update value on parameter server
      switch (o->getType())
      {
        case O_BOOL: 
        {
          bool value = o->getBoolValue();
          n.setParam(parameterName, value);
          break;
        }
        case O_LONG:
        {
          int value = o->getLongValue();
          n.setParam(parameterName, value);
          break;
        }
        case O_STRING:
        {
          std::string value = o->getStringValue();
          n.setParam(parameterName, value);
          break;
        }
        case O_FLOAT:
        {
          double value = o->getDoubleValue();
          n.setParam(parameterName, value);
          break;
        }
        default:
        {
          ROS_ERROR_STREAM("Set for unknown option type (" << o->getType() << "), this should not happen!");
        }
      }
    }
    else
    {
      // Get value from parameter server, it must exist based on the condition above!
      // Here: !o->getGiven() && n.hasParam(parameterName)
      bool ok;
      switch (o->getType())
      {
        case O_BOOL:
        {
          bool value;
          ok = n.getParam(parameterName, value);
          if (ok)
          {
            o->setBoolValue(value);
          }
          else
          {
            ROS_ERROR_STREAM("Could not read parameter " << parameterName << " from server");
          }
          break;
        }
        case O_LONG:
        {
          int value;
          ok = n.getParam(parameterName, value);
          if (ok)
          {
            o->setLongValue(value);
          }
          else
          {
            ROS_ERROR_STREAM("Could not read parameter " << parameterName << " from server");
          }
          break;
        }
        case O_STRING:
        {
          std::string value;
          ok = n.getParam(parameterName, value);
          if (ok)
          {
            o->setStringValue(value);
          }
          else
          {
            ROS_ERROR_STREAM("Could not read parameter " << parameterName << " from server");
          }
          break;
        }
        case O_FLOAT:
        {
          double value;
          ok = n.getParam(parameterName, value);
          if (ok)
          {
            o->setDoubleValue(value);
          }
          else
          {
            ROS_ERROR_STREAM("Could not read parameter " << parameterName << " from server");
          }
          break;
        }
        default:
        {
          ROS_ERROR_STREAM("Get for unknown option type (" << o->getType() << "), this should not happen!");
        }
      }
      
      if (!ok)
      {
        ROS_ERROR_STREAM("Could not get the value for [" << parameterName << "] from the server");
      }
    }
  }
  
  // Print the values of all options
  if (print_all_options_values)
  {
    std::ostringstream stream;
    stream << "Options have the following values:" << std::endl;
    
    // While also checking mandatory options
    for (unsigned int i=0; i<nr_options_; ++i)
    {
      Option * o = &options[i];
      ROS_ASSERT_MSG(!o->getMandatory() || o->getGiven(), "%s", o->getString().c_str());
      
      if (o->getType() != O_BOOL)
      {
        stream << "  " << std::setw(chars_in_longest_name_all+2) << \
                  std::left << o->getName() << " " <<  o->getValue() << std::endl;
      }
      else
      {
        stream << "  " << (o->getGiven() ? "" : "! ") << o->getName() << std::endl;
      }
    }
    
    // Output info
    std::string string = stream.str();
    ROS_INFO("%s", string.c_str());
  }
  // Print specified options
  else if (options_given && print_given_options)
  {
    std::ostringstream stream;
    stream << "Options given:" << std::endl;
    
    // While also checking mandatory options
    for (unsigned int i=0; i<nr_options_; ++i)
    {
      Option * o = &options[i];
      if (o->getGiven())
      {
        if (o->getType() != O_BOOL)
        {
          stream << "  " << std::setw(chars_in_longest_name+2) << std::left << o->getName() << \
                    " " <<  o->getValue() << std::endl;
        }
        else
        {
          stream << "  " << o->getName() << std::endl;
        }
      }
      else //if (o->getMandatory())
      {
        ROS_ASSERT_MSG(!o->getMandatory(), "%s", o->getString().c_str());
      }
    }
    
    // Output info
    std::string string = stream.str();
    ROS_INFO("%s", string.c_str());
  }
  else
  {
    // Check mandatory options
    for (unsigned int i=0; i<nr_options_; ++i)
    {
      Option * o = &options[i];
      ROS_ASSERT_MSG(o->getGiven() || !o->getMandatory(), "%s", o->getString().c_str());
    }
  }
}


}
