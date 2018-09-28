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


#ifndef HZ_CALCULATOR_H
#define HZ_CALCULATOR_H

#include <topic_tools/shape_shifter.h>

namespace find_moving_objects
{

/** This class is used for calculating the message rate of some arbitrary topic.
 * Only a short measurement is performed, so a stable message stream is assumed.
 */
class HZCalculator
{
private:
  bool first_message_received;
  /**< Whether the first message on the given topic has been received. The rate measurement starts when this is true. */
  
  int received_msgs;
  /**< The message count. */
  
  void cb(const topic_tools::ShapeShifter::ConstPtr & msg);
  /**< The callback function counting the number of received messages. */
  
  void cbFirst(const topic_tools::ShapeShifter::ConstPtr & msg);
  /**< The callback function which starts the rate measurement by announcing that the first message was received. */
  
public:
  /** Calculates the rate of the messages received on the given topic.
   * 
   * @param topic The name of the topic for which to calculate the message rate.
   * @return The calculated message rate for the given topic.
   */
  double calc(std::string topic);
};

} // namespace find_moving_objects

#endif // HZ_CALCULATOR_H
