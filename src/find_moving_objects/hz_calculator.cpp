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


#include <find_moving_objects/hz_calculator.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>


// // http://wiki.ros.org/ros_type_introspection/Tutorials/GenericTopicSubscriber
void HZCalculator::cb(const topic_tools::ShapeShifter::ConstPtr & msg) //, const std::string &topic_name)
{
  received_msgs++;
}

void HZCalculator::cbFirst(const topic_tools::ShapeShifter::ConstPtr & msg) //, const std::string &topic_name)
{
  first_message_received = true;
}

double HZCalculator::calc(std::string topic)
{
  // handle to node
  ros::NodeHandle node;
  
  // bind first callback
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callbackFirst;
  callbackFirst = boost::bind(&HZCalculator::cbFirst, this, _1);
  ros::Subscriber subFirst = node.subscribe<topic_tools::ShapeShifter>(topic, 10, callbackFirst);
  
  // bind hz callback
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
  callback = boost::bind(&HZCalculator::cb, this, _1);
  ros::Subscriber sub = node.subscribe<topic_tools::ShapeShifter>(topic, 10, callback);
    
  // spin until target is reached
  const double max_time = 1.2;
  double elapsed_time = 0.0;
  const int max_msgs = 100;
  received_msgs = 0;
  
  // Receive first message
  first_message_received = false;
  while (!first_message_received)
  {
    ros::spinOnce();
    if (!node.ok())
    {
      return 0.0;
    }
  }
  const double start_time = ros::Time::now().toSec();
  while ((elapsed_time < max_time && received_msgs < max_msgs) || received_msgs == 0)
  {
    elapsed_time = ros::Time::now().toSec() - start_time;
    ros::spinOnce();
    if (!node.ok())
    {
      return 0.0;
    }
  }
  
  return received_msgs / elapsed_time;
}
