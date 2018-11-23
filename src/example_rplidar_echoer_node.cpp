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
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;
sensor_msgs::LaserScan msg_dummy;
sensor_msgs::LaserScan::ConstPtr msg_old(new sensor_msgs::LaserScan(msg_dummy));

void cb(const sensor_msgs::LaserScan::ConstPtr & msg_org)
{
  if (msg_old->header.stamp == msg_org->header.stamp)
  {
    return;
  }
  
  msg_old = msg_org;
  sensor_msgs::LaserScan msg = *msg_org;
  msg.header.stamp = ros::Time::now();
  unsigned int size = msg.ranges.size();
//   for (unsigned int i=0; i<size; ++i)
//   {
//     if (msg.ranges[i] == std::numeric_limits<float>::infinity())
//     {
//       msg.ranges[i] = msg.range_max + 10.0;
// //       msg.ranges[i] = 2;
// //       msg.intensities[i] = 1000;
//     }
//     else if (msg.ranges[i] == -std::numeric_limits<float>::infinity())
//     {
//       msg.ranges[i] = msg.range_min - 10.0;
// //       msg.ranges[i] = 2;
// //       msg.intensities[i] = 1000;
//     }
//   }
  
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rplidar_laserscan_echoer");
  ros::NodeHandle n;
  
  // Wait for time to become valid
  ros::Time::waitForValid();

  pub = n.advertise<sensor_msgs::LaserScan>("/lidar/scan_echoed", 10);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/lidar/scan", 10, cb);

  ros::spin();

  return 0;
}
