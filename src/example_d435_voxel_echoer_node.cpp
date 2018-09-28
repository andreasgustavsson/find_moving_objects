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
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void cb(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  sensor_msgs::PointCloud2 msg2 = *msg;
  msg2.header.stamp = ros::Time::now();
  pub.publish(msg2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "d435_voxel_echoer_node");
  ros::NodeHandle n;
  
  // Wait for time to become valid
  ros::Time::waitForValid();

  pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points/filtered_echoed", 10);
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points/filtered", 10, cb);

  ros::spin();

  return 0;
}