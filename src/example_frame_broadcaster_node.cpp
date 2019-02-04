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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>


/* Entry point */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "frame_broadcaster", ros::init_options::AnonymousName);
  ros::NodeHandle node;
  ros::NodeHandle node_priv("~");

  ros::Time now;

  tf2_ros::TransformBroadcaster br;
  std::vector<geometry_msgs::TransformStamped> transforms;
  transforms.resize(4);
  {
    std::vector<std::string> parent_frames;
    parent_frames.push_back("map");
    parent_frames.push_back("odom");
    parent_frames.push_back("base_link");
    parent_frames.push_back("base_link");
    std::vector<std::string> child_frames;
    child_frames.push_back("odom");
    child_frames.push_back("base_link");
    child_frames.push_back("camera_link");
    child_frames.push_back("laser");
    
    // Setup headers and child frames
    for (int i = 0; i<transforms.size(); ++i)
    {
      transforms[i].header.frame_id = parent_frames[i];
      transforms[i].header.seq = 0;
      transforms[i].child_frame_id = child_frames[i];
    }
    
    // Set up static properties
    transforms[0].transform.translation.x = 0.0;
    transforms[0].transform.translation.y = 0.0;
    transforms[0].transform.translation.z = 0.0;
    transforms[0].transform.rotation.x = 0.0;
    transforms[0].transform.rotation.y = 0.0;
    transforms[0].transform.rotation.z = 0.0;
    transforms[0].transform.rotation.w = 1.0;
  }
  
  double dx, dy, radius, dtheta;
  node_priv.param("dx", dx, 0.0);
  node_priv.param("dy", dy, 0.0);
  node_priv.param("radius", radius, 0.0);
  node_priv.param("dtheta", dtheta, 0.0);
  
  const double PI_HALF = M_PI/2;
  const double TWO_PI  = 2*M_PI;
  double x = radius;
  double y = 0.0;
  double theta = 0.0;
  double dxsum = 0.0;
  double dysum = 0.0;

  // Static setup of odom <-> base_link relation
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, dtheta == 0.0 ? 0.0 : theta + PI_HALF);
    transforms[1].transform.rotation.x = q.x();
    transforms[1].transform.rotation.y = q.y();
    transforms[1].transform.rotation.z = q.z();
    transforms[1].transform.rotation.w = q.w();
    
    transforms[1].transform.translation.z = 0.0;
  }
  
  // Set up camera
  {
    tf2::Quaternion q = tf2::Quaternion(0,0,0,1);
    transforms[2].transform.translation.x = 0.035;
    transforms[2].transform.translation.y = -0.025;
    transforms[2].transform.translation.z = -0.1;
    transforms[2].transform.rotation.x = 0.0;
    transforms[2].transform.rotation.y = 0.0;
    transforms[2].transform.rotation.z = 0.0;
    transforms[2].transform.rotation.w = 1.0;
  }
  
  
  // Set up laser
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 3.08923); // 177 degrees rotation around Z axis
    transforms[3].transform.translation.x = -0.053;
    transforms[3].transform.translation.y = -0.036;
    transforms[3].transform.translation.z = 0.03;
    transforms[3].transform.rotation.x = q.x();
    transforms[3].transform.rotation.y = q.y();
    transforms[3].transform.rotation.z = q.z();
    transforms[3].transform.rotation.w = q.w();
  }
    
  // ~Publish rate
  ros::Rate rate(10.0);

  while (node.ok())
  {
    
    now = ros::Time::now();
    
    for (int i = 0; i<transforms.size(); ++i)
    {
      transforms[i].header.stamp = now;
    }
    
    
//     // Odometry frame's location in map
//     br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),
//                                                         tf::Vector3(0,0,0)),
//                                           now,
//                                           "map",
//                                           "odom"));

    // Robot in odometry frame
    transforms[1].transform.translation.x = x;
    transforms[1].transform.translation.y = y;
    
//     br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),
//                                                                        dtheta == 0 ? 0.0 : theta + PI_HALF),
//                                                         tf::Vector3(x,y,0)),
//                                           now,
//                                           "odom",
//                                           "base_link"));

    theta += dtheta;
    x = radius * cos(theta);
    y = radius * sin(theta);
    dxsum+=dx;
    dysum+=dy;
    x+=dxsum;
    y+=dysum;

//     // Camera position on robot
//     br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),
//                                                         tf::Vector3(0.035,-0.025,-0.1)),
//                                           now,
//                                           "base_link",
//                                           "camera_link"));
    
//     // Lidar position on robot
//     br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0),
//                                                                        3.08923), // 177 degrees
//                                                         tf::Vector3(-0.053,-0.036,0.03)),
//                                           now,
//                                           "base_link",
//                                           "laser"));
    
    // Publish transforms
    br.sendTransform(transforms);

    rate.sleep();
  }
  return 0;
}
