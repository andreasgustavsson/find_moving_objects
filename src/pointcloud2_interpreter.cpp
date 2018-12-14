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

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#ifdef NODELET
#include <pluginlib/class_list_macros.h>
#endif
/* C/C++ */
#include <iostream>
#include <cmath>
#include <pthread.h>

/* LOCAL INCLUDES */
#include <find_moving_objects/bank.h>
#include <find_moving_objects/PointCloud2Interpreter.h>


#ifdef NODELET
/* TELL ROS ABOUT THIS NODELET PLUGIN */
PLUGINLIB_EXPORT_CLASS(find_moving_objects::PointCloud2InterpreterNodelet, nodelet::Nodelet)
#endif


namespace find_moving_objects
{

/*
 * Standard Units of Measure and Coordinate Conventions:   http://www.ros.org/reps/rep-0103.html
 * Coordinate Frames for Mobile Platforms:                 http://www.ros.org/reps/rep-0105.html
 */


/* CONFIDENCE CALCULATION FOR BANK */
const double a = -10 / 3;
double root_1=0.35, root_2=0.65; // optimized for bank coverage of 0.5s, adapted in hz calculation
double Bank::calculateConfidence(const MovingObject & mo,
                                 const BankArgument & ba,
                                 const double dt,
                                 const double mo_old_width,
                                 const bool transform_old_time_map_frame_success,
                                 const bool transform_new_time_map_frame_success,
                                 const bool transform_old_time_fixed_frame_success,
                                 const bool transform_new_time_fixed_frame_success,
                                 const bool transform_old_time_base_frame_success,
                                 const bool transform_new_time_base_frame_success)
{
  return ba.ema_alpha * // Using weighting decay decreases the confidence while,
         (ba.base_confidence + // how much we trust the sensor itself,
          (transform_old_time_map_frame_success &&
           transform_new_time_map_frame_success &&
           transform_old_time_fixed_frame_success &&
           transform_new_time_fixed_frame_success &&
           transform_old_time_base_frame_success  &&
           transform_new_time_base_frame_success ? 0.5 : 0.0) + // transform success,
          a*(dt-root_1)*(dt-root_2) + // a well-adapted bank size in relation to the sensor rate and environmental context
          // (dt should be close to 0.5 seconds),
          (-5.0 * fabsf(mo.seen_width - mo_old_width))); // and low difference in width between old and new object,
          // make us more confident
}


/* CONSTRUCTOR */
#ifdef NODELET
PointCloud2InterpreterNodelet::PointCloud2InterpreterNodelet()
#endif
#ifdef NODE
PointCloud2InterpreterNode::PointCloud2InterpreterNode()
#endif
: received_messages(0),
  optimize_nr_scans_in_bank(0.0)
{
#ifdef NODELET
  // Wait for time to become valid, then start bank
  ros::Time::waitForValid();
#endif
  
  // Start collecting tf data
  bank = new find_moving_objects::Bank;
  
#ifdef NODE
  onInit();
#endif
}

/* DESTRUCTOR */
#ifdef NODELET
PointCloud2InterpreterNodelet::~PointCloud2InterpreterNodelet()
#endif
#ifdef NODE
PointCloud2InterpreterNode::~PointCloud2InterpreterNode()
#endif
{
  delete bank;
}



/* CALLBACK FOR FIRST MESSAGE */
#ifdef NODELET
void PointCloud2InterpreterNodelet::pointCloud2CallbackFirst(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
#ifdef NODE
void PointCloud2InterpreterNode::pointCloud2CallbackFirst(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
{ 
  // Debug frame to see e.g. if we are dealing with an optical frame
#ifdef NODELET
  NODELET_DEBUG_STREAM("PointCloud2 sensor is using frame: " << msg->header.frame_id);
#endif
#ifdef NODE
  ROS_DEBUG_STREAM("PointCloud2 sensor is using frame: " << msg->header.frame_id);
#endif

  // Init bank
  if (bank->init(bank_argument, msg) == 0)
  {
    // Use the other callback from now on
#ifdef NODELET
    ros::NodeHandle nh = getNodeHandle();
#endif
#ifdef NODE
    ros::NodeHandle nh;
#endif
    sub = nh.subscribe(subscribe_topic,
                       subscribe_buffer_size,
#ifdef NODELET
                       &PointCloud2InterpreterNodelet::pointCloud2Callback, 
#endif
#ifdef NODE
                       &PointCloud2InterpreterNode::pointCloud2Callback, 
#endif
                       this);
  }
}


/* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
#ifdef NODELET
void PointCloud2InterpreterNodelet::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
#ifdef NODE
void PointCloud2InterpreterNode::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
{
  // Can message be added to bank?
  if (bank->addMessage(msg) != 0)
  {
    // Adding message failed
    return;
  }

  // If so, then find and report objects
  bank->findAndReportMovingObjects();
}


/* CALLBACK FOR WAITING UNTIL THE FIRST MESSAGE WAS RECEIVED - HZ CALCULATION */
#ifdef NODELET
void PointCloud2InterpreterNodelet::waitForFirstMessageCallback(const sensor_msgs::PointCloud2::ConstPtr & msg) 
#endif
#ifdef NODE
void PointCloud2InterpreterNode::waitForFirstMessageCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
{
  // Set start time
  start_time = ros::Time::now().toSec();
  
  // Update subscriber with new callback
#ifdef NODELET
  ros::NodeHandle nh = getNodeHandle();
#endif
#ifdef NODE
  ros::NodeHandle nh;
#endif
  sub = nh.subscribe(subscribe_topic,
                       subscribe_buffer_size,
#ifdef NODELET
                       &PointCloud2InterpreterNodelet::hzCalculationCallback,
#endif
#ifdef NODE
                       &PointCloud2InterpreterNode::hzCalculationCallback,
#endif
                       this);
}


/* CALLBACK FOR HZ CALCULATION */
#ifdef NODELET
void PointCloud2InterpreterNodelet::hzCalculationCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
#ifdef NODE
void PointCloud2InterpreterNode::hzCalculationCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
#endif
{
  // spin until target is reached
  received_messages++;
  const double elapsed_time = ros::Time::now().toSec() - start_time;
  
  // Are we done?
  if (max_time <= elapsed_time ||
      max_messages <= received_messages)
  {
    // Calculate HZ
    const double hz = received_messages / elapsed_time;
    
    // Set nr of messages in bank
    const double nr_scans = optimize_nr_scans_in_bank * hz;
    bank_argument.nr_scans_in_bank = nr_scans - ((long) nr_scans) == 0.0 ? nr_scans + 1 : ceil(nr_scans);
    
    // Sanity check
    if (bank_argument.nr_scans_in_bank < 2)
    {
      bank_argument.nr_scans_in_bank = 2;
    }
    
    root_1 = optimize_nr_scans_in_bank * 0.6;
    root_2 = optimize_nr_scans_in_bank * 1.4;
    
#ifdef NODELET
    NODELET_INFO_STREAM("Topic " << subscribe_topic << " has rate " << hz << "Hz" << 
                        " (based on " << received_messages << " msgs during " << elapsed_time << " seconds)");
    NODELET_INFO_STREAM("Optimized bank size is " << bank_argument.nr_scans_in_bank);
    
    // Update subscriber with new callback
    ros::NodeHandle nh = getNodeHandle();
    sub = nh.subscribe(subscribe_topic,
                       subscribe_buffer_size,
                       &PointCloud2InterpreterNodelet::pointCloud2CallbackFirst,
                       this);
#endif
#ifdef NODE
    ROS_INFO_STREAM("Topic " << subscribe_topic << " has rate " << hz << "Hz" << 
                        " (based on " << received_messages << " msgs during " << elapsed_time << " seconds)");
    ROS_INFO_STREAM("Optimized bank size is " << bank_argument.nr_scans_in_bank);
    
    // Update subscriber with new callback
    ros::NodeHandle nh;
    sub = nh.subscribe(subscribe_topic,
                       subscribe_buffer_size,
                       &PointCloud2InterpreterNode::pointCloud2CallbackFirst,
                       this);
#endif
  }
}


/* ENTRY POINT FOR NODELET AND INIT FOR NODE */
#ifdef NODELET
void PointCloud2InterpreterNodelet::onInit()
{
  // Node handles
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle nh_priv = getPrivateNodeHandle();
#endif
#ifdef NODE
void PointCloud2InterpreterNode::onInit()
{
  // Node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
#endif
  
  // Init bank_argument using parameters
  nh_priv.param("subscribe_topic", subscribe_topic, default_subscribe_topic);
  nh_priv.param("subscribe_buffer_size", subscribe_buffer_size, default_subscribe_buffer_size);
  nh_priv.param("ema_alpha", bank_argument.ema_alpha, default_ema_alpha);
  nh_priv.param("nr_scans_in_bank", bank_argument.nr_scans_in_bank, default_nr_scans_in_bank);
  nh_priv.param("nr_points_per_scan_in_bank", bank_argument.points_per_scan, default_nr_points_per_scan_in_bank);
  nh_priv.param("bank_view_angle", bank_argument.angle_max, default_bank_view_angle);
  bank_argument.angle_max /= 2.0;
  bank_argument.angle_min = -bank_argument.angle_max;
  nh_priv.param("sensor_frame_has_z_axis_forward", bank_argument.sensor_frame_has_z_axis_forward, default_sensor_frame_has_z_axis_forward);
  nh_priv.param("object_threshold_edge_max_delta_range", bank_argument.object_threshold_edge_max_delta_range, default_object_threshold_edge_max_delta_range);
  nh_priv.param("object_threshold_min_nr_points", bank_argument.object_threshold_min_nr_points, default_object_threshold_min_nr_points);
  nh_priv.param("object_threshold_max_distance", bank_argument.object_threshold_max_distance, default_object_threshold_max_distance);
  nh_priv.param("object_threshold_min_speed", bank_argument.object_threshold_min_speed, default_object_threshold_min_speed);
  nh_priv.param("object_threshold_max_delta_width_in_points", bank_argument.object_threshold_max_delta_width_in_points, default_object_threshold_max_delta_width_in_points);
  nh_priv.param("object_threshold_bank_tracking_max_delta_distance", bank_argument.object_threshold_bank_tracking_max_delta_distance, default_object_threshold_bank_tracking_max_delta_distance);
  nh_priv.param("object_threshold_min_confidence", bank_argument.object_threshold_min_confidence, default_object_threshold_min_confidence);
  nh_priv.param("base_confidence", bank_argument.base_confidence, default_base_confidence);
  nh_priv.param("publish_ema", bank_argument.publish_ema, default_publish_ema);
  nh_priv.param("publish_objects_closest_points_markers", bank_argument.publish_objects_closest_point_markers, default_publish_objects_closest_points_markers);
  nh_priv.param("publish_objects_velocity_arrows", bank_argument.publish_objects_velocity_arrows, default_publish_objects_velocity_arrows);
  nh_priv.param("publish_objects_delta_position_lines", bank_argument.publish_objects_delta_position_lines, default_publish_objects_delta_position_lines);
  nh_priv.param("publish_objects_width_lines", bank_argument.publish_objects_width_lines, default_publish_objects_width_lines);
  nh_priv.param("velocity_arrows_use_full_gray_scale", bank_argument.velocity_arrows_use_full_gray_scale, default_velocity_arrows_use_full_gray_scale);
  nh_priv.param("velocity_arrows_use_sensor_frame", bank_argument.velocity_arrows_use_sensor_frame, default_velocity_arrows_use_sensor_frame);
  nh_priv.param("velocity_arrows_use_base_frame", bank_argument.velocity_arrows_use_base_frame, default_velocity_arrows_use_base_frame);
  nh_priv.param("velocity_arrows_use_fixed_frame", bank_argument.velocity_arrows_use_fixed_frame, default_velocity_arrows_use_fixed_frame);
  nh_priv.param("publish_objects", bank_argument.publish_objects, default_publish_objects);
  nh_priv.param("map_frame", bank_argument.map_frame, default_map_frame);
  nh_priv.param("fixed_frame", bank_argument.fixed_frame, default_fixed_frame);
  nh_priv.param("base_frame", bank_argument.base_frame, default_base_frame);
  nh_priv.param("ns_velocity_arrows", bank_argument.velocity_arrow_ns, default_ns_velocity_arrows);
  nh_priv.param("ns_delta_position_lines", bank_argument.delta_position_line_ns, default_ns_delta_position_lines);
  nh_priv.param("ns_width_lines", bank_argument.width_line_ns, default_ns_width_lines);
  nh_priv.param("topic_ema", bank_argument.topic_ema, default_topic_ema);
  nh_priv.param("topic_objects_closest_points_markers", bank_argument.topic_objects_closest_point_markers, default_topic_objects_closest_points_markers);
  nh_priv.param("topic_objects_velocity_arrows", bank_argument.topic_objects_velocity_arrows, default_topic_objects_velocity_arrows);
  nh_priv.param("topic_objects_delta_position_lines", bank_argument.topic_objects_delta_position_lines, default_topic_objects_delta_position_lines);
  nh_priv.param("topic_objects_width_lines", bank_argument.topic_objects_width_lines, default_topic_objects_width_lines);
  nh_priv.param("topic_objects", bank_argument.topic_objects, default_topic_objects);
  nh_priv.param("publish_buffer_size", bank_argument.publish_buffer_size, default_publish_buffer_size);

  // PointCloud2-specific
  nh_priv.param("message_x_coordinate_field_name", bank_argument.PC2_message_x_coordinate_field_name, default_message_x_coordinate_field_name);
  nh_priv.param("message_y_coordinate_field_name", bank_argument.PC2_message_y_coordinate_field_name, default_message_y_coordinate_field_name);
  nh_priv.param("message_z_coordinate_field_name", bank_argument.PC2_message_z_coordinate_field_name, default_message_z_coordinate_field_name);
  nh_priv.param("voxel_leaf_size", bank_argument.PC2_voxel_leaf_size, default_voxel_leaf_size);
  nh_priv.param("threshold_z_min", bank_argument.PC2_threshold_z_min, default_threshold_z_min);
  nh_priv.param("threshold_z_max", bank_argument.PC2_threshold_z_max, default_threshold_z_max);
  
  nh_priv.setParam("dummy", 0.0);
  
  // Z threshold sanity check
  if (bank_argument.PC2_threshold_z_max < bank_argument.PC2_threshold_z_min)
  {
    std::string err = "threshold_z_max cannot be smaller than threshold_z_min";
#ifdef NODELET
    NODELET_ERROR("%s", err.c_str());
#endif
#ifdef NODE
    ROS_ERROR("%s", err.c_str());
#endif
    ROS_BREAK();
  }
  
  // Optimize bank size?
  nh_priv.param("optimize_nr_scans_in_bank", optimize_nr_scans_in_bank, default_optimize_nr_scans_in_bank);
  
  // If optimize_nr_scans_in_bank != 0, then yes
  if (optimize_nr_scans_in_bank != 0.0)
  {
    // Wait for first message to arrive
    sub = nh.subscribe(subscribe_topic,
                       subscribe_buffer_size,
#ifdef NODELET
                       &PointCloud2InterpreterNodelet::waitForFirstMessageCallback,
#endif
#ifdef NODE
                       &PointCloud2InterpreterNode::waitForFirstMessageCallback,
#endif
                       this);
  }
  else
  {
    // Subscribe to the sensor topic using first callback
    sub = nh.subscribe(subscribe_topic,
                       subscribe_buffer_size,
#ifdef NODELET
                       &PointCloud2InterpreterNodelet::pointCloud2CallbackFirst,
#endif
#ifdef NODE
                       &PointCloud2InterpreterNode::pointCloud2CallbackFirst,
#endif
                       this);
  }
}

} // namespace find_moving_objects

#ifdef NODE
using namespace find_moving_objects;

/* ENTRY POINT */
int main (int argc, char ** argv)
{
  // Init ROS
  ros::init(argc, argv, "pointcloud2_interpreter", ros::init_options::AnonymousName);

  // Create and init node object
  PointCloud2InterpreterNode pc2interpreter;
  
  // Enter receive loop
  ros::spin();

  return 0;
}
#endif
