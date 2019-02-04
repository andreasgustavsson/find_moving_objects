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
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/header.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <geometry_msgs/Point.h>

/* C/C++ */
#include <iostream> 
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>
// #include <pthread.h>

/* Local includes */
#include <find_moving_objects/MovingObject.h>
#include <find_moving_objects/MovingObjectArray.h>
#include <find_moving_objects/bank.h>


// namespace geometry_msgs
// {
// Point operator*(float f, Point v1)
// {
//   Point v;
//   v.x = f * v1.x;
//   v.y = f * v1.y;
//   v.z = f * v1.z;
//   return v;
// }
// Point operator*(Point v1, float f)
// {
//   return f*v1;
// }
// Point operator/(Point v1, float f)
// {
//   Point v;
//   v.x = v1.x / f;
//   v.y = v1.y / f;
//   v.z = v1.z / f;
//   return v;
// }
// float operator^(Point v1, Point v2)
// {
//   return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
// }
// Point operator+(Point v1, Point v2)
// {
//   Point v;
//   v.x = v1.x + v2.x;
//   v.y = v1.y + v2.y;
//   v.z = v1.z + v2.z;
//   return v;
// }
// } // namespace geometry_msgs


namespace find_moving_objects
{

const float TWO_PI = 2*M_PI;

/*
 * Constructors
 */
BankArgument::BankArgument()
{
  ema_alpha = 1.0;
  nr_scans_in_bank = 11;
  points_per_scan = 360;
  angle_min = -M_PI;
  angle_max = M_PI;
  sensor_frame_has_z_axis_forward = false;
  object_threshold_edge_max_delta_range = 0.15;
  object_threshold_min_nr_points = 5;
  object_threshold_max_distance = 6.5;
  object_threshold_min_speed = 0.03;
  object_threshold_max_delta_width_in_points = 5;
  object_threshold_min_confidence = 0.67;
  object_threshold_bank_tracking_max_delta_distance = 0.2;
  base_confidence = 0.3;
  publish_objects = true;
  publish_ema = true;
  publish_objects_closest_point_markers = false;
  publish_objects_velocity_arrows = false;
  publish_objects_delta_position_lines = false;
  publish_objects_width_lines = false;
  velocity_arrows_use_full_gray_scale = false;
  velocity_arrows_use_sensor_frame = false;
  velocity_arrows_use_base_frame = false;
  velocity_arrows_use_fixed_frame = false;
  velocity_arrow_ns = "velocity_arrow_ns";
  delta_position_line_ns = "delta_position_line_ns";
  width_line_ns = "width_line_ns";
  topic_objects = "moving_objects_arrays";
  topic_ema = "ema";
  topic_objects_closest_point_markers = "objects_closest_point_markers";
  topic_objects_velocity_arrows = "objects_velocity_arrows";
  topic_objects_delta_position_lines = "objects_delta_position_lines";
  topic_objects_width_lines = "objects_width_lines";
  publish_buffer_size = 10;
  map_frame = "map";
  fixed_frame = "odom";
  base_frame = "base_link";
//   merge_threshold_max_angle_gap = 0.0 / 180.0 * M_PI;
//   merge_threshold_max_end_points_distance_delta = 0.2;
//   merge_threshold_max_velocity_direction_delta = 25.0 / 180.0 * M_PI;
//   merge_threshold_max_speed_delta = 0.2;
  PC2_message_x_coordinate_field_name = "x";
  PC2_message_y_coordinate_field_name = "y";
  PC2_message_z_coordinate_field_name = "z";
  PC2_voxel_leaf_size = 0.02;
  PC2_threshold_z_min = 0.1;
  PC2_threshold_z_max = 1.0;
  
  node_name_suffix = "";
}

std::ostream& operator<<(std::ostream & os, const BankArgument & ba)
{
  os << "Bank Arguments:" << std::endl <<
    "  ema_alpha = " << ba.ema_alpha << std::endl <<
    "  nr_scans_in_bank = " << ba.nr_scans_in_bank << std::endl <<
    "  points_per_scan = " << ba.points_per_scan << std::endl <<
    "  angle_min = " << ba.angle_min << std::endl <<
    "  angle_max = " << ba.angle_max << std::endl <<
    "  sensor_frame_has_z_axis_forward = " << ba.sensor_frame_has_z_axis_forward << std::endl <<
    "  object_threshold_edge_max_delta_range = " << ba.object_threshold_edge_max_delta_range << std::endl <<
    "  object_threshold_min_nr_points = " << ba.object_threshold_min_nr_points << std::endl <<
    "  object_threshold_max_distance = " << ba.object_threshold_max_distance << std::endl <<
    "  object_threshold_min_speed = " << ba.object_threshold_min_speed << std::endl <<
    "  object_threshold_max_delta_width_in_points = " << ba.object_threshold_max_delta_width_in_points << std::endl <<
    "  object_threshold_min_confidence = " << ba.object_threshold_min_confidence << std::endl <<
    "  object_threshold_bank_tracking_max_delta_distance = " <<
    ba.object_threshold_bank_tracking_max_delta_distance << std::endl <<
    "  base_confidence = " << ba.base_confidence << std::endl <<
    "  publish_objects = " << ba.publish_objects << std::endl <<
    "  publish_ema = " << ba.publish_ema << std::endl <<
    "  publish_objects_closest_point_markers = " << ba.publish_objects_closest_point_markers << std::endl <<
    "  publish_objects_velocity_arrows = " << ba.publish_objects_velocity_arrows << std::endl <<
    "  publish_objects_delta_position_lines = " << ba.publish_objects_delta_position_lines << std::endl <<
    "  publish_objects_width_lines = " << ba.publish_objects_width_lines << std::endl <<
    "  velocity_arrows_use_full_gray_scale = " << ba.velocity_arrows_use_full_gray_scale << std::endl <<
    "  velocity_arrows_use_sensor_frame = " << ba.velocity_arrows_use_sensor_frame << std::endl <<
    "  velocity_arrows_use_base_frame = " << ba.velocity_arrows_use_base_frame << std::endl <<
    "  velocity_arrows_use_fixed_frame = " << ba.velocity_arrows_use_fixed_frame << std::endl <<
    "  velocity_arrow_ns = " << ba.velocity_arrow_ns << std::endl <<
    "  delta_position_line_ns = " << ba.delta_position_line_ns << std::endl <<
    "  width_line_ns = " << ba.width_line_ns << std::endl <<
    "  topic_objects = " << ba.topic_objects << std::endl <<
    "  topic_ema = " << ba.topic_ema << std::endl <<
    "  topic_objects_closest_point_markers = " << ba.topic_objects_closest_point_markers << std::endl <<
    "  topic_objects_velocity_arrows = " << ba.topic_objects_velocity_arrows << std::endl <<
    "  topic_objects_delta_position_lines = " << ba.topic_objects_delta_position_lines << std::endl <<
    "  topic_objects_width_lines = " << ba.topic_objects_width_lines << std::endl <<
    "  publish_buffer_size = " << ba.publish_buffer_size << std::endl <<
    "  map_frame = " << ba.map_frame << std::endl <<
    "  fixed_frame = " << ba.fixed_frame << std::endl <<
    "  base_frame = " << ba.base_frame << std::endl <<
//     "  merge_threshold_max_angle_gap = " << ba.merge_threshold_max_angle_gap << std::endl <<
//     "  merge_threshold_max_end_points_distance_delta = " << 
//     ba.merge_threshold_max_end_points_distance_delta << std::endl <<
//     "  merge_threshold_max_velocity_direction_delta = " << 
//     ba.merge_threshold_max_velocity_direction_delta << std::endl <<
//     "  merge_threshold_max_speed_delta = " << ba.merge_threshold_max_speed_delta << std::endl <<
    "  PC2_message_x_coordinate_field_name = " << ba.PC2_message_x_coordinate_field_name << std::endl <<
    "  PC2_message_y_coordinate_field_name = " << ba.PC2_message_y_coordinate_field_name << std::endl <<
    "  PC2_message_z_coordinate_field_name = " << ba.PC2_message_z_coordinate_field_name << std::endl <<
    "  PC2_voxel_leaf_size = " << ba.PC2_voxel_leaf_size << std::endl <<
    "  PC2_threshold_z_min = " << ba.PC2_threshold_z_min << std::endl <<
    "  PC2_threshold_z_max = " << ba.PC2_threshold_z_max << std::endl;
  os << "Private Bank Arguments:" << std::endl <<
    "  sensor_frame = " << ba.sensor_frame << std::endl <<
    "  angle_increment = " << ba.angle_increment << std::endl << 
    "  time_increment = " << ba.time_increment << std::endl << 
    "  scan_time = " << ba.scan_time << std::endl << 
    "  range_min = " << ba.range_min << std::endl <<
    "  range_max = " << ba.range_max << std::endl <<
    "  node_name_suffix = " << ba.node_name_suffix << std::endl;
  
  return os;
}

// Bank::Bank()
// {
//   bank_is_initialized = false;
//   bank_is_filled = false;
//   
//   /* Check endianness in case of PointCloud2 */
//   volatile uint32_t dummy = 0x01234567; // If little endian, then 0x67 is the value at the lowest memory address
//   machine_is_little_endian = (*((uint8_t*)(&dummy))) == 0x67;
//   
//   /* Create handle to this node */
//   node = new ros::NodeHandle;
//   
//   /* Create TF listener */
//   tf_listener = new tf::TransformListener;
// }

Bank::Bank(tf2_ros::Buffer * buffer)
{
  bank_is_initialized = false;
  bank_is_filled = false;
  
  /* Check endianness in case of PointCloud2 */
  volatile uint32_t dummy = 0x01234567; // If little endian, then 0x67 is the value at the lowest memory address
  machine_is_little_endian = (*((uint8_t*)(&dummy))) == 0x67;
  
  /* Create handle to this node */
  node = new ros::NodeHandle;
  
  /* Assign TF listener and buffer */
//   tf_listener = listener;
  tf_buffer = buffer;
}

/*
 * Destructor
 */
Bank::~Bank()
{
  for (int i=0; i<bank_argument.nr_scans_in_bank; ++i)
  {
    free(bank_ranges_ema[i]);
  }
  free(bank_ranges_ema);
  free(bank_stamp);
}


/*
 * Check values of bank arguments.
 */
void BankArgument::check()
{
  ROS_ASSERT_MSG(0.0 <= ema_alpha && ema_alpha <= 1.0,
                 "The EMA weighting decrease coefficient must be a value in [0,1]."); 
  
  ROS_ASSERT_MSG(2 <= nr_scans_in_bank, 
                 "There must be at least 2 messages in the bank. Otherwise, velocities cannot be calculated."); 
  
  ROS_ASSERT_MSG(0 < points_per_scan, 
                 "There must be at least 1 point per scan.");
  
  ROS_ASSERT_MSG(angle_max - angle_min <= TWO_PI, 
                 "Angle interval cannot be larger than 2*PI (360 degrees)."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_edge_max_delta_range, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(1 <= object_threshold_min_nr_points, 
                 "An object must consist of at least 1 point.");
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_max_distance, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_min_speed,
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_max_delta_width_in_points, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(0.0 <= object_threshold_min_confidence && object_threshold_min_confidence <= 1.0, 
                 "Cannot be negative or larger than 1.0."); 

//   ROS_ASSERT_MSG(sensor_frame_has_z_axis_forward);
//   ROS_ASSERT_MSG(base_confidence); 
//   ROS_ASSERT_MSG(publish_objects); 
//   ROS_ASSERT_MSG(publish_ema); 
//   ROS_ASSERT_MSG(publish_objects_closest_point_markers); 
//   ROS_ASSERT_MSG(publish_objects_velocity_arrows); 
//   ROS_ASSERT_MSG(velocity_arrows_use_full_gray_scale); 
//   ROS_ASSERT_MSG(velocity_arrows_use_sensor_frame);  
//   ROS_ASSERT_MSG(velocity_arrows_use_base_frame); 
//   ROS_ASSERT_MSG(velocity_arrows_use_fixed_frame); 
  
  ROS_ASSERT_MSG(!publish_objects_velocity_arrows || velocity_arrow_ns != "", 
                 "If publishing velocity arrows, then a name space for them must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_delta_position_lines || delta_position_line_ns != "", 
                 "If publishing delta position lines, then a name space for them must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_width_lines || width_line_ns != "", 
                 "If publishing width lines, then a name space for them must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects || topic_objects != "", 
                 "If publishing MovingObjectArray messages, then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_ema || topic_ema != "", 
                 "If publishing object points via LaserScan visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_closest_point_markers || topic_objects_closest_point_markers != "", 
                 "If publishing the closest point of each object via LaserScan visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_velocity_arrows || topic_objects_velocity_arrows != "", 
                 "If publishing the velocity of each object via MarkerArray visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_delta_position_lines || topic_objects_delta_position_lines != "", 
                 "If publishing the delta position of each object via MarkerArray visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(!publish_objects_width_lines || topic_objects_width_lines != "", 
                 "If publishing the width of each object via MarkerArray visualization messages, "
                 "then a topic for that must be given."); 
  
  ROS_ASSERT_MSG(1 <= publish_buffer_size, 
                 "Publish buffer size must be at least 1."); 
  
  ROS_ASSERT_MSG(map_frame != "", 
                 "Please specify map frame."); 
  
  ROS_ASSERT_MSG(fixed_frame != "", 
                 "Please specify fixed frame."); 
  
  ROS_ASSERT_MSG(base_frame != "", 
                 "Please specify base frame."); 
  
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_angle_gap && merge_threshold_max_angle_gap <= angle_max - angle_min,
//                  "Invalid gap angle.");
//   
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_end_points_distance_delta,
//                  "Distance delta cannot be negative.");
//   
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_velocity_direction_delta && merge_threshold_max_velocity_direction_delta <= M_PI,
//                  "The maximum angle between two vectors is always greater than 0 and smaller than PI.");
//   
//   ROS_ASSERT_MSG(0.0 <= merge_threshold_max_speed_delta,
//                  "Speed delta cannot be negative.");
}

  
/*
 * Check values of bank arguments, PC2-specific.
 */
void BankArgument::check_PC2()
{
  ROS_ASSERT_MSG(PC2_message_x_coordinate_field_name != "", 
                 "Please specify a field name for x coordinates, or do not alter the default value."); 
  
  ROS_ASSERT_MSG(PC2_message_y_coordinate_field_name != "", 
                 "Please specify a field name for y coordinates, or do not alter the default value."); 
  
  ROS_ASSERT_MSG(PC2_message_z_coordinate_field_name != "", 
                 "Please specify a field name for z coordinates, or do not alter the default value."); 
  
  ROS_ASSERT_MSG(0.0 <= PC2_voxel_leaf_size, 
                 "Cannot be negative."); 
  
  ROS_ASSERT_MSG(PC2_threshold_z_min <= PC2_threshold_z_max, 
                 "Invalid thresholds."); 
}


/*
 * Initialize bank based on information received from the user and sensor
 */
void Bank::initBank(BankArgument bank_argument)
{
  if (bank_is_initialized)
  {
    return;
  }
  
  bank_argument.check();
  
  bank_index_put = -1;
  bank_index_newest = -1;
  
  /* Create publishers */
  pub_ema = 
    node->advertise<sensor_msgs::LaserScan>(bank_argument.topic_ema, 
                                            bank_argument.publish_buffer_size);
  pub_objects_closest_point_markers = 
    node->advertise<sensor_msgs::LaserScan>(bank_argument.topic_objects_closest_point_markers, 
                                            bank_argument.publish_buffer_size);
  pub_objects_velocity_arrows = 
    node->advertise<visualization_msgs::MarkerArray>(bank_argument.topic_objects_velocity_arrows, 
                                                     bank_argument.publish_buffer_size);
  pub_objects_delta_position_lines = 
    node->advertise<visualization_msgs::MarkerArray>(bank_argument.topic_objects_delta_position_lines, 
                                                     bank_argument.publish_buffer_size);
  pub_objects_width_lines = 
    node->advertise<visualization_msgs::MarkerArray>(bank_argument.topic_objects_width_lines, 
                                                     bank_argument.publish_buffer_size);
  pub_objects = 
    node->advertise<MovingObjectArray>(bank_argument.topic_objects, 
                                       bank_argument.publish_buffer_size);
  
  /* Init bank */
  this->bank_argument = bank_argument;
  this->bank_argument.sensor_is_360_degrees = fabsf(bank_argument.angle_max - bank_argument.angle_min - TWO_PI) <= 
                                              2.0 * bank_argument.angle_increment; // Safety margin
  
  bank_stamp = (double *) malloc(bank_argument.nr_scans_in_bank * sizeof(double));
  bank_ranges_ema = (float **) malloc(bank_argument.nr_scans_in_bank * sizeof(float*));
  ROS_ASSERT_MSG(bank_ranges_ema != NULL, "Could not allocate buffer space for messages.");
  for (unsigned int i=0; i<bank_argument.nr_scans_in_bank; ++i)
  {
    bank_ranges_ema[i] = (float *) malloc(bank_argument.points_per_scan * sizeof(float));
    ROS_ASSERT_MSG(bank_ranges_ema[i] != NULL, "Could not allocate buffer space message %d.", i);
  }
  
  /* Init messages to publish - init constant fields */
  // EMA (with detected moving objects/objects)
  if (bank_argument.publish_ema)
  {
    msg_ema.header.frame_id = bank_argument.sensor_frame;
    msg_ema.angle_min       = bank_argument.angle_min;
    msg_ema.angle_max       = bank_argument.angle_max;
    msg_ema.angle_increment = bank_argument.angle_increment;
    msg_ema.time_increment  = bank_argument.time_increment;
    msg_ema.scan_time       = bank_argument.scan_time;
    msg_ema.range_min       = bank_argument.range_min;
    msg_ema.range_max       = bank_argument.range_max;
    msg_ema.ranges.resize(bank_argument.points_per_scan); 
    msg_ema.intensities.resize(bank_argument.points_per_scan);
    bzero(&msg_ema.intensities[0], bank_argument.points_per_scan * sizeof(float));
  }
  // Arrows for position and velocity
  if (bank_argument.publish_objects_velocity_arrows)
  {
    if (bank_argument.velocity_arrows_use_sensor_frame)
    {
      msg_objects_velocity_arrow.header.frame_id = bank_argument.sensor_frame;
    }
    else if (bank_argument.velocity_arrows_use_base_frame)
    {
      msg_objects_velocity_arrow.header.frame_id = bank_argument.base_frame;
    }
    else if (bank_argument.velocity_arrows_use_fixed_frame)
    {
      msg_objects_velocity_arrow.header.frame_id = bank_argument.fixed_frame;
    }
    else // map frame
    {
      msg_objects_velocity_arrow.header.frame_id = bank_argument.map_frame;
    }
    msg_objects_velocity_arrow.ns = bank_argument.velocity_arrow_ns;
    msg_objects_velocity_arrow.type = visualization_msgs::Marker::ARROW;
    msg_objects_velocity_arrow.action = visualization_msgs::Marker::ADD;
    //       msg_objects_velocity_arrow.pose.position.x = 0.0;
    //       msg_objects_velocity_arrow.pose.position.y = 0.0;
    //       msg_objects_velocity_arrow.pose.position.z = 0.0;
    //       msg_objects_velocity_arrow.pose.orientation.x = 0.0;
    //       msg_objects_velocity_arrow.pose.orientation.y = 0.0;
    //       msg_objects_velocity_arrow.pose.orientation.z = 0.0;
    msg_objects_velocity_arrow.pose.orientation.w = 1.0;
    msg_objects_velocity_arrow.scale.x = 0.05; // shaft diameter
    msg_objects_velocity_arrow.scale.y = 0.1;  // arrow head diameter
    //       msg_objects_velocity_arrow.scale.z = 0.0;
    //       msg_objects_velocity_arrow.color.r = 0.0;
    //       msg_objects_velocity_arrow.color.g = 0.0;
    //       msg_objects_velocity_arrow.color.b = 0.0;
    msg_objects_velocity_arrow.color.a = 1.0;
    msg_objects_velocity_arrow.lifetime = ros::Duration(0.4);
    msg_objects_velocity_arrow.frame_locked = true;
    msg_objects_velocity_arrow.points.resize(2);
    // msg_objects_velocity_arrow.points[0].z = 0.0;
    // msg_objects_velocity_arrow.points[1].z = 0.0;
  }
  // Lines for delta position
  if (bank_argument.publish_objects_delta_position_lines)
  {
    msg_objects_delta_position_line.header.frame_id = bank_argument.sensor_frame;
    msg_objects_delta_position_line.ns = bank_argument.delta_position_line_ns;
    msg_objects_delta_position_line.type = visualization_msgs::Marker::LINE_STRIP;
    msg_objects_delta_position_line.action = visualization_msgs::Marker::ADD;
    //       msg_objects_delta_position_line.pose.position.x = 0.0;
    //       msg_objects_delta_position_line.pose.position.y = 0.0;
    //       msg_objects_delta_position_line.pose.position.z = 0.0;
    //       msg_objects_delta_position_line.pose.orientation.x = 0.0;
    //       msg_objects_delta_position_line.pose.orientation.y = 0.0;
    //       msg_objects_delta_position_line.pose.orientation.z = 0.0;
    msg_objects_delta_position_line.pose.orientation.w = 1.0; // No translation or rotation
    msg_objects_delta_position_line.scale.x = 0.04; // diameter
    //       msg_objects_delta_position_line.scale.y = 0.0;  
    //       msg_objects_delta_position_line.scale.z = 0.0;
    //       msg_objects_delta_position_line.color.r = 0.0;
    //       msg_objects_delta_position_line.color.g = 0.0;
    msg_objects_delta_position_line.color.b = 1.0; // Blue lines
    msg_objects_delta_position_line.color.a = 1.0;
    msg_objects_delta_position_line.lifetime = ros::Duration(0.4);
    msg_objects_delta_position_line.frame_locked = true;
    msg_objects_delta_position_line.points.resize(2);
//     msg_objects_delta_position_line.points[0].z = 0.0;
//     msg_objects_delta_position_line.points[1].z = 0.0;
  }
  // Lines for width
  if (bank_argument.publish_objects_width_lines)
  {
    msg_objects_width_line.header.frame_id = bank_argument.sensor_frame;
    msg_objects_width_line.ns = bank_argument.width_line_ns;
    msg_objects_width_line.type = visualization_msgs::Marker::LINE_STRIP;
    msg_objects_width_line.action = visualization_msgs::Marker::ADD;
    //       msg_objects_width_line.pose.position.x = 0.0;
    //       msg_objects_width_line.pose.position.y = 0.0;
    //       msg_objects_width_line.pose.position.z = 0.0;
    //       msg_objects_width_line.pose.orientation.x = 0.0;
    //       msg_objects_width_line.pose.orientation.y = 0.0;
    //       msg_objects_width_line.pose.orientation.z = 0.0;
    msg_objects_width_line.pose.orientation.w = 1.0; // No translation or rotation
    msg_objects_width_line.scale.x = 0.02; // diameter
    //       msg_objects_width_line.scale.y = 0.0;  
    //       msg_objects_width_line.scale.z = 0.0;
    //       msg_objects_width_line.color.r = 0.0;
    msg_objects_width_line.color.g = 1.0; // Green lines
//     msg_objects_width_line.color.b = 1.0;
    msg_objects_width_line.color.a = 1.0;
    msg_objects_width_line.lifetime = ros::Duration(0.4);
    msg_objects_width_line.frame_locked = true;
    msg_objects_width_line.points.resize(2);
//     msg_objects_width_line.points[0].z = 0.0;
//     msg_objects_width_line.points[1].z = 0.0;
  }
  // Laserscan points for closest points
  if (bank_argument.publish_objects_closest_point_markers)
  {
    msg_objects_closest_point_markers.header.frame_id = bank_argument.sensor_frame;
    msg_objects_closest_point_markers.angle_min = bank_argument.angle_min;
    msg_objects_closest_point_markers.angle_max = bank_argument.angle_max;
    msg_objects_closest_point_markers.angle_increment = bank_argument.angle_increment;
    msg_objects_closest_point_markers.time_increment = bank_argument.time_increment;
    msg_objects_closest_point_markers.scan_time = bank_argument.scan_time;
    msg_objects_closest_point_markers.range_min = bank_argument.range_min;
    msg_objects_closest_point_markers.range_max = bank_argument.range_max;
    msg_objects_closest_point_markers.intensities.resize(bank_argument.points_per_scan);
    msg_objects_closest_point_markers.ranges.resize(bank_argument.points_per_scan);
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      msg_objects_closest_point_markers.ranges[i] = msg_objects_closest_point_markers.range_max + 10.0;
      msg_objects_closest_point_markers.intensities[i] = 0.0;
    }
  }
  
  // Nr scan points
  bank_ranges_bytes = sizeof(float) * bank_argument.points_per_scan;
  
  // Init sequence nr
  moa_seq = 0;
  
  bank_is_initialized = true;
}


/* 
 * Recursive tracking of an object through history to get the indices of its middle, 
 * left and right points in the oldest scans, along with the sum of all ranges etc.
 */
void Bank::getOldIndices(const float range_min,
                         const float range_max,
                         const unsigned int object_width_in_points,
                         const int          current_level,
                         const unsigned int levels_searched,
                         const unsigned int index_mean,
                         const unsigned int consecutive_failures_to_find_object,
                         const unsigned int threshold_consecutive_failures_to_find_object,
                         int * index_min_old,
                         int * index_mean_old,
                         int * index_max_old,
                         float * range_sum_old,
                         float * range_at_min_index_old,
                         float * range_at_max_index_old)
{
  // Base case reached?
  if (levels_searched == bank_argument.nr_scans_in_bank)
  {
    return;
  }
  
  // To find the end indices of the object,
  int left = index_mean;
  float prev_range = bank_ranges_ema[current_level][index_mean];
  float range_sum = prev_range;
  int right_upper_limit_out_of_bounds = bank_argument.points_per_scan;
  unsigned int width_in_points = 1; // prev_rang = range at index_mean
  
  // Check range
  if (prev_range < range_min ||
      range_max < prev_range)
  {
    *index_min_old = -1;
    *index_mean_old = -1;
    *index_max_old = -1;
    *range_sum_old = 0;
    *range_at_min_index_old = 0;
    *range_at_max_index_old = 0;
    return;
  }
  
  // Search lower index side, with possible wrap around
  bool stopped = false;
  for (int i=index_mean-1; 0<=i; --i)
  {
    // Same type of range check as in the main code
    const float range = bank_ranges_ema[current_level][i];
    if (range_min <= range &&
        range <= range_max &&
        fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
    {
      left = i;
      prev_range = range;
      range_sum += range;
      width_in_points++;
    }
    else
    {
      stopped = true;
      break;
    }
  }
  if (!stopped)
  {
    // Continue from highest index
    for (int i=bank_argument.points_per_scan-1; index_mean<i; --i)
    {
      // Same type of range check as in the main code
      const float range = bank_ranges_ema[current_level][i];
      if (range_min <= range &&
          range <= range_max &&
          fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
      {
        left = i;
        prev_range = range;
        range_sum += range;
        right_upper_limit_out_of_bounds--;
        width_in_points++;
      }
      else
      {
        break;
      }
    }
  }
  // prev_range holds the range at left
  *range_at_min_index_old = prev_range;
  
  // Search higher index side
  stopped = false;
  int right = index_mean;
  prev_range = bank_ranges_ema[current_level][index_mean];
  for (int i=index_mean+1; i<right_upper_limit_out_of_bounds; ++i)
  {
    // Same type of range check as in the main code
    const float range = bank_ranges_ema[current_level][i];
    if (range_min <= range &&
        range <= range_max &&
        fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
    {
      right = i;
      prev_range = range;
      range_sum += range;
      width_in_points++;
    }
    else
    {
      stopped = true;
      break;
    }
  }
  // Here we must make sure that we have not already wrapped around while going left
  if (!stopped && right_upper_limit_out_of_bounds == bank_argument.points_per_scan)
  {
    // Continue from lowest index (0) - we did not wrap around while going left
    for (int i=0; i<left; ++i)
    {
      // Same type of range check as in the main code
      const float range = bank_ranges_ema[current_level][i];
      if (range_min <= range &&
          range <= range_max &&
          fabsf(range - prev_range) <= bank_argument.object_threshold_edge_max_delta_range)
      {
        right = i;
        prev_range = range;
        range_sum += range;
        width_in_points++;
      }
      else
      {
        break;
      }
    }
  }
  // prev_range holds the range at right
  *range_at_max_index_old = prev_range;
  
  // Did we find a valid object?
  unsigned int misses = consecutive_failures_to_find_object;
  if (width_in_points < bank_argument.object_threshold_min_nr_points  ||
      bank_argument.object_threshold_max_delta_width_in_points < abs(width_in_points - object_width_in_points)  ||
      bank_argument.object_threshold_bank_tracking_max_delta_distance  < 
        fabs(range_sum / width_in_points - *range_sum_old / object_width_in_points))
    // range_sum_old holds the range sum of the previous (newer) scanned object
  {
    // No
    misses++;
    if (threshold_consecutive_failures_to_find_object < misses)
    {
      // Return -1 to signal that no index_mean was found
      *index_min_old = -1;
      *index_mean_old = -1;
      *index_max_old = -1;
      *range_sum_old = 0;
      *range_at_min_index_old = 0;
      *range_at_max_index_old = 0;
      return;
    }
  }
  else
  {
    // Yes
    misses = 0;
  }
  
  // If reaching this point, a valid object was found
  // Update end points
  *index_min_old = left;
  *index_mean_old = (left + (width_in_points-1) / 2) % bank_argument.points_per_scan;
  *index_max_old = right;
  *range_sum_old = range_sum;
  
  // Continue searching based on the new index_mean
  getOldIndices(range_min,
                range_max,
                width_in_points,
                (current_level - 1) < 0 ? bank_argument.nr_scans_in_bank - 1 : current_level - 1, // wrap around
                levels_searched + 1,
                *index_mean_old,
                misses,
                threshold_consecutive_failures_to_find_object,
                index_min_old,
                index_mean_old,
                index_max_old,
                range_sum_old, // *range_sum_old was set to range_sum above
                range_at_min_index_old,
                range_at_max_index_old);
}


// /*
//  * Compares consecutive found objects to see if they are to be considered the same object. If so, then a merge of them
//  * is performed
//  */
// void Bank::mergeFoundObjects(MovingObjectArray * moa)
// {
// //   unsigned int nr_objects_found = moa->objects.size();
//   
//   if (1 < moa->objects.size())
//   {
//   
//     MovingObject * object_1 = &moa->objects[0];
//     MovingObject * object_2;
//     std::vector<MovingObject> moa_merged;
//     
//     // Loop through objects
//     for (unsigned int i=1; i<moa->objects.size(); ++i)
//     {
//       object_2 = &moa->objects[i];
//     
//       // Angle betweeen velocity vectors
// //       float velocity_direction_delta = acosf((object_1->velocity.x * object_2->velocity.x +
// //                                               object_1->velocity.y * object_2->velocity.y +
// //                                               object_1->velocity.z * object_2->velocity.z) / 
// //                                              (object_1->speed * object_2->speed));
//       
//       float velocity_direction_delta = acosf((object_1->velocity ^ object_2->velocity) / 
//                                              (object_1->speed * object_2->speed));
//       
//       float gap_angle = object_2->angle_begin - object_1->angle_end;
//       
// //       std::cout << "Comparing objects " << i-1 << " and " << i << std::endl <<
// //                    "  gap_angle = " << gap_angle * 180 / M_PI << std::endl <<
// //                    "  delta_distance_at_angle_gap_ends = " << fabsf(object_2->distance_at_angle_begin - object_1->distance_at_angle_end) << std::endl <<
// //                    "  delta_speed = " << fabsf(object_1->speed * object_2->speed) << std::endl <<
// //                    "  velocity_direction_delta = " << std::left << std::setw(11) << velocity_direction_delta *180 / M_PI<< 
// //                    "    " << std::left << std::setw(9) << object_1->velocity.x << "  " << std::left << std::setw(9) << object_2->velocity.x << std::endl <<
// //                    "                                            " << std::left << std::setw(9) << object_1->velocity.y << "  " << std::left << std::setw(9) << object_2->velocity.y << std::endl <<
// //                    "                                            " << std::left << std::setw(9) << object_1->velocity.z << "  " << std::left << std::setw(9) << object_2->velocity.z << std::endl;
//       
//       // Compare objects - end of object 1 <-> start of object 2, velocities
//       if (gap_angle <= bank_argument.merge_threshold_max_angle_gap &&
//           fabsf(object_2->distance_at_angle_begin - object_1->distance_at_angle_end) <=
//                                                     bank_argument.merge_threshold_max_end_points_distance_delta &&
//           velocity_direction_delta <= bank_argument.merge_threshold_max_velocity_direction_delta &&
//           fabsf(object_1->speed - object_2->speed) <= bank_argument.merge_threshold_max_speed_delta)
//       {
//         std::cout << "MERGING OBJECTS:" << std::endl; // << 
// //                      *object_1 << std::endl << 
// //                      "---------------------------" << std::endl <<
// //                      *object_2 << std::endl << 
// //                      "---------------------------" << std::endl <<
// //                      "---------------------------" << std::endl;
//         // "Join" objects and add the new object to moa_merged
//         MovingObject mo = *object_1;
//         
//         // Update properties
//         mo.seen_width = sqrt( object_1->distance_at_angle_begin *
//                               object_1->distance_at_angle_begin +
//                               object_2->distance_at_angle_end *
//                               object_2->distance_at_angle_end -
//                               2 * object_1->distance_at_angle_begin *
//                                   object_2->distance_at_angle_end *
//                                   cosf (object_2->angle_end - object_1->angle_begin)
//                             );
//         mo.angle_end = object_2->angle_end;
//         mo.distance_at_angle_end = object_2->distance_at_angle_end;
//         mo.distance = (object_1->distance * 
//                        (object_1->angle_end - object_1->angle_begin) / bank_argument.angle_increment +
//                        object_2->distance *
//                        (object_2->angle_end - object_2->angle_begin) / bank_argument.angle_increment) * // Range sum
//                       bank_argument.angle_increment / (object_2->angle_end - object_1->angle_begin);
//         
//                       // The length of the line between objects 1 and 2
//         float gap_width = sqrt( object_1->distance_at_angle_end *
//                                 object_1->distance_at_angle_end +
//                                 object_2->distance_at_angle_begin *
//                                 object_2->distance_at_angle_begin -
//                                 2 * object_1->distance_at_angle_end *
//                                     object_2->distance_at_angle_begin *
//                                     cosf (gap_angle)
//                               );
//         // The center point of the new object lies this far from object_2 on that line 
//         float shift_factor = (object_1->seen_width + gap_width) / 
//                              (object_1->seen_width + object_2->seen_width + 2 * gap_width);
//         
//         mo.position_in_map_frame = shift_factor * object_1->position_in_map_frame + (1 - shift_factor) * object_2->position_in_map_frame;
//         mo.position_in_fixed_frame = shift_factor * object_1->position_in_fixed_frame + (1 - shift_factor) * object_2->position_in_fixed_frame;
//         mo.position_in_base_frame = shift_factor * object_1->position_in_base_frame + (1 - shift_factor) * object_2->position_in_base_frame;
//         mo.position = shift_factor * object_1->position + (1 - shift_factor) * object_2->position;
//         mo.velocity_in_map_frame = 0.5 * (object_1->velocity_in_map_frame + object_2->velocity_in_map_frame);
//         mo.velocity_in_fixed_frame = 0.5 * (object_1->velocity_in_fixed_frame + object_2->velocity_in_fixed_frame);
//         mo.velocity_in_base_frame = 0.5 * (object_1->velocity_in_base_frame + object_2->velocity_in_base_frame);
//         mo.velocity = 0.5 * (object_1->velocity + object_2->velocity);
//         mo.speed_in_map_frame = sqrt (mo.velocity_in_map_frame ^ mo.velocity_in_map_frame);
//         mo.speed_in_fixed_frame = sqrt (mo.velocity_in_fixed_frame ^ mo.velocity_in_fixed_frame);
//         mo.speed_in_base_frame = sqrt (mo.velocity_in_base_frame ^ mo.velocity_in_base_frame);
//         mo.speed = sqrt (mo.velocity ^ mo.velocity);
//         mo.velocity_normalized_in_map_frame = mo.velocity_in_map_frame / mo.speed_in_map_frame;
//         mo.velocity_normalized_in_fixed_frame = mo.velocity_in_fixed_frame / mo.speed_in_fixed_frame;
//         mo.velocity_normalized_in_base_frame = mo.velocity_in_base_frame / mo.speed_in_base_frame;
//         mo.velocity_normalized = mo.velocity / mo.speed;
//         bool object_1_is_closest = object_1->closest_distance < object_2->closest_distance;
//         mo.closest_point_in_map_frame = object_1_is_closest ? object_1->closest_point_in_map_frame : object_2->closest_point_in_map_frame;
//         mo.closest_point_in_fixed_frame = object_1_is_closest ? object_1->closest_point_in_fixed_frame : object_2->closest_point_in_fixed_frame;
//         mo.closest_point_in_base_frame = object_1_is_closest ? object_1->closest_point_in_base_frame : object_2->closest_point_in_base_frame;
//         mo.closest_point = object_1_is_closest ? object_1->closest_point : object_2->closest_point;
//         mo.closest_distance = object_1_is_closest ? object_1->closest_distance : object_2->closest_distance;
//         mo.angle_for_closest_distance = object_1_is_closest ? object_1->angle_for_closest_distance : object_2->angle_for_closest_distance;
//         mo.confidence = object_1->confidence < object_2->confidence ? object_2->confidence : object_1->confidence;
//         
//         // Manipulate moa, object_1 and i
//         moa->objects.at(i-1) = mo;
//         moa->objects.erase(moa->objects.begin()+i);
//         object_1 = &moa->objects[i-1];
//         i--; // Also updated in loop, we want to redo current i
//       }
//       else
//       {
//         // Go to next two objects
//         object_1 = object_2;
//       }
//     }
//     
//     // TODO: Wrap around if 360 degree sensor
//   }
// }

/*
 * Find and report moving objects based on the current content of the bank
 */
void Bank::findAndReportMovingObjects()
{
  // Is the bank filled with scans?
  if (!bank_is_filled)
  {
    ROS_WARN("Bank is not filled yet-cannot report objects!");
    return;
  }
  
  // Moving object array message
  MovingObjectArray moa;
  
  // Old positions of the objects in moa
  MovingObjectArray moa_old_positions;
  
  /* Find objects in the new scans */
  unsigned int nr_objects_found = 0;
  unsigned int nr_object_points = 0;
  const float range_max = (bank_argument.range_max < bank_argument.object_threshold_max_distance  ?
                           bank_argument.range_max : bank_argument.object_threshold_max_distance);
  const float range_min = bank_argument.range_min;
  unsigned int i=0;
  
  // Stamps
  ros::Time old_time = ros::Time(bank_stamp[bank_index_put]);
  ros::Time new_time = ros::Time(bank_stamp[bank_index_newest]);
  
  // Handle 360 degrees sensors!
  unsigned int upper_limit_out_of_bounds_scan_point = bank_argument.points_per_scan;
  while(i<upper_limit_out_of_bounds_scan_point)
  {
    /* Find first valid scan from where we currently are */
    const float range_i = bank_ranges_ema[bank_index_newest][i];
    float object_range_sum = range_i;
    
    // Is i out-of-range?
    if (range_i < bank_argument.range_min ||
        bank_argument.range_max < range_i)
    {
      i++;
      continue;
    }
    
    // i is a valid scan
    nr_object_points = 1;
    float object_range_min = range_i;
    float object_range_max = range_i;
    unsigned int object_range_min_index = i;
    unsigned int object_range_max_index = i;
    
    // Count valid scans that are within the object threshold
    
    float range_at_angle_begin = range_i; // Might be updated later
    float range_at_angle_end;             // Updated later
    unsigned int index_at_angle_begin = i; // Might be updated later
    unsigned int index_at_angle_end;       // Updated later
    float prev_range = range_i;
    unsigned int j=i+1;
    for (; j<bank_argument.points_per_scan; ++j)
    {
      const float range_j = bank_ranges_ema[bank_index_newest][j];
      
      // Range check
      if (bank_argument.range_min <= range_j  &&
          range_j <= bank_argument.range_max  &&
          fabsf(prev_range - range_j) <= bank_argument.object_threshold_edge_max_delta_range)
      {
        // j is part of the current object
        nr_object_points++;
        object_range_sum += range_j;
        
        // Update min and max ranges
        if (range_j < object_range_min) 
        {
          object_range_min = range_j;
          object_range_min_index = j;
        }
        else if (object_range_max < range_j) 
        {
          object_range_max = range_j;
          object_range_max_index = j;
        }
        prev_range = range_j;
      }
      else
      {
        // j is not part of this object
        break;
      }
    }
    
    // Update range at end
    range_at_angle_end = prev_range;
    index_at_angle_end = j-1; // j is not part of the object
    
    // If i is 0 and sensor is 360 deg, then we must also search higher end of scan points and account for these
    if (i == 0 && bank_argument.sensor_is_360_degrees)
    {
      // Start from i again
      prev_range = range_i;
      //Save prev_range so we can restore it?
      
      // Do not step all the way to j again - it has already been considered
      for (unsigned int k=bank_argument.points_per_scan-1; j<k; k--)
      {
        const float range_k = bank_ranges_ema[bank_index_newest][k];
        
        // Range check
        if (bank_argument.range_min <= range_k  &&
            range_k <= bank_argument.range_max  &&
            fabsf(prev_range - range_k) <= bank_argument.object_threshold_edge_max_delta_range)
        {
          // k is part of the current object
          nr_object_points++;
          object_range_sum += range_k;
          
          // Adapt the loop upper limit
          upper_limit_out_of_bounds_scan_point--;
          
          // Update min and max ranges
          if (range_k < object_range_min) 
          {
            object_range_min = range_k;
            object_range_min_index = k;
          }
          else if (object_range_max < range_k) 
          {
            object_range_max = range_k;
            object_range_max_index = k;
          }
          prev_range = range_k;
        }
        else
        {
          // k is not part of this object
          break;
        }
      }
      
      // Update range at begin; it might not be range_i anymore
      range_at_angle_begin = prev_range;
      index_at_angle_begin = upper_limit_out_of_bounds_scan_point;
    }
    
    /* Evaluate the found object (it consists of at least the ith scan) */
    const float distance = object_range_sum / nr_object_points; // Average distance
    const float object_seen_width = sqrt( range_at_angle_begin * 
                                          range_at_angle_begin + 
                                          range_at_angle_end * 
                                          range_at_angle_end - 
                                          2 * range_at_angle_begin * 
                                              range_at_angle_end * 
                                              cosf (bank_argument.angle_increment * nr_object_points)
                                          ); // This is the seen object width using the law of cosine
    
    // Threshold check
    if (bank_argument.object_threshold_min_nr_points <= nr_object_points)
    {
      // Valid object
      nr_objects_found++;
      
      // Recursively derive the min, mean and max indices and the sum of all ranges of the object (if found) 
      // in the oldest scans in the bank
      const unsigned int index_min = index_at_angle_begin;
      const unsigned int index_max = index_at_angle_end;
      const unsigned int index_mean = (index_min + (nr_object_points-1) / 2) % bank_argument.points_per_scan;
                                      // Accounts for 360 deg sensor => i==0 could mean that index_max < index_min
      int index_min_old = -1;
      int index_mean_old = -1;
      int index_max_old = -1;
      float range_sum_old = object_range_sum;
      float range_at_min_index_old = 0;
      float range_at_max_index_old = 0;
      getOldIndices(range_min,
                    range_max,
                    nr_object_points,
                    (bank_index_newest - 1) < 0 ? bank_argument.nr_scans_in_bank - 1 : bank_index_newest - 1,
                    1, // levels searched
                    index_mean,
                    0, // consecutive misses
                    0, // threshold for consecutive misses; 0 -> allow no misses
                    &index_min_old,
                    &index_mean_old,
                    &index_max_old,
                    &range_sum_old,
                    &range_at_min_index_old,
                    &range_at_max_index_old);
      
      // Could we track object?
      if (0 <= index_mean_old)
      {
        // YES!
        // Create a Moving Object
        MovingObject mo;
        MovingObject mo_old_positions;
        
        // Set the expected information
        mo.map_frame = bank_argument.map_frame;
        mo.fixed_frame = bank_argument.fixed_frame;
        mo.base_frame = bank_argument.base_frame;
        mo.header.frame_id = bank_argument.sensor_frame;
        mo.header.seq = nr_objects_found;
        mo.header.stamp = new_time; // ros::Time(bank_stamp[bank_index_newest]);
        mo.seen_width = object_seen_width;
        mo.angle_begin = index_min * bank_argument.angle_increment + bank_argument.angle_min;
        mo.angle_end   = index_max * bank_argument.angle_increment + bank_argument.angle_min;
        const double angle_mean = index_mean * bank_argument.angle_increment + bank_argument.angle_min;
        mo.distance_at_angle_begin = range_at_angle_begin;
        mo.distance_at_angle_end   = range_at_angle_end;
        // Position is dependent on the distance and angle_mean
        // Reference coordinate system (relation to the Lidar):
        //   x: forward
        //   y: left
        //   z: up        
        mo.distance = distance;
        
        // Optical frame?
        if (bank_argument.sensor_frame_has_z_axis_forward)
        {
          // Yes, Z-axis forward, X-axis right, Y-axis down
          mo.position.x = (double) - distance * sinf(angle_mean);
          mo.position.y = 0.0;
          mo.position.z = (double) distance * cosf(angle_mean);
        }
        else
        {
          // No, X-axis forward, Y-axis left, Z-axis up
          mo.position.x = (double) distance * cosf(angle_mean);
          mo.position.y = (double) distance * sinf(angle_mean);
          mo.position.z = 0.0;
        }
        
        // This will be negated rotation around the Y-axis in the case of an optical frame!
        mo.angle_for_closest_distance = object_range_min_index * bank_argument.angle_increment + 
                                        bank_argument.angle_min;
        mo.closest_distance = object_range_min;
        
        // Optical frame?
        if (bank_argument.sensor_frame_has_z_axis_forward)
        {
          // Yes, Z-axis forward, X-axis right, Y-axis down
          mo.closest_point.x = - object_range_min * sinf(mo.angle_for_closest_distance);
          mo.closest_point.y = 0.0;
          mo.closest_point.z = object_range_min * cosf(mo.angle_for_closest_distance);
        }
        else
        {
          // No, X-axis forward, Y-axis left, Z-axis up
          mo.closest_point.x = object_range_min * cosf(mo.angle_for_closest_distance);
          mo.closest_point.y = object_range_min * sinf(mo.angle_for_closest_distance);
          mo.closest_point.z = 0.0;
        }
        
        // Distance from sensor to object at old time
        const unsigned int nr_object_points_old = (index_min_old <= index_max_old) ? 
                                                  (index_max_old - index_min_old + 1) :
                                                  bank_argument.points_per_scan - (index_min_old - index_max_old) + 1;
        const float distance_old = range_sum_old / nr_object_points_old;
        // distance is found at index_mean_old, this is the angle at which distance is found
        const double distance_angle_old = index_mean_old * bank_argument.angle_increment + bank_argument.angle_min;
        // Covered angle
        const double covered_angle_old = nr_object_points_old * bank_argument.angle_increment;
        // Width of old object
        const double object_seen_width_old = sqrt( range_at_min_index_old * 
                                                   range_at_min_index_old + 
                                                   range_at_max_index_old * 
                                                   range_at_max_index_old - 
                                                   2 * range_at_min_index_old * 
                                                       range_at_max_index_old * 
                                                       cosf (covered_angle_old)
                                                 ); // This is the seen object width using the law of cosine
        // Coordinates at old time
        double x_old;
        double y_old;
        double z_old;
        
        if (bank_argument.sensor_frame_has_z_axis_forward)
        {
          // Yes, Z-axis forward, X-axis right, Y-axis down
          x_old = - distance_old * sinf(distance_angle_old);
          y_old = 0.0;
          z_old = distance_old * cosf(distance_angle_old);
        }
        else
        {
          x_old = distance_old * cosf(distance_angle_old);
          y_old = distance_old * sinf(distance_angle_old);
          z_old = 0.0;
        }
        
        mo_old_positions.position.x = x_old;
        mo_old_positions.position.y = y_old;
        mo_old_positions.position.z = z_old;
        
        // Lookup transformation from old position of sensor_frame to new location of sensor_frame
//         bool transform_old_time_map_frame_success = false;
//         bool transform_new_time_map_frame_success = false;
// //         tf::StampedTransform transform_map_frame_old_time;
// //         tf::StampedTransform transform_map_frame_new_time;
//         geometry_msgs::TransformStamped transform_map_frame_old_time;
//         geometry_msgs::TransformStamped transform_map_frame_new_time;
//         bool transform_old_time_fixed_frame_success = false;
//         bool transform_new_time_fixed_frame_success = false;
// //         tf::StampedTransform transform_fixed_frame_old_time;
// //         tf::StampedTransform transform_fixed_frame_new_time;
//         geometry_msgs::TransformStamped transform_fixed_frame_old_time;
//         geometry_msgs::TransformStamped transform_fixed_frame_new_time;
//         bool transform_old_time_base_frame_success = false;
//         bool transform_new_time_base_frame_success = false;
// //         tf::StampedTransform transform_base_frame_old_time;
// //         tf::StampedTransform transform_base_frame_new_time;
//         geometry_msgs::TransformStamped transform_base_frame_old_time;
//         geometry_msgs::TransformStamped transform_base_frame_new_time;
        
//         // Map frame
//         // OLD time
//         try
//         {
// //           const bool transform_available = 
// //           tf_listener->waitForTransform(bank_argument.map_frame,  // Target
// //                                        bank_argument.sensor_frame, // Source
// //                                        ros::Time(bank_stamp[bank_index_put]), 
// //                                        ros::Duration(1.0)); // Timeout
// //           if (transform_available)
// //           {
// //             tf_listener->lookupTransform(bank_argument.map_frame, 
// //                                         bank_argument.sensor_frame, 
// //                                         ros::Time(bank_stamp[bank_index_put]),
// //                                         transform_map_frame_old_time); // Resulting transform
//           transform_map_frame_old_time = tf_buffer->lookupTransform(bank_argument.map_frame, 
//                                                                    bank_argument.sensor_frame, 
//                                                                    ros::Time(bank_stamp[bank_index_put]));
//           transform_old_time_map_frame_success = true;
// //           }
//             
// //           else
// //           {
// //             ROS_ERROR("Cannot determine transform from %s to %s at old time %f.", bank_argument.sensor_frame.c_str(), \
// //                       bank_argument.map_frame.c_str(), bank_stamp[bank_index_put]);
// //           }
//           
//         }
//         catch (tf2::TransformException ex)
//         {
//           ROS_ERROR("Cannot determine transform from %s to %s at old time %f.\n%s",
//                     bank_argument.sensor_frame.c_str(), 
//                     bank_argument.map_frame.c_str(), 
//                     bank_stamp[bank_index_put], 
//                     ex.what());
// //           ROS_ERROR("%s", ex.what());
// //           transform_old_time_map_frame_success = false;
//         }
//         
//         // NEW time
//         try
//         {
//           const bool transform_available = 
//           tf_listener->waitForTransform(bank_argument.map_frame,  // Target
//                                        bank_argument.sensor_frame, // Source
//                                        ros::Time(bank_stamp[bank_index_newest]), 
//                                        ros::Duration(1.0)); // Timeout
//           if (transform_available)
//           {
//             tf_listener->lookupTransform(bank_argument.map_frame, 
//                                         bank_argument.sensor_frame, 
//                                         ros::Time(bank_stamp[bank_index_newest]),
//                                         transform_map_frame_new_time); // Resulting transform
//           }
//           else
//           {
//             ROS_ERROR("Cannot determine transform from %s to %s at new time %f.", bank_argument.sensor_frame.c_str(), \
//                       bank_argument.map_frame.c_str(), bank_stamp[bank_index_newest]);
//           }
//         }
//         catch (tf::TransformException ex)
//         {
//           ROS_ERROR("%s", ex.what());
//           transform_new_time_map_frame_success = false;
//         }
//         
//         // Fixed frame
//         // OLD time
//         try
//         {
//           const bool transform_available = 
//           tf_listener->waitForTransform(bank_argument.fixed_frame,  // Target
//                                        bank_argument.sensor_frame, // Source
//                                        ros::Time(bank_stamp[bank_index_put]), 
//                                        ros::Duration(1.0)); // Timeout
//           if (transform_available)
//           {
//             tf_listener->lookupTransform(bank_argument.fixed_frame, 
//                                         bank_argument.sensor_frame, 
//                                         ros::Time(bank_stamp[bank_index_put]),
//                                         transform_fixed_frame_old_time); // Resulting transform
//           }
//           else
//           {
//             ROS_ERROR("Cannot determine transform from %s to %s at old time %f.", bank_argument.sensor_frame.c_str(), \
//                       bank_argument.fixed_frame.c_str(), bank_stamp[bank_index_put]);
//           }
//         }
//         catch (tf::TransformException ex)
//         {
//           ROS_ERROR("%s", ex.what());
//           transform_old_time_fixed_frame_success = false;
//         }
//         
//         // NEW time
//         try
//         {
//           const bool transform_available = 
//           tf_listener->waitForTransform(bank_argument.fixed_frame,  // Target
//                                        bank_argument.sensor_frame, // Source
//                                        ros::Time(bank_stamp[bank_index_newest]), 
//                                        ros::Duration(1.0)); // Timeout
//           if (transform_available)
//           {
//             tf_listener->lookupTransform(bank_argument.fixed_frame, 
//                                         bank_argument.sensor_frame, 
//                                         ros::Time(bank_stamp[bank_index_newest]),
//                                         transform_fixed_frame_new_time); // Resulting transform
//           }
//           else
//           {
//             ROS_ERROR("Cannot determine transform from %s to %s at new time %f.", bank_argument.sensor_frame.c_str(), \
//                       bank_argument.fixed_frame.c_str(), bank_stamp[bank_index_newest]);
//           }
//         }
//         catch (tf::TransformException ex)
//         {
//           ROS_ERROR("%s", ex.what());
//           transform_new_time_fixed_frame_success = false;
//         }
//         
//         // Base frame
//         // OLD time
//         try
//         {
//           const bool transform_available = 
//           tf_listener->waitForTransform(bank_argument.base_frame,  // Target
//                                        bank_argument.sensor_frame, // Source
//                                        ros::Time(bank_stamp[bank_index_put]), 
//                                        ros::Duration(1.0)); // Timeout
//           if (transform_available)
//           {
//             tf_listener->lookupTransform(bank_argument.base_frame, 
//                                         bank_argument.sensor_frame, 
//                                         ros::Time(bank_stamp[bank_index_put]),
//                                         transform_base_frame_old_time); // Resulting transform
//           }
//           else
//           {
//             ROS_ERROR("Cannot determine transform from %s to %s at old time %f.", bank_argument.sensor_frame.c_str(), \
//                       bank_argument.base_frame.c_str(), bank_stamp[bank_index_put]);
//           }
//         }
//         catch (tf::TransformException ex)
//         {
//           ROS_ERROR("%s", ex.what());
//           transform_old_time_base_frame_success = false;
//         }
//         
//         // NEW time
//         try
//         {
//           const bool transform_available = 
//           tf_listener->waitForTransform(bank_argument.base_frame,  // Target
//                                        bank_argument.sensor_frame, // Source
//                                        ros::Time(bank_stamp[bank_index_newest]), 
//                                        ros::Duration(1.0)); // Timeout
//           if (transform_available)
//           {
//             tf_listener->lookupTransform(bank_argument.base_frame, 
//                                         bank_argument.sensor_frame, 
//                                         ros::Time(bank_stamp[bank_index_newest]),
//                                         transform_base_frame_new_time); // Resulting transform
//           }
//           else
//           {
//             ROS_ERROR("Cannot determine transform from %s to %s at new time %f.", bank_argument.sensor_frame.c_str(), 
//                       bank_argument.base_frame.c_str(), bank_stamp[bank_index_newest]);
//           }
//         }
//         catch (tf::TransformException ex)
//         {
//           ROS_ERROR("%s", ex.what());
//           transform_new_time_base_frame_success = false;
//         }
//         
//         // Coordinates translated
//         tf::Stamped<tf::Point> old_point(tf::Point(x_old, y_old, z_old), 
//                                          ros::Time(bank_stamp[bank_index_put]), 
//                                          bank_argument.sensor_frame);
//         tf::Stamped<tf::Point> new_point(tf::Point(mo.position.x, mo.position.y, mo.position.z), 
//                                          ros::Time(bank_stamp[bank_index_newest]), 
//                                          bank_argument.sensor_frame);
//         tf::Stamped<tf::Point> closest_point(tf::Point(mo.closest_point.x, mo.closest_point.y, mo.closest_point.z), 
//                                              ros::Time(bank_stamp[bank_index_newest]), 
//                                              bank_argument.sensor_frame);
//         
//         tf::Point old_point_in_map_frame;
//         tf::Point new_point_in_map_frame;
//         tf::Point closest_point_in_map_frame;
//         
//         tf::Point old_point_in_fixed_frame;
//         tf::Point new_point_in_fixed_frame;
//         tf::Point closest_point_in_fixed_frame;
//         
//         tf::Point old_point_in_base_frame;
//         tf::Point new_point_in_base_frame;
//         tf::Point closest_point_in_base_frame;
//         
//         if (transform_old_time_map_frame_success && transform_new_time_map_frame_success)
//         {
//           old_point_in_map_frame = transform_map_frame_old_time * old_point;
//           new_point_in_map_frame = transform_map_frame_new_time * new_point;
//           closest_point_in_map_frame = transform_map_frame_new_time * closest_point;
//         }
//         else
//         {
//           old_point_in_map_frame = old_point;
//           new_point_in_map_frame = new_point;
//           closest_point_in_map_frame = closest_point;
//         }
//         
//         if (transform_old_time_fixed_frame_success && transform_new_time_fixed_frame_success)
//         {
//           old_point_in_fixed_frame = transform_fixed_frame_old_time * old_point;
//           new_point_in_fixed_frame = transform_fixed_frame_new_time * new_point;
//           closest_point_in_fixed_frame = transform_fixed_frame_new_time * closest_point;
//         }
//         else
//         {
//           old_point_in_fixed_frame = old_point;
//           new_point_in_fixed_frame = new_point;
//           closest_point_in_fixed_frame = closest_point;
//         }
//         
//         if (transform_old_time_base_frame_success && transform_new_time_base_frame_success)
//         {
//           old_point_in_base_frame = transform_base_frame_old_time * old_point;
//           new_point_in_base_frame = transform_base_frame_new_time * new_point;
//           closest_point_in_base_frame = transform_base_frame_new_time * closest_point;
//         }
//         else
//         {
//           old_point_in_base_frame = old_point;
//           new_point_in_base_frame = new_point;
//           closest_point_in_base_frame = closest_point;
//         }
        
        geometry_msgs::PointStamped in;
        geometry_msgs::PointStamped out;
        
        // Transform old point into map, fixed and base frames at old_time
        in.header.frame_id = bank_argument.sensor_frame;
        in.header.stamp = old_time;
        in.point = mo_old_positions.position;
        
        try {
          tf_buffer->transform(in, // mo_old_positions.position, 
                              out, // mo_old_positions.position_in_map_frame, 
                              bank_argument.map_frame, 
                              old_time,
                              bank_argument.fixed_frame);
          mo_old_positions.position_in_map_frame = out.point;
          
          
          tf_buffer->transform(in, // mo_old_positions.position, 
                              out, // mo_old_positions.position_in_fixed_frame, 
                              bank_argument.fixed_frame, 
                              old_time,
                              bank_argument.fixed_frame);
          mo_old_positions.position_in_fixed_frame = out.point;
          
          tf_buffer->transform(in, // mo_old_positions.position, 
                              out, // mo_old_positions.position_in_base_frame, 
                              bank_argument.base_frame, 
                              old_time,
                              bank_argument.fixed_frame);
          mo_old_positions.position_in_base_frame = out.point;
          
          // Transform new point into map, fixed and base frames at new_time
          in.header.stamp = new_time;
          in.point = mo.position;
          
          tf_buffer->transform(in, // mo.position, 
                              out, // mo.position_in_map_frame, 
                              bank_argument.map_frame, 
                              new_time,
                              bank_argument.fixed_frame);
          mo.position_in_map_frame = out.point;
          
          tf_buffer->transform(in, // mo.position, 
                              out, // mo.position_in_fixed_frame, 
                              bank_argument.fixed_frame, 
                              new_time,
                              bank_argument.fixed_frame);
          mo.position_in_fixed_frame = out.point;
          
          tf_buffer->transform(in, // mo.position, 
                              out, // mo.position_in_base_frame, 
                              bank_argument.base_frame, 
                              new_time,
                              bank_argument.fixed_frame);
          mo.position_in_base_frame = out.point;
          
          // Transform closest point into map, fixed and base frames at new_time
          in.point = mo.closest_point;
          
          tf_buffer->transform(in, // mo.closest_point, 
                              out, // mo.closest_point_in_map_frame, 
                              bank_argument.map_frame, 
                              new_time,
                              bank_argument.fixed_frame);
          mo.closest_point_in_map_frame = out.point;
          
          tf_buffer->transform(in, // mo.closest_point, 
                              out, // mo.closest_point_in_fixed_frame, 
                              bank_argument.fixed_frame, 
                              new_time,
                              bank_argument.fixed_frame);
          mo.closest_point_in_fixed_frame = out.point;
          
          tf_buffer->transform(in, // mo.closest_point, 
                              out, // mo.closest_point_in_base_frame, 
                              bank_argument.base_frame, 
                              new_time,
                              bank_argument.fixed_frame);
          mo.closest_point_in_base_frame = out.point;
        } 
        catch (tf2::TransformException e)
        {
          ROS_ERROR_STREAM("Caught some exception: " << e.what());
        }
        
        // Check how object has moved
        const double dx_map    = mo.position_in_map_frame.x   - mo_old_positions.position_in_map_frame.x;
        const double dy_map    = mo.position_in_map_frame.y   - mo_old_positions.position_in_map_frame.y;
        const double dz_map    = mo.position_in_map_frame.z   - mo_old_positions.position_in_map_frame.z;
        const double dx_fixed  = mo.position_in_fixed_frame.x - mo_old_positions.position_in_fixed_frame.x;
        const double dy_fixed  = mo.position_in_fixed_frame.y - mo_old_positions.position_in_fixed_frame.y;
        const double dz_fixed  = mo.position_in_fixed_frame.z - mo_old_positions.position_in_fixed_frame.z;
        const double dx_base   = mo.position_in_base_frame.x  - mo_old_positions.position_in_base_frame.x;
        const double dy_base   = mo.position_in_base_frame.y  - mo_old_positions.position_in_base_frame.y;
        const double dz_base   = mo.position_in_base_frame.z  - mo_old_positions.position_in_base_frame.z;
        const double dx_sensor = mo.position.x - x_old;
        const double dy_sensor = mo.position.y - y_old;
        const double dz_sensor = mo.position.z - z_old;
        
        
//         // Set old position in map_frame
//         mo_old_positions.position_in_map_frame.x = old_point_in_map_frame.x();
//         mo_old_positions.position_in_map_frame.y = old_point_in_map_frame.y();
//         mo_old_positions.position_in_map_frame.z = old_point_in_map_frame.z();
//         
//         // Set old position in fixed_frame
//         mo_old_positions.position_in_fixed_frame.x = old_point_in_fixed_frame.x();
//         mo_old_positions.position_in_fixed_frame.y = old_point_in_fixed_frame.y();
//         mo_old_positions.position_in_fixed_frame.z = old_point_in_fixed_frame.z();
//         
//         // Set old position in base_frame
//         mo_old_positions.position_in_base_frame.x = old_point_in_base_frame.x();
//         mo_old_positions.position_in_base_frame.y = old_point_in_base_frame.y();
//         mo_old_positions.position_in_base_frame.z = old_point_in_base_frame.z();
//         
//         // Set position in map_frame
//         mo.position_in_map_frame.x = new_point_in_map_frame.x();
//         mo.position_in_map_frame.y = new_point_in_map_frame.y();
//         mo.position_in_map_frame.z = new_point_in_map_frame.z();
//         
//         // Set position in fixed_frame
//         mo.position_in_fixed_frame.x = new_point_in_fixed_frame.x();
//         mo.position_in_fixed_frame.y = new_point_in_fixed_frame.y();
//         mo.position_in_fixed_frame.z = new_point_in_fixed_frame.z();
//         
//         // Set position in base_frame
//         mo.position_in_base_frame.x = new_point_in_base_frame.x();
//         mo.position_in_base_frame.y = new_point_in_base_frame.y();
//         mo.position_in_base_frame.z = new_point_in_base_frame.z();
//         
//         // Set closest point in map_frame
//         mo.closest_point_in_map_frame.x = closest_point_in_map_frame.x();
//         mo.closest_point_in_map_frame.y = closest_point_in_map_frame.y();
//         mo.closest_point_in_map_frame.z = closest_point_in_map_frame.z();
//         
//         // Set closest point in fixed_frame
//         mo.closest_point_in_fixed_frame.x = closest_point_in_fixed_frame.x();
//         mo.closest_point_in_fixed_frame.y = closest_point_in_fixed_frame.y();
//         mo.closest_point_in_fixed_frame.z = closest_point_in_fixed_frame.z();
//         
//         // Set closest point in base_frame
//         mo.closest_point_in_base_frame.x = closest_point_in_base_frame.x();
//         mo.closest_point_in_base_frame.y = closest_point_in_base_frame.y();
//         mo.closest_point_in_base_frame.z = closest_point_in_base_frame.z();
//         
//         // Check how object has moved
//         const float dx_map =   new_point_in_map_frame.x() -   old_point_in_map_frame.x();
//         const float dy_map =   new_point_in_map_frame.y() -   old_point_in_map_frame.y();
//         const float dz_map =   new_point_in_map_frame.z() -   old_point_in_map_frame.z();
//         const float dx_fixed = new_point_in_fixed_frame.x() - old_point_in_fixed_frame.x();
//         const float dy_fixed = new_point_in_fixed_frame.y() - old_point_in_fixed_frame.y();
//         const float dz_fixed = new_point_in_fixed_frame.z() - old_point_in_fixed_frame.z();
//         const float dx_base =  new_point_in_base_frame.x() -  old_point_in_base_frame.x();
//         const float dy_base =  new_point_in_base_frame.y() -  old_point_in_base_frame.y();
//         const float dz_base =  new_point_in_base_frame.z() -  old_point_in_base_frame.z();
//         const float dx_sensor = mo.position.x - x_old;
//         const float dy_sensor = mo.position.y - y_old;
//         const float dz_sensor = mo.position.z - z_old;
        
        // And with what velocity
//         const double dt = bank_stamp[bank_index_newest] - bank_stamp[bank_index_put];
        const double dt = mo.header.stamp.toSec() - bank_stamp[bank_index_put];
//         ROS_ERROR_STREAM("newest stamp = " << mo.header.stamp.toSec() << std::endl << "oldest stamp = " << bank_stamp[bank_index_put] << std::endl << "dt = " << dt << std::endl);
        mo.velocity.x = dx_sensor / dt;
        mo.velocity.y = dy_sensor / dt;
        mo.velocity.z = dz_sensor / dt;
        mo.velocity_in_map_frame.x = dx_map / dt;
        mo.velocity_in_map_frame.y = dy_map / dt;
        mo.velocity_in_map_frame.z = dz_map / dt;
        mo.velocity_in_fixed_frame.x = dx_fixed / dt;
        mo.velocity_in_fixed_frame.y = dy_fixed / dt;
        mo.velocity_in_fixed_frame.z = dz_fixed / dt;
        mo.velocity_in_base_frame.x = dx_base / dt;
        mo.velocity_in_base_frame.y = dy_base / dt;
        mo.velocity_in_base_frame.z = dz_base / dt;
        
        // Calculate speed and normalized velocity
        mo.speed = sqrt(mo.velocity.x * mo.velocity.x  +  
                        mo.velocity.y * mo.velocity.y  +  
                        mo.velocity.z * mo.velocity.z);
        mo.speed_in_map_frame = sqrt(mo.velocity_in_map_frame.x * mo.velocity_in_map_frame.x  +
                                     mo.velocity_in_map_frame.y * mo.velocity_in_map_frame.y  +
                                     mo.velocity_in_map_frame.z * mo.velocity_in_map_frame.z);
        mo.speed_in_fixed_frame = sqrt(mo.velocity_in_fixed_frame.x * mo.velocity_in_fixed_frame.x  +
                                       mo.velocity_in_fixed_frame.y * mo.velocity_in_fixed_frame.y  +
                                       mo.velocity_in_fixed_frame.z * mo.velocity_in_fixed_frame.z);
        mo.speed_in_base_frame = sqrt(mo.velocity_in_base_frame.x * mo.velocity_in_base_frame.x  +
                                      mo.velocity_in_base_frame.y * mo.velocity_in_base_frame.y  +
                                      mo.velocity_in_base_frame.z * mo.velocity_in_base_frame.z);
        
        // Avoid division by 0
        if (0 < mo.speed)
        {
          mo.velocity_normalized.x = mo.velocity.x / mo.speed;
          mo.velocity_normalized.y = mo.velocity.y / mo.speed;
          mo.velocity_normalized.z = mo.velocity.z / mo.speed;
        }
        else
        {
          mo.velocity_normalized.x = 0.0;
          mo.velocity_normalized.y = 0.0;
          mo.velocity_normalized.z = 0.0;
        }
        if (0 < mo.speed_in_map_frame)
        {
          mo.velocity_normalized_in_map_frame.x = mo.velocity_in_map_frame.x / mo.speed_in_map_frame;
          mo.velocity_normalized_in_map_frame.y = mo.velocity_in_map_frame.y / mo.speed_in_map_frame;
          mo.velocity_normalized_in_map_frame.z = mo.velocity_in_map_frame.z / mo.speed_in_map_frame;
        }
        else
        {
          mo.velocity_normalized_in_map_frame.x = 0.0;
          mo.velocity_normalized_in_map_frame.y = 0.0;
          mo.velocity_normalized_in_map_frame.z = 0.0;
        }
        if (0 < mo.speed_in_fixed_frame)
        {
          mo.velocity_normalized_in_fixed_frame.x = mo.velocity_in_fixed_frame.x / mo.speed_in_fixed_frame;
          mo.velocity_normalized_in_fixed_frame.y = mo.velocity_in_fixed_frame.y / mo.speed_in_fixed_frame;
          mo.velocity_normalized_in_fixed_frame.z = mo.velocity_in_fixed_frame.z / mo.speed_in_fixed_frame;
        }
        else
        {
          mo.velocity_normalized_in_fixed_frame.x = 0.0;
          mo.velocity_normalized_in_fixed_frame.y = 0.0;
          mo.velocity_normalized_in_fixed_frame.z = 0.0;
        }
        if (0 < mo.speed_in_base_frame)
        {
          mo.velocity_normalized_in_base_frame.x = mo.velocity_in_base_frame.x / mo.speed_in_base_frame;
          mo.velocity_normalized_in_base_frame.y = mo.velocity_in_base_frame.y / mo.speed_in_base_frame;
          mo.velocity_normalized_in_base_frame.z = mo.velocity_in_base_frame.z / mo.speed_in_base_frame;
        }
        else
        {
          mo.velocity_normalized_in_base_frame.x = 0.0;
          mo.velocity_normalized_in_base_frame.y = 0.0;
          mo.velocity_normalized_in_base_frame.z = 0.0;
        }
        
        // Threshold check
        if (bank_argument.object_threshold_min_speed <= mo.speed || 
            bank_argument.object_threshold_min_speed <= mo.speed_in_map_frame || 
            bank_argument.object_threshold_min_speed <= mo.speed_in_fixed_frame || 
            bank_argument.object_threshold_min_speed <= mo.speed_in_base_frame)
        {
          // We believe that the object is moving in relation to at least one of the frames
          ROS_DEBUG_STREAM("Moving object:" << std::endl \
                        << "               (sensor)  x=" << std::setw(12) << std::left << mo.position.x \
                        <<                       "   y=" << std::setw(12) << std::left << mo.position.y \
                        <<                       "   z=" << std::setw(12) << std::left << mo.position.z \
                        <<                       std::endl \
                        << "                        vx=" << std::setw(12) << std::left << mo.velocity.x \
                        <<                       "  vy=" << std::setw(12) << std::left << mo.velocity.y \
                        <<                       "  vz=" << std::setw(12) << std::left << mo.velocity.z \
                        <<                       "  speed=" << mo.speed \
                        <<                       std::endl \
                        << "               (map)     x=" << std::setw(12) << std::left << mo.position_in_map_frame.x  \
                        <<                       "   y=" << std::setw(12) << std::left << mo.position_in_map_frame.y \
                        <<                       "   z=" << std::setw(12) << std::left << mo.position_in_map_frame.z \
                        <<                       std::endl \
                        << "                        vx=" << std::setw(12) << std::left << mo.velocity_in_map_frame.x \
                        <<                       "  vy=" << std::setw(12) << std::left << mo.velocity_in_map_frame.y \
                        <<                       "  vz=" << std::setw(12) << std::left << mo.velocity_in_map_frame.z  \
                        <<                       "  speed=" << mo.speed_in_map_frame \
                        <<                       std::endl \
                        << "               (fixed)   x=" << std::setw(12) << std::left << mo.position_in_fixed_frame.x \
                        <<                       "   y=" << std::setw(12) << std::left << mo.position_in_fixed_frame.y \
                        <<                       "   z=" << std::setw(12) << std::left << mo.position_in_fixed_frame.z \
                        <<                       std::endl \
                        << "                        vx=" << std::setw(12) << std::left << mo.velocity_in_fixed_frame.x \
                        <<                       "  vy=" << std::setw(12) << std::left << mo.velocity_in_fixed_frame.y \
                        <<                       "  vz=" << std::setw(12) << std::left << mo.velocity_in_fixed_frame.z \
                        <<                       "  speed=" << mo.speed_in_fixed_frame \
                        <<                       std::endl \
                        << "               (base)    x=" << std::setw(12) << std::left << mo.position_in_base_frame.x \
                        <<                       "   y=" << std::setw(12) << std::left << mo.position_in_base_frame.y \
                        <<                       "   z=" << std::setw(12) << std::left << mo.position_in_base_frame.z \
                        <<                       std::endl \
                        << "                        vx=" << std::setw(12) << std::left << mo.velocity_in_base_frame.x \
                        <<                       "  vy=" << std::setw(12) << std::left << mo.velocity_in_base_frame.y \
                        <<                       "  vz=" << std::setw(12) << std::left << mo.velocity_in_base_frame.z \
                        <<                       "  speed=" << mo.speed_in_base_frame \
                        <<                       std::endl);
          
          // Calculate confidence value using the user-defined function
          mo.confidence = calculateConfidence(mo, 
                                              bank_argument, 
                                              dt, 
                                              object_seen_width_old);
          
          // Bound the value to [0,1]
          mo.confidence = (mo.confidence < 0.0  ?  0.0  :  mo.confidence);
          mo.confidence = (mo.confidence < 1.0  ?  mo.confidence  :  1.0);
          
          // Are we confident enough to report this object?
          if (bank_argument.object_threshold_min_confidence <= mo.confidence)
          {
            // Adapt EMA message intensities
            if (bank_argument.publish_ema)
            {
              // Are we avoiding wrapping around the bank edges?
              if (index_min <= index_max)
              {
                // YES
                for (unsigned int k=index_min; k<=index_max; ++k)
                {
                  msg_ema.intensities[k] = 300.0f;
                }
              }
              else
              {
                // NO - we are wrapping around
                // index_max < index_min
                for (unsigned int k=index_min; k<bank_argument.points_per_scan; ++k)
                {
                  msg_ema.intensities[k] = 300.0f;
                }
                for (unsigned int k=0; k<index_max; ++k)
                {
                  msg_ema.intensities[k] = 300.0f;
                }
              }
            }
            
            // Push back the moving object info to the msg
            moa.objects.push_back(mo);
            moa_old_positions.objects.push_back(mo_old_positions);
          }
        }
      }
    }
    
    i = index_at_angle_end + 1;
    nr_object_points = 0;
  }
  
  // Filter found objects
//   mergeFoundObjects(&moa);
  
  // Moving object array message
  ++moa_seq;
  if (bank_argument.publish_objects && 0 < moa.objects.size())
  {
    moa.origin_node_name = ros::this_node::getName() + bank_argument.node_name_suffix;
    
    // Publish MOA message
    pub_objects.publish(moa);
  }
  
  // Save timestamp
  ros::Time now = ros::Time::now();
  
  // EMA message
  if (bank_argument.publish_ema)
  {
    // Copy ranges and set header
    memcpy(msg_ema.ranges.data(), &bank_ranges_ema[bank_index_newest][0], bank_ranges_bytes);
    msg_ema.header.seq = moa_seq;
    msg_ema.header.stamp = now;
    
    // Publish EMA message
    pub_ema.publish(msg_ema);
  }
  
  // Update headers of the marker, arrow, delta position and width messages
  if (bank_argument.publish_objects_closest_point_markers)
  {
    msg_objects_closest_point_markers.header.stamp = now;
    msg_objects_closest_point_markers.header.seq = moa_seq;
  }
  if (bank_argument.publish_objects_velocity_arrows)
  {
    msg_objects_velocity_arrow.header.stamp = now;
    msg_objects_velocity_arrow.header.seq = moa_seq;
  }
  if (bank_argument.publish_objects_delta_position_lines)
  {
    msg_objects_delta_position_line.header.stamp = now;
    msg_objects_delta_position_line.header.seq = moa_seq;
  }
  if (bank_argument.publish_objects_width_lines)
  {
    msg_objects_width_line.header.stamp = now;
    msg_objects_width_line.header.seq = moa_seq;
  }
  
  // Go through found objects
  MovingObject * mo;
  MovingObject * mo_old_positions;
  const unsigned int nr_moving_objects_found = moa.objects.size();
  for (unsigned int i=0; i<nr_moving_objects_found; ++i)
  {
    mo = &moa.objects[i];
    mo_old_positions = &moa_old_positions.objects[i];
    
    // Laserscan Marker (square)
    if (bank_argument.publish_objects_closest_point_markers)
    {
      // Find index for closest range for this object - reverse calculation
      const unsigned int distance_min_index = 
        round((mo->angle_for_closest_distance - bank_argument.angle_min) / bank_argument.angle_increment);
      msg_objects_closest_point_markers.ranges[distance_min_index] = mo->closest_distance;
      msg_objects_closest_point_markers.intensities[distance_min_index] = 1000;
    }
    
    // Visualization Marker (velocity arrow)
    if (bank_argument.publish_objects_velocity_arrows)
    {
      msg_objects_velocity_arrow.id = i;
      if (bank_argument.velocity_arrows_use_sensor_frame)
      {
        // Origin: (the size of points is 2)
        msg_objects_velocity_arrow.points[0].x = mo->position.x;
        msg_objects_velocity_arrow.points[0].y = mo->position.y;
        msg_objects_velocity_arrow.points[0].z = mo->position.z;
        // End:
        msg_objects_velocity_arrow.points[1].x = mo->position.x + mo->velocity.x;
        msg_objects_velocity_arrow.points[1].y = mo->position.y + mo->velocity.y;
        msg_objects_velocity_arrow.points[1].z = mo->position.z + mo->velocity.z;
      }
      else if (bank_argument.velocity_arrows_use_base_frame)
      {
        // Origin (the size of points is 2)
        msg_objects_velocity_arrow.points[0].x = mo->position_in_base_frame.x;
        msg_objects_velocity_arrow.points[0].y = mo->position_in_base_frame.y;
        msg_objects_velocity_arrow.points[0].z = mo->position_in_base_frame.z;
        // End:
        msg_objects_velocity_arrow.points[1].x = mo->position_in_base_frame.x + mo->velocity_in_base_frame.x;
        msg_objects_velocity_arrow.points[1].y = mo->position_in_base_frame.y + mo->velocity_in_base_frame.y;
        msg_objects_velocity_arrow.points[1].z = mo->position_in_base_frame.z + mo->velocity_in_base_frame.z;
      }
      else if (bank_argument.velocity_arrows_use_fixed_frame)
      {
        // Origin (the size of points is 2)
        msg_objects_velocity_arrow.points[0].x = mo->position_in_fixed_frame.x;
        msg_objects_velocity_arrow.points[0].y = mo->position_in_fixed_frame.y;
        msg_objects_velocity_arrow.points[0].z = mo->position_in_fixed_frame.z;
        // End:
        msg_objects_velocity_arrow.points[1].x = mo->position_in_fixed_frame.x + mo->velocity_in_fixed_frame.x;
        msg_objects_velocity_arrow.points[1].y = mo->position_in_fixed_frame.y + mo->velocity_in_fixed_frame.y;
        msg_objects_velocity_arrow.points[1].z = mo->position_in_fixed_frame.z + mo->velocity_in_fixed_frame.z;
      }
      else
      {
        // Origin (the size of points is 2)
        msg_objects_velocity_arrow.points[0].x = mo->position_in_map_frame.x;
        msg_objects_velocity_arrow.points[0].y = mo->position_in_map_frame.y;
        msg_objects_velocity_arrow.points[0].z = mo->position_in_map_frame.z;
        // End:
        msg_objects_velocity_arrow.points[1].x = mo->position_in_map_frame.x + mo->velocity_in_map_frame.x;
        msg_objects_velocity_arrow.points[1].y = mo->position_in_map_frame.y + mo->velocity_in_map_frame.y;
        msg_objects_velocity_arrow.points[1].z = mo->position_in_map_frame.z + mo->velocity_in_map_frame.z;
      }
      
      // Color of the arrow represents the confidence black=low, white=high
      if (bank_argument.velocity_arrows_use_full_gray_scale && bank_argument.object_threshold_min_confidence < 1)
      {
        const double adapted_confidence = (mo->confidence - bank_argument.object_threshold_min_confidence) / 
                                          (1 - bank_argument.object_threshold_min_confidence);
        msg_objects_velocity_arrow.color.r = adapted_confidence;
        msg_objects_velocity_arrow.color.g = adapted_confidence;
        msg_objects_velocity_arrow.color.b = adapted_confidence;
      }
      else
      {
        msg_objects_velocity_arrow.color.r = mo->confidence;
        msg_objects_velocity_arrow.color.g = mo->confidence;
        msg_objects_velocity_arrow.color.b = mo->confidence;
      }
      
      // Add to array of markers
      msg_objects_velocity_arrows.markers.push_back(msg_objects_velocity_arrow);
    }
    
    // Visualization Marker (delta position)
    if (bank_argument.publish_objects_delta_position_lines)
    {
      msg_objects_delta_position_line.id = i;

      // Copy line end points
      msg_objects_delta_position_line.points[0].x = mo_old_positions->position.x;
      msg_objects_delta_position_line.points[0].y = mo_old_positions->position.y;
      msg_objects_delta_position_line.points[0].z = mo_old_positions->position.z;
      msg_objects_delta_position_line.points[1].x = mo->position.x;
      msg_objects_delta_position_line.points[1].y = mo->position.y;
      msg_objects_delta_position_line.points[1].z = mo->position.z;

      // Add to array of markers
      msg_objects_delta_position_lines.markers.push_back(msg_objects_delta_position_line);
    }
    
    // Visualization Marker (width)
    if (bank_argument.publish_objects_width_lines)
    {
      msg_objects_width_line.id = i;

      // Calculate line end points
      if (!bank_argument.sensor_frame_has_z_axis_forward)
      {
        // angle_min
        msg_objects_width_line.points[0].x = mo->distance_at_angle_begin * cosf(mo->angle_begin);
        msg_objects_width_line.points[0].y = mo->distance_at_angle_begin * sinf(mo->angle_begin);
        msg_objects_width_line.points[0].z = 0.0;
        // angle_max
        msg_objects_width_line.points[1].x = mo->distance_at_angle_end * cosf(mo->angle_end);
        msg_objects_width_line.points[1].y = mo->distance_at_angle_end * sinf(mo->angle_end);
        msg_objects_width_line.points[1].z = 0.0;
      }
      else
      {
        // angle_min
        msg_objects_width_line.points[0].x = -mo->distance_at_angle_begin * sinf(mo->angle_begin);
        msg_objects_width_line.points[0].y = 0.0;
        msg_objects_width_line.points[0].z = mo->distance_at_angle_begin * cosf(mo->angle_begin);
        // angle_max
        msg_objects_width_line.points[1].x = -mo->distance_at_angle_end * sinf(mo->angle_end);
        msg_objects_width_line.points[1].y = 0.0;
        msg_objects_width_line.points[1].z = mo->distance_at_angle_end * cosf(mo->angle_end);
      }
      
      // Add to array of markers
      msg_objects_width_lines.markers.push_back(msg_objects_width_line);
    }
  }
  
  // Publish if we are supposed to
  if (bank_argument.publish_objects_closest_point_markers)
  {
    pub_objects_closest_point_markers.publish(msg_objects_closest_point_markers);
  }
  
  // Dito
  if (bank_argument.publish_objects_velocity_arrows)
  {
    pub_objects_velocity_arrows.publish(msg_objects_velocity_arrows);
  }
  
  // Dito
  if (bank_argument.publish_objects_delta_position_lines)
  {
    pub_objects_delta_position_lines.publish(msg_objects_delta_position_lines);
  }
  
  // Dito
  if (bank_argument.publish_objects_width_lines)
  {
    pub_objects_width_lines.publish(msg_objects_width_lines);
  }
  
  // Reset range and intensity of markers and delete found objects
  if (bank_argument.publish_objects_closest_point_markers)
  {
    for (unsigned int i=0; i<nr_moving_objects_found; ++i)
    {
      mo = &moa.objects[i];
      const unsigned int distance_min_index = 
        round((mo->angle_for_closest_distance - bank_argument.angle_min) / bank_argument.angle_increment);
      msg_objects_closest_point_markers.ranges[distance_min_index] = msg_objects_closest_point_markers.range_max + 10.0;
      msg_objects_closest_point_markers.intensities[distance_min_index] = 0.0;
    }
  }
  if (bank_argument.publish_objects_velocity_arrows)
  {
    msg_objects_velocity_arrows.markers.clear();
  }
  if (bank_argument.publish_objects_delta_position_lines)
  {
    msg_objects_delta_position_lines.markers.clear();
  }
  if (bank_argument.publish_objects_width_lines)
  {
    msg_objects_width_lines.markers.clear();
  }
  if (bank_argument.publish_ema)
  {
    bzero(msg_ema.intensities.data(), bank_ranges_bytes);
  }
}


/* HANDLING ENDIANNESS */
void Bank::reverseBytes(byte_t * bytes, unsigned int nr_bytes)
{
  const unsigned int half_way = nr_bytes/2;
  for (unsigned int i=0; i<half_way; ++i)
  {
    const byte_t tmp_byte = bytes[i];
    bytes[i] = bytes[nr_bytes - i - 1];
    bytes[nr_bytes - i - 1] = tmp_byte;
  }
}


/*
 * Read offsets and nr of bytes from PointCloud2 message
 */
int Bank::getOffsetsAndBytes(BankArgument bank_argument, sensor_msgs::PointCloud2::ConstPtr msg)
{
  PC2_message_x_offset = -1;
  PC2_message_x_bytes = -1;
  PC2_message_y_offset = -1;
  PC2_message_y_bytes = -1;
  PC2_message_z_offset = -1;
  PC2_message_z_bytes = -1;
  const bool must_reverse_bytes = (msg->is_bigendian != !machine_is_little_endian);
  const unsigned int fields = msg->fields.size();
  byte_t tmp_byte4[4]; // Used for reading offset, count = 1 (ROS: uint32)
  byte_t tmp_byte;     // Used for reading datatype 
  
  for (unsigned int i=0; i<fields; ++i)
  {
    // X
    if (strcmp(bank_argument.PC2_message_x_coordinate_field_name.c_str(), 
               msg->fields[i].name.c_str()) == 0)
    {
      // Read offset
      memcpy(&tmp_byte4[0], &msg->fields[i].offset, 4);
      if (must_reverse_bytes)
      {
        reverseBytes(tmp_byte4, 4);
      }
      memcpy(&PC2_message_x_offset, tmp_byte4, 4);
      
      // Read datatype
      tmp_byte = msg->fields[i].datatype;
      if (tmp_byte == sensor_msgs::PointField::INT8 || 
          tmp_byte == sensor_msgs::PointField::UINT8)
      {
        PC2_message_x_bytes = 1;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT16 || 
               tmp_byte == sensor_msgs::PointField::UINT16)
      {
        PC2_message_x_bytes = 2;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT32 || 
               tmp_byte == sensor_msgs::PointField::UINT32 || 
               tmp_byte == sensor_msgs::PointField::FLOAT32)
      {
        PC2_message_x_bytes = 4;
      }
      else if (tmp_byte == sensor_msgs::PointField::FLOAT64)
      {
        PC2_message_x_bytes = 8;
      }
      else
      {
        ROS_ERROR("Cannot determine number of bytes for X coordinate");
        return -1;
      }
    }
    // Y
    else if (strcmp(bank_argument.PC2_message_y_coordinate_field_name.c_str(), 
                    msg->fields[i].name.c_str()) == 0)
    {
      // Read offset
      memcpy(&tmp_byte4[0], &msg->fields[i].offset, 4);
      if (must_reverse_bytes)
      {
        reverseBytes(tmp_byte4, 4);
      }
      memcpy(&PC2_message_y_offset, tmp_byte4, 4);
      
      // Read datatype
      tmp_byte = msg->fields[i].datatype;
      if (tmp_byte == sensor_msgs::PointField::INT8 || 
          tmp_byte == sensor_msgs::PointField::UINT8)
      {
        PC2_message_y_bytes = 1;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT16 || 
               tmp_byte == sensor_msgs::PointField::UINT16)
      {
        PC2_message_y_bytes = 2;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT32 || 
               tmp_byte == sensor_msgs::PointField::UINT32 || 
               tmp_byte == sensor_msgs::PointField::FLOAT32)
      {
        PC2_message_y_bytes = 4;
      }
      else if (tmp_byte == sensor_msgs::PointField::FLOAT64)
      {
        PC2_message_y_bytes = 8;
      }
      else
      {
        ROS_ERROR("Cannot determine number of bytes for Y coordinate");
        return -1;
      }
    }
    // Z
    else if (strcmp(bank_argument.PC2_message_z_coordinate_field_name.c_str(), 
                    msg->fields[i].name.c_str()) == 0)
    {
      // Read offset
      memcpy(&tmp_byte4[0], &msg->fields[i].offset, 4);
      if (must_reverse_bytes)
      {
        reverseBytes(tmp_byte4, 4);
      }
      memcpy(&PC2_message_z_offset, tmp_byte4, 4);
      
      // Read datatype
      tmp_byte = msg->fields[i].datatype;
      if (tmp_byte == sensor_msgs::PointField::INT8 || 
          tmp_byte == sensor_msgs::PointField::UINT8)
      {
        PC2_message_z_bytes = 1;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT16 || 
               tmp_byte == sensor_msgs::PointField::UINT16)
      {
        PC2_message_z_bytes = 2;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT32 || 
               tmp_byte == sensor_msgs::PointField::UINT32 || 
               tmp_byte == sensor_msgs::PointField::FLOAT32)
      {
        PC2_message_z_bytes = 4;
      }
      else if (tmp_byte == sensor_msgs::PointField::FLOAT64)
      {
        PC2_message_z_bytes = 8;
      }
      else
      {
        ROS_ERROR("Cannot determine number of bytes for Z coordinate");
        return -1;
      }
    }
  }
  
  if (0 <= PC2_message_x_offset &&
      0 <= PC2_message_x_bytes  &&
      0 <= PC2_message_y_offset &&
      0 <= PC2_message_y_bytes  &&
      0 <= PC2_message_z_offset &&
      0 <= PC2_message_z_bytes)
  {
    return 0;
  }
  else
  {
    ROS_ERROR("Could not determine offsets and bytes");
    return -1;
  }
}

int Bank::getOffsetsAndBytes(BankArgument bank_argument, const sensor_msgs::PointCloud2 * msg)
{
  PC2_message_x_offset = -1;
  PC2_message_x_bytes = -1;
  PC2_message_y_offset = -1;
  PC2_message_y_bytes = -1;
  PC2_message_z_offset = -1;
  PC2_message_z_bytes = -1;
  const bool must_reverse_bytes = (msg->is_bigendian != !machine_is_little_endian);
  const unsigned int fields = msg->fields.size();
  byte_t tmp_byte4[4]; // Used for reading offset, count = 1 (ROS: uint32)
  byte_t tmp_byte;     // Used for reading datatype 
  
  for (unsigned int i=0; i<fields; ++i)
  {
    // X
    if (strcmp(bank_argument.PC2_message_x_coordinate_field_name.c_str(), 
               msg->fields[i].name.c_str()) == 0)
    {
      // Read offset
      memcpy(&tmp_byte4[0], &msg->fields[i].offset, 4);
      if (must_reverse_bytes)
      {
        reverseBytes(tmp_byte4, 4);
      }
      memcpy(&PC2_message_x_offset, tmp_byte4, 4);
      
      // Read datatype
      tmp_byte = msg->fields[i].datatype;
      if (tmp_byte == sensor_msgs::PointField::INT8 || 
          tmp_byte == sensor_msgs::PointField::UINT8)
      {
        PC2_message_x_bytes = 1;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT16 || 
               tmp_byte == sensor_msgs::PointField::UINT16)
      {
        PC2_message_x_bytes = 2;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT32 || 
               tmp_byte == sensor_msgs::PointField::UINT32 || 
               tmp_byte == sensor_msgs::PointField::FLOAT32)
      {
        PC2_message_x_bytes = 4;
      }
      else if (tmp_byte == sensor_msgs::PointField::FLOAT64)
      {
        PC2_message_x_bytes = 8;
      }
      else
      {
        ROS_ERROR("Cannot determine number of bytes for X coordinate");
        return -1;
      }
    }
    // Y
    else if (strcmp(bank_argument.PC2_message_y_coordinate_field_name.c_str(), 
                    msg->fields[i].name.c_str()) == 0)
    {
      // Read offset
      memcpy(&tmp_byte4[0], &msg->fields[i].offset, 4);
      if (must_reverse_bytes)
      {
        reverseBytes(tmp_byte4, 4);
      }
      memcpy(&PC2_message_y_offset, tmp_byte4, 4);
      
      // Read datatype
      tmp_byte = msg->fields[i].datatype;
      if (tmp_byte == sensor_msgs::PointField::INT8 || 
          tmp_byte == sensor_msgs::PointField::UINT8)
      {
        PC2_message_y_bytes = 1;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT16 || 
               tmp_byte == sensor_msgs::PointField::UINT16)
      {
        PC2_message_y_bytes = 2;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT32 || 
               tmp_byte == sensor_msgs::PointField::UINT32 || 
               tmp_byte == sensor_msgs::PointField::FLOAT32)
      {
        PC2_message_y_bytes = 4;
      }
      else if (tmp_byte == sensor_msgs::PointField::FLOAT64)
      {
        PC2_message_y_bytes = 8;
      }
      else
      {
        ROS_ERROR("Cannot determine number of bytes for Y coordinate");
        return -1;
      }
    }
    // Z
    else if (strcmp(bank_argument.PC2_message_z_coordinate_field_name.c_str(), 
                    msg->fields[i].name.c_str()) == 0)
    {
      // Read offset
      memcpy(&tmp_byte4[0], &msg->fields[i].offset, 4);
      if (must_reverse_bytes)
      {
        reverseBytes(tmp_byte4, 4);
      }
      memcpy(&PC2_message_z_offset, tmp_byte4, 4);
      
      // Read datatype
      tmp_byte = msg->fields[i].datatype;
      if (tmp_byte == sensor_msgs::PointField::INT8 || 
          tmp_byte == sensor_msgs::PointField::UINT8)
      {
        PC2_message_z_bytes = 1;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT16 || 
               tmp_byte == sensor_msgs::PointField::UINT16)
      {
        PC2_message_z_bytes = 2;
      }
      else if (tmp_byte == sensor_msgs::PointField::INT32 || 
               tmp_byte == sensor_msgs::PointField::UINT32 || 
               tmp_byte == sensor_msgs::PointField::FLOAT32)
      {
        PC2_message_z_bytes = 4;
      }
      else if (tmp_byte == sensor_msgs::PointField::FLOAT64)
      {
        PC2_message_z_bytes = 8;
      }
      else
      {
        ROS_ERROR("Cannot determine number of bytes for Z coordinate");
        return -1;
      }
    }
  }
  
  if (0 <= PC2_message_x_offset &&
      0 <= PC2_message_x_bytes  &&
      0 <= PC2_message_y_offset &&
      0 <= PC2_message_y_bytes  &&
      0 <= PC2_message_z_offset &&
      0 <= PC2_message_z_bytes)
  {
    return 0;
  }
  else
  {
    ROS_ERROR("Could not determine offsets and bytes");
    return -1;
  }
}


/* DATA POINT HANDLING */
void Bank::readPoint(const byte_t * start_of_point,
                      const bool must_reverse_bytes,
                      double * x,
                      double * y,
                      double * z)
{
  uint8_t x_array[PC2_message_x_bytes];
  uint8_t y_array[PC2_message_y_bytes];
  uint8_t z_array[PC2_message_z_bytes];
  memcpy(x_array, start_of_point + PC2_message_x_offset, PC2_message_x_bytes);
  memcpy(y_array, start_of_point + PC2_message_y_offset, PC2_message_y_bytes);
  memcpy(z_array, start_of_point + PC2_message_z_offset, PC2_message_z_bytes);
  
  // Does the msg have different byte order compared to the CPU?
  if (must_reverse_bytes)
  {
    reverseBytes(x_array, PC2_message_x_bytes);
    reverseBytes(y_array, PC2_message_y_bytes);
    reverseBytes(z_array, PC2_message_z_bytes);
  }
  
  // Read the coordinates for this point
  // X
  if (PC2_message_x_bytes == sizeof(float))
  {
    float xf;
    memcpy(&xf, x_array, PC2_message_x_bytes);
    *x = xf;
  }
  else if (PC2_message_x_bytes == sizeof(double))
  {
    memcpy(x, x_array, PC2_message_x_bytes);
  }
  else
  {
    ROS_ERROR("Cannot determine how to read X coordinate for this point!");
    return;
  }
  // Y
  if (PC2_message_y_bytes == sizeof(float))
  {
    float yf;
    memcpy(&yf, y_array, PC2_message_y_bytes);
    *y = yf;
  }
  else if (PC2_message_y_bytes == sizeof(double))
  {
    memcpy(y, y_array, PC2_message_y_bytes);
  }
  else
  {
    ROS_ERROR("Cannot determine how to read Y coordinate for this point!");
    return;
  }
  // Z
  if (PC2_message_z_bytes == sizeof(float))
  {
    float zf;
    memcpy(&zf, z_array, PC2_message_z_bytes);
    *z = zf;
  }
  else if (PC2_message_z_bytes == sizeof(double))
  {
    memcpy(z, z_array, PC2_message_z_bytes);
  }
  else
  {
    ROS_ERROR("Cannot determine how to read Z coordinate for this point!");
    return;
  }
}


/* BANK HANDLING */
// Resets the range bank[i] for each i to a value that is larger than the largest allowed (threshold_distance_max)
void Bank::resetPutPoints()
{
  const double range = bank_argument.object_threshold_max_distance + 10.0;
  float * bank_put = bank_ranges_ema[bank_index_put];
  for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
  {
    bank_put[i] = range;
  }
}


// Assumes that threshold_distance_max < bank[i] (i.e. that values have been reset)
// and that bank_view_angle is centered at the x-axis.
// Reads all points from msg and puts them at bank[i] such that i corresponds to the angle at which 
// the point is found in the x,y plane of the sensor.
// Tries to fill several i for one and the same point if needed based on the voxel leaf size.
unsigned int Bank::putPoints(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  const bool must_reverse_bytes = (msg->is_bigendian != !machine_is_little_endian);
  float * bank_put = bank_ranges_ema[bank_index_put];
  const double bank_view_angle = bank_argument.angle_max - bank_argument.angle_min;
  const double bank_view_angle_half = bank_view_angle / 2;
  const double voxel_leaf_size_half = bank_argument.PC2_voxel_leaf_size / 2;
  const double inverted_bank_resolution = bank_argument.points_per_scan / bank_view_angle;
  const int bank_index_max = bank_argument.points_per_scan - 1;
  const unsigned int rows = msg->height;
  const unsigned int bytes_per_row = msg->row_step;
  const unsigned int bytes_per_point = msg->point_step;
  
  // Loop through rows
  unsigned int added_points_out = 0;
  for (unsigned int i=0; i<rows; i++)
  {
    // Loop through points in each row
    const unsigned int row_offset = i * bytes_per_row;
    for (unsigned int j=0; j<bytes_per_row; j+=bytes_per_point)
    {
      double x, y, z;
      const uint8_t * start_of_point = &msg->data[row_offset + j];
      
      readPoint(start_of_point,
                must_reverse_bytes,
                &x,
                &y,
                &z);
      
      // Is this point outside the considered volume?
      if (!bank_argument.sensor_frame_has_z_axis_forward)
      {
        // Assume Z-axis is pointing up and X-axis forward
        if (z < bank_argument.PC2_threshold_z_min || 
            bank_argument.PC2_threshold_z_max < z ||
            x < 0.02) 
        {
          continue;
        }
      }
      else
      {
        // Assume Y-axis is pointing down and Z-axis forward
        if (-y < bank_argument.PC2_threshold_z_min || 
            bank_argument.PC2_threshold_z_max < -y ||
            z < 0.02) 
        {
          continue;
        }
      }
      
      // Sanity check
      if (std::isnan(x) || std::isnan(y) || std::isnan(z))
      {
        ROS_DEBUG_STREAM("Skipping point (" << x << "," << y << "," << z << ")");
        continue;
      }
      
      // Another valid point
      added_points_out = added_points_out + 1;
      
      // Calculate index (indices) of point in bank
      const double range = sqrt(x*x + y*y + z*z); // TODO?
      double point_angle_min;
      double point_angle_max;
      if (!bank_argument.sensor_frame_has_z_axis_forward)
      {
        // Assume Z-axis is pointing up, 0.02 <= x
        point_angle_min = atan((y - voxel_leaf_size_half) / x);
        point_angle_max = atan((y + voxel_leaf_size_half) / x);
      }
      else
      {
        // Assume Y-axis is pointing down, 0.02 <= z
        point_angle_min = atan((-x - voxel_leaf_size_half) / z);
        point_angle_max = atan((-x + voxel_leaf_size_half) / z);
      }

      
      const int bank_index_point_min = 
        (0 > (point_angle_min + bank_view_angle_half) * inverted_bank_resolution ?
        0: // MAX of 0 and next row
        (point_angle_min + bank_view_angle_half) * inverted_bank_resolution);
      const int bank_index_point_max = 
        (bank_index_max < (point_angle_max + bank_view_angle_half) * inverted_bank_resolution ?
        bank_index_max : // MIN of bank_index_max and next row
        (point_angle_max + bank_view_angle_half) * inverted_bank_resolution);
            
      ROS_DEBUG_STREAM("The point (" << x << "," << y << "," << z << ") is added in the bank between indices " << \
           std::setw(4) << std::left << bank_index_point_min << " and " << bank_index_point_max << std::endl);
      
      // Fill all indices covered by this point
      // Check if there is already a range at the given index, only add if this point is closer
      for (int p=bank_index_point_min; p<=bank_index_point_max; ++p)
      {
        if (range < bank_put[p])
        {
          bank_put[p] = range;
        }
      }
    }
  }
  
  return added_points_out;
}

unsigned int Bank::putPoints(const sensor_msgs::PointCloud2 * msg)
{
  const bool must_reverse_bytes = (msg->is_bigendian != !machine_is_little_endian);
  float * bank_put = bank_ranges_ema[bank_index_put];
  const double bank_view_angle = bank_argument.angle_max - bank_argument.angle_min;
  const double bank_view_angle_half = bank_view_angle / 2;
  const double voxel_leaf_size_half = bank_argument.PC2_voxel_leaf_size / 2;
  const double inverted_bank_resolution = bank_argument.points_per_scan / bank_view_angle;
  const int bank_index_max = bank_argument.points_per_scan - 1;
  const unsigned int rows = msg->height;
  const unsigned int bytes_per_row = msg->row_step;
  const unsigned int bytes_per_point = msg->point_step;
  
  // Loop through rows
  unsigned int added_points_out = 0;
  for (unsigned int i=0; i<rows; i++)
  {
    // Loop through points in each row
    const unsigned int row_offset = i * bytes_per_row;
    for (unsigned int j=0; j<bytes_per_row; j+=bytes_per_point)
    {
      double x, y, z;
      const uint8_t * start_of_point = &msg->data[row_offset + j];
      
      readPoint(start_of_point,
                must_reverse_bytes,
                &x,
                &y,
                &z);
      
      // Is this point outside the considered volume?
      if (!bank_argument.sensor_frame_has_z_axis_forward)
      {
        // Assume Z-axis is pointing up and X-axis forward
        if (z < bank_argument.PC2_threshold_z_min || 
            bank_argument.PC2_threshold_z_max < z ||
            x < 0.02) 
        {
          continue;
        }
      }
      else
      {
        // Assume Y-axis is pointing down and Z-axis forward
        if (-y < bank_argument.PC2_threshold_z_min || 
            bank_argument.PC2_threshold_z_max < -y ||
            z < 0.02) 
        {
          continue;
        }
      }
      
      // Sanity check
      if (std::isnan(x) || std::isnan(y) || std::isnan(z))
      {
        ROS_DEBUG_STREAM("Skipping point (" << x << "," << y << "," << z << ")");
        continue;
      }
      
      // Another valid point
      added_points_out = added_points_out + 1;
      
      // Calculate index (indices) of point in bank
      const double range = sqrt(x*x + y*y + z*z); // TODO?
      double point_angle_min;
      double point_angle_max;
      if (!bank_argument.sensor_frame_has_z_axis_forward)
      {
        // Assume Z-axis is pointing up, 0.02 <= x
        point_angle_min = atan((y - voxel_leaf_size_half) / x);
        point_angle_max = atan((y + voxel_leaf_size_half) / x);
      }
      else
      {
        // Assume Y-axis is pointing down, 0.02 <= z
        point_angle_min = atan((-x - voxel_leaf_size_half) / z);
        point_angle_max = atan((-x + voxel_leaf_size_half) / z);
      }

      
      const int bank_index_point_min = 
        (0 > (point_angle_min + bank_view_angle_half) * inverted_bank_resolution ?
        0: // MAX of 0 and next row
        (point_angle_min + bank_view_angle_half) * inverted_bank_resolution);
      const int bank_index_point_max = 
        (bank_index_max < (point_angle_max + bank_view_angle_half) * inverted_bank_resolution ?
        bank_index_max : // MIN of bank_index_max and next row
        (point_angle_max + bank_view_angle_half) * inverted_bank_resolution);
            
      ROS_DEBUG_STREAM("The point (" << x << "," << y << "," << z << ") is added in the bank between indices " << \
           std::setw(4) << std::left << bank_index_point_min << " and " << bank_index_point_max << std::endl);
      
      // Fill all indices covered by this point
      // Check if there is already a range at the given index, only add if this point is closer
      for (int p=bank_index_point_min; p<=bank_index_point_max; ++p)
      {
        if (range < bank_put[p])
        {
          bank_put[p] = range;
        }
      }
    }
  }
  
  return added_points_out;
}


// Assumes that bank[bank_index_put] is filled with ranges from a new message 
// (i.e. that indices have not yet been updated).
// These values are EMA-adapted based on the previous set of EMA-adapted values at bank[index_previous]
void Bank::emaPutMessage()
{
  const double alpha = bank_argument.ema_alpha;
  
  // No need to do this if alpha < 1.0!
  if (alpha < 1.0)
  {
    const double alpha_prev = 1.0 - alpha;
    float * bank_put = bank_ranges_ema[bank_index_put];
    float * bank_prev = bank_ranges_ema[bank_index_newest];

    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      bank_put[i] = alpha * bank_put[i] + 
                    alpha_prev * bank_prev[i];
    }
  }
}


// Debug/print bank column/msg
std::string Bank::getStringPutPoints()
{
  float * bank_put = bank_ranges_ema[bank_index_put];  
  std::ostringstream stream;
  stream << "Bank points (at put index):";
  for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
  {
    stream << " " << bank_put[i];
  }
  stream << std::endl;
  std::string string = stream.str();
  return string;
}


// Init indices
inline void Bank::initIndex()
{
  bank_index_put = 1;
  bank_index_newest = 0;
}


// Advance indices
inline void Bank::advanceIndex()
{
  bank_index_put = (bank_index_put + 1) % bank_argument.nr_scans_in_bank; // points to the oldest message
  bank_index_newest = (bank_index_newest + 1) % bank_argument.nr_scans_in_bank; // points to the newest/this message
}


// Init bank based on LaserScan msg
long Bank::init(BankArgument bank_argument, const sensor_msgs::LaserScan * msg)
{
  if (!bank_is_initialized)
  {
    if (!bank_argument.sensor_frame_has_z_axis_forward && strstr(msg->header.frame_id.c_str(), "_optical") != NULL)
    {
      ROS_WARN_STREAM("The sensor frame (" << msg->header.frame_id.c_str() << ") seems to be a camera/optical frame. "
                      "Perhaps sensor_frame_has_z_axis_forward should be set?");
    }
    else if (bank_argument.sensor_frame_has_z_axis_forward)
    {
      ROS_WARN_STREAM("Please note that setting sensor_frame_has_z_axis_forward will cause the ema and "
                      "objects_closest_points messages to be shown incorrectly.");
    }
  }
  
  bank_argument.sensor_frame    = msg->header.frame_id;
  bank_argument.points_per_scan = msg->ranges.size();
  bank_argument.angle_min       = msg->angle_min;
  bank_argument.angle_max       = msg->angle_max;
  bank_argument.angle_increment = msg->angle_increment;
  bank_argument.time_increment  = msg->time_increment;
  bank_argument.scan_time       = msg->scan_time;
  bank_argument.range_min       = msg->range_min;
  bank_argument.range_max       = msg->range_max;
  resolution                    = bank_argument.angle_increment;
  
  initBank(bank_argument);
  
  ROS_DEBUG_STREAM("Bank arguments:" << std::endl << bank_argument);
  
  return addFirstMessage(msg);
}


// Add FIRST LaserScan message to bank - no ema
long Bank::addFirstMessage(const sensor_msgs::LaserScan * msg)
{
  bank_stamp[0] = msg->header.stamp.toSec();
  
//   if (bank_argument.ema_alpha != 1.0)
//   {
    float * bank_put = bank_ranges_ema[0];
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      if (msg->ranges[i] == std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else if (msg->ranges[i] == -std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_min - 0.01;
      }
//       else if (msg->ranges[i] != msg->ranges[i])
      else if (std::isnan(msg->ranges[i]))
      {
        // The range is NaN
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else
      {
        // Turn infinity into large value
        bank_put[i] = msg->ranges[i];
      }
    }
//   }
//   else
//   {
//     memcpy(&bank_ranges_ema[0][0], msg->ranges.data(), bank_ranges_bytes);
//   }
  
  initIndex(); // set put to 1 and newest to 0
  bank_is_filled = false;
  
  ROS_DEBUG_STREAM("First message (LaserScan):" << std::endl << *msg);
  
  return 0;
}

// Add LaserScan message and perform EMA
long Bank::addMessage(const sensor_msgs::LaserScan * msg)
{
  // Save timestamp
  bank_stamp[bank_index_put] = msg->header.stamp.toSec();
  
  // Save EMA of ranges, if applicable, otherwise, perform memcpy
  const double alpha = bank_argument.ema_alpha;
//   if (alpha != 1.0)
//   {
    const double alpha_prev = 1.0 - bank_argument.ema_alpha;
    float * bank_put = bank_ranges_ema[bank_index_put];
    float * bank_newest = bank_ranges_ema[bank_index_newest];
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      if (msg->ranges[i] == std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else if (msg->ranges[i] == -std::numeric_limits<float>::infinity())
      {
        bank_put[i] = bank_argument.range_min - 0.01;
      }
      else if (msg->ranges[i] != msg->ranges[i])
      {
        // The range is NaN
        bank_put[i] = bank_argument.range_max + 0.01;
      }
      else
      {
        // Turn infinity into large value
        bank_put[i] = alpha * msg->ranges[i]  +  alpha_prev * bank_newest[i];
      }
    }
//   }
//   else
//   {
//     memcpy(&bank_ranges_ema[bank_index_put][0], msg->ranges.data(), bank_ranges_bytes);
//   }
  
  advanceIndex();
  if (!bank_is_filled && bank_index_put < bank_index_newest)
  {
    bank_is_filled = true;
  }
  
  return 0;
}


// Init bank based on PointCloud2 msg
// This function only works on local copies of the input parameters, so it can safely be called again using the same 
// parameters if offsets and bytes could not be read from the message
long Bank::init(BankArgument bank_argument, const sensor_msgs::PointCloud2 * msg, 
                                            const bool discard_message_if_no_points_added)
{
  ROS_DEBUG("Init bank (%s)", msg->header.frame_id.c_str());
  bank_argument.sensor_frame = msg->header.frame_id;
  if (!bank_is_initialized)
  {
    if (!bank_argument.sensor_frame_has_z_axis_forward && strstr(msg->header.frame_id.c_str(), "_optical") != NULL)
    {
      ROS_WARN_STREAM("The sensor frame (" << msg->header.frame_id.c_str() << ") seems to be a camera/optical frame. "
                      "Perhaps sensor_frame_has_z_axis_forward should be set?");
    }
    else if (bank_argument.sensor_frame_has_z_axis_forward)
    {
      ROS_WARN_STREAM("Please note that setting sensor_frame_has_z_axis_forward will cause the ema and "
                      "objects_closest_points messages to be shown incorrectly.");
    }
  }
  
  if (bank_argument.points_per_scan <= 1)
  {
    bank_argument.angle_increment = 0.0000001;
  }
  else
  {
    bank_argument.angle_increment = (bank_argument.angle_max - bank_argument.angle_min) / 
                                    (bank_argument.points_per_scan - 1);
  }
  bank_argument.time_increment  = 0;
  bank_argument.scan_time       = 0;
  bank_argument.range_min       = 0.01;
  bank_argument.range_max       = bank_argument.object_threshold_max_distance;
  
  resolution = bank_argument.angle_increment;
  
  if (getOffsetsAndBytes(bank_argument, msg))
  {
    ROS_ERROR("Cannot read offsets and bytes from message!");
    return -1;
  }
  
  bank_argument.check_PC2();
  initBank(bank_argument);  // Will return immediately in case it has been called before
  return addFirstMessage(msg, discard_message_if_no_points_added);
}


// Add FIRST PointCloud2 message to bank - no EMA
long Bank::addFirstMessage(const sensor_msgs::PointCloud2 * msg, 
                           const bool discard_message_if_no_points_added)
{
  // Save timestamp
  bank_stamp[0] = msg->header.stamp.toSec();
  
  // Set put index so that we can use the helper functions
  bank_index_put = 0;
  
  // Reset ranges so that new values can be added to bank position
  resetPutPoints();
  
  // Add points if possible
  const unsigned int added_points = putPoints(msg);
  
  // If no points were added, then redo the process for this message
  if (added_points == 0)
  {
    ROS_WARN("Could not add any points from the PointCloud2 message");
    if (discard_message_if_no_points_added)
    {
      return -1;
    }
  }
  
  ROS_DEBUG_STREAM("First message (PointCloud2):" << std::endl << *msg);
  
  ROS_DEBUG("%s", getStringPutPoints().c_str());
  
  // Set put to 1 and newest to 0
  initIndex();
  bank_is_filled = false;
  
  return 0;
}

// Add PointCloud2 message and perform EMA
long Bank::addMessage(const sensor_msgs::PointCloud2 * msg, 
                      const bool discard_message_if_no_points_added)
{
  // Copy timestamp
  bank_stamp[bank_index_put] = msg->header.stamp.toSec();
  
  // Reset ranges so that new values can be added to bank position
  resetPutPoints();
  
  // Read the message and put the points in the bank
  const unsigned int added_points = putPoints(msg);
  
  // If no points were added, then redo the process for this message
  if (added_points == 0)
  {
    ROS_WARN("Could not add any points from the PointCloud2 message");
    if (discard_message_if_no_points_added)
    {
      return -1;
    }
  }
  
  ROS_DEBUG("%s", getStringPutPoints().c_str());
  
  // EMA-adapt the new ranges
  emaPutMessage();
  
  // Update indices
  advanceIndex();
  if (bank_index_put < bank_index_newest)
  {
    bank_is_filled = true;
  }
  
  return 0;
}

} // namespace find_moving_objects
