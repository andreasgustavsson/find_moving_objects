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
#include <sensor_msgs/LaserScan.h>

/* C/C++ */
#include <iostream>
#include <cmath>
#include <pthread.h>

/* LOCAL INCLUDES */
#include <find_moving_objects/option.h>
#include <find_moving_objects/bank.h>
#include <find_moving_objects/hz_calculator.h>

using namespace find_moving_objects;

/*
 * Standard Units of Measure and Coordinate Conventions:   http://www.ros.org/reps/rep-0103.html
 * Coordinate Frames for Mobile Platforms:                 http://www.ros.org/reps/rep-0105.html
 */


/* HANDLE TO THIS NODE */
ros::NodeHandle * g_node;
bool first_message_received;


/* CONFIDENCE CALCULATION FOR BANK */
const double a = -10 / 3;
double find_moving_objects::Bank::calculateConfidence(const find_moving_objects::MovingObject & mo,
                                                      const find_moving_objects::BankArgument & ba,
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
          a*(dt*dt-1.0*dt+0.16) + // a well-adapted bank size in relation to the sensor rate and environmental context
          // (dt should be close to 0.5 seconds),
          (-5.0 * fabsf(mo.seen_width - mo_old_width)));// + // and low difference in width between old and new object,
          // make us more confident
}


/* USER OPTIONS */
Option g_options[] = {
  Option(false, "--map_frame",
         "The frame used as map frame",
         std::string("map")),
  Option(false, "--fixed_frame",
         "The frame used as fixed/odometry frame",
         std::string("odom")),
  Option(false, "--base_frame",
         "The base frame of the robot",
         std::string("base_link")),
  Option(false, "--mounting_angle_shift_z",
         "Can be used for offsetting the mounting angle of the sensor",
         0.0, -M_PI, M_PI), // Set value to -(2.34731f+(-0.776829f))/2 = −0,7852405 if the frames
         // base_laser{,_ema} are not tf:ed;
         // setting this value to a value other than 0.0 causes the incoming message to be doubly copied (NOT GOOD!)...
  Option(false, "--nr_scans_in_bank",
         "Size of the bank containing subsequent, EMA:ed messages (ignored if the below option is not 0.0 seconds)",
         11, 2, 20),
  Option(false, "--optimize_nr_scans_in_bank",
         "If not 0.0, then the bank size is optimized to cover the given time period (seconds)",
         0.0, 0.0, std::numeric_limits<float>::max()),
  Option(false, "--ema_alpha",
         "EMA coefficient representing the degree of weighting decrease",
         1.0, 0.0, 1.0),
  Option(false, "--subscribe_buffer_size",
         "Subscription queue size (should be 1 if only the latest sensor reading is of interest)",
         1, 1, 100),
  Option(false, "--subscribe_topic",
         "Topic on which sensor data is published",
         std::string("/scan_filtered_echoed")),
  Option(false, "--no_publish_objects",
         "Do not publish any MOA messages",
         false),
  Option(false, "--publish_ema",
         "Publish EMA:ed data with objects marked as a LaserScan message",
         false),
  Option(false, "--publish_objects_closest_point_markers",
         "Publish closest points of each object as a LaserScan message",
         false),
  Option(false, "--publish_objects_velocity_arrows",
         "Publish position (arrow base) and velocity (arrow length) of objects as a MarkerArray message",
         false),
  Option(false, "--publish_objects_delta_position_lines",
         "Publish delta position line of objects as a MarkerArray message (lines)",
         false),
  Option(false, "--publish_objects_width_lines",
         "Publish width line of objects as a MarkerArray message (lines)",
         false),
  Option(false, "--publish_buffer_size",
         "Publishing queue size",
         1, 1, 100),
  Option(false, "--topic_objects",
         "Topic for publishing MOA messages",
         std::string("/moving_objects")),
  Option(false, "--topic_ema",
         "Topic for publishing EMA:ed data",
         std::string("/laserscan_ema")),
  Option(false, "--topic_objects_closest_point_markers",
         "Topic for publishing closest points of the objects",
         std::string("/laserscan_objects_closest_point_markers")),
  Option(false, "--topic_objects_velocity_arrows",
         "Topic for publishing positions and velocities",
         std::string("/laserscan_objects_velocity_arrows")),
  Option(false, "--topic_objects_delta_position_lines",
         "Topic for publishing delta position lines",
         std::string("/laserscan_objects_delta_position_lines")),
  Option(false, "--topic_objects_width_lines",
         "Topic for publishing width lines",
         std::string("/laserscan_objects_width_lines")),
  Option(false, "--velocity_arrows_use_full_gray_scale",
         "Shift color/confidence of arrow to full gray scale (white is high confidence)",
         false),
  Option(false, "--velocity_arrows_use_sensor_frame",
         "Show velocity of object in sensor frame instead of map",
         false),
  Option(false, "--velocity_arrows_use_base_frame",
         "Show velocity of object in base frame instead of map",
         false),
  Option(false, "--velocity_arrows_use_fixed_frame",
         "Show velocity of object in fixed frame instead of map",
         false),
  Option(false, "--object_threshold_edge_max_delta_range",
         "Maximum distance between two consecutive scan points belonging to the same object",
         0.15, 0.0, std::numeric_limits<float>::max()), // 0.072
  Option(false, "--object_threshold_min_nr_points",
         "Objects must consist of at least this number of consecutive scan points",
         4, 1, 100000),
  Option(false, "--object_threshold_max_distance",
         "Object must consist of scan points with maximum this range",
         6.5, 0.0, std::numeric_limits<float>::max()),
  Option(false, "--object_threshold_min_speed",
         "Minimum speed of object to consider it moving",
         0.03, 0.0, std::numeric_limits<float>::max()),
  Option(false, "--object_threshold_max_delta_width_in_points",
         "Maximum size difference in points to consider old and current object instances the same",
         5, 0, 100000),
  Option(false, "--object_threshold_bank_tracking_max_delta_distance",
         "Maximum distance an object is allowed to move between two consecutive scans while tracking it through bank",
         0.2, 0.0, std::numeric_limits<float>::max()),
  Option(false, "--object_threshold_min_confidence",
         "Minimum confidence of object for publishing it",
         0.7, 0.0, 1.0), // 0.65
  Option(false, "--base_confidence",
         "How much we trust the sensor data",
         0.3, 0.0, 1.0),
};


/* USER OPTION INDICES FOR EASY ACCESS */
typedef enum {
  O_I_MAP_FRAME = 0,
  O_I_FIXED_FRAME,
  O_I_BASE_FRAME,
  O_I_MOUNTING_ANGLE_SHIFT_Z,
  O_I_NR_SCANS_IN_BANK,
  O_I_OPTIMIZE_NR_SCANS_IN_BANK,
  O_I_EMA_ALPHA,
  O_I_SUBSCRIBE_BUFFER_SIZE,
  O_I_SUBSCRIBE_TOPIC, // Topic on which we expect the sensor_msgs::LaserScan messages
  O_I_NO_PUBLISH_OBJECTS,
  O_I_PUBLISH_EMA,
  O_I_PUBLISH_OBJECTS_CLOSEST_POINT_MARKERS,
  O_I_PUBLISH_OBJECTS_VELOCITY_ARROWS,
  O_I_PUBLISH_OBJECTS_DELTA_POSITION_LINES,
  O_I_PUBLISH_OBJECTS_WIDTH_LINES,
  O_I_PUBLISH_BUFFER_SIZE,
  O_I_TOPIC_OBJECTS,
  O_I_TOPIC_EMA,
  O_I_TOPIC_OBJECTS_CLOSEST_POINT_MARKERS,
  O_I_TOPIC_OBJECTS_VELOCITY_ARROWS,
  O_I_TOPIC_OBJECTS_DELTA_POSITION_LINES,
  O_I_TOPIC_OBJECTS_WIDTH_LINES,
  O_I_VELOCITY_ARROWS_USE_FULL_GRAY_SCALE,
  O_I_VELOCITY_ARROWS_USE_SENSOR_FRAME,
  O_I_VELOCITY_ARROWS_USE_BASE_FRAME,
  O_I_VELOCITY_ARROWS_USE_FIXED_FRAME,
  O_I_OBJECT_THRESHOLD_EDGE_MAX_DELTA_RANGE,
  O_I_OBJECT_THRESHOLD_MIN_NR_POINTS,
  O_I_OBJECT_THRESHOLD_MAX_DISTANCE,
  O_I_OBJECT_THRESHOLD_MIN_SPEED,
  O_I_OBJECT_THRESHOLD_MAX_DELTA_WIDTH_IN_POINTS,
  O_I_OBJECT_THRESHOLD_BANK_TRACKING_MAX_DELTA_DISTANCE,
  O_I_OBJECT_THRESHOLD_MIN_CONFIDENCE,
  O_I_BASE_CONFIDENCE,
  NR_OPTIONS
} option_index_t;


/* BANK AND ARGUMENT */
find_moving_objects::Bank * bank;
find_moving_objects::BankArgument bank_argument;


/* LOCAL MESSAGE POINTER */
sensor_msgs::LaserScan::ConstPtr msg_old;


/* CALLBACK FOR FIRST MESSAGE */
void laserScanCallbackFirst(const sensor_msgs::LaserScan::ConstPtr & msg_org)
{
  // Working copy of msg
  sensor_msgs::LaserScan::ConstPtr msg;

  // Should we rotate the reading around the z-axis?
  if (g_options[O_I_MOUNTING_ANGLE_SHIFT_Z].getDoubleValue() != 0.0)
  {
    sensor_msgs::LaserScan msg_copy = *msg_org;
    // Change angles
    msg_copy.angle_min = msg_copy.angle_min + g_options[O_I_MOUNTING_ANGLE_SHIFT_Z].getDoubleValue();
    msg_copy.angle_max = msg_copy.angle_max + g_options[O_I_MOUNTING_ANGLE_SHIFT_Z].getDoubleValue();
    // Create ConstPtr by COPYING the message again...
    sensor_msgs::LaserScan::ConstPtr msg_copy_ptr(new sensor_msgs::LaserScan(msg_copy));
    // Update buffer
    msg = msg_copy_ptr;
  }
  else
  {
    msg = msg_org;
  }

  // Keep reference to message
  msg_old = msg;

  // Init bank
  bank_argument.points_per_scan = msg->ranges.size();
  if (bank->init(bank_argument, msg) == 0)
  {
    first_message_received = true;
  }
}


/* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg_org)
{
  // Working copy of msg
  sensor_msgs::LaserScan::ConstPtr msg;

  // Should we rotate the reading around the z-axis?
  if (g_options[O_I_MOUNTING_ANGLE_SHIFT_Z].getDoubleValue() != 0.0)
  {
    sensor_msgs::LaserScan msg_copy = *msg_org;
    // Change angles
    msg_copy.angle_min = msg_copy.angle_min + g_options[O_I_MOUNTING_ANGLE_SHIFT_Z].getDoubleValue();
    msg_copy.angle_max = msg_copy.angle_max + g_options[O_I_MOUNTING_ANGLE_SHIFT_Z].getDoubleValue();
    // Create ConstPtr by COPYING the message again...
    sensor_msgs::LaserScan::ConstPtr msg_copy_ptr(new sensor_msgs::LaserScan(msg_copy));
    // Update buffer
    msg = msg_copy_ptr;
  }
  else
  {
    msg = msg_org;
  }

  // Check if duplicate message was received
  if (msg->header.stamp.toSec() == msg_old->header.stamp.toSec())
  {
    bool same_msg = true;
    for (unsigned int i=0; i<bank_argument.points_per_scan; ++i)
    {
      if (msg->ranges[i] != msg_old->ranges[i])
      {
        same_msg = false;
        break;
      }
    }
    // Discard this message?
    if (same_msg)
    {
      return;
    }
  }
  // Message is ok

  // Keep reference to message
  msg_old = msg;

  // Add message to bank and find and report moving objects
  bank->addMessage(msg);
  bank->findAndReportMovingObjects();
}


/* ENTRY POINT */
int main (int argc, char ** argv)
{
  // Init
  first_message_received = false;

  // Init ROS
  ros::init(argc, argv, "mo_finder_laserscan", ros::init_options::AnonymousName);
  g_node = new ros::NodeHandle;
  
  // Wait for time to become valid, then start bank
  ros::Time::waitForValid();
  bank = new find_moving_objects::Bank;

  // Scan arguments
  Option::scanArgs(argc, argv, g_options);

  // Init bank_argument with user options
  bank_argument.ema_alpha = g_options[O_I_EMA_ALPHA].getDoubleValue();
  bank_argument.nr_scans_in_bank = g_options[O_I_NR_SCANS_IN_BANK].getLongValue();
  bank_argument.object_threshold_edge_max_delta_range =
    g_options[O_I_OBJECT_THRESHOLD_EDGE_MAX_DELTA_RANGE].getDoubleValue();
  bank_argument.object_threshold_min_nr_points = g_options[O_I_OBJECT_THRESHOLD_MIN_NR_POINTS].getLongValue();
  bank_argument.object_threshold_max_distance = g_options[O_I_OBJECT_THRESHOLD_MAX_DISTANCE].getDoubleValue(); // m
  bank_argument.object_threshold_min_speed = g_options[O_I_OBJECT_THRESHOLD_MIN_SPEED].getDoubleValue(); // m/s
  bank_argument.object_threshold_max_delta_width_in_points =
    g_options[O_I_OBJECT_THRESHOLD_MAX_DELTA_WIDTH_IN_POINTS].getLongValue();
  bank_argument.object_threshold_bank_tracking_max_delta_distance =
    g_options[O_I_OBJECT_THRESHOLD_BANK_TRACKING_MAX_DELTA_DISTANCE].getDoubleValue();
  bank_argument.object_threshold_min_confidence = g_options[O_I_OBJECT_THRESHOLD_MIN_CONFIDENCE].getDoubleValue();
  bank_argument.base_confidence = g_options[O_I_BASE_CONFIDENCE].getDoubleValue();
  bank_argument.publish_ema = g_options[O_I_PUBLISH_EMA].getBoolValue();
  bank_argument.publish_objects_closest_point_markers =
    g_options[O_I_PUBLISH_OBJECTS_CLOSEST_POINT_MARKERS].getBoolValue();
  bank_argument.publish_objects_velocity_arrows = g_options[O_I_PUBLISH_OBJECTS_VELOCITY_ARROWS].getBoolValue();
  bank_argument.publish_objects_delta_position_lines =
    g_options[O_I_PUBLISH_OBJECTS_DELTA_POSITION_LINES].getBoolValue();
  bank_argument.publish_objects_width_lines =
    g_options[O_I_PUBLISH_OBJECTS_WIDTH_LINES].getBoolValue();
  bank_argument.velocity_arrows_use_full_gray_scale = g_options[O_I_VELOCITY_ARROWS_USE_FULL_GRAY_SCALE].getBoolValue();
  bank_argument.velocity_arrows_use_sensor_frame = g_options[O_I_VELOCITY_ARROWS_USE_SENSOR_FRAME].getBoolValue();
  bank_argument.velocity_arrows_use_base_frame = g_options[O_I_VELOCITY_ARROWS_USE_BASE_FRAME].getBoolValue();
  bank_argument.velocity_arrows_use_fixed_frame = g_options[O_I_VELOCITY_ARROWS_USE_FIXED_FRAME].getBoolValue();
  bank_argument.publish_objects = !g_options[O_I_NO_PUBLISH_OBJECTS].getBoolValue();
  bank_argument.map_frame = g_options[O_I_MAP_FRAME].getStringValue();
  bank_argument.fixed_frame = g_options[O_I_FIXED_FRAME].getStringValue();
  bank_argument.base_frame = g_options[O_I_BASE_FRAME].getStringValue();
  bank_argument.velocity_arrow_ns = "laserscan_velocity_arrow";
  bank_argument.delta_position_line_ns = "laserscan_delta_position_line";
  bank_argument.width_line_ns = "laserscan_width_line";
  bank_argument.topic_ema = g_options[O_I_TOPIC_EMA].getStringValue();
  bank_argument.topic_objects_closest_point_markers =
    g_options[O_I_TOPIC_OBJECTS_CLOSEST_POINT_MARKERS].getStringValue();
  bank_argument.topic_objects_velocity_arrows = g_options[O_I_TOPIC_OBJECTS_VELOCITY_ARROWS].getStringValue();
  bank_argument.topic_objects_delta_position_lines = g_options[O_I_TOPIC_OBJECTS_DELTA_POSITION_LINES].getStringValue();
  bank_argument.topic_objects_width_lines = g_options[O_I_TOPIC_OBJECTS_WIDTH_LINES].getStringValue();
  bank_argument.topic_objects = g_options[O_I_TOPIC_OBJECTS].getStringValue();
  bank_argument.publish_buffer_size = g_options[O_I_PUBLISH_BUFFER_SIZE].getLongValue();

  // Optimize bank size?
  if (g_options[O_I_OPTIMIZE_NR_SCANS_IN_BANK].getDoubleValue() != 0.0)
  {
    HZCalculator hzc;
    const double hz = hzc.calc(g_options[O_I_SUBSCRIBE_TOPIC].getStringValue());

    // Set nr of messages in bank
    const double nr_scans = g_options[O_I_OPTIMIZE_NR_SCANS_IN_BANK].getDoubleValue() * hz;
    bank_argument.nr_scans_in_bank = nr_scans - ((const long) nr_scans) == 0.0 ? nr_scans + 1 : ceil(nr_scans);
    
    // Sanity check
    if (bank_argument.nr_scans_in_bank < 2)
    {
      bank_argument.nr_scans_in_bank = 2;
    }
    
    ROS_INFO_STREAM("Optimized bank size is " << bank_argument.nr_scans_in_bank);
  }
  
  // Receive first message and init bank
  {
    // Subscribe to the sensor topic using first callback
    ros::Subscriber sub = g_node->subscribe(g_options[O_I_SUBSCRIBE_TOPIC].getStringValue(),
                                            g_options[O_I_SUBSCRIBE_BUFFER_SIZE].getLongValue(),
                                            laserScanCallbackFirst);

    // Receive first message
    while (!first_message_received)
    {
      ros::spinOnce();
      if (!g_node->ok())
      {
        return 0;
      }
    }
  } // un-scope sub object

  // Subscribe to sensor topic using new callback
  ros::Subscriber sub = g_node->subscribe(g_options[O_I_SUBSCRIBE_TOPIC].getStringValue(),
                                          g_options[O_I_SUBSCRIBE_BUFFER_SIZE].getLongValue(),
                                          laserScanCallback);

  // Enter receive loop
  ros::spin();

  return 0;
}
