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

/* C/C++ */
#include <iostream>
#include <cmath>
#include <pthread.h>

/* LOCAL INCLUDES */
#include <find_moving_objects/option.h>
#include <find_moving_objects/bank.h>

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
          a*(dt*dt-1.2*dt+0.27) +// a well-adapted bank size in relation to the sensor rate and environmental context,
          (-5.0 * fabsf(mo.seen_width - mo_old_width))); // and low difference in width between old and new object,
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
  Option(false, "--sensor_frame_has_z_axis_forward",
         "The sensor is using an optical frame (x right, y down and z forward)",
         false),
  Option(false, "--nr_scans_in_bank",
         "Size of the bank containing subsequent, EMA:ed messages",
         5, 2, 20),
  Option(false, "--nr_points_per_scan_in_bank",
         "Determines the resolution of the bank",
         360, 1, 1800), // 720
  Option(false, "--bank_view_angle",
         "View angle, centered on the X axis",
         M_PI, 0.0, 2*M_PI), // Resolution is 0.5 degrees by default
  Option(false, "--ema_alpha",
         "EMA coefficient representing the degree of weighting decrease",
         1.0, 0.0, 1.0),
  Option(false, "--subscribe_buffer_size",
         "Subscription queue size (should be 1 if only the latest sensor reading is of interest))",
         1, 1, 1000),
  Option(false, "--subscribe_topic",
         "Topic on which sensor data is published",
         std::string("/cloud_filtered_echoed")),
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
         1, 1, 1000),
  Option(false, "--topic_objects",
         "Topic for publishing MOA messages",
         std::string("/moving_objects")),
  Option(false, "--topic_ema",
         "Topic for publishing EMA:ed data",
         std::string("/pointcloud2_ema")),
  Option(false, "--topic_objects_closest_point_markers",
         "Topic for publishing closest points of the objects",
         std::string("/pointcloud2_objects_closest_point_markers")),
  Option(false, "--topic_objects_velocity_arrows",
         "Topic for publishing position and velocity",
         std::string("/pointcloud2_objects_velocity_arrows")),
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
         "Show velocity of object in relation to sensor instead of map",
         false),
  Option(false, "--velocity_arrows_use_base_frame",
         "Show velocity of object in base frame instead of map",
         false),
  Option(false, "--velocity_arrows_use_fixed_frame",
         "Show velocity of object in fixed frame instead of map",
         false),
  Option(false, "--message_x_coordinate_field_name",
         "Name of X coordinate offset field in the point cloud messages",
         std::string("x")),
  Option(false, "--message_y_coordinate_field_name",
         "Name of Y coordinate offset field in the point cloud messages",
         std::string("y")),
  Option(false, "--message_z_coordinate_field_name",
         "Name of Z coordinate offset field in the point cloud messages",
         std::string("z")),
  Option(true,  "--voxel_leaf_size",
         "Approximate distance between two point cloud points",
         -1.0, 0.001, 0.1),
  Option(false, "--threshold_z_min",
         "Points with Z coordinates smaller than this are discarded",
         0.0, 0.0, std::numeric_limits<double>::max()),
  Option(false, "--threshold_z_max",
         "Points with Z coordinates larger than this are discarded",
         0.6, 0.0, std::numeric_limits<double>::max()),
  Option(false, "--object_threshold_edge_max_delta_range",
         "Maximum distance between two consecutive scan points belonging to the same object",
         0.15, 0.0, std::numeric_limits<double>::max()), // 0.072
  Option(false, "--object_threshold_min_nr_points",
         "Objects must consist of at least this number of consecutive scan points",
         3, 1, 100000), // 5
  Option(false, "--object_threshold_max_distance",
         "Object must consist of scan points with maximum this range",
         6.5, 0.0, std::numeric_limits<double>::max()),
  Option(false, "--object_threshold_min_speed",
         "Minimum speed of object to consider it moving",
         0.1, 0.0, std::numeric_limits<double>::max()),
  Option(false, "--object_threshold_max_delta_width_in_points",
         "Maximum size difference in points to consider old and current object instances the same",
         15, 0, 100000),
  Option(false, "--object_threshold_bank_tracking_max_delta_distance",
         "Maximum distance an object is allowed to move between two consecutive scans while tracking it through the bank",
         std::numeric_limits<double>::max(), 0.0, std::numeric_limits<double>::max()),
  Option(false, "--object_threshold_min_confidence",
         "Minimum confidence of object for publishing it",
         0.7, 0.0, 1.0), // 0.65
  Option(false, "--base_confidence",
         "How much we trust the sensor data",
         0.1, 0.0, 1.0),
};


/* USER OPTION INDICES FOR EASY ACCESS */
typedef enum {
  O_I_MAP_FRAME = 0,
  O_I_FIXED_FRAME,
  O_I_BASE_FRAME,
  O_I_SENSOR_FRAME_HAS_Z_AXIS_FORWARD,
  O_I_NR_MESSAGES_IN_BANK,
  O_I_NR_POINTS_PER_MESSAGE_IN_BANK,
  O_I_BANK_VIEW_ANGLE,
  O_I_EMA_ALPHA,
  O_I_SUBSCRIBE_BUFFER_SIZE,
  O_I_SUBSCRIBE_TOPIC,
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
  O_I_MESSAGE_X_COORDINATE_FIELD_NAME,
  O_I_MESSAGE_Y_COORDINATE_FIELD_NAME,
  O_I_MESSAGE_Z_COORDINATE_FIELD_NAME,
  O_I_VOXEL_LEAF_SIZE,
  O_I_THRESHOLD_Z_MIN,
  O_I_THRESHOLD_Z_MAX,
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
sensor_msgs::PointCloud2::ConstPtr msg_old;


/* CALLBACK FOR FIRST MESSAGE */
void pointCloud2CallbackFirst(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  // Keep reference to message
  msg_old = msg;
  // Debug frame to see e.g. if we are dealing with an optical frame
  ROS_DEBUG_STREAM("Pointcloud2 sensor is using frame: " << msg->header.frame_id);

  // Init bank
  if (bank->init(bank_argument, msg) == 0)
  {
    first_message_received = true;
  }
}


/* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  // Keep reference to message
  msg_old = msg;

  // Was message added to bank?
  if (bank->addMessage(msg) != 0)
  {
    // Adding message failed
    return;
  }

  // If so, then find and report objects
  bank->findAndReportMovingObjects();
}


/* ENTRY POINT */
int main (int argc, char ** argv)
{
  // Init
  first_message_received = false;

  // Init ROS
  ros::init(argc, argv, "mo_finder_pointcloud2", ros::init_options::AnonymousName);
  g_node = new ros::NodeHandle;
  bank = new find_moving_objects::Bank;

  // Scan arguments
  Option::scanArgs(argc, argv, g_options);

  // Init bank_argument using user options
  bank_argument.ema_alpha = g_options[O_I_EMA_ALPHA].getDoubleValue();
  bank_argument.nr_scans_in_bank = g_options[O_I_NR_MESSAGES_IN_BANK].getLongValue();
  bank_argument.points_per_scan = g_options[O_I_NR_POINTS_PER_MESSAGE_IN_BANK].getLongValue();
  bank_argument.angle_max = g_options[O_I_BANK_VIEW_ANGLE].getDoubleValue() / 2;
  bank_argument.angle_min = -bank_argument.angle_max;
  bank_argument.sensor_frame_has_z_axis_forward = g_options[O_I_SENSOR_FRAME_HAS_Z_AXIS_FORWARD].getBoolValue();
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
  bank_argument.velocity_arrow_ns = "pointcloud2_velocity_arrow";
  bank_argument.delta_position_line_ns = "pointcloud2_delta_position_line";
  bank_argument.width_line_ns = "pointcloud2_width_line";
  bank_argument.topic_ema = g_options[O_I_TOPIC_EMA].getStringValue();
  bank_argument.topic_objects_closest_point_markers =
    g_options[O_I_TOPIC_OBJECTS_CLOSEST_POINT_MARKERS].getStringValue();
  bank_argument.topic_objects_velocity_arrows = g_options[O_I_TOPIC_OBJECTS_VELOCITY_ARROWS].getStringValue();
  bank_argument.topic_objects_delta_position_lines = g_options[O_I_TOPIC_OBJECTS_DELTA_POSITION_LINES].getStringValue();
  bank_argument.topic_objects_width_lines = g_options[O_I_TOPIC_OBJECTS_WIDTH_LINES].getStringValue();
  bank_argument.topic_objects = g_options[O_I_TOPIC_OBJECTS].getStringValue();
  bank_argument.publish_buffer_size = g_options[O_I_PUBLISH_BUFFER_SIZE].getLongValue();

  // PointCloud2-specific
  bank_argument.PC2_message_x_coordinate_field_name = g_options[O_I_MESSAGE_X_COORDINATE_FIELD_NAME].getStringValue();
  bank_argument.PC2_message_y_coordinate_field_name = g_options[O_I_MESSAGE_Y_COORDINATE_FIELD_NAME].getStringValue();
  bank_argument.PC2_message_z_coordinate_field_name = g_options[O_I_MESSAGE_Z_COORDINATE_FIELD_NAME].getStringValue();
  bank_argument.PC2_voxel_leaf_size = g_options[O_I_VOXEL_LEAF_SIZE].getDoubleValue();
  bank_argument.PC2_threshold_z_min = g_options[O_I_THRESHOLD_Z_MIN].getDoubleValue();
  bank_argument.PC2_threshold_z_max = g_options[O_I_THRESHOLD_Z_MAX].getDoubleValue();

  // Z threshold sanity check
  if (g_options[O_I_THRESHOLD_Z_MAX].getDoubleValue() < g_options[O_I_THRESHOLD_Z_MIN].getDoubleValue())
  {
    std::string err = g_options[O_I_THRESHOLD_Z_MAX].getName() + " cannot be smaller than " +
                      g_options[O_I_THRESHOLD_Z_MIN].getName();
    ROS_ERROR("%s", err.c_str());
    ROS_BREAK();
  }

  // Receive first message and init bank
  {
    // Subscribe to the sensor topic using first callback
    ros::Subscriber sub = g_node->subscribe(g_options[O_I_SUBSCRIBE_TOPIC].getStringValue(),
                                            g_options[O_I_SUBSCRIBE_BUFFER_SIZE].getLongValue(),
                                            pointCloud2CallbackFirst);

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
                                          pointCloud2Callback);

  // Enter receive loop
  ros::spin();

  return 0;
}
