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

/**
 * This is a ROS node that takes MovingObjectArray messages as input.
 * When a new message is received, the node caches that message and 
 * compares the objects it contains to the objects of the messages the 
 * node has received from other senders. If an object has been detected
 * by at least one other sender, then its confidence value is increased
 * before the message is forwarded.
 */


/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* C/C++ */
#include <iostream>
#include <sstream>
#include <cmath>
#include <pthread.h>

/* Local includes */
#include <find_moving_objects/MovingObject.h>
#include <find_moving_objects/MovingObjectArray.h>

using namespace find_moving_objects;

#define MIN(X,Y) (X<Y?X:Y)
#define MAX(X,Y) (X>Y?X:Y)

#define WL(W) std::setw(W) << std::left
#define WR(W) std::setw(W) << std::right
const double TWO_PI = 2 * M_PI;

/* HANDLE TO THIS NODE */
ros::NodeHandle * g_node;


/* PUBLISHERS */
ros::Publisher g_pub_objects_closest_point_markers;
ros::Publisher g_pub_objects_velocity_arrows;
ros::Publisher g_pub_moaf;


/* MOVINGOBJECTARRAY MSG HANDLING */
std::vector<find_moving_objects::MovingObjectArray::ConstPtr> g_msg_buffer_moa;
pthread_mutex_t g_mutex_moa = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_cond_moa = PTHREAD_COND_INITIALIZER;
bool g_available_moa = false;
int g_sender_index = -1;
int g_nr_senders = 0;


/* CALLBACK - USE EXTRA WORKER THREAD TO DO THE ACTUAL WORK */
void moaCallback(const find_moving_objects::MovingObjectArray::ConstPtr & msg)
{
  // Save names of sending nodes so that we can keep messages apart, only used in this function, so make it a static var
  static std::vector<std::string> g_msg_buffer_moa_senders;
  
  // Local search so that we can avoid locking the mutex
  int sender_index = 0;
  for (; sender_index<g_nr_senders; ++sender_index)
  {
    if (strcmp(msg->origin_node_name.c_str(), g_msg_buffer_moa_senders[sender_index].c_str()) == 0)
    {
      // Found!
      break;
    }
  }
  // If sender_index == g_nr_senders here, then the sender is unknown to us
  
  // CS start
  pthread_mutex_lock(&g_mutex_moa);
  
  if (sender_index == g_nr_senders)
  {
    // If sender is not in list of known senders, then add its name and the msg to the buffers
    g_msg_buffer_moa_senders.push_back(msg->origin_node_name);
    g_msg_buffer_moa.push_back(msg); // Is the whole message copied when the new ConstPtr is created?
    // Make sure we know that we added a sender
    g_nr_senders++;
  }
  else
  {
    // Replace the cached message 
    g_msg_buffer_moa[sender_index] = msg;
  }
  
  // Point to sender
  g_sender_index = sender_index;
  
  // Signal that a new message is available
  g_available_moa = true;
  pthread_cond_signal(&g_cond_moa);
  
  // CS end
  pthread_mutex_unlock(&g_mutex_moa);
  
  /*
   * Now, the buffer of sender names and msgs are up-to-date, 
   * and g_sender_index is pointing to the newly arrived msg.
   * When accessing the buffers or g_sender_index in the worker thread, the mutex must be taken.
   * The newly arrived msg and g_sender_index should be immediately saved locally, as soon as the mutex is taken.
   * Then some unprotected work can be done before each of the other senders' cached msgs are locally saved, 
   * under mutex protection, and then evaluated locally.
   * This allows the cache of msgs to be updated even if the worker thread is currently doing some evaluation.
   * Note, however, that only the latest msg is ever considered, which means that msgs can be dropped and that
   * senders might thus suffer from never having their msgs evaluated and forwarded; this node might be a bottleneck.
   */
}


/* WORKER THREAD */
void * moaHandlerBody(void * arg)
{
  // Private nodehandle
  ros::NodeHandle nh_priv("~");
  
  // Read parameters
  bool verbose;
  bool print_received_objects;
  bool publish_objects;
  bool publish_objects_closest_points_markers;
  bool publish_objects_velocity_arrows;
  bool velocity_arrows_use_full_gray_scale;
  bool velocity_arrows_use_sensor_frame_param;
  bool velocity_arrows_use_base_frame_param;
  bool velocity_arrows_use_fixed_frame_param;
  std::string topic_moving_objects_enhanced;
  std::string topic_objects_closest_point_markers;
  std::string topic_objects_velocity_arrows;
  double threshold_min_confidence;
  double threshold_max_delta_time_for_different_sources;
  double threshold_max_delta_position;
  double threshold_max_delta_velocity;
  bool ignore_z_map_coordinate_for_position;
  nh_priv.param("verbose", verbose, false);
  nh_priv.param("print_received_objects", print_received_objects, false);
  nh_priv.param("publish_objects", publish_objects, true);
  nh_priv.param("publish_objects_closest_points_markers", publish_objects_closest_points_markers, true);
  nh_priv.param("publish_objects_velocity_arrows", publish_objects_velocity_arrows, true);
  nh_priv.param("velocity_arrows_use_full_gray_scale", velocity_arrows_use_full_gray_scale, false);
  nh_priv.param("velocity_arrows_use_sensor_frame", velocity_arrows_use_sensor_frame_param, false);
  nh_priv.param("velocity_arrows_use_base_frame", velocity_arrows_use_base_frame_param, false);
  nh_priv.param("velocity_arrows_use_fixed_frame", velocity_arrows_use_fixed_frame_param, false);
  nh_priv.param("threshold_min_confidence", threshold_min_confidence, 0.0);
  nh_priv.param("threshold_max_delta_time_for_different_sources", threshold_max_delta_time_for_different_sources, 0.2);
  nh_priv.param("threshold_max_delta_position", threshold_max_delta_position, 0.1);
  nh_priv.param("threshold_max_delta_velocity", threshold_max_delta_velocity, 0.1);
  nh_priv.param("ignore_z_map_coordinate_for_position", ignore_z_map_coordinate_for_position, true);
  
  
  
  bool velocity_arrows_use_sensor_frame = false;
  bool velocity_arrows_use_base_frame = false;
  bool velocity_arrows_use_fixed_frame = false;
  if      (velocity_arrows_use_sensor_frame_param)
    velocity_arrows_use_sensor_frame = true;
  else if (velocity_arrows_use_base_frame_param)
    velocity_arrows_use_base_frame = true;
  else if (velocity_arrows_use_fixed_frame_param)
    velocity_arrows_use_fixed_frame = true;
  const double threshold_max_delta_position_line_squared = threshold_max_delta_position * 
                                                           threshold_max_delta_position;
  const double threshold_max_delta_velocity_squared = threshold_max_delta_velocity * threshold_max_delta_velocity;
  
  
  // Local message pointer
  find_moving_objects::MovingObjectArray::ConstPtr msg_sender;
  find_moving_objects::MovingObjectArray::ConstPtr msg_other;
  int sender_index = -1;
  
  // Init msgs
  sensor_msgs::LaserScan msg_objects_closest_point_markers;
  const unsigned int points = 720;
  const double range_min = 0.0;
  const double range_max = 100.0;
  const double resolution = TWO_PI / points;
  const double inverted_resolution = points / TWO_PI;
  const double out_of_range = range_max + 10.0;
  if (publish_objects_closest_points_markers)
  {
    msg_objects_closest_point_markers.header.seq = 0;
    msg_objects_closest_point_markers.angle_min = -M_PI;
    msg_objects_closest_point_markers.angle_max = M_PI;
    msg_objects_closest_point_markers.angle_increment = resolution;
    msg_objects_closest_point_markers.time_increment = 0.0;
    msg_objects_closest_point_markers.scan_time = 0.0;
    msg_objects_closest_point_markers.range_min = range_min;
    msg_objects_closest_point_markers.range_max = range_max;
    msg_objects_closest_point_markers.intensities.resize(points);
    msg_objects_closest_point_markers.ranges.resize(points);
    for (unsigned int i=0; i<points; ++i)
    {
      msg_objects_closest_point_markers.ranges[i] = out_of_range;
      msg_objects_closest_point_markers.intensities[i] = 0.0;
    }
  }
  visualization_msgs::Marker msg_objects_velocity_arrow;
  if (publish_objects_velocity_arrows)
  {
    msg_objects_velocity_arrow.ns = "fused_velocity_arrow";
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
    msg_objects_velocity_arrow.points[0].z = 0.0;
    msg_objects_velocity_arrow.points[1].z = 0.0;
  }
  visualization_msgs::MarkerArray msg_objects_velocity_arrows;
  unsigned int arrow_seq = 0;
  
  // Spin
  while (g_node->ok())
  {
    // Count of senders
    unsigned int nr_senders = 0;
  
    // CS start - wait for a message to arrive
    pthread_mutex_lock(&g_mutex_moa);
    // Wait for message to be available
    while (!g_available_moa)
    {
      // Exit?
      if (!g_node->ok())
      {
        ROS_BREAK();
      }
      pthread_cond_wait(&g_cond_moa, &g_mutex_moa);
    }
    // Read that message and put it in the appropriate buffer/space
    g_available_moa = false;
    msg_sender = g_msg_buffer_moa[g_sender_index];
    sender_index = g_sender_index;
    nr_senders = g_nr_senders;
    
    // CS end - we now have a message to work on and the reference to it will not be modified
    pthread_mutex_unlock(&g_mutex_moa);
    
    /* msg_sender points to the newly arrived msg, which comes from the sender with index sender_index */
    
    if (print_received_objects)
    {
      std::ostringstream stream;
      stream << "Received moving objects from " << msg_sender->origin_node_name \
             << " (sender " << sender_index+1 << "/" << nr_senders << "):" << std::endl \
             << *msg_sender << std::endl;
      std::string string = stream.str();
      ROS_DEBUG("%s", string.c_str());
    }
    
    // Look at each object and see if we can find an appropriate object in the latest message from each other sender
    // Make sure there are objects in the received msg
    const unsigned int nr_sender_objects = msg_sender->objects.size();
    if (0 < nr_sender_objects)
    {
      // Init output msg
      find_moving_objects::MovingObjectArray moa;
      moa.origin_node_name = ros::this_node::getName();
      
      /* 
       * Only send objects included in the current msg!
       * Any other object should already have been reported!
       */
      
      // Sender stamp
      const double sender_stamp = msg_sender->objects[0].header.stamp.toSec();
      
      // Loop over each object in the incoming msg
      unsigned int nr_objects = 0; // Count of objects to send
      for (unsigned int j=0; j<nr_sender_objects; ++j)
      {
        // Local pointer to the jth sender object
        const find_moving_objects::MovingObject * sender_mo = & (msg_sender->objects[j]);
        
        // Keep track of how many other senders have a matching object and the sum of the confidences
        unsigned int nr_matching_senders = 0;
        double confidence_sum = 0.0;
        
        // Loop over each sender
        for (unsigned int i=0; i<nr_senders; ++i)
        {
          // Make sure we are not looking at our own sender/message
          if (i != sender_index)
          {
            // Point to the other message and update nr_senders in case new senders have reported seen objects
            pthread_mutex_lock(&g_mutex_moa);
            msg_other = g_msg_buffer_moa[i];
            nr_senders = g_nr_senders;
            pthread_mutex_unlock(&g_mutex_moa);
            
            // Get number of objects reported by the other sender
            const unsigned int nr_other_objects = msg_other->objects.size();

            // Make sure there are objects in the other msg
            if (0 < nr_other_objects)
            {
              // Other stamp
              const double other_stamp = msg_other->objects[0].header.stamp.toSec();
              
              // Compare stamps to see if objects occur with low enough difference in time
              if (fabs(sender_stamp - other_stamp) < threshold_max_delta_time_for_different_sources)
              {                
                // Loop over other objects to find a corresponding one
                for (unsigned int k=0; k<nr_other_objects; ++k)
                {
                  // Local pointer to the kth sender object
                  const find_moving_objects::MovingObject * other_mo = & (msg_other->objects[k]);
                  
                  // Compare position and velocity in global frame (assume this frame is the same for all sources)
                  const double dx = sender_mo->position_in_map_frame.x - other_mo->position_in_map_frame.x;
                  const double dy = sender_mo->position_in_map_frame.y - other_mo->position_in_map_frame.y;
                  const double dz = ignore_z_map_coordinate_for_position ?
                                    0.0 :
                                    sender_mo->position_in_map_frame.z - other_mo->position_in_map_frame.z;
                  const double dp2 = dx*dx + dy*dy + dz*dz;
                  
                  const double dvx = sender_mo->velocity_in_map_frame.x - other_mo->velocity_in_map_frame.x;
                  const double dvy = sender_mo->velocity_in_map_frame.y - other_mo->velocity_in_map_frame.y;
                  const double dvz = sender_mo->velocity_in_map_frame.z - other_mo->velocity_in_map_frame.z;
                  const double dv2  = dvx*dvx + dvy*dvy + dvz*dvz;
                  
//                   ROS_WARN_STREAM("dp2 = " << dp2 << "  (" << threshold_max_delta_position_line_squared << ")   dv2 = "
//                      << dv2 << "  (" << threshold_max_delta_velocity_squared << ")");
                  
                  // Are the objects quite the same?
                  // Adapt confidence accordingly
                  if (dp2 < threshold_max_delta_position_line_squared &&
                      dv2 < threshold_max_delta_velocity_squared)
                  {
                    // We found another sender that has a message sent within the given time threshold and with an
                    // object matching the current one
                    nr_matching_senders++;
                    confidence_sum += other_mo->confidence;
                    break; // Go to next sender
                  }
                }
              }
            }
          }
        }
        
        ROS_DEBUG_STREAM("Increasing confidence of object based on " << nr_matching_senders << " matching senders");
        
        // Update confidence of object for the object in the output msg
        double confidence = sender_mo->confidence;
        if (0 < nr_matching_senders)
        {
          confidence += confidence_sum / nr_matching_senders;
          
          // Put confidence inside [0,1]
          confidence = MIN(1.0, confidence);
          confidence = MAX(0.0, confidence);
        }
        
        // Add object to output moa if confidence is high enough
        if (threshold_min_confidence <= confidence)
        {
          moa.objects.push_back(*sender_mo);
          nr_objects++;
          moa.objects.back().confidence = confidence;
        }
      }
            
      // Do we have any objects?
      if (0 < nr_objects)
      {
        // Send moa msg
        if (publish_objects)
        {
          g_pub_moaf.publish(moa);
        }
      
        // Publish closest point markers
        if (publish_objects_closest_points_markers)
        {
          // Update sequence number and stamp
          msg_objects_closest_point_markers.header.seq++;
          msg_objects_closest_point_markers.header.stamp = moa.objects[0].header.stamp;
          
          // Use sensor frame
          msg_objects_closest_point_markers.header.frame_id = moa.objects[0].header.frame_id;
          
          // Vector for remembering which range indices have been marked
          std::vector<int> indices;
          unsigned int nr_indices = 0;
          for (unsigned int a=0; a<nr_objects; ++a)
          {
            // Calculate angle in sensor frame where object is to be found
            double angle = moa.objects[a].angle_for_closest_distance; // Angle in [-PI,PI], hopefully
            angle += M_PI; // Shift by PI with intention to start with 0 angle in the direction of the negative x axis
            // Put angle in [0, 2PI) so that index can be calculated
            if (angle < 0 || TWO_PI <= angle)
            {
              angle -= floor(angle / TWO_PI) * TWO_PI;
            }
            
            // Calculate index
            int index = angle * inverted_resolution;
            
            // Save range index
            indices.push_back(index);
            nr_indices++;
            
            // Mark closest point
            msg_objects_closest_point_markers.ranges[index] = moa.objects[a].closest_distance;
          }
          
          // Publish
          g_pub_objects_closest_point_markers.publish(msg_objects_closest_point_markers);
          
          // Reset ranges
          for (unsigned k=0; k<nr_indices; ++k)
          {
            msg_objects_closest_point_markers.ranges[indices[k]] = out_of_range;
          }
        }
        
        // Publish velocity arrows
        if (publish_objects_velocity_arrows)
        {
          msg_objects_velocity_arrow.header.seq = ++arrow_seq;
          
          for (unsigned int a=0; a<nr_objects; ++a)
          {
            msg_objects_velocity_arrow.header.stamp = moa.objects[a].header.stamp;
            if (velocity_arrows_use_sensor_frame)
            {
              msg_objects_velocity_arrow.header.frame_id = moa.objects[a].header.frame_id;
              msg_objects_velocity_arrow.points[0].x = moa.objects[a].position.x;
              msg_objects_velocity_arrow.points[0].y = moa.objects[a].position.y;
              msg_objects_velocity_arrow.points[1].x = moa.objects[a].position.x + moa.objects[a].velocity.x;
              msg_objects_velocity_arrow.points[1].y = moa.objects[a].position.y + moa.objects[a].velocity.y;
            }
            else if (velocity_arrows_use_base_frame)
            {
              msg_objects_velocity_arrow.header.frame_id = moa.objects[a].base_frame;
              msg_objects_velocity_arrow.points[0].x = moa.objects[a].position_in_base_frame.x;
              msg_objects_velocity_arrow.points[0].y = moa.objects[a].position_in_base_frame.y;
              msg_objects_velocity_arrow.points[1].x = moa.objects[a].position_in_base_frame.x + 
                                                       moa.objects[a].velocity_in_base_frame.x;
              msg_objects_velocity_arrow.points[1].y = moa.objects[a].position_in_base_frame.y + 
                                                       moa.objects[a].velocity_in_base_frame.y;
            }
            else if (velocity_arrows_use_fixed_frame)
            {
              msg_objects_velocity_arrow.header.frame_id = moa.objects[a].fixed_frame;
              msg_objects_velocity_arrow.points[0].x = moa.objects[a].position_in_fixed_frame.x;
              msg_objects_velocity_arrow.points[0].y = moa.objects[a].position_in_fixed_frame.y;
              msg_objects_velocity_arrow.points[1].x = moa.objects[a].position_in_fixed_frame.x + 
                                                       moa.objects[a].velocity_in_fixed_frame.x;
              msg_objects_velocity_arrow.points[1].y = moa.objects[a].position_in_fixed_frame.y + 
                                                      moa.objects[a].velocity_in_fixed_frame.y;
            }
            else // map frame
            {
              msg_objects_velocity_arrow.header.frame_id = moa.objects[a].map_frame;
              msg_objects_velocity_arrow.points[0].x = moa.objects[a].position_in_map_frame.x;
              msg_objects_velocity_arrow.points[0].y = moa.objects[a].position_in_map_frame.y;
              msg_objects_velocity_arrow.points[1].x = moa.objects[a].position_in_map_frame.x + 
                                                       moa.objects[a].velocity_in_map_frame.x;
              msg_objects_velocity_arrow.points[1].y = moa.objects[a].position_in_map_frame.y + 
                                                       moa.objects[a].velocity_in_map_frame.y;
            }
            msg_objects_velocity_arrow.id = a;
            
            // Color of the arrow represents the confidence black=low, white=high
            float adapted_confidence = moa.objects[a].confidence;
            if (velocity_arrows_use_full_gray_scale && threshold_min_confidence < 1)
            {
              adapted_confidence = (moa.objects[a].confidence - threshold_min_confidence) / 
                                   (1 - threshold_min_confidence);
            }
            msg_objects_velocity_arrow.color.r = adapted_confidence;
            msg_objects_velocity_arrow.color.g = adapted_confidence;
            msg_objects_velocity_arrow.color.b = adapted_confidence;
            
            // Add to array of markers
            msg_objects_velocity_arrows.markers.push_back(msg_objects_velocity_arrow);
          }
          
          // Publish the msg
          g_pub_objects_velocity_arrows.publish(msg_objects_velocity_arrows);
          
          // Clear msg from all arrows
          msg_objects_velocity_arrows.markers.clear();
        }
      }
    } // Here, the moa msg is out of scope and moa.objects[] is thus destroyed
  }
  
  return NULL;
}


/* ENTRY POINT */
int main(int argc, char** argv)
{  
  // Init ROS
  ros::init(argc, argv, "mo_confidence_enhancer", ros::init_options::AnonymousName);
  g_node = new ros::NodeHandle;
  ros::NodeHandle nh_priv("~");
  
  // Wait for time to become valid
  ros::Time::waitForValid();
  
  // Init publisher
  std::string topic_moving_objects_enhanced;
  std::string topic_objects_velocity_arrows;
  std::string topic_objects_closest_points_markers;
  int publish_buffer_size;
  nh_priv.param("topic_moving_objects_enhanced", topic_moving_objects_enhanced, std::string("moving_objects_enhanced"));
  nh_priv.param("topic_objects_velocity_arrows", topic_objects_velocity_arrows, std::string("objects_velocity_arrows"));
  nh_priv.param("topic_objects_closest_points_markers", topic_objects_closest_points_markers, std::string("objects_closest_points_markers"));
  nh_priv.param("publish_buffer_size", publish_buffer_size, 2);
  g_pub_moaf = g_node->advertise<find_moving_objects::MovingObjectArray>
                                                 (topic_moving_objects_enhanced,
                                                  publish_buffer_size);
  g_pub_objects_velocity_arrows = g_node->advertise<visualization_msgs::MarkerArray>
                                                 (topic_objects_velocity_arrows,
                                                  publish_buffer_size);
  g_pub_objects_closest_point_markers = g_node->advertise<sensor_msgs::LaserScan>
                                                 (topic_objects_closest_points_markers,
                                                  publish_buffer_size);
  
  // MovingObjectArray handler thread
  pthread_t moa_handler;
  if (pthread_create(&moa_handler, NULL, moaHandlerBody, NULL))
  {
    ROS_ERROR("Failed to create thread for handling MovingObjectArray messages");
    ROS_BREAK();
  }
  
  // Subscribe to interpreter results
  
  std::string subscribe_topic;
  nh_priv.param("subscribe_topic", subscribe_topic, std::string("moving_objects"));
  int subscribe_buffer_size;
  nh_priv.param("subscribe_buffer_size", subscribe_buffer_size, 10);
  ros::Subscriber sub = g_node->subscribe(subscribe_topic, 
                                          subscribe_buffer_size, 
                                          moaCallback);
 
  // Start main ROS loop
  ros::spin();
  
  return 0;
}
