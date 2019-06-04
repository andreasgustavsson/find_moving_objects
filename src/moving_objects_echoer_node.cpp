#include <ros/ros.h>
#include <find_moving_objects/MovingObjectArray.h>
#include <find_moving_objects/MovingObject.h>

using namespace std;



void callback(const find_moving_objects::MovingObjectArray::ConstPtr & msg)
{
  cout << "Received message from node " << msg->origin_node_name
       << " containing " << msg->objects.size() << " detected moving objects." <<  endl;
       
  for (int i=0; i<msg->objects.size(); ++i)
  {
    cout << "  Object " << i << " has center point at (" 
         << msg->objects[i].position.x << "," 
         << msg->objects[i].position.y << "," 
         << msg->objects[i].position.z << ") relative the sensor (i.e. in frame " 
         << msg->objects[i].header.frame_id << ")"
         << endl << "  and at (" 
         << msg->objects[i].position_in_map_frame.x << "," 
         << msg->objects[i].position_in_map_frame.y << "," 
         << msg->objects[i].position_in_map_frame.z << ") in the map (i.e. in frame "
         << msg->objects[i].map_frame << ")" << endl
         << "  and is travelling with relative speed "
         << msg->objects[i].speed << "m/s" << endl
         << "  and absolute speed "
         << msg->objects[i].speed_in_map_frame << "m/s." << endl;
  }
  
  // See msg/MovingObject.msg for the other fields in the message which can be accessed in the same way.
}



int main (int argc, char ** argv)
{
  // Init ROS
  ros::init(argc, argv, "moving_object_echoer_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  
  // Wait for time to become valid
  ros::Time::waitForValid();
  
  // Init publisher
  string topic;
  int buffer_size;
  nh_priv.param("topic", topic, string("moving_objects"));
  nh_priv.param("buffer_size", buffer_size, 10);
  
  ros::Subscriber sub = nh.subscribe(topic, buffer_size, callback);
 
  // Start main ROS loop
  ros::spin();
  
  return 0;
}