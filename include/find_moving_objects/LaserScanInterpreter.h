#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef LSARRAY
#include <find_moving_objects/LaserScanArray.h>
#else
#include <sensor_msgs/LaserScan.h>
#endif

#include <find_moving_objects/bank.h>

#ifdef NODELET
#include <nodelet/nodelet.h>
#endif
        
namespace find_moving_objects
{

#ifdef NODELET
# ifdef LSARRAY
class LaserScanArrayInterpreterNodelet : public nodelet::Nodelet
# else
class LaserScanInterpreterNodelet : public nodelet::Nodelet
# endif
#endif
#ifdef NODE
# ifdef LSARRAY
class LaserScanArrayInterpreterNode
# else
class LaserScanInterpreterNode
# endif
#endif
{
private:
  /* DEFAULT PARAMETER VALUES */
  #include "laserscan_interpreter_default_parameter_values.h"
  
  /* SUBSCRIBE INFO */
  std::string subscribe_topic;
  int subscribe_buffer_size;
  
  /* HZ CALCULATION */
  double optimize_nr_scans_in_bank;
  int received_messages;
  const int max_messages = 100;
  double start_time;
  const double max_time = 1.5;
  
  /* BANK AND ARGUMENT */
  std::vector<Bank *> banks;
  std::vector<BankArgument> bank_arguments;
  
  /* TF LISTENER, BUFFER AND TARGET FRAME */
  tf2_ros::Buffer * tf_buffer;
  tf2_ros::TransformListener * tf_listener;
  std::vector<std::string> tf_filter_target_frames;
  
  /* NODE HANDLES (ROS must be initialized when object is created) */
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  
  /* STATES OF MESSAGE RECEIVING */
  typedef enum
  {
    WAIT_FOR_FIRST_MESSAGE_HZ,
    CALCULATE_HZ,
    INIT_BANKS,
    FIND_MOVING_OBJECTS
  } state_t;
  state_t state;
  
#ifdef LSARRAY  
  /* MESSAGE FILTER */
  message_filters::Subscriber<find_moving_objects::LaserScanArray> * tf_subscriber;
  tf2_ros::MessageFilter<find_moving_objects::LaserScanArray> * tf_filter;
  
  /* CALLBACK */
  void laserScanArrayCallback(const find_moving_objects::LaserScanArray::ConstPtr & msg);
#else
  /* MESSAGE FILTER */
  message_filters::Subscriber<sensor_msgs::LaserScan> * tf_subscriber;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> * tf_filter;
  
  /* CALLBACK */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
#endif
  
public:
  /* CONSTRUCTOR & DESTRUCTOR */
#ifdef NODELET
# ifdef LSARRAY
  LaserScanArrayInterpreterNodelet();
  ~LaserScanArrayInterpreterNodelet();
# else
  LaserScanInterpreterNodelet();
  ~LaserScanInterpreterNodelet();
# endif
  
  virtual void onInit();
#endif
#ifdef NODE
# ifdef LSARRAY
  LaserScanArrayInterpreterNode();
  ~LaserScanArrayInterpreterNode();
# else
  LaserScanInterpreterNode();
  ~LaserScanInterpreterNode();
# endif
  
  void onInit();
#endif
};

} // namespace find_moving_objects
