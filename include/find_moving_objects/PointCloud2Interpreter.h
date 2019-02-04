#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef PC2ARRAY
#include <find_moving_objects/PointCloud2Array.h>
#else
#include <sensor_msgs/PointCloud2.h>
#endif

#include <find_moving_objects/bank.h>

#ifdef NODELET
#include <nodelet/nodelet.h>
#endif

namespace find_moving_objects
{

#ifdef NODELET
# ifdef PC2ARRAY
class PointCloud2ArrayInterpreterNodelet : public nodelet::Nodelet
# else
class PointCloud2InterpreterNodelet : public nodelet::Nodelet
# endif
#endif
#ifdef NODE
# ifdef PC2ARRAY
class PointCloud2ArrayInterpreterNode
# else
class PointCloud2InterpreterNode
# endif
#endif
{
private:
  /* DEFAULT PARAMETER VALUES */
  #include "pointcloud2_interpreter_default_parameter_values.h"
  
  /* SUBSCRIBE INFO */
  std::string subscribe_topic;
  int subscribe_buffer_size;
  
  /* HZ CALCULATION */
  double optimize_nr_scans_in_bank;
  double max_confidence_for_dt_match;
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
  
#ifdef PC2ARRAY
  /* MESSAGE FILTER */
  message_filters::Subscriber<find_moving_objects::PointCloud2Array> * tf_subscriber;
  tf2_ros::MessageFilter<find_moving_objects::PointCloud2Array> * tf_filter;
  
  /* CALLBACK */
  void pointCloud2ArrayCallback(const find_moving_objects::PointCloud2Array::ConstPtr & msg);
#else
  /* MESSAGE FILTER */
  message_filters::Subscriber<sensor_msgs::PointCloud2> * tf_subscriber;
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2> * tf_filter;
  
  /* CALLBACK */
  void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & msg);
#endif
  
public:
  /* CONSTRUCTOR & DESTRUCTOR */
#ifdef NODELET
# ifdef PC2ARRAY
  PointCloud2ArrayInterpreterNodelet();
  ~PointCloud2ArrayInterpreterNodelet();
# else
  PointCloud2InterpreterNodelet();
  ~PointCloud2InterpreterNodelet();
# endif
  
  virtual void onInit();
#endif
#ifdef NODE
# ifdef PC2ARRAY
  PointCloud2ArrayInterpreterNode();
  ~PointCloud2ArrayInterpreterNode();
# else
  PointCloud2InterpreterNode();
  ~PointCloud2InterpreterNode();
# endif
  
  void onInit();
#endif
};

} // namespace find_moving_objects
