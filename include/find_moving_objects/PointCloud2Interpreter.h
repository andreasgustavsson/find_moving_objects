#include <ros/ros.h>

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
  
  /* SUBSCRIBER */
  ros::Subscriber sub;
  std::string subscribe_topic;
  int subscribe_buffer_size;
  
  /* HZ CALCULATION */
  double optimize_nr_scans_in_bank;
  int received_messages;
  const int max_messages = 100;
  double start_time;
  const double max_time = 1.5;
  
#ifdef PC2ARRAY
  /* BANK AND ARGUMENT */
  std::vector<Bank *> banks;
  std::vector<BankArgument> bank_arguments;
  
  /* TF listener */
  tf::TransformListener * tfListener;
  
  /* CALLBACK FOR FIRST MESSAGE */
  void pointCloud2ArrayCallbackFirst(const find_moving_objects::PointCloud2Array::ConstPtr & msg);
  
  /* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
  void pointCloud2ArrayCallback(const find_moving_objects::PointCloud2Array::ConstPtr & msg);
  
  /* CALLBACK FOR WAITING UNTIL THE FIRST MESSAGE WAS RECEIVED - HZ CALCULATION */
  void waitForFirstMessageCallback(const find_moving_objects::PointCloud2Array::ConstPtr & msg);
  
  /* CALLBACK FOR HZ CALCULATION */
  void hzCalculationCallback(const find_moving_objects::PointCloud2Array::ConstPtr & msg);
#else
  /* BANK AND ARGUMENT */
  Bank * bank;
  BankArgument bank_argument;

  /* CALLBACK FOR FIRST MESSAGE */
  void pointCloud2CallbackFirst(const sensor_msgs::PointCloud2::ConstPtr & msg);
  
  /* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
  void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr & msg);
  
  /* CALLBACK FOR WAITING UNTIL THE FIRST MESSAGE WAS RECEIVED - HZ CALCULATION */
  void waitForFirstMessageCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);
  
  /* CALLBACK FOR HZ CALCULATION */
  void hzCalculationCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);
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
