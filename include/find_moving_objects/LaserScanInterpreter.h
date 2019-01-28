#include <ros/ros.h>

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
  
#ifdef LSARRAY
  /* BANK AND ARGUMENT */
  std::vector<Bank *> banks;
  std::vector<BankArgument> bank_arguments;
  
  /* TF listener */
  tf::TransformListener * tfListener;
  
  /* CALLBACK FOR FIRST MESSAGE */
  void laserScanArrayCallbackFirst(const find_moving_objects::LaserScanArray::ConstPtr & msg);
  
  /* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
  void laserScanArrayCallback(const find_moving_objects::LaserScanArray::ConstPtr & msg);
  
  /* CALLBACK FOR WAITING UNTIL THE FIRST MESSAGE WAS RECEIVED - HZ CALCULATION */
  void waitForFirstMessageCallback(const find_moving_objects::LaserScanArray::ConstPtr & msg);
  
  /* CALLBACK FOR HZ CALCULATION */
  void hzCalculationCallback(const find_moving_objects::LaserScanArray::ConstPtr & msg);
#else
  /* BANK AND ARGUMENT */
  Bank * bank;
  BankArgument bank_argument;

  /* LOCAL MESSAGE POINTER */
  sensor_msgs::LaserScan::ConstPtr msg_old;

  /* CALLBACK FOR FIRST MESSAGE */
  void laserScanCallbackFirst(const sensor_msgs::LaserScan::ConstPtr & msg);
  
  /* CALLBACK FOR ALL BUT THE FIRST MESSAGE */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
  
  /* CALLBACK FOR WAITING UNTIL THE FIRST MESSAGE WAS RECEIVED - HZ CALCULATION */
  void waitForFirstMessageCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
  
  /* CALLBACK FOR HZ CALCULATION */
  void hzCalculationCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
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
