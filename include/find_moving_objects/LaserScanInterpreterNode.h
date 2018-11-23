#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <find_moving_objects/bank.h>

namespace find_moving_objects
{

class LaserScanInterpreterNode
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
  
public:
  /* CONSTRUCTOR & DESTRUCTOR */
  LaserScanInterpreterNode();
  ~LaserScanInterpreterNode();
  
  void onInit();
};

} // namespace find_moving_objects
