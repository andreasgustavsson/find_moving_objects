# find_moving_objects

A ROS library that can be used to find moving objects. It derives their positions and velocities,
based on a 2D LaserScan or a 3D PointCloud2 data stream.

The file src/find_moving_objects/bank.h declares a class called BankArgument. An object of this 
class is taken as input by the main class, Bank, also declared in this file. BankArgument mainly 
consists of variables that control the behavior of the Bank object. Please refer to 
src/find_moving_objects/bank.h or a doxygen-generated documentation for each variable.
Note that Bank declares the function virtual double calculateConfidence(...), but it is up to the
user of Bank to define it!

The package defines two executable ROS nodes which use the Bank; one for interpreting a LaserScan data 
stream and one for interpreting a PointCloud2 data stream. There are also two corresponding nodelets.

The nodelet for interpreting PointCloud2 data streams and the node interpreting LaserScan data streams
are used in the provided launch file. This launch file can be used to run the interpreters on live or
recorded (an example bag file is provided) sensor data. The sensors supported by the launch file are 
the following.
* Intel Realsense D435 depth camera (for PointCloud2 data at about 30Hz)
* Slamtec rplidar A2 (for LaserScan data at about 12Hz)

NOTE: If running the \*_bag.launch file, then bags/\*.bag.tar.gz must first be extracted to 
      bags/\*.bag. Also note that rosbag and rviz are used by the launch file.

The default values for the BankArgument variables (which are set by the default ROS parameter values as
found in the launch/includes/\*.launch.xml files) have been tested and work well for an office setting 
and a robot moving at about 0.5 m/s, equipped with the above-mentioned sensors.

NOTE: If running the launch file for real sensors, then you need the realsense2_camera and rplidar_ros 
      packages, and the librealsense camera driver installed. These are not dependencies of 
      find_moving_objects because this package can be used with several different sensors, not only these.
      The launch file is merely given as an example, and can be used by those who have the named packages
      installed. Also note that rviz is used by the launch file.

This library is developed for, and tested on, ROS Kinetic.
