# find_moving_objects

A ROS library that can be used to find moving objects. It derives their positions and velocities,
based on a 2D laser scan or a 3D point cloud 2 data stream.

The file src/find_moving_objects/bank.h declares a class called BankArgument. An object of this 
class is taken as input by the main class, Bank, also declared in this file. BankArgument mainly 
consists of variables that control the behavior of the Bank object. Please refer to 
src/find_moving_objects/bank.h or a doxygen-generated documentation for each variable.
Note that Bank declares the function virtual double calculateConfidence(...), but it is up to the
user of Bank to define it!

The package defines two executables which use the Bank; one for interpreting a laser scan data 
stream and one for interpreting a point cloud data stream. These are used in the provided launch 
files.

NOTE: If running the \*_bag.launch file, then bags/\*.bag.tar.gz must first be extracted to 
      bags/*.bag. Also note that rosbag and rviz are used by the launch file.

The values for the BankArgument variables, as specified by the two executables and by the launch 
files have been tested and work well for an office setting and a robot moving at about 0.5 m/s, 
equipped with a LIDAR which produces scans at a frequency of about 12 Hz and a depth camera, 
filtered by a voxel grid which produces point clouds at a rate of about 15 Hz. The used sensors 
were a Slamtec rplidar A2M8 (LIDAR) and an Intel Realsense D435 (camera).

NOTE: If running the live launch file, then you need the realsense2_camera and rplidar_ros packages,
      and the librealsense camera driver installed. These are not depencies of find_moving_objects
      because this package can be used with several different sensors, not only these. The launch
      file is merely given as an example, and can be used by those who have the named packages
      installed. Also note that rviz is used by the launch file.

This library is developed for, and tested on, ROS Kinetic.
