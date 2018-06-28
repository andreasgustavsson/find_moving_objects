# find_moving_objects

A ROS library that can be used to find moving objects. It derives their positions and velocities,
based on a 2D laser scan or a 3D point cloud 2 data stream.

The file src/find_moving_objects/bank.h declares a class called BankArgument. An object of this 
class is taken as input by the main class, Bank, also declared in this file. BankArgument mainly 
consists of variables that control the behavior of the Bank object. Please refer to 
src/find_moving_objects/bank.h or a doxygen-generated documentation for each variable. The default
values for the variables work well for an office setting and a robot moving at about 0.5 m/s, 
equipped with a LIDAR which produces scans at a frequency of about 25 Hz.
