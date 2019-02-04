# stamp is the time at which the sensor scanned this 
# object. 
# frame_id is the frame (i.e. coordinate system) of the 
# sensor - this is the frame in which the given relative 
# information below (the variables position, velocity, 
# speed, angle_* etc.) is valid.
# seq is not really used.
Header header

# The frame which is considered world-fixed and never 
# moves. Note that the robot's position in this frame 
# can be discontinuous.
# The variables *_in_map_frame below are specified in 
# this frame.
string map_frame

# The frame in which the movement of the robot is always
# continuous and in which all transforms are made.
# The variables *_in_fixed_frame below are 
# specified in this frame.
string fixed_frame

# The frame which is considered fixed on the robot and 
# in which the variables *_in_base_frame below are
# specified.
string base_frame

# The width of the object as seen by the given 
# sensor (calculated based on angle_begin, angle_end,
# distance_angle_begin and distance_angle_end as 
# given below using the law of cosine).
float64 seen_width

# In frame header.frame_id, angles are measured around 
# the positive Z axis (counterclockwise; Z is pointing up)
# with zero angle being forward along the x axis, provided
# that the frame is not an optical/camera frame.
# In case of an optical frame (Z is pointing forward and Y
# is pointing down), then angles are measured around the 
# negative Y axis.
# The angles given specify where the object is to be
# found in the frame header.frame_id (this is thus 
# relative to the X-axis of the sensor in a regular frame
# and relative the Z-axis, but negated, in an optical frame).
float64 angle_begin
float64 angle_end

# The scanned distances to the object's end points as 
# specified by angle_begin and angle_end.
float64 distance_at_angle_begin
float64 distance_at_angle_end

# The averaged distance (approximated to be at 
# (angle_begin+angle_end)/2) from the sensor to the seen
# object.
float64 distance

# Position and velocity of the object.
geometry_msgs/Point position_in_map_frame
geometry_msgs/Point position_in_fixed_frame
geometry_msgs/Point position_in_base_frame
geometry_msgs/Point position
geometry_msgs/Vector3 velocity_in_map_frame
geometry_msgs/Vector3 velocity_in_fixed_frame
geometry_msgs/Vector3 velocity_in_base_frame
geometry_msgs/Vector3 velocity

# For convenience, the normalized velocity vector is
# also given, along with the scale factor (speed).
geometry_msgs/Vector3 velocity_normalized_in_map_frame
geometry_msgs/Vector3 velocity_normalized_in_fixed_frame
geometry_msgs/Vector3 velocity_normalized_in_base_frame
geometry_msgs/Vector3 velocity_normalized
float64 speed_in_map_frame
float64 speed_in_fixed_frame
float64 speed_in_base_frame
float64 speed

# The point on the object closest to the sensor.
geometry_msgs/Point closest_point_in_map_frame
geometry_msgs/Point closest_point_in_fixed_frame
geometry_msgs/Point closest_point_in_base_frame
geometry_msgs/Point closest_point

# For convenience, the distance from the sensor to the 
# closest point on the object and at what angle it is to 
# be found in relation to the sensor (i.e. 
# header.frame_id).
float64 closest_distance
float64 angle_for_closest_distance

# A measure on how confident the sending node is about 
# the specified information.
float64 confidence