# If using this message type, then:
#   the time stamp of this message and the ones in the array must be highly related, or better yet, equal; and
#   the frame_id of each message in the array must be transformable into the frame_id of this message.
# Basically, all PointCloud2 messages in the array should come from different sensor entities on the same sensor unit.
# This message type could, for example, be used for a multi-diode Lidar or similar.

Header header
sensor_msgs/LaserScan[] msgs