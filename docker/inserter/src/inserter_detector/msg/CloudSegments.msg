# This message holds a list of segmented point clouds.
# The fields are correlated by index, i.e. names[1] is the
# name for the data at poses[1] and clouds[1].
#
# All values are broadcast in world frame.
#
# This message type aims to be similar to gazebo_msgs/ModelStates
# (https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/msg/ModelStates.msg)

# Name of the segment
string[] name

# Centroid of the segment
geometry_msgs/Point[] centroid

# Disparity image of the segment
sensor_msgs/Image[] image

# Performance and other stats
CloudSegmentsStats stats
