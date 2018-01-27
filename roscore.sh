#!/bin/bash
# /etc/init.d/roscore.sh
### BEGIN INIT INFO
# Provides:          roscore
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start roscore at boot time
### END INIT INFO
PATH=/sbin:/bin:/usr/bin
export ROS_HOSTNAME=localhost
source /opt/ros/kinetic/setup.bash
source /home/pi/KTBot/catkin_ws/devel/setup.bash
roslaunch kt_core ktbot_core.launch 2>&1 >> /var/log/roscore.log
# /opt/ros/kinetic/bin/roscore 2>&1 >> /var/log/roscore.log

