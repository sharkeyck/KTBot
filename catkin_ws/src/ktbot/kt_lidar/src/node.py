#!/usr/bin/env python

import roslib; roslib.load_manifest("kt_lidar")
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.broadcaster import TransformBroadcaster

from driver import Lidar

class LidarNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('kt_lidar')

        self.port = rospy.get_param('~port', "/dev/ttyACM1")
        rospy.loginfo("Using port: %s"%(self.port))

        scan_link = rospy.get_param('~frame_id','base_laser_link')
        self.scan = LaserScan(header=rospy.Header(frame_id=scan_link))
        self.scan.angle_min = 0
        self.scan.angle_max = 6.26
        self.scan.angle_increment = 0.017437326
        self.scan.range_min = Lidar.MIN_RANGE
        self.scan.range_max = Lidar.MAX_RANGE

        # TODO REMOVE
        self.odomBroadcaster = TransformBroadcaster()

        self.lidar = Lidar(self.port, cb=self.onLidarData)
        self.scanPub = rospy.Publisher('base_scan', LaserScan, queue_size=10)


    def onLidarData(self, ranges):
        # prepare laser scan
        self.scan.header.stamp = rospy.Time.now()
        self.scan.ranges = ranges
        self.scanPub.publish(self.scan)
        self.odomBroadcaster.sendTransform( (0,0, 0), (0,0,0,1),
                self.scan.header.stamp, "base_link", "odom" )

    def loop(self):
        # main loop of driver
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    robot = LidarNode()
    robot.loop()

