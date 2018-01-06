#!/usr/bin/env python

import roslib; roslib.load_manifest("kt_wheel_controller")
import rospy

from geometry_msgs.msg import Twist
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
from ktbot import MotorStatus
from driver import WheelController, WHEEL_CIRCUMFERENCE_METERS, BASE_WIDTH_METERS, MAX_SPEED_MPS

class WheelControllerNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('kt_wheel_controller')

        self.port = rospy.get_param('~port', "/dev/ttyACM1")
        rospy.loginfo("Using port: %s"%(self.port))

        self.odomBroadcaster = TransformBroadcaster()
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.statusPub = rospy.Publisher('motor_status', MotorStatus, queue_size=10)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)

        self.controller = WheelController(self.port, encoderCb=self.onWheelStatus, infrequentCb=self.onInfrequentStats)
        self.prevWheelPos = [None, None]

    def onInfrequentStats(self, temperatures, voltages):
        status = MotorStatus()
        status.temp_c = temperatures
        status.vin_mv = voltages
        self.statusPub(status)

    def onWheelStatus(self, leftWheelAngle, rightWheelAngle):
        dt = (scan.header.stamp - then).to_sec()
        then = scan.header.stamp

        # Get delta in meters driven for each wheel
        d_left = (leftWheelAngle - self.prevWheelPos[0])/360.0 * WHEEL_CIRCUMFERENCE_METERS
        d_right = (rightWheelAngle - self.prevWheelPos[1])/360.0 * WHEEL_CIRCUMFERENCE_METERS
        self.prevWheelPos = [leftWheelAngle, rightWheelAngle]

        # Split into forward distance traveled and radians turned
        dx = (d_left+d_right)/2
        dth = (d_right-d_left)/BASE_WIDTH_METERS

        # Get x and y change in position in local coordinates
        x = cos(dth)*dx
        y = -sin(dth)*dx

        # Update odom (global?) coordinates
        self.x += cos(self.th)*x - sin(self.th)*y
        self.y += sin(self.th)*x + cos(self.th)*y
        self.th += dth

        # prepare transform from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self.th/2.0)
        quaternion.w = cos(self.th/2.0)

        # prepare odometry
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = dx/dt
        odom.twist.twist.angular.z = dth/dt
        self.odomPub(odom)

        self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            then, "base_link", "odom" )

    def cmdVelUpdate(self, req):
        # Forward velocity
        x = req.linear.x * 1000

        # Difference between wheel speeds for turning
        th = req.angular.z * (BASE_WIDTH_METERS/2)

        # Faster wheel speed
        k = max(abs(x-th),abs(x+th))

        # Limit the max speed if it exceeds MAX_SPEED_MPS
        if k > MAX_SPEED_MPS:
            x = x*MAX_SPEED_MPS/k
            th = th*MAX_SPEED_MPS/k

        # Set velocity for both wheels
        self.controller.set_cmd_vel([int(x-th), int(x+th)])

    def loop(self):
        # main loop of driver
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    robot = WheelControllerNode()
    robot.loop()
