#!/usr/bin/env python
# Simple simulation of wheel controller code

import roslib
roslib.load_manifest("kt_wheel_controller")
import rospy
from node import WheelControllerNode

class FakeSerial():
  in_waiting = False

  def read(self):
    return None

  def write(self, val):
    return None

if __name__ == "__main__":
  ser = FakeSerial()
  robot = WheelControllerNode(ser)
  robot.loop()
