#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from random import random
import numpy as np
import argparse

def issue_commands(publishers):
  for pub in publishers:
    point = JointTrajectoryPoint()
    point.positions = [random()*2*np.pi, random()+0.1, random()*0.05]
    point.time_from_start = rospy.Duration(1) # max(dur) / self._speed_scale

    cmd = JointTrajectory()
    cmd.header = Header()
    cmd.joint_names = ['joint1', 'joint2', 'joint5']
    cmd.points = [point]

    pub.publish(cmd)

def main():
  parser = argparse.ArgumentParser(description='Issues random commands to a number of robots')
  parser.add_argument('--num_bots', type=int, help='number of robots', default=1)
  args = parser.parse_args()

  rospy.loginfo('Command generator: generating for ' + str(args.num_bots) + 'robots')
  publishers = []
  for i in xrange(args.num_bots):
    topic = '/inserter{0}/arm_controller/command'.format(i)
    rospy.loginfo('Commanding ' + topic)
    publishers.append(rospy.Publisher(topic, JointTrajectory, queue_size=1))

  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(0.5) # hz

  while not rospy.is_shutdown():
      issue_commands(publishers)
      rate.sleep()
