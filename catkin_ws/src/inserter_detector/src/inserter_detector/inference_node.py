#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState, PointCloud2
from inserter_detector import convert, model

def main():
  pub = rospy.Publisher('output', JointState, queue_size=10)
  rospy.init_node('inference_node', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  model_json_path = rospy.get_param('model_json_path')
  weights_path = rospy.get_param('weights_path')

  rospy.loginfo("Loading model from {0}, weights from {1}".format(model_json_path, weights_path))
  detector = model.Model(model_json_path, weights_path)
  rospy.loginfo("Model loaded")

  def callback(cloud):
    rospy.loginfo(rospy.get_caller_id() + " got point cloud")
    images = np.array([convert.cloud_to_image(cloud).reshape((convert.IMG_SIZE, convert.IMG_SIZE, 1))])
    result = detector.predict_joint_states(images)[0]
    result.header.stamp = rospy.Time.now()
    pub.publish(result)

  rospy.Subscriber("input", PointCloud2, callback)
  rospy.spin()
