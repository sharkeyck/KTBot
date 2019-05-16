#!/usr/bin/env python
import rospy
import numpy as np
import argparse
import sys
from sensor_msgs.msg import JointState, PointCloud2
from inserter_detector_py import convert, model
from inserter_detector.msg import CloudSegments

def main():
  parser = argparse.ArgumentParser(description='Does a few example predictions from a TF.Record file')
  parser.add_argument('--tfrecord_path', type=str, help='input .tfrecord file')
  parser.add_argument('--model_json_path', type=str, help='path to saved model json file')
  parser.add_argument('--weights_path', type=str, help='path to saved model weights')
  args = parser.parse_args(rospy.myargv(argv=sys.argv[1:]))

  pubs = dict()
  rospy.init_node('inference_node', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  rospy.loginfo("Loading model from {0}, weights from {1}".format(args.model_json_path, args.weights_path))
  detector = model.Model(args.model_json_path, args.weights_path)
  rospy.loginfo("Model loaded")

  def callback(cloud_segments):
    # rospy.loginfo(rospy.get_caller_id() + " got point cloud")
    images = np.stack([np.frombuffer(cloud_segments.image[i].data, np.uint8).astype(np.float32).reshape((64, 64, 1)) for i in xrange(len(cloud_segments.name))])
    results = detector.predict_joint_states(images)
    for i, r in enumerate(results):
      n = cloud_segments.name[i].split('/')[0]
      if pubs.get(n) == None:
        pubs[n] = rospy.Publisher(n + '/joint_states_predicted', JointState, queue_size=10)

      r.header.stamp = rospy.Time.now()
      pubs[n].publish(r)

  rospy.Subscriber("input", CloudSegments, callback)
  rospy.spin()
