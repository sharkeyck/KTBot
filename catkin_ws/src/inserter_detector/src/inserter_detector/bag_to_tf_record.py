#!/usr/bin/python

import rosbag
import argparse
import tensorflow as tf
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from inserter_detector.train_pose_detection import loadDataset

NUM_POINTS = 50

def convert(infile, outfile, joint_state_topic, pc_topic):
  topics = [pc_topic, joint_state_topic]
  print 'Reading from:', infile
  bag = rosbag.Bag(infile)
  num_messages = bag.get_message_count(topic_filters=topics);
  print num_messages, 'messages'

  print 'Writing to:', outfile
  writer = tf.io.TFRecordWriter(outfile)

  def _joint_feature(jointstate):
    # jointstate is sensor_msgs/JointState
    return tf.train.Feature(float_list=tf.train.FloatList(value=jointstate.position))

  def _cloud_feature(cloud):
    # cloud is sensor_msgs/PointCloud2
    gen = pc2.read_points(cloud, skip_nans=True, field_names=('x', 'y', 'z'))
    arr = np.array(list(gen))

    # Downsample to exactly 50 points
    fixed = np.random.permutation(arr)[:NUM_POINTS]
    flat = [item for sublist in fixed for item in sublist]
    if len(flat) != NUM_POINTS*3:
      raise Exception('Found example with only ' + len(flat) + ' points, need at least ' + NUM_POINTS)

    return tf.train.Feature(float_list=tf.train.FloatList(value=flat))

  latest_joint_state = None

  i = 0
  count = 0
  for topic, msg, t in bag.read_messages(topics=topics):
      i += 1
      if not i % 1000:
        print 'Export: {}/{}'.format(i, num_messages)

      if topic == joint_state_topic:
        latest_joint_state = msg
        continue

      if topic != pc_topic:
        raise Exception('Unknown message on topic ', topic)

      example = tf.train.Example(features=tf.train.Features(feature={
        'label': _joint_feature(latest_joint_state),
        'cloud_flat': _cloud_feature(msg),
      }))

      count += 1
      writer.write(example.SerializeToString())

  print 'Closing'

  writer.close()
  bag.close()

  print 'Finished writing', count, 'examples'

def verify(filepath):
  print 'Verifying...'
  dataset = loadDataset([filepath], NUM_POINTS)

  # Create a one-shot iterator
  iterator = dataset.make_one_shot_iterator()

  # Get example value
  next_element = iterator.get_next()
  with tf.Session() as sess:
    print 'Example:', sess.run(next_element)

def main():
  parser = argparse.ArgumentParser(description='Converts .bag file to TFRecord.')
  parser.add_argument('--input', type=str, help='input .bag file')
  parser.add_argument('--output', type=str, help='output .tfrecord file')
  parser.add_argument('--verify_only', dest='verify_only', action='store_true', help='do not write; only verify')
  parser.set_defaults(verify_only=False)

  args = parser.parse_args()

  pc_topic = '/realsense/depth_registered/points'
  joint_state_topic = '/inserter/joint_states'


  if args.verify_only:
    print 'Skipping conversion'
  else:
    convert(args.input, args.output, joint_state_topic, pc_topic)

  verify(args.output)


